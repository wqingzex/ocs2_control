"""Recording and playback of rosbag2 files: topic sets, listing and deleting bags, the commands behind start.sh.

    python3 -m robot_app.recording --config C --ws-root W record [NAME] [--topics joint|xr|collect|/a,/b]
    ... play NAME [--rate R] [--loop] [--paused] [--start SECONDS] [--xr] | bags [--delete NAME --yes]"""
from __future__ import annotations

import argparse
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import yaml

from launch_helpers.devices.camera import camera_specs
from .config import TOPIC_RE, ConfigError, load

JOINT_TOPICS = ("/joint_states", "/fsm_state")
XR_TOPICS = (
    "/teleop/head_pose", "/teleop/left_ee_pose", "/teleop/right_ee_pose", "/teleop/controller_state",
    "/teleop/trigger_values", "/teleop/connection_status",
    "/left_target", "/right_target", "/left_current_pose", "/right_current_pose", "/fsm_state",
    # Recorded to see why the base moved. Never replayed: a recorded left stick key and stick values played back to a
    # robot would drive the base.
    "/teleop/thumbstick_axes", "/base_velocity_command",
)
XR_REPLAY_TOPICS = XR_TOPICS[:5]  # what the controller needs from /xr_target_node (the first five, no sticks)
SET_NAMES = ("joint", "xr", "collect")
NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")
MIN_FREE_BYTES = 2 * 1024 ** 3  # refuse to start a recording on a nearly full disk


class RecordingError(RuntimeError):
    """A recording or playback request cannot be carried out (message is user facing)."""


def bag_root(cfg: Dict[str, Any], ws_root: str) -> str:
    folder = cfg["record"]["dir"] or "bags"
    return os.path.abspath(folder if os.path.isabs(folder) else os.path.join(ws_root, folder))


def camera_topics(cfg: Dict[str, Any]) -> List[str]:
    return [f"/{spec.name}/color/image_raw/compressed" for spec in camera_specs(cfg)]


def topics_for(cfg: Dict[str, Any], selection: str = "joint") -> List[str]:
    """Topics of a set (joint | xr | collect) or of a comma separated list, plus record.extra_topics."""
    if selection == "joint":
        topics = list(JOINT_TOPICS)
    elif selection == "xr":
        topics = list(XR_TOPICS)
    elif selection == "collect":
        cameras = camera_topics(cfg)
        if not cameras:
            raise RecordingError("数采需要相机：先运行 ./scripts/setup.sh cam，或检查 cameras.enable")
        topics = list(JOINT_TOPICS) + cameras
    else:
        topics = [t.strip() for t in selection.split(",") if t.strip()]
        bad = [t for t in topics if not TOPIC_RE.match(t)]
        if not topics or bad:
            raise RecordingError(f"话题集应为 {'/'.join(SET_NAMES)} 或逗号分隔的话题名，无效: {bad or selection!r}")
    return list(dict.fromkeys(topics + list(cfg["record"]["extra_topics"])))


def check_name(name: str) -> str:
    if not NAME_RE.match(name):
        raise RecordingError(f"录包名只能用字母、数字、_ . -，且以字母或数字开头（最长 64）: {name!r}")
    return name


def new_bag_name(selection: str, now: Optional[float] = None) -> str:
    prefix = selection if selection in SET_NAMES else "custom"
    return f"{prefix}_{time.strftime('%Y%m%d_%H%M%S', time.localtime(now))}"


def free_bytes(path: str) -> int:
    while not os.path.exists(path):
        path = os.path.dirname(path) or "/"
    return shutil.disk_usage(path).free


def record_command(cfg: Dict[str, Any], ws_root: str, selection: str = "joint",
                   name: Optional[str] = None) -> Tuple[Tuple[str, ...], str]:
    """(ros2 bag record command, bag directory); creates the bag root, the bag itself is made by ros2."""
    topics = topics_for(cfg, selection)
    root = bag_root(cfg, ws_root)
    path = os.path.join(root, check_name(name) if name else new_bag_name(selection))
    if os.path.exists(path):
        raise RecordingError(f"录包已存在: {path}")
    os.makedirs(root, exist_ok=True)
    if free_bytes(root) < MIN_FREE_BYTES:
        raise RecordingError(f"[SYS-002] 磁盘剩余空间不足 {MIN_FREE_BYTES // 1024 ** 3} GB，已拒绝开始录制: {root}")
    return ("ros2", "bag", "record", "-o", path, "--topics", *topics), path


def bag_info(path: str) -> Dict[str, Any]:
    """Summary of one bag directory; a bag still being written has no metadata.yaml yet."""
    size = sum(os.path.getsize(os.path.join(path, f)) for f in os.listdir(path) if os.path.isfile(os.path.join(path, f)))
    info: Dict[str, Any] = {"name": os.path.basename(path), "size_bytes": size, "duration_s": None, "messages": None,
                            "topics": [], "started": os.path.getmtime(path), "recording": True}
    try:
        with open(os.path.join(path, "metadata.yaml"), encoding="utf-8") as handle:
            meta = yaml.safe_load(handle)["rosbag2_bagfile_information"]
        info.update(duration_s=meta["duration"]["nanoseconds"] / 1e9, messages=meta["message_count"], recording=False,
                    topics=[t["topic_metadata"]["name"] for t in meta["topics_with_message_count"]],
                    started=meta["starting_time"]["nanoseconds_since_epoch"] / 1e9)
    except (OSError, yaml.YAMLError, KeyError, TypeError):
        pass  # no (or unreadable) metadata: shown as recording / unknown
    return info


def list_bags(root: str) -> List[Dict[str, Any]]:
    """Bags under root, newest first."""
    if not os.path.isdir(root):
        return []
    paths = [os.path.join(root, n) for n in os.listdir(root) if NAME_RE.match(n) and os.path.isdir(os.path.join(root, n))]
    return sorted((bag_info(p) for p in paths), key=lambda info: info["started"], reverse=True)


def bag_path(root: str, name: str) -> str:
    """Directory of a bag under root; refuses anything that is not a plain direct child."""
    path = os.path.join(root, check_name(name))
    if os.path.islink(path) or not os.path.isdir(path) or os.path.dirname(os.path.realpath(path)) != os.path.realpath(root):
        raise RecordingError(f"录包不存在: {name}")
    return path


def resolve_bag(root: str, ref: str) -> str:
    """A bag by its name under root, or else by a directory path."""
    try:
        return bag_path(root, ref)
    except RecordingError:
        if os.path.isdir(ref):
            return os.path.abspath(ref)
        raise


def delete_bag(root: str, name: str) -> None:
    shutil.rmtree(bag_path(root, name))


def play_command(path: str, rate: float = 1.0, loop: bool = False, paused: bool = False, start: float = 0.0,
                 xr: bool = False) -> Tuple[str, ...]:
    if rate <= 0 or start < 0:
        raise RecordingError("--rate 必须大于 0，--start 不能为负")
    cmd: Tuple[str, ...] = ("ros2", "bag", "play", path)
    if rate != 1.0:
        cmd += ("--rate", str(rate))
    if loop:
        cmd += ("--loop",)
    if paused:
        cmd += ("--start-paused",)
    if start:
        cmd += ("--start-offset", str(start))
    if xr:
        cmd += ("--topics", *XR_REPLAY_TOPICS)
    return cmd


def format_size(size: float) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if size < 1024 or unit == "GB":
            return f"{size:.0f} {unit}" if unit == "B" else f"{size:.1f} {unit}"
        size /= 1024
    return ""


def format_duration(seconds: Optional[float]) -> str:
    return "-" if seconds is None else f"{int(seconds) // 60:02d}:{int(seconds) % 60:02d}"


def print_bags(bags: Sequence[Dict[str, Any]], out=None) -> None:
    from .units_cli import _pad
    out = out or sys.stdout
    print(_pad("名称", 34) + _pad("时长", 8) + _pad("大小", 11) + _pad("消息数", 9) + "话题", file=out)
    for bag in bags:
        topics = "录制中/未完成" if bag["recording"] else " ".join(bag["topics"])
        print(_pad(bag["name"], 34) + _pad(format_duration(bag["duration_s"]), 8) + _pad(format_size(bag["size_bytes"]), 11)
              + _pad("-" if bag["messages"] is None else str(bag["messages"]), 9) + topics, file=out)


def _play(cmd: Sequence[str], xr: bool) -> int:
    stub = subprocess.Popen(["ros2", "run", "xr_teleop", "xr_stub.py", "--silent"]) if xr else None
    try:
        return subprocess.call(list(cmd))
    finally:
        if stub is not None:
            stub.send_signal(signal.SIGINT)
            stub.wait(timeout=5)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="start.sh", description="robot_control_suite recording and playback")
    parser.add_argument("--config", required=True)
    parser.add_argument("--ws-root", required=True)
    parser.add_argument("--set", dest="overrides", action="append", default=[], metavar="SECTION.FIELD=VALUE")
    sub = parser.add_subparsers(dest="cmd", required=True)
    p_rec = sub.add_parser("record")
    p_rec.add_argument("name", nargs="?")
    p_rec.add_argument("--topics", default="joint", help="joint (default) | xr | collect | /a,/b")
    p_play = sub.add_parser("play")
    p_play.add_argument("bag")
    p_play.add_argument("--rate", type=float, default=1.0)
    p_play.add_argument("--loop", action="store_true")
    p_play.add_argument("--paused", action="store_true", help="start paused (resume with space)")
    p_play.add_argument("--start", type=float, default=0.0, metavar="SECONDS")
    p_play.add_argument("--xr", action="store_true", help="replay the XR input to the controller (xr_stub stands in)")
    p_bags = sub.add_parser("bags")
    p_bags.add_argument("--delete", metavar="NAME")
    p_bags.add_argument("--yes", action="store_true")
    args = parser.parse_args(argv)

    try:
        cfg = load(args.config, args.overrides)
    except ConfigError as exc:
        print(f"[配置错误] {args.config}: {exc}", file=sys.stderr)
        return 2
    root = bag_root(cfg, args.ws_root)
    try:
        if args.cmd == "record":
            cmd, path = record_command(cfg, args.ws_root, args.topics, args.name)
            print(f">>> 录制 {' '.join(cmd[6:])}\n>>> 保存到 {path}（Ctrl+C 结束）", flush=True)
            os.execvp(cmd[0], list(cmd))
        if args.cmd == "play":
            path = resolve_bag(root, args.bag)
            cmd = play_command(path, args.rate, args.loop, args.paused, args.start, args.xr)
            print(">>> " + " ".join(cmd), flush=True)
            return _play(cmd, args.xr)
        if args.delete:
            if not args.yes and not (sys.stdin.isatty() and input(f"删除录包 {args.delete}？[y/N] ").strip().lower() == "y"):
                print("已取消（非交互环境请加 --yes）", file=sys.stderr)
                return 1
            delete_bag(root, args.delete)
            print(f">>> 已删除 {args.delete}")
            return 0
        print_bags(list_bags(root))
        return 0
    except RecordingError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
