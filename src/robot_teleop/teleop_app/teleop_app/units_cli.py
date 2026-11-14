"""Unit control CLI behind start.sh (exit codes: 0 ok, 1 unit error / aborted, 2 config error).

    ... --config C --ws-root W up [unit ...] [--foreground] | down [unit ...] [--yes] | restart | status [--hz] | logs"""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from typing import Callable, List, Optional

from .config import ConfigError, load
from .control_ready import FAILED, PENDING, wait_ready
from xr_teleop import xr_ready
from .diag import clock_warning, make_bundle, report_path
from .events import session_info
from .ros_probe import FSM_HOLD, fsm_name
from .robot import viz_launch
from robot_runtime.supervisor import Supervisor, UnitError, UnitStatus
from .units import build_units


def _fmt_uptime(seconds: Optional[float]) -> str:
    if seconds is None:
        return "-"
    seconds = int(seconds)
    return f"{seconds // 3600:02d}:{seconds % 3600 // 60:02d}:{seconds % 60:02d}"


def _pad(text: str, width: int) -> str:
    """Left-align by terminal display width (CJK characters take two columns)."""
    import unicodedata
    shown = sum(2 if unicodedata.east_asian_width(c) in ("W", "F") else 1 for c in text)
    return text + " " * max(1, width - shown)


def print_table(statuses: List[UnitStatus], rates: Optional[dict] = None, out=None) -> None:
    out = out or sys.stdout
    print(_pad("模块", 30) + _pad("状态", 6) + _pad("PID", 9) + _pad("运行时长", 10) + _pad("话题 Hz", 40) + "最近错误",
          file=out)
    for s in statuses:
        rate = (rates or {}).get(s.name, "")
        err = (s.last_error or "")[:80]
        print(_pad(f"{s.name}({s.title})", 30) + _pad("运行" if s.running else "停止", 6) + _pad(str(s.pid or "-"), 9)
              + _pad(_fmt_uptime(s.uptime_s), 10) + _pad(rate, 40) + err, file=out)


def make_hold_hook(assume_yes: bool, ask: Callable[[str], bool],
                   probe_factory: Optional[Callable[[], object]] = None) -> Callable[[str], None]:
    """pre_stop hook: bring the FSM to HOLD before the controllers are stopped (asks first)."""
    def default_factory():
        from .ros_probe import RosProbe
        return RosProbe("teleop_app_stop")

    factory = probe_factory or default_factory

    def hook(name: str) -> None:
        if name != "control":
            return
        try:
            probe = factory()
        except Exception as exc:  # no ROS environment: cannot check, ask the operator
            if not (assume_yes or ask(f"无法读取 FSM 状态（{exc}），仍然停止运控？")):
                raise UnitError("已取消停止运控")
            return
        try:
            state = probe.wait_fsm(2.0)
            if state in (None, FSM_HOLD):
                return
            if not (assume_yes or ask(f"当前 FSM 为 {fsm_name(state)}，先切到 HOLD 再停止运控？")):
                raise UnitError("已取消停止运控")
            if not probe.request_hold(5.0):
                if not (assume_yes or ask("5 秒内未回到 HOLD，仍然停止运控？")):
                    raise UnitError("已取消停止运控（FSM 未回到 HOLD）")
        finally:
            probe.close()
    return hook


def _ask_tty(question: str) -> bool:
    if not sys.stdin.isatty():
        print(f"{question} [非交互环境，按否处理；可加 --yes]", file=sys.stderr)
        return False
    return input(f"{question} [y/N] ").strip().lower() in ("y", "yes")


def _rates(units, seconds: float = 2.0) -> dict:
    from .ros_probe import RosProbe
    probe = RosProbe("teleop_app_status")
    try:
        topics = [t for u in units.units.values() for t in u.topics]
        probe.watch(topics)
        time.sleep(0.5)
        probe.watch(topics)  # topics discovered late
        time.sleep(seconds)
        result = {}
        for unit in units.units.values():
            parts = []
            for topic in unit.topics:
                r = probe.rate(topic, seconds)
                parts.append(f"{topic.rsplit('/', 1)[-1]}={'-' if r is None else r}")
            result[unit.name] = " ".join(parts)
        if probe.fsm_state is not None and "control" in result:
            result["control"] = f"FSM={fsm_name(probe.fsm_state)}"
        return result
    finally:
        probe.close()



def _control_came_up(sup: Supervisor, status: UnitStatus) -> bool:
    """Wait until every controller is active; say so, or say clearly why not (the launch process outlives a failure)."""
    print(">>> 等待控制器全部激活 ...", flush=True)
    verdict = wait_ready(status.log, lambda: sup.is_running("control"))
    if verdict.state == PENDING:
        print("[警告] 运控还在启动，超时仍有控制器没有激活；看 logs/control/latest.log，不要当作已经可用", file=sys.stderr)
        return True
    if verdict.state != FAILED:
        print(">>> 运控就绪：控制器全部激活")
        return True
    print("[错误] 运控启动失败，机器人没有被控制：", file=sys.stderr)
    for line in verdict.reasons:
        print(f"    {line[:200]}", file=sys.stderr)
    print("    完整日志: logs/control/latest.log。先 ./scripts/start.sh down control 停掉，"
          "确认没有别的运控在运行（ps -eo pid,args | grep ros2_control_node），再重新启动", file=sys.stderr)
    return False


XR_WAIT_S = 30.0  # the headset connects within seconds once its app points at the PC Service


def _xr_came_up(sup: Supervisor, status: UnitStatus) -> bool:
    """Wait for the first headset frame; a bridge that waits for a missing headset is a warning, a dead one an error."""
    print(">>> 等待头显连接 ...", flush=True)
    verdict = wait_ready(status.log, lambda: sup.is_running("xr"), XR_WAIT_S, judge=xr_ready.judge)
    if verdict.state == xr_ready.READY:
        print(f">>> 头显已连接，正在接收追踪数据（设备 ID {verdict.reasons[0]}）")
        return True
    if verdict.state == xr_ready.FAILED:
        print("[错误] XR 桥没有在运行：", file=sys.stderr)
        for line in verdict.reasons:
            print(f"    {line[:200]}", file=sys.stderr)
        print("    完整日志: logs/xr/latest.log", file=sys.stderr)
        return False
    print(f"[警告] {XR_WAIT_S:.0f} 秒内头显没有连上，现在遥操作不会有数据：", file=sys.stderr)
    for line in verdict.reasons:
        print(f"    {line[:200]}", file=sys.stderr)
    print("    在头显的 XRoboToolkit App 里选 PC Service 地址（setup.sh xr 打印过）并开启 Data & Control 的 Send；"
          "桥会继续等，连上后 /teleop/connection_status 变 true。日志: logs/xr/latest.log", file=sys.stderr)
    return True


def up_names(units, names: List[str], extra: List[str], no_rviz: bool = False) -> Optional[List[str]]:
    """Units for `up`: the named ones (default: all regular ones) plus the extra on-demand units (--with).
    Naming the controller brings RViz with it, since the panels are how its state is watched (--no-rviz: headless)."""
    extra = list(extra)
    if "control" in names and "rviz" in units.names() and "rviz" not in names + extra and not no_rviz:
        extra.append("rviz")
    if not extra:
        return names or None
    return (names or units.start_order()) + extra


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="start.sh", description="W2 teleop units")
    parser.add_argument("--config", required=True)
    parser.add_argument("--ws-root", required=True)
    parser.add_argument("--set", dest="overrides", action="append", default=[], metavar="SECTION.FIELD=VALUE",
                        help="override a config field for this run (repeatable)")
    sub = parser.add_subparsers(dest="cmd", required=True)
    p_up = sub.add_parser("up")
    p_up.add_argument("units", nargs="*")
    p_up.add_argument("--foreground", action="store_true", help="wait; Ctrl+C stops what was started")
    p_up.add_argument("--with", dest="extra", action="append", default=[], metavar="UNIT",
                      help="also start an on-demand unit (e.g. rviz); repeatable")
    p_up.add_argument("--no-rviz", action="store_true", help="naming control does not start RViz with it")
    p_down = sub.add_parser("down")
    p_down.add_argument("units", nargs="*")
    p_down.add_argument("--yes", action="store_true", help="do not ask before leaving OCS2")
    p_restart = sub.add_parser("restart")
    p_restart.add_argument("unit")
    p_restart.add_argument("--yes", action="store_true")
    p_status = sub.add_parser("status")
    p_status.add_argument("--hz", action="store_true", help="also measure topic rates (2 s)")
    sub.add_parser("viz", help="show the robot model in RViz only (no hardware, no controller)")
    p_logs = sub.add_parser("logs")
    p_logs.add_argument("unit")
    p_logs.add_argument("-f", "--follow", action="store_true")
    p_logs.add_argument("-n", type=int, default=100)
    p_diag = sub.add_parser("diag", help="collect what is needed to find out what went wrong (logs of the last minutes, "
                                         "fault codes, health, the flight recorder rings) into one file and print the report")
    p_diag.add_argument("--minutes", type=float, default=10.0, help="how far back to look (default 10)")
    p_diag.add_argument("reason", nargs="*", help="a note on what happened")
    args = parser.parse_args(argv)

    # diag must work when the config is broken: it is needed most then, and it only needs the path.
    if args.cmd == "diag":
        if args.minutes <= 0:
            print("[错误] --minutes 要大于 0", file=sys.stderr)
            return 2
        bundle = make_bundle(args.ws_root, os.path.abspath(args.config), args.minutes, " ".join(args.reason))
        with open(report_path(bundle), encoding="utf-8") as handle:
            print(handle.read())
        print(f">>> 诊断包已生成: {bundle}\n>>> 报告（同样在包里）: {report_path(bundle)}")
        return 0
    try:
        cfg = load(args.config, args.overrides)
    except ConfigError as exc:
        print(f"[配置错误] {args.config}: {exc}", file=sys.stderr)
        return 2
    if args.cmd == "viz":
        package, launch_file, launch_args = viz_launch(cfg)
        cmd = ["ros2", "launch", package, launch_file, *(f"{k}:={v}" for k, v in launch_args.items())]
        print(">>> " + " ".join(cmd[1:]), flush=True)
        os.execvp(cmd[0], cmd)
    units = build_units(cfg, os.path.abspath(args.config), args.ws_root)
    sup = Supervisor(units, os.path.join(args.ws_root, "run"), os.path.join(args.ws_root, "logs"))

    try:
        if args.cmd == "up":
            sup.events.record("session", source="start.sh", **session_info(args.ws_root, cfg))
            names = up_names(units, args.units, args.extra, args.no_rviz)
            for name in units.launch_order(names):
                status = sup.start(name)
                print(f">>> {status.title}（{name}）已启动，日志: logs/{name}/latest.log")
                if name == "control" and status.log and not _control_came_up(sup, status):
                    return 1  # the units after it (RViz, joystick ...) depend on it
                if name == "xr" and status.log and not _xr_came_up(sup, status):
                    return 1
            if not args.foreground:
                print_table(sup.status_all())
                return 0
            print(">>> 前台运行中，Ctrl+C 停止本次启动的模块（运控会先切 HOLD）")
            stop = {"flag": False}
            signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
            signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))
            while not stop["flag"]:
                time.sleep(0.5)
            sup.down(units.start_order(names), pre_stop=make_hold_hook(True, _ask_tty))
            print(">>> 已全部停止")
            return 0
        if args.cmd == "down":
            for status in sup.down(args.units or None, pre_stop=make_hold_hook(args.yes, _ask_tty)):
                print(f">>> {status.title}（{status.name}）已停止")
            return 0
        if args.cmd == "restart":
            sup.stop(args.unit, pre_stop=make_hold_hook(args.yes, _ask_tty))
            status = sup.start(args.unit)
            print(f">>> {status.title}（{args.unit}）已重启")
            return 0
        if args.cmd == "status":
            print_table(sup.status_all(), _rates(units) if args.hz else None)
            warning = clock_warning()
            if warning:
                print(f"\n[警告] {warning}")
            return 0
        if args.cmd == "logs":
            units[args.unit]  # validates the name
            path = sup.log_path(args.unit)
            if not path:
                print(f"{args.unit} 还没有日志", file=sys.stderr)
                return 1
            tail = ["tail", "-n", str(args.n)] + (["-F"] if args.follow else []) + [path]
            return subprocess.call(tail)
    except (UnitError, KeyError, ValueError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
