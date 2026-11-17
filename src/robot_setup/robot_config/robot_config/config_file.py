"""The config file of the machine: edit it in place keeping comments, the confirmation receipts of the setup steps, and
which steps still need the operator.

Edit: ruamel.yaml round trip (comments and order stay), the receipt of a confirmed camera or hand mapping (a fingerprint of
the fields in config.py). Status: whether cameras / ee / dyn still need the operator, and the configuration summary
(`check`)."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
import tempfile
from datetime import datetime, timezone
from typing import Any, Dict, NamedTuple, Optional, Sequence

import yaml

from .config import DYN_LENGTH, FIELDS, ROLES, ConfigError, ee_bus, ee_side, ee_type, load, validate_camera_assignments

# ---- edit the file, confirmation receipts -------------------------------------------------------------------

def _yaml():
    try:
        from ruamel.yaml import YAML
    except ImportError as exc:
        raise RuntimeError("缺少 ruamel.yaml：sudo apt install python3-ruamel.yaml（或 rosdep install）") from exc
    yaml = YAML()  # round-trip: comments, key order and flow style are preserved
    yaml.preserve_quotes = True
    yaml.width = 4096
    return yaml


def load_rt(path: str):
    yaml = _yaml()
    with open(path, encoding="utf-8") as handle:
        data = yaml.load(handle)
    return yaml, ({} if data is None else data)


def save_rt(path: str, yaml, data) -> None:
    directory = os.path.dirname(os.path.abspath(path))
    fd, tmp = tempfile.mkstemp(prefix=".tmp_", suffix=".yaml", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            yaml.dump(data, handle)
        if os.path.exists(path):
            shutil.copymode(path, tmp)  # mkstemp makes the file 0600; keep the mode the file had
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.remove(tmp)
        raise


def _section(data, key: str):
    """The mapping at key (dotted: robot.arm.interface), created when missing."""
    from ruamel.yaml.comments import CommentedMap
    node = data
    for part in key.split("."):
        if part not in node or node[part] is None:
            node[part] = CommentedMap()
        node = node[part]
    return node


def _apply(target, values: Dict[str, Any]) -> None:
    """Write values into a mapping; a dict value is merged into the mapping under that key, however deep."""
    for key, value in values.items():
        if isinstance(value, dict):
            _apply(_section(target, key), value)
        else:
            target[key] = value


def set_cameras(path: str, assignments: Dict[str, Dict[str, Any]], *, enabled: Optional[bool] = None) -> None:
    """assignments: role -> {serial, usb_port?, rotation?, enable?}; other fields are left untouched."""
    unknown = set(assignments) - set(ROLES)
    if unknown:
        raise ValueError(f"未知相机角色: {sorted(unknown)}")
    validate_camera_assignments(assignments)  # never write values start.sh would then refuse
    yaml, data = load_rt(path)
    cameras = _section(data, "cameras")
    if enabled is not None:
        cameras["enable"] = enabled
    for role, values in assignments.items():
        entry = _section(cameras, role)
        for key in ("enable", "serial", "usb_port", "rotation"):
            if key in values and values[key] is not None:
                entry[key] = values[key]
    save_rt(path, yaml, data)


def set_fields(path: str, section: str, values: Dict[str, Any]) -> None:
    """Set scalar fields of one section, dotted for a nested one (e.g. robot.arm.interface)."""
    yaml, data = load_rt(path)
    _apply(_section(data, section), values)
    save_rt(path, yaml, data)


def set_float_list(path: str, section: str, key: str, values) -> None:
    """A list of numbers in flow style ([a, b, c]), the way such lists are written by hand."""
    from ruamel.yaml.comments import CommentedSeq
    yaml, data = load_rt(path)
    seq = CommentedSeq([float(v) for v in values])
    seq.fa.set_flow_style()
    _section(data, section)[key] = seq
    save_rt(path, yaml, data)


def set_usb_ports(profile_path: str, left: Optional[str], right: Optional[str]) -> None:
    """robot.end_effector.interface.left.port / right.port (serial ports of USB hands)."""
    values = {side: {"port": port} for side, port in (("left", left), ("right", right)) if port}
    if values:
        set_fields(profile_path, "robot.end_effector.interface", values)


def camera_serials(path: str) -> Dict[str, str]:
    """role -> serial currently written in the config ('' if unset)."""
    _, data = load_rt(path)
    cameras = data.get("cameras") or {}
    return {role: str((cameras.get(role) or {}).get("serial", "") or "") for role in ROLES}


def save_setup(path: str, section: str, changes: Dict[str, Dict], original: str) -> None:
    """Commit one reviewed step, preserving comments and rejecting concurrent edits."""
    import fcntl
    with open(path + ".setup.lock", "a", encoding="utf-8") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        with open(path, encoding="utf-8") as handle:
            if handle.read() != original:
                raise ValueError("配置文件已被其他程序修改，请取消本次配置并重新进入")
        yaml, data = load_rt(path)
        for name, values in changes.items():
            _apply(_section(data, name), values)
        _section(data, "setup")[section] = receipt(data, section)
        save_rt(path, yaml, data)


# ---- confirmation receipts ----
#
# A receipt belongs to the device mapping the operator reviewed: it stores a fingerprint of the fields that
# mapping consists of (mappings.FIELDS); a step is confirmed only while its fingerprint still matches, so a changed
# serial, port or rotation asks again.


def fingerprint(raw, section):
    """Hash of the mapping fields of a section (cameras, ee)."""
    if section not in FIELDS:
        raise KeyError(f"no mapping fields for '{section}'; known: {sorted(FIELDS)}")
    value = FIELDS[section](raw)
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False).encode()).hexdigest()


def confirmed(raw, section):
    setup = raw.get('setup')
    receipt = setup.get(section) if isinstance(setup, dict) else None
    if not isinstance(receipt, dict):
        return False
    return receipt.get("fingerprint") == fingerprint(raw, section) and receipt.get("confirmed") is True


def receipt(raw, section):
    return {"confirmed": True, "fingerprint": fingerprint(raw, section),
            "confirmed_at": datetime.now(timezone.utc).isoformat(timespec="seconds")}


# ---- which steps still need the operator --------------------------------------------------------------------

CONFIGURED, DEFAULT, NEEDED = "已配置", "默认值", "需配置"
DYN_SIDES = ("left", "right")


class Status(NamedTuple):
    state: str
    reason: str

    @property
    def needed(self) -> bool:
        return self.state == NEEDED


def dyn_values(raw: Dict[str, Any], side: str) -> Optional[Sequence[float]]:
    """The load override written for one side, or None when it is empty or not 10 numbers."""
    value = (((raw.get("robot") or {}).get("end_effector") or {}).get("dyn_param") or {}).get(side)
    numbers = isinstance(value, (list, tuple)) and len(value) == DYN_LENGTH and \
        all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in value)
    return list(value) if numbers else None


def camera_status(raw: Dict[str, Any]) -> Status:
    if (raw.get("cameras") or {}).get("enable") is False:
        return Status(DEFAULT, "相机已关闭（cameras.enable: false）")
    if confirmed(raw, "cameras"):
        return Status(CONFIGURED, "相机映射已确认")
    return Status(NEEDED, "相机映射尚未确认")


def ee_status(raw: Dict[str, Any]) -> Status:
    robot = raw.get("robot") or {}
    if ee_type(robot) == "none":
        return Status(DEFAULT, "没有末端（robot.end_effector.type: none）")
    if ee_bus(robot) == "ethernet":
        return Status(DEFAULT, "以太网（Wuji）：左右手由驱动扫描后按 hand_side 匹配，不用 ./scripts/setup.sh ee 确认")
    if ee_bus(robot) == "usb_can":
        return Status(DEFAULT, "USB-CAN：左右手按 robot.end_effector.interface.left.can / right.can 区分，"
                               "不用 ./scripts/setup.sh ee 确认")
    if confirmed(raw, "ee"):
        return Status(CONFIGURED, "左右手实物已确认")
    return Status(NEEDED, "左右手尚未确认")


def dyn_status(raw: Dict[str, Any]) -> Status:
    robot = raw.get("robot") or {}
    hand = ee_type(robot)
    left, right = (dyn_values(raw, side) for side in DYN_SIDES)
    if robot.get("hardware", "real") != "real":
        return Status(DEFAULT, "不是真机（仿真不需要负载辨识）")
    if left and right:
        return Status(CONFIGURED, "已填写左右臂的负载参数")
    if left or right:
        only = "左" if left else "右"
        return Status(NEEDED, f"只填写了{only}臂的负载参数，另一侧会用默认值")
    if hand == "none":
        return Status(DEFAULT, "没有末端，负载为 0")
    if hand in ("linkerhand_l6", "wuji_hand2"):
        return Status(DEFAULT, f"使用 {hand} 的内置辨识值")
    return Status(DEFAULT, f"{hand} 没有内置辨识值，使用驱动的默认值（0）；要更准运行 ./scripts/setup.sh dyn")


STATUS = {"cameras": camera_status, "ee": ee_status, "dyn": dyn_status}


def check(args):
    if len(args) != 1:
        print('用法：python3 -m robot_config.config_status check config/robot.local.yaml', file=sys.stderr)
        return 2
    path = args[0]
    try:
        cfg = load(path)
    except ConfigError as exc:
        print(f'[配置错误] {path}: {exc}', file=sys.stderr)
        return 2
    with open(path, encoding='utf-8') as handle:
        raw = yaml.safe_load(handle) or {}
    print(f'配置: {path}')
    robot = cfg['robot']
    print(f"  机器人: {robot['entry']}，硬件模式: {robot['hardware']}")
    eef = robot['end_effector']
    print(f"  末端: {eef['type']}，接线: {eef['interface']['bus']}")
    if eef['type'] != 'none' and eef['interface']['bus'] == 'usb_rs485':
        for side, label in (('left', '左手'), ('right', '右手')):
            port = ee_side(raw.get('robot') or {}, side).get('port') or ''
            note = '' if port.startswith('/dev/serial/by-id/') else '  [未按序列号配置：./scripts/setup.sh ee]'
            print(f"    {label}: {port or '（未配置）'}{note}")
    cams = cfg['cameras']
    for role in ROLES:
        camera = cams[role]
        description = f"{camera['serial']}，旋转 {camera['rotation']}°" if cams['enable'] and camera['enable'] else '关闭'
        print(f'  相机 {role}: {description}')
    for section, title, enabled in (('cameras', '相机画面与位置', cams['enable']), ('ee', '左右手实物', robot['end_effector']['type'] != 'none')):
        if enabled:
            status = '已确认' if confirmed(raw, section) else '尚未确认或配置已变化'
            command = 'cam' if section == 'cameras' else 'ee'
            print(f'  {title}: {status}（./scripts/setup.sh {command}）')
    return 0


def main(argv=None) -> int:
    """status --config FILE STEP | check FILE (the commands of setup_steps)."""
    args = sys.argv[1:] if argv is None else list(argv)
    if args and args[0] == "check":
        return check(args[1:])
    parser = argparse.ArgumentParser(prog="config_status")
    parser.add_argument("command", choices=["status"])
    parser.add_argument("--config", required=True)
    parser.add_argument("step", choices=sorted(STATUS))
    options = parser.parse_args(args)
    _, raw = load_rt(options.config)
    status = STATUS[options.step](raw)
    print(f"{status.state}：{status.reason}")
    return 1 if status.needed else 0
