"""Which setup steps still need the operator, and the configuration summary.

A confirmed result or a usable default needs nothing.
    python3 -m robot_config.config_status status --config C cameras|ee|dyn   (exit 0: nothing to do, 1: needs setup)
    python3 -m robot_config.config_status check C                              (summary; exit 2: invalid config)"""
from __future__ import annotations

import argparse
import sys
from typing import Any, Dict, NamedTuple, Optional, Sequence

import yaml

from .config import DYN_LENGTH, ROLES, ConfigError, ee_bus, ee_side, ee_type, load
from .config_edit import confirmed, load_rt


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


if __name__ == "__main__":
    sys.exit(main())
