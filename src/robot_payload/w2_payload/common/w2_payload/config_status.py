"""Which setup steps still need the operator, and the configuration summary.

A confirmed result or a usable default needs nothing.
    python3 -m w2_payload.config_status status --config C cameras|hand|dyn   (exit 0: nothing to do, 1: needs setup)
    python3 -m w2_payload.config_status check C                              (summary; exit 2: invalid config)"""
from __future__ import annotations

import argparse
import sys
from typing import Any, Dict, NamedTuple, Optional, Sequence

import yaml

from .config import ROLES, ConfigError, load
from .config_edit import confirmed, load_rt


def _register_devices() -> None:
    """Importing a device package registers how its mapping is fingerprinted (config_edit.register_fingerprint)."""
    import w2_camera.camera_setup  # noqa: F401
    import w2_hand.hand_setup  # noqa: F401

CONFIGURED, DEFAULT, NEEDED = "已配置", "默认值", "需配置"
DYN_SIDES = ("left", "right")
DYN_LENGTH = 10  # m, mx, my, mz, ixx, ixy, ixz, iyy, iyz, izz


class Status(NamedTuple):
    state: str
    reason: str

    @property
    def needed(self) -> bool:
        return self.state == NEEDED


def dyn_values(raw: Dict[str, Any], side: str) -> Optional[Sequence[float]]:
    """The load override written for one side, or None when it is empty or not 10 numbers."""
    value = (raw.get("defaults") or {}).get(f"{side}_dyn_param")
    numbers = isinstance(value, (list, tuple)) and len(value) == DYN_LENGTH and \
        all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in value)
    return list(value) if numbers else None


def camera_status(raw: Dict[str, Any]) -> Status:
    _register_devices()
    if (raw.get("cameras") or {}).get("enable") is False:
        return Status(DEFAULT, "相机已关闭（cameras.enable: false）")
    if confirmed(raw, "cameras"):
        return Status(CONFIGURED, "相机映射已确认")
    return Status(NEEDED, "相机映射尚未确认")


def hand_status(raw: Dict[str, Any]) -> Status:
    _register_devices()
    robot = raw.get("robot") or {}
    if robot.get("hand", "linkerhand_l6") == "none":
        return Status(DEFAULT, "没有灵巧手（robot.hand: none）")
    if confirmed(raw, "hand"):
        return Status(CONFIGURED, "左右手实物已确认")
    return Status(NEEDED, "左右手尚未确认")


def dyn_status(raw: Dict[str, Any]) -> Status:
    robot = raw.get("robot") or {}
    hand = robot.get("hand", "linkerhand_l6")
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
    if hand == "linkerhand_l6":
        return Status(DEFAULT, "使用 linkerhand_l6 的内置辨识值")
    return Status(DEFAULT, f"{hand} 没有内置辨识值，使用驱动的默认值（0）；要更准运行 ./scripts/setup.sh dyn")


STATUS = {"cameras": camera_status, "hand": hand_status, "dyn": dyn_status}


def check(args):
    if len(args) != 1:
        print('用法：python3 -m w2_payload.config_status check config/robot.local.yaml', file=sys.stderr)
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
    print(f"  手: {robot['hand']}，接线: {robot['hand_drive']}")
    hardware = raw.get('hardware') or {}
    if robot['hand'] != 'none' and robot['hand_drive'] == 'usb':
        for side, label in (('left', '左手'), ('right', '右手')):
            port = hardware.get('usb_' + side + '_port', '')
            note = '' if port.startswith('/dev/serial/by-id/') else '  [未按序列号配置：./scripts/setup.sh hand]'
            print(f"    {label}: {port or '（未配置）'}{note}")
    cams = cfg['cameras']
    for role in ROLES:
        camera = cams[role]
        description = f"{camera['serial']}，旋转 {camera['rotation']}°" if cams['enable'] and camera['enable'] else '关闭'
        print(f'  相机 {role}: {description}')
    for section, title, enabled in (('cameras', '相机画面与位置', cams['enable']), ('hand', '左右手实物', robot['hand'] != 'none')):
        if enabled:
            status = '已确认' if confirmed(raw, section) else '尚未确认或配置已变化'
            command = 'cam' if section == 'cameras' else 'hand'
            print(f'  {title}: {status}（./scripts/setup.sh {command}）')
    return 0


def main(argv=None) -> int:
    _register_devices()
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
