"""Which setup steps still need the operator: a confirmed result or a usable default needs nothing.

    python3 -m w2_payload.setup.status --config C cameras|hand|dyn   (exit 0: nothing to do, 1: needs setup)"""
from __future__ import annotations

import argparse
import sys
from typing import Any, Dict, NamedTuple, Optional, Sequence

from ..config import edit as config_edit
from .state import confirmed

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
    if (raw.get("cameras") or {}).get("enable") is False:
        return Status(DEFAULT, "相机已关闭（cameras.enable: false）")
    if confirmed(raw, "cameras"):
        return Status(CONFIGURED, "相机映射已确认")
    return Status(NEEDED, "相机映射尚未确认")


def hand_status(raw: Dict[str, Any]) -> Status:
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


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="status")
    parser.add_argument("--config", required=True)
    parser.add_argument("step", choices=sorted(STATUS))
    args = parser.parse_args(argv)
    _, raw = config_edit.load_rt(args.config)
    status = STATUS[args.step](raw)
    print(f"{status.state}：{status.reason}")
    return 1 if status.needed else 0


if __name__ == "__main__":
    sys.exit(main())
