"""Tool load identification (./scripts/setup.sh dyn): run the Marvin wizard per arm, then write the result to
defaults.<side>_dyn_param. Without it the robot description picks the load by end effector type."""
from __future__ import annotations

import json
import math
import os
import subprocess
import time
from typing import Callable, List, Optional, Sequence

from . import config_edit
from .device_setup import main_for
from .config_status import DYN_LENGTH, DYN_SIDES, dyn_status, dyn_values

WIZARD = ("ros2", "run", "marvin_hardware", "tool_dyn_identify_wizard")
ARM_OF_SIDE = {"left": "A", "right": "B"}  # the wizard's own names for the arms
YES = ("y", "yes", "是")


def output_dir(ws_root: str) -> str:
    return os.path.join(ws_root, "logs", "dyn_identify")


def result_path(ws_root: str, arm: str) -> str:
    return os.path.join(output_dir(ws_root), f"result_{arm}.json")


MTIME_TOLERANCE_S = 0.01  # file timestamps can lag time.time() by up to ~1 ms (coarse kernel clock, overlayfs)


def read_result(path: str, newer_than: float) -> Optional[List[float]]:
    """The 10 identified values the wizard wrote after newer_than, or None."""
    try:
        if os.path.getmtime(path) < newer_than - MTIME_TOLERANCE_S:
            return None
        with open(path, encoding="utf-8") as handle:
            values = json.load(handle)["params"]
    except (OSError, ValueError, KeyError, TypeError):
        return None
    ok = isinstance(values, list) and len(values) == DYN_LENGTH and \
        all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values)
    return [float(v) for v in values] if ok else None


def remember(ws_root: str, side: str, values: Optional[Sequence[float]]) -> str:
    """Keep the value that is about to be replaced; returns the file it went to."""
    os.makedirs(output_dir(ws_root), exist_ok=True)
    path = os.path.join(output_dir(ws_root), f"previous_{side}_{time.strftime('%Y%m%d_%H%M%S')}.json")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump({"side": side, "params": list(values) if values else []}, handle)
    return path


def step_dyn(config_path: str, ws_root: str, io, runner: Callable[..., int] = subprocess.call) -> bool:
    """Identify the arms the operator picks (default: none, keep the current values)."""
    _, raw = config_edit.load_rt(config_path)
    status = dyn_status(raw)
    io.info(f"负载参数 {status.state}：{status.reason}")
    if (raw.get("robot") or {}).get("hardware", "real") != "real":
        return True
    ok = True
    for side in DYN_SIDES:
        arm = ARM_OF_SIDE[side]
        label = "左" if side == "left" else "右"
        if io.ask(f"辨识{label}臂的工具负载？需要真机，约 10 分钟，手臂会运动 y/n", "n").lower() not in YES:
            continue
        started = time.time()
        code = runner(list(WIZARD), env=dict(os.environ, TOOL_DYN_ARM=arm), cwd=ws_root)
        values = read_result(result_path(ws_root, arm), started)
        if code != 0 or values is None:
            io.error(f"{label}臂辨识没有得到结果（退出码 {code}）；日志见 logs/dyn_identify/")
            ok = False
            continue
        _, raw = config_edit.load_rt(config_path)
        old = dyn_values(raw, side)
        io.info(f"{label}臂辨识结果: {[round(v, 6) for v in values]}")
        io.info(f"当前值: {list(old) if old else '空（使用默认）'}")
        if io.ask("写入配置？y/n", "y").lower() not in YES:
            io.info("未写入，保持原值")
            continue
        io.info(f"原值已记录到 {remember(ws_root, side, old)}")
        config_edit.set_float_list(config_path, "defaults", f"{side}_dyn_param", values)
        io.info(f"已写入 defaults.{side}_dyn_param")
    return ok


def reset_dyn(config_path: str, ws_root: str, io) -> bool:
    """Back to the defaults: both overrides are emptied (the old values are kept in logs/dyn_identify)."""
    _, raw = config_edit.load_rt(config_path)
    for side in DYN_SIDES:
        old = dyn_values(raw, side)
        if old:
            io.info(f"{side} 原值已记录到 {remember(ws_root, side, old)}")
        config_edit.set_float_list(config_path, "defaults", f"{side}_dyn_param", [])
    io.info("已清空 defaults.left_dyn_param / right_dyn_param，使用默认负载")
    return True


def main(argv=None) -> int:
    return main_for(step_dyn, argv, reset=reset_dyn)


if __name__ == '__main__':
    import sys
    sys.exit(main())
