"""Bus measurement through the end effector of this machine: the command behind `start.sh bus_probe`.

    python3 -m robot_app.bus_probe --config C [--set SECTION.FIELD=VALUE] [--side left|right|both] [bus_probe options]

The device, its side and its serial port come from the config (robot.end_effector). Options this module does not know
(--duration, --cycle-hz, --io-mode, --yes, ...) go to the bus_probe program of end_effector_hardware (--help there)."""
from __future__ import annotations

import argparse
import subprocess
import sys
from typing import Any, Dict, List, Optional, Sequence

from .config import ConfigError, load

PROBE_DEVICES = ("linkerhand_l6", "xhand1")  # the end effectors that sit on a USB-RS485 adapter
CONTROLLER_PATTERN = "lib/controller_manager/ros2_control_node"


class ProbeError(RuntimeError):
    """The measurement cannot be started for this configuration (message is user facing)."""


def probe_commands(cfg: Dict[str, Any], side: str = "both", extra: Sequence[str] = ()) -> List[List[str]]:
    """One bus_probe command line per side to measure."""
    end_effector = cfg["robot"]["end_effector"]
    device = end_effector["type"]
    if device not in PROBE_DEVICES:
        raise ProbeError(f"末端 {device} 没有可测的 USB 总线（支持：{', '.join(PROBE_DEVICES)}；"
                         "--end-effector 可以临时指定）")
    bus = end_effector["interface"]["bus"]
    if bus != "usb_rs485":
        raise ProbeError(f"末端总线是 {bus}：目前只能测 usb_rs485（arm_rs485 的总线只在臂驱动运行时才有）")
    commands = []
    for name in ("left", "right") if side == "both" else (side,):
        port = end_effector["interface"][name]["port"]
        if not port:
            raise ProbeError(f"robot.end_effector.interface.{name}.port 没有配置：先运行 ./scripts/setup.sh ee")
        commands.append(["ros2", "run", "end_effector_hardware", "bus_probe", "--device", device, "--side", name,
                         "--port", port, *extra])
    return commands


def controller_running() -> bool:
    return subprocess.run(["pgrep", "-f", CONTROLLER_PATTERN], capture_output=True).returncode == 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="start.sh bus_probe", description=__doc__.splitlines()[0],
        epilog="passed on to bus_probe: --duration S  --warmup S  --cycle-hz N  --io-mode async|sync  --max-gap-ms N  "
               "--csv FILE  --yes")
    parser.add_argument("--config", required=True)
    parser.add_argument("--set", dest="overrides", action="append", default=[], metavar="SECTION.FIELD=VALUE")
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    args, extra = parser.parse_known_args(argv)

    try:
        cfg = load(args.config, args.overrides)
    except ConfigError as exc:
        print(f"[配置错误] {args.config}: {exc}", file=sys.stderr)
        return 2
    try:
        commands = probe_commands(cfg, args.side, extra)
    except ProbeError as exc:
        print(f"[错误] {exc}", file=sys.stderr)
        return 2
    if controller_running():
        print("[错误] 运控（ros2_control_node）正在运行，串口被它占着：先 ./scripts/start.sh down", file=sys.stderr)
        return 2
    code = 0
    for command in commands:
        code = max(code, subprocess.run(command, check=False).returncode)
    return code


if __name__ == "__main__":
    sys.exit(main())
