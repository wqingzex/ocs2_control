"""Gamepad discovery for the console: the devices `ros2 run joy joy_enumerate_devices` lists."""
from __future__ import annotations

import re
import subprocess
from typing import Any, Dict, List

JOYSTICK_LINE = re.compile(r"^\s*(\d+)\s*:\s*([^:]+?)\s*:")
ENUMERATE_TIMEOUT_S = 3.0


def parse_joysticks(text: str) -> List[Dict[str, Any]]:
    """Parse `ros2 run joy joy_enumerate_devices` output: lines like '0 : name : true : ...'."""
    return [{"id": int(m.group(1)), "name": m.group(2)}
            for m in (JOYSTICK_LINE.match(line) for line in text.splitlines()) if m]


def enumerate_joysticks() -> List[Dict[str, Any]]:
    try:
        out = subprocess.run(["ros2", "run", "joy", "joy_enumerate_devices"], capture_output=True, text=True,
                             check=False, timeout=ENUMERATE_TIMEOUT_S).stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return []
    return parse_joysticks(out)
