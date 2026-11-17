"""The unit of the gamepad node (on demand: it needs the controllers running)."""
from __future__ import annotations

from typing import Any, Dict

from robot_runtime.units import Unit


def joystick_units(cfg: Dict[str, Any]) -> Dict[str, Unit]:
    cmd = ("ros2", "launch", "robot_startup", "joystick.launch.py")
    if cfg["joystick"]["device"]:
        cmd += (f"joy_dev:={cfg['joystick']['device']}",)
    return {"joystick": Unit(name="joystick", title="手柄遥操", cmd=cmd, depends=("control",), on_demand=True)}
