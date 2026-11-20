"""The unit of the simulated chassis (hardware = mock_components): it moves with base_velocity_command."""
from __future__ import annotations

from typing import Any, Dict

from robot_runtime.units import Unit


def base_units(cfg: Dict[str, Any]) -> Dict[str, Unit]:
    robot = cfg["robot"]
    # Isaac simulates its own base; a real chassis has no driver here yet; a dual-arm entry has no chassis.
    if robot["hardware"] != "mock_components" or not robot.get("base", {}).get("type"):
        return {}
    return {"base": Unit(name="base", title="底盘（仿真）", cmd=("python3", "-m", "robot_app.base_sim"),
                         topics=("/base_odom",))}
