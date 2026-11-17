"""Map the 'robot' section of the config to robot_bringup launch arguments (as the former quick_start.sh)."""
from __future__ import annotations

from typing import Any, Dict, Tuple

from .config import ENTRIES


def controller_launch(cfg: Dict[str, Any], profile_path: str) -> Tuple[str, Dict[str, str]]:
    """Return (launch file in robot_bringup, launch arguments). RViz is not included (own unit)."""
    robot = cfg["robot"]
    launch_file, robot_name = ENTRIES[robot["entry"]]
    eef = robot["end_effector"]
    args: Dict[str, str] = {
        "robot": robot_name,
        "launch_mode": "control_only",
        "hardware": robot["hardware"],
        # The same values as in the config; passed explicitly so the command line shows them (the launch would also
        # read them from robot_profile).
        "end_effector": eef["type"],
    }
    if eef["type"] != "none" and robot["hardware"] == "real":
        args["end_effector_bus"] = eef["interface"]["bus"]

    args["robot_profile"] = profile_path  # the same file carries the parts' interfaces and the control section
    return launch_file, args


# entry -> (visualization launch file in robot_visualize, extra launch arguments)
VIZ_LAUNCH = {
    "split_body": ("humanoid.launch.py", {}),
    "whole_body": ("humanoid.launch.py", {}),
    "dual_arm": ("manipulator.launch.py", {"arms": "dual"}),
}


def viz_launch(cfg: Dict[str, Any]) -> Tuple[str, str, Dict[str, str]]:
    """(package, launch file, arguments) that only show the model in RViz: no hardware, no controller."""
    launch_file, extra = VIZ_LAUNCH[cfg["robot"]["entry"]]
    return "robot_visualize", launch_file, {"robot": ENTRIES[cfg["robot"]["entry"]][1], **extra}
