"""Map the 'robot' section of the config to robot_bringup launch arguments (as the former quick_start.sh)."""
from __future__ import annotations

from typing import Any, Dict, Tuple

from .config import ENTRIES


def controller_launch(cfg: Dict[str, Any], profile_path: str) -> Tuple[str, Dict[str, str]]:
    """Return (launch file in robot_bringup, launch arguments). RViz is not included (own unit)."""
    robot = cfg["robot"]
    launch_file, robot_name = ENTRIES[robot["entry"]]
    args: Dict[str, str] = {
        "robot": robot_name,
        "launch_mode": "control_only",
        # The end effector comes from robot.hand only.
        "use_profile_eef": "false",
    }

    hardware = robot["hardware"]
    hand = robot["hand"]
    if hand != "none":
        args["type"] = hand
        if hardware == "real":
            # Keep the low-level xacro/SDK selector internal to the launch adapter.
            drive = "marvin" if robot["hand_drive"] == "485" else robot["hand_drive"]
            args["xacro_left_ee_drive_type"] = drive
            args["xacro_right_ee_drive_type"] = drive
            # USB hands need the real_usb hardware profile (same rule as the former quick_start.sh).
            hardware = "real_usb" if drive == "usb" else "real"
    args["hardware"] = hardware

    args["robot_profile"] = profile_path  # the same file carries platform / hardware / control
    return launch_file, args


# entry -> (visualization launch file in robot_visualize, extra launch arguments)
VIZ_LAUNCH = {
    "split_body": ("humanoid.launch.py", {}),
    "whole_body": ("humanoid.launch.py", {}),
    "dual_arm": ("manipulator.launch.py", {"type": "dual"}),
}


def viz_launch(cfg: Dict[str, Any]) -> Tuple[str, str, Dict[str, str]]:
    """(package, launch file, arguments) that only show the model in RViz: no hardware, no controller."""
    launch_file, extra = VIZ_LAUNCH[cfg["robot"]["entry"]]
    return "robot_visualize", launch_file, {"robot": ENTRIES[cfg["robot"]["entry"]][1], **extra}
