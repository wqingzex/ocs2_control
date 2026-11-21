"""The units of a run: the controllers and RViz are built here, the devices' units come from their packages."""
from __future__ import annotations

import glob
import os
from dataclasses import replace
from typing import Any, Dict, Optional, Tuple

from launch_helpers.devices.base import base_units
from robot_config.config import DEXCAP_PORTS, teleop_type
from robot_log_tools.logger import LOG_MODE_ENV  # set for every unit; the C++ components read it too
from robot_runtime.units import Unit, UnitSet
from launch_helpers.devices.camera import camera_units
from launch_helpers.devices.xr import xr_units

from .robot import controller_launch


def display_env(environ: Optional[Dict[str, str]] = None) -> Tuple[Tuple[str, str], ...]:
    """DISPLAY / XAUTHORITY for the RViz window on the robot's own screen.

    Keeps the caller's DISPLAY; otherwise uses the lowest local X display and the desktop user's Xauthority."""
    environ = os.environ if environ is None else environ
    if environ.get("DISPLAY"):
        return ()
    display_numbers = [os.path.basename(path)[1:] for path in glob.glob("/tmp/.X11-unix/X[0-9]*")]
    display = min((int(number) for number in display_numbers if number.isdecimal()), default=None)
    if display is None:
        return ()
    env = [("DISPLAY", f":{display}")]
    candidates = [os.path.join(environ.get("HOME", ""), ".Xauthority"),
                  f"/run/user/{os.getuid()}/gdm/Xauthority"]
    candidates += sorted(glob.glob(f"/run/user/{os.getuid()}/.mutter-Xwaylandauth.*"))
    xauth = next((p for p in candidates if p and os.path.isfile(p)), None)
    if xauth and not environ.get("XAUTHORITY"):
        env.append(("XAUTHORITY", xauth))
    return tuple(env)


def dexcap_launch_args(cfg: Dict[str, Any]) -> Dict[str, str]:
    ports = cfg["teleop"]["dexcap"]
    robot = cfg["robot"]
    eef = {"linkerhand_l6": "hand", "jodell_rg75": "gripper"}.get(robot["end_effector"]["type"], "none")
    args = {}
    for device, key in zip(("upper_body", "left_hand", "right_hand"), DEXCAP_PORTS):
        args[f"start_{device}"] = "true" if ports[key] else "false"
        if ports[key]:
            args[f"{device}_serial_port"] = ports[key]
    args["arm_mapper"] = "true" if ports["upper_body_port"] and robot["entry"] != "whole_body" else "false"
    for side in ("left", "right"):
        args[f"{side}_eef"] = eef if ports[f"{side}_hand_port"] else "none"
    return args


def build_units(cfg: Dict[str, Any], config_path: str, ws_root: str) -> UnitSet:
    """Units enabled in the config, in the order status lists them: the controllers and RViz are ours, every
    device package (XR headset, gamepad, cameras) supplies its own units."""
    xr = xr_units(cfg)
    base = base_units(cfg)
    launch_file, args = controller_launch(cfg, config_path)

    def controller_cmd(launch_args: Dict[str, str]) -> Tuple[str, ...]:
        return ("ros2", "launch", "robot_startup", launch_file, *(f"{k}:={v}" for k, v in launch_args.items()))

    control = Unit(
        name="control",
        title="运控",
        cmd=controller_cmd(args),  # launch_mode:=control_only, no RViz
        # /fsm_state is latched (published on change only), so its rate says nothing; /joint_states does.
        topics=("/joint_states",),
        depends=tuple(base),  # the simulated base is up first: its TF is there when the controller looks for one
    )
    # Same launch file in rviz_only mode, so the RViz panels get the controller names they need.
    rviz = Unit(
        name="rviz",
        title="RViz",
        cmd=controller_cmd({**args, "launch_mode": "rviz_only"}),
        depends=("control",),
        env=display_env(),
        on_demand=True,
    )
    units: Dict[str, Unit] = {}
    units.update({name: unit for name, unit in xr.items() if name == "pc_service"})
    units.update(base)
    units.update({"control": control, "rviz": rviz})
    units.update({name: unit for name, unit in xr.items() if name != "pc_service"})
    if teleop_type(cfg) == "dexcap":
        args = dexcap_launch_args(cfg)
        sensors = (("upper_body_port", "/dexcap/left_arm"), ("left_hand_port", "/dexcap/left_hand"),
                   ("right_hand_port", "/dexcap/right_hand"))
        units["dexcap"] = Unit(
            name="dexcap", title="DexCap", depends=("control",),
            cmd=("ros2", "launch", "robot_startup", "dexcap.launch.py", *(f"{key}:={value}" for key, value in args.items())),
            topics=tuple(topic for key, topic in sensors if cfg["teleop"]["dexcap"][key]),
        )
    units.update(camera_units(cfg, config_path))
    # The log mode is one fact for the whole run: every process gets it in the environment and applies it once.
    for name, unit in units.items():
        units[name] = replace(unit, env=unit.env + ((LOG_MODE_ENV, cfg["log"]["mode"]),))
    return UnitSet(units)
