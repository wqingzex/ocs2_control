"""The units of a run: the controllers and RViz are built here, the devices' units come from their packages."""
from __future__ import annotations

import glob
import os
from dataclasses import replace
from typing import Any, Dict, Optional, Tuple

from joystick_teleop.units import joystick_units
from robot_log_tools.log import LOG_MODE_ENV  # set for every unit; the C++ components read it too
from robot_runtime.units import Unit, UnitSet
from camera.units import camera_units
from xr_teleop.units import xr_units

from .robot import controller_launch


def display_env(environ: Optional[Dict[str, str]] = None) -> Tuple[Tuple[str, str], ...]:
    """DISPLAY / XAUTHORITY for the RViz window on the robot's own screen.

    Keeps the caller's DISPLAY; otherwise local X server :0 with the desktop user's Xauthority (e.g. over SSH)."""
    environ = os.environ if environ is None else environ
    if environ.get("DISPLAY"):
        return ()
    if not os.path.exists("/tmp/.X11-unix/X0"):
        return ()
    env = [("DISPLAY", ":0")]
    candidates = [os.path.join(environ.get("HOME", ""), ".Xauthority"),
                  f"/run/user/{os.getuid()}/gdm/Xauthority"]
    candidates += sorted(glob.glob(f"/run/user/{os.getuid()}/.mutter-Xwaylandauth.*"))
    xauth = next((p for p in candidates if p and os.path.isfile(p)), None)
    if xauth and not environ.get("XAUTHORITY"):
        env.append(("XAUTHORITY", xauth))
    return tuple(env)


def build_units(cfg: Dict[str, Any], config_path: str, ws_root: str) -> UnitSet:
    """Units enabled in the config, in the order status lists them: the controllers and RViz are ours, every
    device package (XR headset, gamepad, cameras) supplies its own units."""
    xr = xr_units(cfg)
    launch_file, args = controller_launch(cfg, config_path)

    def controller_cmd(launch_args: Dict[str, str]) -> Tuple[str, ...]:
        return ("ros2", "launch", "robot_bringup", launch_file, *(f"{k}:={v}" for k, v in launch_args.items()))

    control = Unit(
        name="control",
        title="运控",
        cmd=controller_cmd(args),  # launch_mode:=control_only, no RViz
        # /fsm_state is latched (published on change only), so its rate says nothing; /joint_states does.
        topics=("/joint_states",),
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
    units.update({"control": control, "rviz": rviz})
    units.update(joystick_units(cfg))
    units.update({name: unit for name, unit in xr.items() if name != "pc_service"})
    units.update(camera_units(cfg, config_path))
    # The log mode is one fact for the whole run: every process gets it in the environment and applies it once.
    for name, unit in units.items():
        units[name] = replace(unit, env=unit.env + ((LOG_MODE_ENV, cfg["log"]["mode"]),))
    return UnitSet(units)
