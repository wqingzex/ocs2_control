"""Units built from the config, one process group and log each: pc_service, control, rviz (on demand),
joystick (on demand), xr (xr_bridge_node as /xr_target_node) and one Orbbec driver per camera.
"""
from __future__ import annotations

import glob
import json
import os
from dataclasses import dataclass, field, replace
from typing import Any, Dict, List, Optional, Tuple

from w2_payload.devices.cameras import camera_specs
from .robot import controller_launch

CAMERA_TITLES = {"camera_head": "头部相机", "camera_left_wrist": "左腕相机", "camera_right_wrist": "右腕相机"}


LOG_MODE_ENV = "ROBOT_LOG_MODE"  # read by robot_log::debugMode() in every component


@dataclass(frozen=True)
class Unit:
    name: str
    title: str                       # shown in status / console
    cmd: Tuple[str, ...]             # empty for pc_service (special handling)
    depends: Tuple[str, ...] = ()
    topics: Tuple[str, ...] = ()     # health topics (message rate)
    kind: str = "process"            # process | pc_service
    env: Tuple[Tuple[str, str], ...] = ()  # extra environment variables for the process
    on_demand: bool = False          # skipped by "start everything"; started only when named

    @property
    def is_camera(self) -> bool:
        return self.name.startswith("camera_")


@dataclass
class UnitSet:
    units: Dict[str, Unit] = field(default_factory=dict)

    def names(self) -> List[str]:
        return list(self.units)

    def __contains__(self, name: str) -> bool:
        return name in self.units

    def __getitem__(self, name: str) -> Unit:
        if name not in self.units:
            raise KeyError(f"未知模块 '{name}'，可选: {', '.join(self.units)}")
        return self.units[name]

    def start_order(self, names: Optional[List[str]] = None) -> List[str]:
        """The given units (default: all but on-demand ones) plus their dependencies, dependencies first."""
        order: List[str] = []

        def visit(name: str, stack: Tuple[str, ...]):
            if name in order:
                return
            if name in stack:
                raise ValueError(f"模块依赖成环: {' -> '.join(stack + (name,))}")
            for dep in self[name].depends:
                visit(dep, stack + (name,))
            order.append(name)

        for name in names or [n for n, u in self.units.items() if not u.on_demand]:
            visit(name, ())
        return order

    def launch_order(self, names: Optional[List[str]] = None) -> List[str]:
        """What to start for names: like start_order, but on-demand units never pull in their dependencies."""
        requested = names or [n for n, u in self.units.items() if not u.on_demand]
        regular = [n for n in requested if not self[n].on_demand]
        order = self.start_order(regular) if regular else []
        return order + [n for n in requested if self[n].on_demand and n not in order]

    def stop_order(self, names: Optional[List[str]] = None) -> List[str]:
        """The given units (default: all) plus everything that depends on them, dependents first."""
        wanted = set(names or self.names())
        changed = True
        while changed:
            changed = False
            for unit in self.units.values():
                if unit.name not in wanted and wanted.intersection(unit.depends):
                    wanted.add(unit.name)
                    changed = True
        return [n for n in reversed(self.start_order(self.names())) if n in wanted]


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
    """Units enabled in the config, in a sensible start order."""
    units: Dict[str, Unit] = {}
    xr = cfg["xr"]

    if xr["enable"] and xr["start_pc_service"]:
        units["pc_service"] = Unit(name="pc_service", title="PC Service", cmd=(), kind="pc_service")

    launch_file, args = controller_launch(cfg, config_path)

    def controller_cmd(launch_args: Dict[str, str]) -> Tuple[str, ...]:
        return ("ros2", "launch", "robot_bringup", launch_file, *(f"{k}:={v}" for k, v in launch_args.items()))

    units["control"] = Unit(
        name="control",
        title="运控",
        cmd=controller_cmd(args),  # launch_mode:=control_only, no RViz
        # /fsm_state is latched (published on change only), so its rate says nothing; /joint_states does.
        topics=("/joint_states",),
    )
    # Same launch file in rviz_only mode, so the RViz panels get the controller names they need.
    units["rviz"] = Unit(
        name="rviz",
        title="RViz",
        cmd=controller_cmd({**args, "launch_mode": "rviz_only"}),
        depends=("control",),
        env=display_env(),
        on_demand=True,
    )

    joystick_cmd = ("ros2", "launch", "arms_teleop", "joystick_teleop.launch.py")
    if cfg["joystick"]["device"]:
        joystick_cmd += (f"joy_dev:={cfg['joystick']['device']}",)
    units["joystick"] = Unit(name="joystick", title="手柄遥操", cmd=joystick_cmd, depends=("control",), on_demand=True)

    if xr["enable"]:
        units["xr"] = Unit(
            name="xr",
            title="VR",
            cmd=("ros2", "run", "fa_teleop", "xr_bridge_node", "--ros-args",
                 "-r", "__node:=xr_target_node",
                 "-p", f"frame_timeout_ms:={int(xr['frame_timeout_ms'])}",
                 "-p", f"reconnect_interval_s:={float(xr['reconnect_interval_s'])}",
                 "-p", f"device_id:={json.dumps(xr['device_id'])}",
                 "-p", f"control_scheme:={xr['control_scheme']}"),
            depends=("pc_service",) if "pc_service" in units else (),
            topics=("/teleop/left_ee_pose", "/teleop/connection_status"),
        )

    for spec in camera_specs(cfg):
        units[spec.name] = Unit(
            name=spec.name,
            title=CAMERA_TITLES.get(spec.name, spec.name),
            cmd=("ros2", "launch", "w2_payload", "cameras.launch.py", f"config:={config_path}", f"only:={spec.name}"),
            topics=(f"/{spec.name}/color/image_raw/compressed",),
        )
    # The log mode is one fact for the whole run: every process gets it in the environment and applies it once.
    for name, unit in units.items():
        units[name] = replace(unit, env=unit.env + ((LOG_MODE_ENV, cfg["log"]["mode"]),))
    return UnitSet(units)
