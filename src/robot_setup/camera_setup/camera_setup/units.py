"""The units of the cameras: one Orbbec driver process per enabled camera."""
from __future__ import annotations

from typing import Any, Dict

from robot_runtime.units import Unit

from .camera import DRIVER_READY_LOG, DRIVER_READY_TIMEOUT_S, camera_specs

CAMERA_TITLES = {"camera_head": "头部相机", "camera_left_wrist": "左腕相机", "camera_right_wrist": "右腕相机"}
GROUP = "camera"


def camera_units(cfg: Dict[str, Any], config_path: str) -> Dict[str, Unit]:
    units: Dict[str, Unit] = {}
    for spec in camera_specs(cfg):
        units[spec.name] = Unit(
            name=spec.name,
            title=CAMERA_TITLES.get(spec.name, spec.name),
            cmd=("ros2", "launch", "camera_setup", "cameras.launch.py", f"config:={config_path}", f"only:={spec.name}"),
            topics=(f"/{spec.name}/color/image_raw/compressed",),
            group=GROUP,
            ready_log=DRIVER_READY_LOG,       # the drivers share one device lock: one camera is initialized at a time
            ready_timeout_s=DRIVER_READY_TIMEOUT_S,
        )
    return units
