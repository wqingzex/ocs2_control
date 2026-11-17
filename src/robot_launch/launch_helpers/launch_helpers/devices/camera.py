"""Orbbec cameras: how to start them.

camera_specs turns a validated robot config into one driver launch spec per enabled camera (head Gemini 335Lg,
wrists Gemini 305; topics /camera_<name>/color/image_raw/compressed); camera_units describes the processes robot_app
supervises. Discovery and the setup window are in camera_setup."""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Tuple

from robot_runtime.units import Unit

WIDTH, HEIGHT = 640, 480


@dataclass
class CameraSpec:
    name: str                       # ROS camera_name, e.g. camera_left_wrist
    launch_file: str                # file in orbbec_camera/launch
    args: Dict[str, str]            # launch arguments
    remaps: List[Tuple[str, str]] = field(default_factory=list)
    config_yaml: Dict[str, Any] = field(default_factory=dict)  # written to config_file_path (wrist)


def wrist_eye(rotation: int) -> str:
    """Gemini 305 has two color eyes; camera_setup uses the right eye, the left one at 90/270°."""
    return "left" if rotation in (90, 270) else "right"


def _wrist_yaml(eye: str, rotation: int, fps: int) -> Dict[str, Any]:
    """Keys gemini305.launch.py does not take as launch args (camera_setup gemini305_*_wrist.yaml)."""
    other = "left" if eye == "right" else "right"
    return {
        "device_preset": "Dual Color Streams",
        "enable_color": False,
        "enable_depth": False,
        "enable_left_ir": False,
        "enable_right_ir": False,
        "enable_point_cloud": False,
        f"enable_{eye}_color": True,
        f"{eye}_color_width": WIDTH,
        f"{eye}_color_height": HEIGHT,
        f"{eye}_color_fps": fps,
        f"{eye}_color_format": "YUYV",
        f"{eye}_color_rotation": rotation,
        f"{eye}_color.image_raw.enable_pub_plugins": ["image_transport/compressed"],
        # The preset forces both eyes on; keep the unused one minimal so the USB 2 link is not saturated.
        f"enable_{other}_color": False,
        f"{other}_color_width": WIDTH,
        f"{other}_color_height": HEIGHT,
        f"{other}_color_fps": 5,
        f"{other}_color.image_raw.enable_pub_plugins": ["image_transport/compressed"],  # not raw, theora, zstd
    }


def _common_color_args(cam: Dict[str, Any]) -> Dict[str, str]:
    """Imaging of one camera: fixed white balance; exposure -1 (with gain -1) = auto, otherwise fixed exposure and gain."""
    auto = cam["exposure"] == -1
    return {
        "color_white_balance": str(cam["white_balance"]),
        "color_ae_max_exposure": str(cam["ae_max_exposure"]),
        "enable_color_auto_exposure_priority": "false",
        "enable_color_auto_white_balance": "false",
        "enable_color_auto_exposure": "true" if auto else "false",
        "color_exposure": str(cam["exposure"]),
        "color_gain": str(cam["gain"]),
        "color_powerline_freq": "50hz",
        "color_anti_flicker": "true",
        "publish_tf": "false",
        "diagnostic_period": "0.0",
    }


def _device_args(name: str, cam: Dict[str, Any]) -> Dict[str, str]:
    args = {"camera_name": name, "serial_number": cam["serial"]}
    if cam["usb_port"]:
        args["usb_port"] = cam["usb_port"]
    return args


def camera_specs(cfg: Dict[str, Any]) -> List[CameraSpec]:
    """Specs for every enabled camera in a validated config."""
    cams = cfg["cameras"]
    if not cams["enable"]:
        return []
    fps = int(cams["fps"])
    specs: List[CameraSpec] = []

    for side in ("left", "right"):
        cam = cams[f"{side}_wrist"]
        if not cam["enable"]:
            continue
        name = f"camera_{side}_wrist"
        eye = wrist_eye(int(cam["rotation"]))
        args = _device_args(name, cam)
        args.update(_common_color_args(cam))
        args.update({
            "ae_reference_stream": "color",
            "ae_strategy": "motion",
        })
        specs.append(CameraSpec(
            name=name,
            launch_file="gemini305.launch.py",
            args=args,
            # Publish the eye in use under <name>/color regardless of rotation.
            remaps=[
                (f"/{name}/{eye}_color/image_raw/compressed", f"/{name}/color/image_raw/compressed"),
                (f"/{name}/{eye}_color/camera_info", f"/{name}/color/camera_info"),
            ],
            config_yaml=_wrist_yaml(eye, int(cam["rotation"]), fps),
        ))

    head = cams["head"]
    if head["enable"]:
        name = "camera_head"
        args = _device_args(name, head)
        args.update(_common_color_args(head))
        # Native 640x480 profile; the SDK rotates decoded frames, so the format must be YUYV (not MJPG).
        args.update({
            "enable_color": "true",
            "color_width": str(WIDTH),
            "color_height": str(HEIGHT),
            "color_format": "YUYV",
            "color_rotation": str(head["rotation"]),
            "color_fps": str(fps),
            "color.image_raw.enable_pub_plugins": '["image_transport/compressed"]',
            "enable_depth": "false",
            "enable_left_ir": "false",
            "enable_right_ir": "false",
            "enable_point_cloud": "false",
        })
        specs.append(CameraSpec(name=name, launch_file="gemini_330_series.launch.py", args=args))
    return specs


def runtime_dir() -> str:
    """Private per-user directory for generated driver YAML (like camera_setup)."""
    base = os.environ.get("XDG_RUNTIME_DIR") or "/tmp"
    path = os.path.join(base, f"fa_w2_camera-{os.getuid()}")
    os.makedirs(path, mode=0o700, exist_ok=True)
    return path


# The driver logs this once the device is initialized; until then it holds the shared device lock, and a second
# driver that starts meanwhile gives up after 5 x 100 ms. Cameras are therefore started one after another.
DRIVER_READY_LOG = "Initialize device cost"
DRIVER_READY_TIMEOUT_S = 30.0


CAMERA_TITLES = {"camera_head": "头部相机", "camera_left_wrist": "左腕相机", "camera_right_wrist": "右腕相机"}
GROUP = "camera"


def camera_units(cfg: Dict[str, Any], config_path: str) -> Dict[str, Unit]:
    units: Dict[str, Unit] = {}
    for spec in camera_specs(cfg):
        units[spec.name] = Unit(
            name=spec.name,
            title=CAMERA_TITLES.get(spec.name, spec.name),
            cmd=("ros2", "launch", "robot_startup", "cameras.launch.py", f"config:={config_path}", f"only:={spec.name}"),
            topics=(f"/{spec.name}/color/image_raw/compressed",),
            group=GROUP,
            ready_log=DRIVER_READY_LOG,       # the drivers share one device lock: one camera is initialized at a time
            ready_timeout_s=DRIVER_READY_TIMEOUT_S,
        )
    return units
