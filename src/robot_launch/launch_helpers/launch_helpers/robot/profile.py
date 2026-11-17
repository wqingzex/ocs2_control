"""
Launch argument utilities: prefixed xacro_/hardware_/control_ routing and robot profile.
"""

from __future__ import annotations

import os
import sys
from typing import Any, Dict, List, Optional, Tuple

import yaml

XACRO_PREFIX = "xacro_"
HARDWARE_PREFIX = "hardware_"
CONTROL_PREFIX = "control_"

REAL_HARDWARE = frozenset({"real"})

# End effector arguments: resolved by resolve_end_effectors / resolve_end_effector_buses, never copied as plain
# xacro overrides. Names as robot.end_effector / robot.end_effector_bus in robot.local.yaml.
_EEF_XACRO_KEYS = frozenset({
    "end_effector", "left_end_effector", "right_end_effector",
    "end_effector_bus", "left_end_effector_bus", "right_end_effector_bus",
})


def _strip_eef_key(value: str) -> str:
    return str(value or "").strip()


def _strip_prefix(key: str, prefix: str) -> Optional[str]:
    if key.startswith(prefix):
        return key[len(prefix):]
    return None


def extract_prefixed_args(
    launch_configurations: Dict[str, str],
    prefix: str,
    argv: Optional[List[str]] = None,
) -> Dict[str, str]:
    result: Dict[str, str] = {}
    for key, value in launch_configurations.items():
        if value is None:
            continue
        stripped = _strip_prefix(key, prefix)
        if stripped is not None and str(value).strip():
            result[stripped] = str(value).strip()

    if argv is None:
        argv = sys.argv
    for token in argv:
        if ":=" not in token:
            continue
        key, _, raw_value = token.partition(":=")
        stripped = _strip_prefix(key, prefix)
        if stripped is not None and raw_value.strip():
            result[stripped] = raw_value.strip()
    return result


# config key (platform.*) -> xacro argument
_PLATFORM_XACRO_KEYS = {"chassis": "base", "variant": "variant", "chassis_joints_movable": "base_joints_movable"}


def create_platform_launch_arguments():
    """Launch args for humanoid / mobile-base xacro (not used by bare manipulators)."""
    from launch.actions import DeclareLaunchArgument

    return [
        DeclareLaunchArgument(
            "base",
            default_value="",
            description="Xacro mobile base model key (robot-specific). Empty means no base parameter passed to xacro.",
        ),
        DeclareLaunchArgument(
            "base_joints_movable",
            default_value="",
            description="Xacro flag for movable base joints (true/false). Empty means no base_joints_movable parameter passed to xacro.",
        ),
    ]


def normalize_robot_profile(data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Normalize robot profile to {xacro, hardware, control} for launch code.

    New schema (recommended):
      platform:  chassis (xacro: base) / variant / arm_ctrl_mode — always apply (also when end effectors come from the config)
      robot.end_effector / robot.end_effector_bus: loaded into profile["eef"] (launch arguments override them)
      control.patch: inline ros2_control overrides (deep-merged after compose)
    """
    if not isinstance(data, dict) or not data:
        return {}

    platform = data.get("platform")
    defaults = data.get("defaults")
    has_control = isinstance(data.get("control"), dict)
    if (
        not isinstance(platform, dict)
        and not isinstance(defaults, dict)
        and not has_control
        and not isinstance(data.get("robot"), dict)
    ):
        return data

    legacy_x = data.get("xacro") if isinstance(data.get("xacro"), dict) else {}
    legacy_hw = data.get("hardware") if isinstance(data.get("hardware"), dict) else {}
    legacy_c = data.get("control") if isinstance(data.get("control"), dict) else {}

    xacro: Dict[str, Any] = {}
    hardware = dict(legacy_hw)
    control: Dict[str, Any] = {}

    plat = platform if isinstance(platform, dict) else {}
    for key, xacro_name in _PLATFORM_XACRO_KEYS.items():
        if key in plat:
            xacro[xacro_name] = plat[key]
        elif key in legacy_x:
            xacro[xacro_name] = legacy_x[key]

    arm_mode = plat.get("arm_ctrl_mode") or legacy_hw.get("arm_ctrl_mode")
    if arm_mode is not None and str(arm_mode).strip():
        hardware["arm_ctrl_mode"] = str(arm_mode).strip()

    if isinstance(defaults, dict) and "end_effectors" in defaults:
        raise ValueError("defaults.end_effectors was removed: write robot.end_effector / robot.end_effector_bus "
                         "in the same file instead")
    robot = data.get("robot") if isinstance(data.get("robot"), dict) else {}
    eef: Dict[str, str] = {}
    for key, name in (("end_effector", "end_effector"), ("end_effector_bus", "bus")):
        value = _strip_eef_key(str(robot.get(key, "") or ""))
        if value:
            eef[name] = value

    patch = legacy_c.get("patch")
    if isinstance(patch, dict) and patch:
        control["patch"] = patch

    return {"xacro": xacro, "eef": eef, "hardware": hardware, "control": control}


def load_robot_profile(profile_path: str) -> Dict[str, Any]:
    if not profile_path or not os.path.isfile(profile_path):
        return {}
    with open(profile_path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        return {}
    return normalize_robot_profile(data)


def resolve_profile_path(launch_configurations: Dict[str, str]) -> str:
    profile = launch_configurations.get("robot_profile", "").strip()
    if profile and os.path.isfile(profile):
        return os.path.abspath(profile)
    return ""


def _cli_launch_value(launch_configurations: Dict[str, str], key: str) -> str:
    """Read a launch arg from LaunchConfiguration or ``key:=value`` on the CLI."""
    value = str(launch_configurations.get(key, "") or "").strip()
    if value:
        return value
    prefix = f"{key}:="
    for token in sys.argv:
        if token.startswith(prefix):
            return token[len(prefix) :].strip()
    return ""


def _side_value(launch_configurations: Dict[str, str], profile: Optional[Dict[str, Any]], side: str,
                key: str, profile_key: str) -> str:
    """left_<key> / right_<key>, else <key>, from the launch; else the profile's robot.<key>."""
    for name in (f"{side}_{key}", key):
        value = _strip_eef_key(_cli_launch_value(launch_configurations, name))
        if value:
            return value
    if profile:
        return _strip_eef_key(str((profile.get("eef") or {}).get(profile_key, "") or ""))
    return ""


def resolve_end_effectors(
    launch_configurations: Dict[str, str],
    profile: Optional[Dict[str, Any]] = None,
) -> Tuple[str, str]:
    """Left / right end effector (robot.end_effector names; '' = none) for xacro and ros2_control compose."""
    sides = tuple(_side_value(launch_configurations, profile, side, "end_effector", "end_effector")
                  for side in ("left", "right"))
    return tuple("" if value == "none" else value for value in sides)  # type: ignore[return-value]


def resolve_end_effector_buses(
    launch_configurations: Dict[str, str],
    profile: Optional[Dict[str, Any]] = None,
) -> Tuple[str, str]:
    """Left / right end effector bus (usb_rs485 | arm_rs485 | usb_can | ethernet; '' = the xacro default)."""
    return tuple(_side_value(launch_configurations, profile, side, "end_effector_bus", "bus")  # type: ignore
                 for side in ("left", "right"))


resolve_side_eef_types = resolve_end_effectors  # used by control.compose


def resolve_control_patch(profile: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """ros2_control overrides from robot.local.yaml control.patch (deep-merged after compose)."""
    if not profile:
        return {}
    control_section = profile.get("control") or {}
    if not isinstance(control_section, dict):
        return {}
    patch = control_section.get("patch")
    return dict(patch) if isinstance(patch, dict) else {}


def resolve_control_sides(
    launch_configurations: Dict[str, str],
    profile: Optional[Dict[str, Any]] = None,
) -> Tuple[str, str]:
    """Alias for :func:`resolve_end_effectors` (ros2_control compose)."""
    return resolve_end_effectors(launch_configurations, profile)


def resolve_robot_variant(
    launch_configurations: Dict[str, str],
    profile: Optional[Dict[str, Any]] = None,
) -> str:
    """Platform variant from launch ``variant`` or profile ``platform.variant``."""
    variant = _strip_eef_key(_cli_launch_value(launch_configurations, "variant"))
    if variant:
        return variant
    if profile:
        xacro_section = profile.get("xacro") or {}
        if isinstance(xacro_section, dict):
            profile_variant = _strip_eef_key(str(xacro_section.get("variant", "") or ""))
            if profile_variant:
                return profile_variant
    return ""


def build_xacro_mappings(
    hardware: str,
    launch_configurations: Dict[str, str],
    robot_profile: Optional[str] = None,
) -> Dict[str, str]:
    profile = load_robot_profile(robot_profile) if robot_profile else {}
    profile_xacro = profile.get("xacro") or {}
    if not isinstance(profile_xacro, dict):
        profile_xacro = {}

    mappings: Dict[str, str] = {"ros2_control_hardware_type": hardware}

    for key, value in profile_xacro.items():
        if str(key) in _EEF_XACRO_KEYS:
            continue
        if value is not None and str(value).strip():
            mappings[str(key)] = str(value).strip()

    xacro_overrides = extract_prefixed_args(launch_configurations, XACRO_PREFIX)
    for key in _EEF_XACRO_KEYS:
        xacro_overrides.pop(key, None)
    mappings.update(xacro_overrides)

    if hardware in REAL_HARDWARE:
        profile_hardware = profile.get("hardware") or {}
        if isinstance(profile_hardware, dict):
            for key, value in profile_hardware.items():
                if value is not None and str(value).strip():
                    mappings[str(key)] = str(value).strip()
        mappings.update(extract_prefixed_args(launch_configurations, HARDWARE_PREFIX))

    left, right = resolve_end_effectors(launch_configurations, profile)
    for side, value in (("left", left), ("right", right)):
        if value:
            mappings[f"{side}_end_effector"] = value
    if hardware in REAL_HARDWARE:
        for side, value in zip(("left", "right"), resolve_end_effector_buses(launch_configurations, profile)):
            if value:
                mappings[f"{side}_end_effector_bus"] = value

    arms = _strip_eef_key(_cli_launch_value(launch_configurations, "arms"))
    if arms:
        mappings["arms"] = arms

    if "collider" not in mappings:
        mappings["collider"] = "simple"

    launch_variant = _strip_eef_key(_cli_launch_value(launch_configurations, "variant"))
    if launch_variant:
        mappings["variant"] = launch_variant

    return mappings


def create_eef_side_launch_arguments():
    from launch.actions import DeclareLaunchArgument

    names = "jodell_rg75 | linkerhand_l6 | xhand1 | wuji_hand2 | none"
    buses = "usb_rs485 | arm_rs485 | usb_can | ethernet"
    return [
        DeclareLaunchArgument("end_effector", default_value="",
                              description=f"End effector on both arms ({names}); default robot.end_effector"),
        DeclareLaunchArgument("left_end_effector", default_value="", description="Left side only; see end_effector"),
        DeclareLaunchArgument("right_end_effector", default_value="", description="Right side only; see end_effector"),
        DeclareLaunchArgument("end_effector_bus", default_value="",
                              description=f"Where the end effectors are wired ({buses}); default robot.end_effector_bus"),
        DeclareLaunchArgument("left_end_effector_bus", default_value="", description="Left side only"),
        DeclareLaunchArgument("right_end_effector_bus", default_value="", description="Right side only"),
    ]


def create_robot_profile_launch_arguments():
    from launch.actions import DeclareLaunchArgument

    return [
        DeclareLaunchArgument(
            "robot_profile",
            default_value="",
            description="Path to robot.local.yaml (machine hardware profile)",
        ),
        *create_eef_side_launch_arguments(),
    ]
