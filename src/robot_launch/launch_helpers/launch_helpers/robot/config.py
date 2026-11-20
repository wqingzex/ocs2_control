"""Robot ros2_control config loading: package lookup, type/variant overlays, merged-yaml metadata."""

from dataclasses import dataclass
import hashlib
import json
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
import yaml

from ..control.compose import (
    compose_control_config,
    is_compose_asymmetric,
    resolve_compose_type_key,
)


_config_cache = {}

@dataclass(frozen=True)
class RobotConfigMeta:
    """Metadata from load_robot_config; single source for merged-file decisions."""

    compose_applied: bool
    type_yaml_found: bool
    patch_applied: bool
    variant_overlay_applied: bool
    needs_merged_file: bool
    merged_yaml_path: str
    base_config_path: str = ""


EMPTY_ROBOT_CONFIG_META = RobotConfigMeta(
    compose_applied=False,
    type_yaml_found=False,
    patch_applied=False,
    variant_overlay_applied=False,
    needs_merged_file=False,
    merged_yaml_path="",
    base_config_path="",
)


def _build_robot_config_meta(
    *,
    compose_applied: bool,
    type_yaml_found: bool,
    patch_applied: bool,
    variant_overlay_applied: bool,
    config: dict,
    config_path: str,
    yaml_only: bool,
) -> RobotConfigMeta:
    if yaml_only:
        return EMPTY_ROBOT_CONFIG_META
    needs_merged = compose_applied or patch_applied or variant_overlay_applied
    merged_path = ""
    if needs_merged and config:
        merged_path = write_temp_ros2_control_yaml(config, quiet=True)
    return RobotConfigMeta(
        compose_applied=compose_applied,
        type_yaml_found=type_yaml_found,
        patch_applied=patch_applied,
        variant_overlay_applied=variant_overlay_applied,
        needs_merged_file=needs_merged,
        merged_yaml_path=merged_path,
        base_config_path=config_path or "",
    )


def clear_config_cache():
    """
    清除配置缓存
    
    在开发或测试时，如果配置文件被修改，可以调用此函数清除缓存
    """
    global _config_cache
    _config_cache.clear()
    print("[INFO] Config cache cleared")

def get_robot_package_path(robot_name):
    """
    Get common robot-related paths.
    
    Args:
        robot_name (str): Name of the robot (e.g., 'cr5', 'arx5', etc.)
        
    Returns:
        str: Path to the robot description package (ament PackageNotFoundError when it is not installed)
        
    Example:
        >>> robot_path = get_robot_package_path('cr5')
        >>> print(robot_path)
        '/opt/ros/humble/share/cr5_description'
    """
    return get_package_share_directory(robot_name + "_description")


def _generate_progressive_type_candidates(robot_type):
    """
    生成渐进匹配的type候选列表。
    
    例如，对于'ccs_left_rg75'，会生成：
    ['ccs_left_rg75', 'ccs_left', 'ccs']
    
    Args:
        robot_type (str): 完整的robot type字符串
        
    Returns:
        list: 渐进缩短的type候选列表
    """
    if not robot_type or not robot_type.strip():
        return []
    
    candidates = []
    current_type = robot_type.strip()
    
    # 添加完整名称
    candidates.append(current_type)
    
    # 逐步缩短：每次去掉最后一个下划线分隔的部分
    while '_' in current_type:
        # 找到最后一个下划线的位置
        last_underscore_idx = current_type.rfind('_')
        # 截取到最后一个下划线之前的部分
        current_type = current_type[:last_underscore_idx]
        if current_type:  # 确保不为空
            candidates.append(current_type)
    
    return candidates


def _deep_merge_dicts(base, override):
    """
    Recursively merge dictionaries without mutating inputs.

    Values from override win when both sides define the same key.
    """
    if not isinstance(base, dict):
        base = {}
    if not isinstance(override, dict):
        return override

    merged = dict(base)
    for key, value in override.items():
        if key in merged and isinstance(merged[key], dict) and isinstance(value, dict):
            merged[key] = _deep_merge_dicts(merged[key], value)
        else:
            merged[key] = value
    return merged


def write_temp_ros2_control_yaml(config_dict, *, quiet: bool = False):
    """Write merged ros2_control config to a temp YAML file for ros2_control_node."""
    fd, path = tempfile.mkstemp(suffix=".yaml", prefix="ros2_control_merged_")
    with os.fdopen(fd, "w", encoding="utf-8") as handle:
        yaml.safe_dump(config_dict, handle, default_flow_style=False, sort_keys=False)
    if not quiet:
        print(f"[INFO] Wrote merged ros2_control config: {path}")
    return path


def _strip_eef_controllers_from_config(config):
    """Remove per-side gripper/hand sections before EEF compose merge."""
    for controller_name in list(config.keys()):
        if controller_name.endswith("_gripper_controller") or controller_name.endswith(
            "_hand_controller"
        ):
            config.pop(controller_name, None)
        if controller_name == "controller_manager":
            cm_params = config.get("controller_manager", {}).get("ros__parameters", {})
            if isinstance(cm_params, dict):
                for cm_name in list(cm_params.keys()):
                    if cm_name.endswith("_gripper_controller") or cm_name.endswith(
                        "_hand_controller"
                    ):
                        cm_params.pop(cm_name, None)


def load_robot_config(
    robot_name,
    config_type="ros2_control",
    robot_type="",
    control_left="",
    control_right="",
    control_patch=None,
    robot_variant="",
    yaml_only=False,
):
    """
    Load ros2_control config with optional per-side compose and profile patch merge.

    Merge order: common.yaml + type yaml + variant yaml overlay + compose + control.patch

    When yaml_only is True, skip symmetric registry/template compose fallback (used by
    control_compose enrichment to avoid recursion).
    """
    effective_type, left, right = resolve_compose_type_key(robot_type, control_left, control_right)
    asymmetric = is_compose_asymmetric(left, right)
    patch = control_patch if isinstance(control_patch, dict) else {}
    variant_key = str(robot_variant or "").strip()
    patch_stamp = (
        hashlib.md5(json.dumps(patch, sort_keys=True).encode()).hexdigest() if patch else ""
    )

    cache_key = (
        f"{robot_name}_{config_type}_{effective_type}_{left}_{right}"
        f"_{variant_key}_{patch_stamp}"
    )
    if not yaml_only and cache_key in _config_cache:
        return _config_cache[cache_key]

    robot_pkg_path = get_robot_package_path(robot_name)

    compose_applied = False
    if config_type == "ros2_control":
        config_dir = os.path.join(robot_pkg_path, "config", "ros2_control")
        default_config_file = "ros2_controllers.yaml"
    else:
        config_dir = os.path.join(robot_pkg_path, "config", config_type)
        default_config_file = f"{config_type}.yaml"

    config_path = None
    config_file = None
    type_yaml_found = False

    if effective_type and effective_type.strip() and not asymmetric:
        type_candidates = _generate_progressive_type_candidates(effective_type)
        for candidate_type in type_candidates:
            candidate_file = f"{candidate_type}.yaml"
            candidate_path = os.path.join(config_dir, candidate_file)
            if os.path.exists(candidate_path):
                config_path = candidate_path
                config_file = candidate_file
                type_yaml_found = True
                print(f"[INFO] Using {config_type} config file (progressive match): {config_file}")
                break
        if config_path is None:
            config_file = default_config_file
            config_path = os.path.join(config_dir, config_file)
            print(f"[INFO] Progressive type matching failed, using default: {config_file}")
    else:
        config_file = default_config_file
        config_path = os.path.join(config_dir, config_file)
        if asymmetric:
            print(f"[INFO] Asymmetric EEF compose: {left} + {right}, base config: {config_file}")

    common_config_path = os.path.join(config_dir, "common.yaml")
    common_config = {}
    if config_type == "ros2_control" and os.path.exists(common_config_path):
        with open(common_config_path, "r") as file:
            common_config = yaml.safe_load(file) or {}

    if common_config:
        print(
            f"[INFO] Loaded {config_type} config: {config_path} "
            f"(+ {common_config_path})"
        )
    else:
        print(f"[INFO] Loaded {config_type} config: {config_path}")

    with open(config_path, "r") as file:
        config = yaml.safe_load(file) or {}

    config = _deep_merge_dicts(common_config, config)

    variant_overlay_applied = False
    if variant_key:
        variant_config_path = os.path.join(config_dir, f"{variant_key}.yaml")
        if os.path.isfile(variant_config_path):
            with open(variant_config_path, "r") as file:
                variant_config = yaml.safe_load(file) or {}
            config = _deep_merge_dicts(config, variant_config)
            variant_overlay_applied = True
            print(
                f"[INFO] Merged {config_type} variant overlay: "
                f"{os.path.basename(variant_config_path)}"
            )

    use_compose = asymmetric or (
        not yaml_only
        and not type_yaml_found
        and bool(effective_type and effective_type.strip())
    )
    if use_compose:
        _strip_eef_controllers_from_config(config)

    if use_compose:
        composed = compose_control_config(left, right, robot_name, base_config=config)
        if composed:
            config = _deep_merge_dicts(config, composed)
            compose_applied = True
            if asymmetric:
                print(f"[INFO] Merged composed control config for {left} + {right}")
            else:
                print(
                    f"[INFO] Merged symmetric EEF compose (template fallback) "
                    f"for {effective_type}"
                )

    patch_applied = bool(patch)
    if patch_applied:
        config = _deep_merge_dicts(config, patch)
        print("[INFO] Merged control.patch from robot profile")

    meta = _build_robot_config_meta(
        compose_applied=compose_applied,
        type_yaml_found=type_yaml_found,
        patch_applied=patch_applied,
        variant_overlay_applied=variant_overlay_applied,
        config=config,
        config_path=config_path or "",
        yaml_only=yaml_only,
    )

    result = (config, config_path, meta)
    if not yaml_only:
        _config_cache[cache_key] = result
    return result




def _info_controller_order(launch_mode):
    mode = (launch_mode or "").strip()
    if mode == "whole_body":
        return ("whole_body_controller", "arm_controller")
    if mode in ("split_body", "dual_arm"):
        # Do not fall back to whole_body_controller; planning uses the arm .info only.
        return ("arm_controller",)
    return ("whole_body_controller", "arm_controller")


def resolve_info_file_path(config, launch_mode, robot_name):
    """
    Path of the OCS2 .info the controller of this launch mode loads, resolved the same way as the
    controller (ArmMpcTask / WholeBodyMpcTask):
    <info_package or <robot_name>_description>/config/ocs2/<info_file_name>.info, where robot_name is the
    controller's robot_name parameter (falling back to the launch robot). A controller without info_file_name and
    a missing description package raise.
    """
    order = _info_controller_order(launch_mode)
    controller_key = order[0]
    params = {}
    if isinstance(config, dict):
        for key in order:
            candidate = config.get(key, {}).get("ros__parameters", {})
            if isinstance(candidate, dict) and str(candidate.get("info_file_name", "") or "").strip():
                controller_key, params = key, candidate
                break
        else:
            candidate = config.get(controller_key, {}).get("ros__parameters", {})
            params = candidate if isinstance(candidate, dict) else {}

    info_file_name = str(params.get("info_file_name", "") or "").strip().removesuffix(".info")
    if not info_file_name:
        raise ValueError(f"'{controller_key}' has no info_file_name in the ros2_control config of '{robot_name}'")
    package = str(params.get("info_package", "") or "").strip()
    if not package:
        package = str(params.get("robot_name", "") or robot_name).strip() + "_description"
    share = get_package_share_directory(package)
    return os.path.join(share, "config", "ocs2", f"{info_file_name}.info")
