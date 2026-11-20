"""Build controller_manager and robot_state_publisher nodes."""

from __future__ import annotations

import os
from typing import Any, Dict, List, Optional

import yaml
from launch_ros.actions import Node

from .compose import is_compose_asymmetric
from ..robot.profile import (
    load_robot_profile,
    resolve_control_patch,
    resolve_control_sides,
    resolve_profile_path,
    resolve_robot_variant,
)
from ..robot.config import COMMON_CONFIG_FILE, DEFAULT_CONFIG_FILES, EMPTY_ROBOT_CONFIG_META, RobotConfigMeta, get_robot_package_path, load_robot_config
from ..robot.urdf import get_ros2_control_robot_description
from ..launch_utils import create_rmw_zenohd_node


def _parse_remappings(remappings_str: str) -> List[tuple]:
    additional = []
    for remap_pair in (remappings_str or "").split(";"):
        if not remap_pair.strip():
            continue
        if ":" not in remap_pair:
            raise ValueError(f"remapping '{remap_pair}' of '{remappings_str}' is not <from>:<to>")
        from_topic, to_topic = remap_pair.split(":", 1)
        additional.append((from_topic.strip(), to_topic.strip()))
    return additional


def _build_ros2_control_node_parameters(
    *,
    ros2_controllers_config: Optional[Dict[str, Any]],
    ros2_controllers_path: Optional[str],
    meta: RobotConfigMeta,
    use_sim_time: bool,
    ocs2_planning_param_file: str,
    robot_type: str,
    asymmetric: bool,
) -> List[Any]:
    if ros2_controllers_config is None and ros2_controllers_path is None:
        return []

    config_filename = os.path.basename(ros2_controllers_path or "")
    is_type_specific_config = bool(
        ros2_controllers_path and config_filename not in DEFAULT_CONFIG_FILES
    )

    config_has_robot_type = False
    if isinstance(ros2_controllers_config, dict):
        config_robot_type = (
            ros2_controllers_config.get("arm_controller", {})
            .get("ros__parameters", {})
            .get("robot_type")
        )
        config_has_robot_type = config_robot_type is not None

    node_parameters: List[Any] = []
    if meta.needs_merged_file and meta.merged_yaml_path:
        node_parameters.append(meta.merged_yaml_path)
        print(f"[INFO] Using merged ros2_control config file: {meta.merged_yaml_path}")
    else:
        common_config_path = os.path.join(
            os.path.dirname(ros2_controllers_path or ""), COMMON_CONFIG_FILE
        )
        if os.path.exists(common_config_path) and common_config_path != ros2_controllers_path:
            node_parameters.append(common_config_path)
            print(f"[INFO] Loading common ros2_control defaults from: {common_config_path}")
        if ros2_controllers_path:
            node_parameters.append(ros2_controllers_path)  # common.yaml itself when there is no type file

    node_parameters.append({"use_sim_time": use_sim_time})

    if ocs2_planning_param_file and os.path.isfile(ocs2_planning_param_file):
        node_parameters.append(ocs2_planning_param_file)

    if not is_type_specific_config and not asymmetric:
        if robot_type and robot_type.strip():
            node_parameters.append({"robot_type": robot_type})
            print(f"[INFO] Using launch arg robot_type '{robot_type}' (fallback to default config)")
    elif not config_has_robot_type and not asymmetric:
        if robot_type and robot_type.strip():
            node_parameters.append({"robot_type": robot_type})
            print(
                f"[INFO] Using launch arg robot_type '{robot_type}' "
                "(type-specific config has no robot_type)"
            )
    elif config_has_robot_type:
        print(f"[INFO] Using robot_type from config file '{config_filename}'")

    return node_parameters


def create_controller_manager_nodes(
    *,
    robot_name: str,
    robot_type: str = "",
    hardware: str = "mock_components",
    use_sim_time: bool = False,
    remappings_str: str = "",
    ocs2_planning_param_file: str = "",
    ros2_controllers_override: str = "",
    launch_configurations: Optional[Dict[str, str]] = None,
    preloaded_config: Optional[Dict[str, Any]] = None,
    preloaded_meta: Optional[RobotConfigMeta] = None,
) -> List[Any]:
    """
    Create robot_state_publisher, ros2_control_node, and
    joint_state_broadcaster spawner.

    When preloaded_config/meta are provided (OCS2 parent launch), skip reloading
    ros2_control config but still resolve profile for URDF xacro.
    """
    if hardware == "gz":
        raise ValueError(
            "hardware 'gz' (Gazebo) is no longer supported; use real, mock_components or isaac"
        )

    configs = launch_configurations or {}
    robot_profile_path = resolve_profile_path(configs)
    profile = load_robot_profile(robot_profile_path) if robot_profile_path else {}
    control_left, control_right = resolve_control_sides(configs, profile)
    control_patch = resolve_control_patch(profile)
    robot_variant = resolve_robot_variant(configs, profile)
    asymmetric = is_compose_asymmetric(control_left, control_right)

    get_robot_package_path(robot_name)  # a missing description package stops the launch here

    robot_description = get_ros2_control_robot_description(
        robot_name,
        robot_type=robot_type,
        hardware=hardware,
        launch_configurations=configs,
        robot_profile=robot_profile_path or None,
    )
    if robot_description is None:
        print(f"[ERROR] Failed to generate robot_description for '{robot_name}'")
        return []

    nodes: List[Any] = []

    rmw_zenohd_node = create_rmw_zenohd_node()
    if rmw_zenohd_node is not None:
        nodes.append(rmw_zenohd_node)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "publish_frequency": 100.0,
                "use_tf_static": True,
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )
    nodes.append(robot_state_publisher)

    meta = preloaded_meta or EMPTY_ROBOT_CONFIG_META
    ros2_controllers_config = preloaded_config
    ros2_controllers_path: Optional[str] = None

    override = (ros2_controllers_override or meta.merged_yaml_path or "").strip()
    if override and os.path.isfile(override):
        with open(override, "r", encoding="utf-8") as handle:
            ros2_controllers_config = yaml.safe_load(handle) or {}
        ros2_controllers_path = override
        if not meta.merged_yaml_path:
            meta = RobotConfigMeta(
                compose_applied=meta.compose_applied,
                type_yaml_found=meta.type_yaml_found,
                patch_applied=meta.patch_applied,
                variant_overlay_applied=meta.variant_overlay_applied,
                needs_merged_file=True,
                merged_yaml_path=override,
                base_config_path=meta.base_config_path,
            )
    elif preloaded_config is not None:
        ros2_controllers_path = meta.base_config_path or meta.merged_yaml_path or None
    else:
        ros2_controllers_config, ros2_controllers_path, meta = load_robot_config(
            robot_name,
            "ros2_control",
            robot_type,
            control_left=control_left,
            control_right=control_right,
            control_patch=control_patch,
            robot_variant=robot_variant,
        )

    node_parameters = _build_ros2_control_node_parameters(
        ros2_controllers_config=ros2_controllers_config,
        ros2_controllers_path=ros2_controllers_path,
        meta=meta,
        use_sim_time=use_sim_time,
        ocs2_planning_param_file=ocs2_planning_param_file,
        robot_type=robot_type,
        asymmetric=asymmetric,
    )
    if node_parameters:
        default_remappings = [
            ("/controller_manager/robot_description", "/robot_description"),
        ]
        all_remappings = default_remappings + _parse_remappings(remappings_str)
        nodes.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=node_parameters,
                remappings=all_remappings,
                output="screen",
            )
        )
    else:
        print(f"[WARN] No controller config found for robot '{robot_name}', skipping ros2_control_node")

    nodes.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        )
    )

    return nodes
