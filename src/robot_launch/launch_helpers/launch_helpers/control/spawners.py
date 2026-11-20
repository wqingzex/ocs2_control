"""
Common controller utilities for launch files.

This module provides utility functions for controller detection and management.
"""

import os
import tempfile
import xml.etree.ElementTree as ET
from typing import Any, Dict, Optional

import yaml
from launch_ros.actions import Node


def wrap_spawner_controller_params(
    controller_name: str, ros_parameters: Dict[str, Any]
) -> Dict[str, Dict[str, Dict[str, Any]]]:
    """
    Wrap parameters for ``controller_manager spawner``.

    ROS 2 Jazzy spawner only forwards ``--params-file`` entries that contain
    ``<controller_name>: {ros__parameters: ...}``. Flat dicts are ignored by the
    controller plugin (e.g. ``planning_urdf_path`` never reaches arm_controller).
    """
    return {controller_name: {"ros__parameters": dict(ros_parameters)}}


def prepare_ros2_controllers_override_path(meta) -> str:
    """Return pre-written merged ros2_control YAML path from load_robot_config meta."""
    from ..robot.config import EMPTY_ROBOT_CONFIG_META, RobotConfigMeta

    if not isinstance(meta, RobotConfigMeta):
        return ""
    if meta is EMPTY_ROBOT_CONFIG_META or not meta.needs_merged_file:
        return ""
    return meta.merged_yaml_path or ""


def write_spawner_controller_param_file(
    controller_name: str,
    ros_parameters: Dict[str, Any],
    *,
    quiet: bool = False,
) -> str:
    """
    Write a YAML file for ``spawner -p`` / ``--param-file`` (ROS 2 Jazzy).

    Prefer this over ``Node(parameters=[...])``: launch-generated param files are
    often flat under ``/**`` and are ignored by controller_manager spawner.
    """
    fd, path = tempfile.mkstemp(
        prefix=f"{controller_name}_spawner_",
        suffix=".yaml",
        dir="/tmp",
    )
    os.close(fd)
    payload = wrap_spawner_controller_params(controller_name, ros_parameters)
    with open(path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(payload, handle, default_flow_style=False)
    if not quiet:
        print(f"[INFO] Spawner param file for {controller_name}: {path}")
    return path


def _extract_joints_from_urdf(robot_description):
    """
    从 URDF/XML 字符串中提取所有 joint 名称。

    robot_description 不是合法 XML 时直接抛 ET.ParseError：它就是 controller_manager 要加载的那份，
    解析不了应当在这里停下，而不是返回空集合、跳过关节校验把所有控制器都 spawn 出来。
    """
    if not robot_description:
        return set()
    root = ET.fromstring(robot_description)
    return {joint.get("name") for joint in root.findall(".//joint") if joint.get("name")}


def _extract_sensors_from_urdf(robot_description):
    """Names of the <sensor> elements of the ros2_control blocks (e.g. left_ft_sensor).

    A robot_description that is not XML raises (ET.ParseError): it is the same text the controller manager loads, so
    the launch must stop there instead of silently starting without the FT broadcasters.
    """
    if not robot_description:
        return set()
    root = ET.fromstring(robot_description)
    return {s.get("name") for s in root.findall(".//ros2_control/sensor") if s.get("name")}


# The MPC controllers are the arm and the whole-body plugin; the joint, hand and gripper plugins are not.
MPC_CONTROLLER_TYPES = ("arm_controller/ArmController", "whole_body_controller/WholeBodyController")


def _is_mpc_controller(name: str, controller_config, config: Dict[str, Any]) -> bool:
    """True when the controller_manager entry is an MPC controller (arm or whole-body plugin)."""
    controller_type = controller_config if isinstance(controller_config, str) else (
        controller_config.get("type", "") if isinstance(controller_config, dict) else "")
    return controller_type in MPC_CONTROLLER_TYPES


def _controller_passes_joint_check(
    name: str,
    config: Dict[str, Any],
    patterns: list,
    robot_description: Optional[str],
    available_joints: set,
    skipped_controllers: set,
) -> bool:
    """Return True if controller should be included."""
    if not any(pattern.lower() in name.lower() for pattern in patterns):
        return False

    controller_params = config.get(name, {}).get("ros__parameters", {})
    sensor_name = controller_params.get("sensor_name")
    if sensor_name:
        # A sensor broadcaster (FT): only when the robot has that sensor; without a description it cannot be known.
        if sensor_name in _extract_sensors_from_urdf(robot_description):
            print(f"[INFO] Detected controller: {name} with sensor '{sensor_name}' (verified)")
            return True
        skipped_controllers.add(name)
        return False

    if not robot_description or not available_joints:
        return True

    joint_name = controller_params.get("joint")
    joint_names = controller_params.get("joints")

    if joint_name:
        if joint_name not in available_joints:
            print(
                f"[WARN] Controller '{name}' specifies joint '{joint_name}' "
                "which does not exist in robot_description, skipping"
            )
            skipped_controllers.add(name)
            return False
        print(f"[INFO] Detected controller: {name} with joint '{joint_name}' (verified)")
        return True

    if joint_names:
        missing = [j for j in joint_names if j not in available_joints]
        if missing:
            print(
                f"[WARN] Controller '{name}' specifies joints {missing} "
                "which do not exist in robot_description, skipping"
            )
            skipped_controllers.add(name)
            return False
        print(f"[INFO] Detected controller: {name} with {len(joint_names)} joints (verified)")
        return True

    print(
        f"[WARN] Controller '{name}' has no joint info in config, skipping (robot_description provided)"
    )
    skipped_controllers.add(name)
    return False


def detect_controllers(
    robot_name,
    robot_type="",
    patterns=None,
    robot_description=None,
    control_left="",
    control_right="",
    control_patch=None,
    ros2_control_config=None,
):
    """
    Detect controllers from ROS2 controller configuration.
    
    如果提供了 robot_description，会检查控制器配置中指定的 joint 是否真的存在于 xacro 中。
    只有 joint 存在的控制器才会被返回。
    
    Args:
        robot_name (str): Name of the robot
        robot_type (str): Robot type/variant
        patterns (list): List of patterns to match controller names
        robot_description (str, optional): URDF/XML 格式的机器人描述字符串，用于验证 joint 是否存在
        
    Returns:
        list: List of detected controllers (filtered by joint existence if robot_description is provided)
        
    Example:
        >>> controllers = detect_controllers('cr5', 'x5', ['hand', 'gripper'])
        >>> print(controllers)
        [{'name': 'hand_controller', 'type': 'joint_trajectory_controller', ...}]
        
        >>> # 带 joint 验证
        >>> controllers = detect_controllers('cr5', 'x5', ['hand'], robot_description=xml_string)
    """
    # Import here to avoid circular imports
    from ..robot.config import load_robot_config
    
    if patterns is None:
        patterns = ['hand', 'gripper']

    if ros2_control_config is not None:
        config = ros2_control_config
    else:
        config, _, _meta = load_robot_config(
            robot_name,
            "ros2_control",
            robot_type,
            control_left=control_left,
            control_right=control_right,
            control_patch=control_patch,
        )
    
    if config is None:
        print(f"[WARN] No controllers will be detected for robot '{robot_name}'")
        return []
    
    # 如果提供了 robot_description，提取所有 joint 名称
    available_joints = set()
    if robot_description:
        available_joints = _extract_joints_from_urdf(robot_description)
        if available_joints:
            print(f"[INFO] Found {len(available_joints)} joints in robot_description for validation")
    
    controllers = []
    skipped_controllers = set()

    controller_manager = config.get("controller_manager", {}).get("ros__parameters", {})

    for controller_name, controller_config in controller_manager.items():
        if controller_name in skipped_controllers:
            continue
        # The MPC controllers are started by the main spawner of the launch file, never through this name
        # matching (a name like whole_body_controller contains "body" and would otherwise be taken for a joint one).
        if _is_mpc_controller(controller_name, controller_config, config):
            continue
        if not _controller_passes_joint_check(
            controller_name,
            config,
            patterns,
            robot_description,
            available_joints,
            skipped_controllers,
        ):
            continue

        if isinstance(controller_config, str):
            controller_type = controller_config
        elif isinstance(controller_config, dict):
            controller_type = controller_config.get("type", "")
        else:
            controller_type = ""

        if not (robot_description and available_joints):
            print(f"[INFO] Detected controller: {controller_name} ({controller_type})")

        controllers.append(
            {
                "name": controller_name,
                "type": controller_type,
                "config": controller_config,
            }
        )

    print(f"[INFO] Total controllers detected: {len(controllers)}")
    return controllers


def detect_and_spawn_controllers(
    config,
    patterns,
    *,
    robot_description=None,
    use_sim_time=False,
):
    """Detect controllers from preloaded config and return spawner nodes."""
    if config is None:
        return [], []
    controllers = detect_controllers(
        robot_name="",
        patterns=patterns,
        robot_description=robot_description,
        ros2_control_config=config,
    )
    return controllers, create_controller_spawners(controllers, use_sim_time)


def create_controller_spawners(controllers, use_sim_time=False):
    """
    Create spawner nodes for controllers.
    
    Args:
        controllers (list): List of controller configurations
        use_sim_time (bool): Whether to use simulation time
        
    Returns:
        list: List of Node objects for controller spawners
        
    Example:
        >>> controllers = detect_controllers('cr5', 'x5', ['hand'])
        >>> spawners = create_controller_spawners(controllers, use_sim_time=True)
        >>> print(len(spawners))
        1
    """
    spawners = []
    
    for controller in controllers:
        controller_name = controller['name']
        
        print(f"[INFO] Creating spawner for controller: {controller_name}")
        
        spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller_name],
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
        )
        
        spawners.append(spawner)
    
    return spawners
