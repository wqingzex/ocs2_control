"""One-shot bringup from a teleop YAML: PC Service, controllers, xr_bridge_node, cameras.

    ros2 launch robot_app teleop.launch.py config:=<ws_root>/config/robot.local.yaml"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_app import load
from robot_app.robot import controller_launch
from robot_app.units import dexcap_launch_args
from robot_config.config import teleop_type, xr_settings
from launch_helpers.devices import xr as pc_service


def _xr_bridge_installed() -> bool:
    exe = os.path.join(get_package_prefix("xr_teleop"), "lib", "xr_teleop", "xr_bridge_node")
    return os.path.exists(exe)


def launch_setup(context):
    config_path = LaunchConfiguration("config").perform(context)
    cfg = load(config_path)
    actions = [LogInfo(msg=f"robot_control_suite teleop bringup: {config_path}")]

    # 1. PC Service
    selected, xr = teleop_type(cfg), xr_settings(cfg)
    if selected == "xr" and xr["start_pc_service"]:
        started = pc_service.start_pc_service(xr["pc_service_script"], network_interface=xr["network_interface"])
        if started:
            actions.append(LogInfo(msg=f"XRoboToolkit PC Service started (pid {started})"))
            actions.append(RegisterEventHandler(OnShutdown(
                on_shutdown=[OpaqueFunction(function=lambda _ctx: pc_service.stop_pc_service(started) or [])])))
        else:
            actions.append(LogInfo(msg="XRoboToolkit PC Service already running"))

    # 2. Controllers (+ target_generator / XrInputHandler)
    launch_file, args = controller_launch(cfg, os.path.abspath(config_path))
    controller_dir = os.path.join(get_package_share_directory("robot_startup"), "launch")
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(controller_dir, launch_file)),
        launch_arguments=args.items()))

    # 3. XR bridge
    if selected == "xr":
        if not _xr_bridge_installed():
            raise RuntimeError(
                "xr_teleop xr_bridge_node is not built: the XRoboToolkit PC Service SDK was not found at "
                "build time. Run setup.sh with teleop.type: xr to prepare the PC Service and bridge")
        actions.append(Node(
            package="xr_teleop",
            executable="xr_bridge_node",
            name="xr_target_node",
            output="screen",
            parameters=[{
                "frame_timeout_ms": int(xr["frame_timeout_ms"]),
                "reconnect_interval_s": float(xr["reconnect_interval_s"]),
                "device_id": xr["device_id"],
            }],
        ))
    elif selected == "dexcap":
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(controller_dir, "dexcap.launch.py")),
            launch_arguments=dexcap_launch_args(cfg).items()))

    # 4. Cameras
    if cfg["cameras"]["enable"]:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("robot_startup"), "launch", "cameras.launch.py")),
            launch_arguments={"config": config_path}.items()))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", description="robot config YAML (config/robot.local.yaml, made by ./scripts/setup.sh config)"),
        OpaqueFunction(function=launch_setup),
    ])
