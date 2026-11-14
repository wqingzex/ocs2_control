"""One-shot bringup from a teleop YAML: PC Service, controllers, xr_bridge_node, cameras.

    ros2 launch teleop_app teleop.launch.py config:=<ws_root>/config/robot.local.yaml"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from teleop_app import load
from teleop_app.robot import controller_launch
from xr_teleop import pc_service


def _xr_bridge_installed() -> bool:
    exe = os.path.join(get_package_prefix("xr_teleop"), "lib", "xr_teleop", "xr_bridge_node")
    return os.path.exists(exe)


def launch_setup(context):
    config_path = LaunchConfiguration("config").perform(context)
    cfg = load(config_path)
    actions = [LogInfo(msg=f"W2 teleop bringup: {config_path}")]

    # 1. PC Service
    xr = cfg["xr"]
    if xr["enable"] and xr["start_pc_service"]:
        started = pc_service.start(xr["pc_service_script"])
        if started:
            actions.append(LogInfo(msg=f"XRoboToolkit PC Service started (pid {started})"))
            actions.append(RegisterEventHandler(OnShutdown(
                on_shutdown=[OpaqueFunction(function=lambda _ctx: pc_service.stop(started) or [])])))
        else:
            actions.append(LogInfo(msg="XRoboToolkit PC Service already running"))

    # 2. Controllers (+ teleop_command / VRInputHandler)
    launch_file, args = controller_launch(cfg, os.path.abspath(config_path))
    controller_dir = os.path.join(get_package_share_directory("robot_bringup"), "launch")
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(controller_dir, launch_file)),
        launch_arguments=args.items()))

    # 3. VR bridge
    if xr["enable"]:
        if not _xr_bridge_installed():
            raise RuntimeError(
                "xr_teleop xr_bridge_node is not built: the XRoboToolkit PC Service SDK was not found at "
                "build time (install the PC Service deb, then rebuild xr_teleop), or set xr.enable: false")
        actions.append(Node(
            package="xr_teleop",
            executable="xr_bridge_node",
            name="xr_target_node",  # VRInputHandler::XR_NODE_NAME — VR is enabled only while it exists
            output="screen",
            parameters=[{
                "frame_timeout_ms": int(xr["frame_timeout_ms"]),
                "reconnect_interval_s": float(xr["reconnect_interval_s"]),
                "control_scheme": xr["control_scheme"],
                "device_id": xr["device_id"],
            }],
        ))

    # 4. Cameras
    if cfg["cameras"]["enable"]:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("w2_camera"), "launch", "cameras.launch.py")),
            launch_arguments={"config": config_path}.items()))
    return actions


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory("teleop_app"), "config", "teleop.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config, description="robot config YAML (config/robot.local.yaml)"),
        OpaqueFunction(function=launch_setup),
    ])
