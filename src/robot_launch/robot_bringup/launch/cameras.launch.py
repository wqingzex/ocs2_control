"""Start the Orbbec cameras of a teleop YAML, one driver process each (only:= picks some).

    ros2 launch robot_bringup cameras.launch.py config:=/path/to/robot.local.yaml [only:=camera_head]"""
import os
import re

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare

from launch_helpers.devices.camera import camera_specs, runtime_dir
from robot_config.config import load


def camera_actions(cfg, only=(), name_prefix=""):
    if name_prefix and not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name_prefix):
        raise ValueError("invalid camera name prefix")
    actions = []
    orbbec_launch_dir = os.path.join(FindPackageShare("orbbec_camera").find("orbbec_camera"), "launch")
    specs = camera_specs(cfg)
    unknown = set(only) - {s.name for s in specs}
    if unknown:
        raise RuntimeError(f"only:= names cameras that are not enabled in the config: {sorted(unknown)}")
    for spec in specs:
        if only and spec.name not in only:
            continue
        if name_prefix:
            old, new = spec.name, name_prefix + spec.name
            rename = lambda topic: topic.replace(f"/{old}/", f"/{new}/")
            spec.name = spec.args["camera_name"] = new
            spec.remaps = [(rename(src), rename(dst)) for src, dst in spec.remaps]
        args = dict(spec.args)
        if spec.config_yaml:
            path = os.path.join(runtime_dir(), f"{spec.name}.yaml")
            with open(path, "w", encoding="utf-8") as handle:
                yaml.safe_dump(spec.config_yaml, handle, sort_keys=False)
            os.chmod(path, 0o600)
            args["config_file_path"] = path
        include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(orbbec_launch_dir, spec.launch_file)),
            launch_arguments=args.items(),
        )
        actions.append(GroupAction([*(SetRemap(src=s, dst=d) for s, d in spec.remaps), include]))
        actions.append(LogInfo(msg=f"{spec.name}: {spec.launch_file} serial={args['serial_number']}"))
    if not actions:
        actions.append(LogInfo(msg="cameras disabled in config"))
    return actions


def launch_setup(context):
    only = [n for n in LaunchConfiguration("only").perform(context).split(",") if n]
    return camera_actions(load(LaunchConfiguration("config").perform(context)), only,
                          LaunchConfiguration("name_prefix").perform(context))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", description="shared robot YAML (config/robot.local.yaml)"),
        DeclareLaunchArgument("only", default_value="",
                              description="comma-separated camera names to start (default: all enabled)"),
        DeclareLaunchArgument("name_prefix", default_value="", description="private name prefix for setup previews"),
        OpaqueFunction(function=launch_setup),
    ])
