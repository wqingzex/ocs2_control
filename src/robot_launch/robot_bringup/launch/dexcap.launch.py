"""DexCap teleoperation: device nodes, arm joint mapping and glove mapping in one launch.

  ros2 launch robot_bringup dexcap.launch.py                                   # exoskeleton + both gloves, arm mapping
  ros2 launch robot_bringup dexcap.launch.py left_eef:=hand right_eef:=hand    # + gloves drive the LinkerHand L6
  ros2 launch robot_bringup dexcap.launch.py left_eef:=gripper right_eef:=gripper
  ros2 launch robot_bringup dexcap.launch.py arm_mapper:=false                 # raw /dexcap/* topics only

The arm mapping streams to /arm_controller/joint_target_position: run the robot in split_body or dual_arm (in
whole_body the arms belong to whole_body_controller). It only follows while the controller is in MOVEJ.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch_ros.actions import Node

EEF_CHOICES = ("none", "hand", "gripper")


def _check_serial_port(path, what):
    """Fail early with a readable message instead of an SDK error deep in the node."""
    if not os.path.exists(path):
        raise RuntimeError(f"{what}: serial port {path} does not exist (ls /dev/ttyACM* /dev/ttyUSB* /dev/serial/by-id/)")
    if not os.access(path, os.R_OK | os.W_OK):
        raise RuntimeError(f"{what}: no read/write access to {path}; the user must be in the dialout group "
                           "(in the container: same GID as on the host)")


def _flag(context, name):
    return context.launch_configurations[name].strip().lower() == "true"


def launch_setup(context):
    share = get_package_share_directory("dexcap_teleop")
    cfg = context.launch_configurations
    actions = []

    # ---- device nodes: raw JointState on /dexcap/{waist,left_arm,right_arm,left_hand,right_hand} ----------------
    if _flag(context, "start_upper_body"):
        _check_serial_port(cfg["upper_body_serial_port"], "DexCap upper body")
        actions.append(Node(
            package="dexcap_teleop", executable="dexcap_upper_body_node", name="dexcap_upper_body", output="screen",
            parameters=[{"serial_port": cfg["upper_body_serial_port"],
                         "publish_rate": float(cfg["upper_body_publish_rate"])}]))
    # The SDK's device discovery is unreliable when two glove processes start at the same moment: stagger them.
    for delay, side in ((0.5, "left"), (1.0, "right")):
        if not _flag(context, f"start_{side}_hand"):
            continue
        port = cfg[f"{side}_hand_serial_port"]
        _check_serial_port(port, f"DexCap {side} glove")
        actions.append(TimerAction(period=delay, actions=[Node(
            package="dexcap_teleop", executable="dexcap_glove_node", name=f"dexcap_{side}_hand", output="screen",
            parameters=[{"side": side, "serial_port": port, "publish_rate": float(cfg["hand_publish_rate"])}])]))

    # ---- exoskeleton arms -> dual-arm joint targets ---------------------------------------------------------------
    if _flag(context, "arm_mapper"):
        actions.append(Node(
            package="dexcap_teleop", executable="arm_joint_mapper", name="arm_joint_mapper", output="screen",
            parameters=[os.path.join(share, "config", "arm_joint_mapper.yaml")]))

    # ---- gloves -> hand / gripper -----------------------------------------------------------------------------
    for side in ("left", "right"):
        eef = cfg[f"{side}_eef"].strip().lower()
        if eef not in EEF_CHOICES:
            raise RuntimeError(f"{side}_eef must be one of {list(EEF_CHOICES)}, got '{eef}'")
        if eef == "hand":
            actions.append(Node(
                package="dexcap_teleop", executable="dexcap_hand_mapper", name=f"dexcap_{side}_hand_mapper",
                output="screen", parameters=[os.path.join(share, "config", f"hand_mapper_{side}.yaml")]))
        elif eef == "gripper":
            urdf = os.path.join(share, "urdf", "DexGlove_L_v4.urdf" if side == "left" else "DexGlove_R_v4.urdf")
            with open(urdf, "r", encoding="utf-8") as handle:
                robot_description = handle.read()
            actions.append(Node(
                package="dexcap_teleop", executable="dexcap_gripper_mapper", name=f"dexcap_{side}_gripper_mapper",
                output="screen",
                parameters=[{
                    "robot_description": robot_description,
                    "hand_side": side,
                    "input_topic": f"/dexcap/{side}_hand",
                    "pinch_distance_topic": f"/dexcap/{side}_hand/pinch_distance",
                    "gripper_target_topic": f"/{side}_gripper_controller/target_percent",
                    "publish_gripper_target": True,
                    "pinch_closed_distance": float(cfg["pinch_closed_distance"]),
                    "pinch_open_distance": float(cfg["pinch_open_distance"]),
                    "gripper_filter_alpha": 0.25,
                    "gripper_min_change": 0.005,
                    "input_timeout": 0.2,
                }]))
    return actions


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("start_upper_body", default_value="true", description="exoskeleton (waist + arms)"),
        DeclareLaunchArgument("start_left_hand", default_value="true", description="left glove"),
        DeclareLaunchArgument("start_right_hand", default_value="true", description="right glove"),
        DeclareLaunchArgument("upper_body_serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("left_hand_serial_port", default_value="/dev/ttyUSB1"),
        DeclareLaunchArgument("right_hand_serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("upper_body_publish_rate", default_value="1000.0"),
        DeclareLaunchArgument("hand_publish_rate", default_value="40.0"),
        DeclareLaunchArgument("arm_mapper", default_value="true",
                              description="map the exoskeleton arms to /arm_controller/joint_target_position"),
        DeclareLaunchArgument("left_eef", default_value="none", description="none | hand (LinkerHand L6) | gripper"),
        DeclareLaunchArgument("right_eef", default_value="none", description="none | hand (LinkerHand L6) | gripper"),
        DeclareLaunchArgument("pinch_closed_distance", default_value="0.040"),
        DeclareLaunchArgument("pinch_open_distance", default_value="0.110"),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])
