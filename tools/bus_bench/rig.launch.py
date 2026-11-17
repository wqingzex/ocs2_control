"""tools/bus_bench: robot_state_publisher + ros2_control_node + joint_state_broadcaster for rig.xacro.

Everything comes from the environment that bus_bench.py sets (BB_*), so the launch file has no arguments to forward:
BB_DEVICE, BB_SIDE, BB_BUS, BB_IO_MODE, BB_TORQUE, BB_MOCK, BB_CONTROLLERS (the controllers yaml, which carries the
update rate), BB_MOVE=1 (also spawn joint_trajectory_controller, for the step response).
"""
import os
from pathlib import Path

import xacro
from launch import LaunchDescription
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parent


def generate_launch_description():
    env = os.environ
    mappings = {
        "device": env.get("BB_DEVICE", "xhand1"),
        "side": env.get("BB_SIDE", "left"),
        "bus": env.get("BB_BUS", ""),
        "io_mode": env.get("BB_IO_MODE", "async"),
        "torque": env.get("BB_TORQUE", "0.8"),
        "mock": env.get("BB_MOCK", "false"),
    }
    description = {"robot_description": xacro.process_file(HERE / "rig.xacro", mappings=mappings).toxml()}
    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[description]),
        Node(package="controller_manager", executable="ros2_control_node", output="screen",
             parameters=[description, Path(env["BB_CONTROLLERS"])]),
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"]),
    ]
    if env.get("BB_MOVE") == "1":
        nodes.append(Node(package="controller_manager", executable="spawner", arguments=["joint_trajectory_controller"]))
    return LaunchDescription(nodes)
