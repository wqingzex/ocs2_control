import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from launch_helpers import (
    create_launch_mode_arguments,
    create_robot_profile_launch_arguments,
    resolve_info_file_path,
    prepare_teleop_command_parameters,
)

_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _launch_dir not in sys.path:
    sys.path.insert(0, _launch_dir)
import _bringup_common as common


def launch_setup(context, *args, **kwargs):
    ctx = common.build_context(context)

    planning_robot_name = common.resolve_planning_robot_name_from_config(
        ctx.config, "arm_controller", ctx.robot_name
    )

    planning_urdf_params = common.validate_planning_urdf(
        planning_robot_name,
        ctx.launch_configurations,
        ctx.hardware,
        ctx.profile_path or None,
        planning_scope="arms",
    )
    if planning_urdf_params is None:
        return []

    main_spawner, ocs2_planning_param_file = common.create_main_controller_spawner(
        "arm_controller", planning_urdf_params
    )
    controller_stack_nodes = common.create_controller_stack_nodes(
        ctx,
        ocs2_planning_param_file=ocs2_planning_param_file,
    )

    enable_gripper = (
        context.launch_configurations.get("enable_gripper", "true").lower() == "true"
    )
    hand_controllers, hand_spawners = common.setup_hand_controllers(ctx, enable_gripper)
    hand_names = [c["name"] for c in hand_controllers] if enable_gripper else []

    # Same .info the arm_controller loads (info_package / robot_name + info_file_name).
    task_file_path = resolve_info_file_path(ctx.config, "dual_arm", ctx.robot_name)
    if task_file_path is None:
        return []
    print(f"[INFO] Using task file for TeleopCommand: {task_file_path}")

    teleop_command = None
    teleop_command_params = prepare_teleop_command_parameters(
        task_file_path=task_file_path,
        config_file_path=None,
        hand_controllers=hand_names or None,
    )
    if teleop_command_params and (
        context.launch_configurations.get("enable_teleop_command", "true").lower()
        == "true"
    ):
        teleop_command = Node(
            package="teleop_command",
            executable="teleop_command_node",
            name="teleop_command",
            output="screen",
            parameters=teleop_command_params + [{"use_sim_time": ctx.use_sim_time}],
        )

    rviz_node = common.build_rviz_node(
        ctx,
        common.resolve_rviz_config(ctx.robot_name, "dual_arm.rviz"),
        hand_names,
        ["arm_controller"] + hand_names,
    )

    return common.assemble_nodes(
        ctx,
        rviz_node=rviz_node,
        controller_stack_nodes=controller_stack_nodes,
        main_spawner=main_spawner,
        extra_spawners=hand_spawners,
        optional_nodes=[teleop_command] if teleop_command else [],
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="cr5"),
            DeclareLaunchArgument("type", default_value=""),
            DeclareLaunchArgument("hardware", default_value="mock_components"),
            DeclareLaunchArgument("enable_teleop_command", default_value="true"),
            DeclareLaunchArgument("enable_gripper", default_value="true"),
            *create_launch_mode_arguments(),
        ]
        + create_robot_profile_launch_arguments()
        + [OpaqueFunction(function=launch_setup)]
    )
