import os
import sys
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from launch_helpers import (
    create_launch_mode_arguments,
    create_robot_profile_launch_arguments,
    resolve_info_file_path,
    parse_task_info,
    prepare_target_generator_parameters,
)

_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _launch_dir not in sys.path:
    sys.path.insert(0, _launch_dir)
import _bringup_common as common


def launch_setup(context, *args, **kwargs):
    ctx = common.build_context(context)

    planning_robot_name = common.resolve_planning_robot_name_from_config(
        ctx.config, "whole_body_controller", ctx.robot_name
    )

    # The main controller of this launch is the "whole_body_controller" instance; its plugin is the whole-body
    # controller on a wheeled humanoid and the arm controller on robots that drive the body with the arm task
    # (then the head needs its own controller).
    whole_body_available = False
    mpc_task = ""
    if ctx.config:
        try:
            cm_params = ctx.config.get("controller_manager", {}).get("ros__parameters", {})
            controller_type = cm_params.get("whole_body_controller", {}).get("type", "")
            if controller_type == "whole_body_controller/WholeBodyController":
                mpc_task = "whole_body"
            elif controller_type == "arm_controller/ArmController":
                mpc_task = "arm"
            whole_body_available = mpc_task == "whole_body"
            print(
                f"[INFO] whole_body_controller: type = {controller_type}, mpc_task = {mpc_task!r}, "
                f"whole_body_available = {whole_body_available}"
            )
        except Exception as exc:
            print(f"[WARN] Failed to parse whole_body_controller from config: {exc}")

    planning_urdf_params = common.validate_planning_urdf(
        planning_robot_name,
        ctx.launch_configurations,
        ctx.hardware,
        ctx.profile_path or None,
        planning_scope="full",
    )
    if planning_urdf_params is None:
        return []

    main_spawner, ocs2_planning_param_file = common.create_main_controller_spawner(
        "whole_body_controller", planning_urdf_params
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

    body_spawners = []
    joint_controller_names = ["whole_body_controller"]
    is_arm_controller_type = mpc_task == "arm"
    if is_arm_controller_type:
        head_controllers, body_spawners = common.setup_body_controllers(ctx, ["head"])
        if head_controllers:
            joint_controller_names.extend(c["name"] for c in head_controllers)
            print(
                f"[INFO] ARM type detected in whole_body: spawning head controllers "
                f"{[c['name'] for c in head_controllers]}"
            )
        else:
            print("[WARN] ARM type detected in whole_body but no head controller found to spawn")

    if hand_names:
        joint_controller_names.extend(hand_names)

    # Same .info the whole_body_controller loads (info_package / robot_name + info_file_name).
    task_file_path = resolve_info_file_path(ctx.config, "whole_body", planning_robot_name)
    if task_file_path is None:
        return []
    print(f"[INFO] Using task file for TargetGenerator: {task_file_path}")

    _dual_arm, _control_base_frame, _auto_marker_frame = parse_task_info(task_file_path)
    if whole_body_available:
        marker_fixed_frame = "world"
    elif _auto_marker_frame:
        marker_fixed_frame = _auto_marker_frame
    else:
        marker_fixed_frame = _control_base_frame
    print(
        f"[INFO] marker_fixed_frame={marker_fixed_frame} "
        f"(whole_body_available={whole_body_available}, control_base_frame={_control_base_frame})"
    )

    target_generator = None
    target_generator_params = prepare_target_generator_parameters(
        task_file_path=task_file_path,
        config_file_path=None,
        hand_controllers=hand_names or None,
        marker_fixed_frame=marker_fixed_frame,
    )
    if target_generator_params and (
        context.launch_configurations.get("enable_target_generator", "true").lower()
        == "true"
    ):
        target_generator = Node(
            package="target_generator",
            executable="target_generator_node",
            name="target_generator",
            output="screen",
            parameters=target_generator_params + [{"use_sim_time": ctx.use_sim_time}],
        )

    rviz_config_path = common.resolve_rviz_config(ctx.robot_name, "whole_body.rviz")
    if is_arm_controller_type and marker_fixed_frame:
        try:
            with open(rviz_config_path, "r", encoding="utf-8") as handle:
                rviz_content = handle.read()
            patched_content = rviz_content.replace(
                "Fixed Frame: world", f"Fixed Frame: {marker_fixed_frame}"
            )
            tmp_name = f"whole_body_{ctx.robot_name}_{ctx.robot_type or 'default'}_arm_fixedframe.rviz"
            tmp_rviz_config_path = os.path.join(tempfile.gettempdir(), tmp_name)
            with open(tmp_rviz_config_path, "w", encoding="utf-8") as handle:
                handle.write(patched_content)
            rviz_config_path = tmp_rviz_config_path
            print(
                f"[INFO] ARM type: RViz Fixed Frame={marker_fixed_frame}: {rviz_config_path}"
            )
        except Exception as exc:
            print(f"[WARN] Failed to patch RViz Fixed Frame for ARM type: {exc}")

    rviz_node = common.build_rviz_node(
        ctx,
        rviz_config_path,
        hand_names,
        joint_controller_names,
        extra_rviz_parameters=[{"whole_body_available": whole_body_available}],
    )

    return common.assemble_nodes(
        ctx,
        rviz_node=rviz_node,
        controller_stack_nodes=controller_stack_nodes,
        main_spawner=main_spawner,
        extra_spawners=hand_spawners + body_spawners,
        optional_nodes=[target_generator] if target_generator else [],
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="cr5"),
            DeclareLaunchArgument("hardware", default_value="mock_components"),
            DeclareLaunchArgument("enable_target_generator", default_value="true"),
            DeclareLaunchArgument("enable_gripper", default_value="true"),
            *create_launch_mode_arguments(),
        ]
        + create_robot_profile_launch_arguments()
        + [OpaqueFunction(function=launch_setup)]
    )
