import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Import launch_helpers utilities
from launch_helpers import (
    detect_controllers,
    create_controller_manager_nodes,
    create_controller_spawners,
    get_ros2_control_robot_description
)

# All utility functions are now imported from launch_helpers

def launch_setup(context, *args, **kwargs):
    """Launch setup function using OpaqueFunction"""
    robot_name = context.launch_configurations['robot']
    robot_type = context.launch_configurations.get('arms', '')
    hardware = context.launch_configurations.get('hardware', 'mock_components')

    # 基本参数
    use_sim_time = hardware == 'isaac'

    # robot_state_publisher、ros2_control_node 和 joint_state_broadcaster
    controller_manager_nodes = create_controller_manager_nodes(
        robot_name=robot_name,
        robot_type=robot_type,
        hardware=hardware,
        use_sim_time=use_sim_time,
        launch_configurations={
            'robot': robot_name,
            'arms': robot_type,
            'hardware': hardware,
            'use_sim_time': str(use_sim_time),
        },
    )

    # Detect head controllers using launch_helpers (only if head is enabled)
    enable_head = context.launch_configurations.get('enable_head', 'true').lower() == 'true'
    head_controllers = []
    head_controller_spawners = []

    if enable_head:
        # Get ros2_control robot_description to verify joints exist in xacro
        # This uses the same logic as controller_manager.launch.py
        robot_description = get_ros2_control_robot_description(robot_name, robot_type, hardware)
        
        # Detect controllers matching head pattern
        # Pass robot_description to verify joints exist in xacro
        head_controllers = detect_controllers(robot_name, robot_type, ['head'], robot_description=robot_description)
        head_controller_spawners = create_controller_spawners(head_controllers, use_sim_time)

    # Detect body controllers using launch_helpers (only if body is enabled)
    enable_body = context.launch_configurations.get('enable_body', 'true').lower() == 'true'
    body_controllers = []
    body_controller_spawners = []

    if enable_body:
        # Get ros2_control robot_description to verify joints exist in xacro
        # This uses the same logic as controller_manager.launch.py
        robot_description = get_ros2_control_robot_description(robot_name, robot_type, hardware)
        
        # Detect controllers matching body pattern
        # Pass robot_description to verify joints exist in xacro
        body_controllers = detect_controllers(robot_name, robot_type, ['body'], robot_description=robot_description)
        body_controller_spawners = create_controller_spawners(body_controllers, use_sim_time)

    # RViz for visualization (optional)
    rviz_node = None
    use_rviz = context.launch_configurations.get('use_rviz', 'true').lower() == 'true'
    if use_rviz:
        rviz_base = os.path.join(
            get_package_share_directory("robot_bringup"), "config",
        )
        rviz_full_config = os.path.join(rviz_base, "head_body.rviz")
        
        # Check if config file exists, if not skip RViz
        if not os.path.exists(rviz_full_config):
            print(f"[WARN] RViz config file not found: {rviz_full_config}, skipping RViz")
        else:
            rviz_node = Node(
                package="rviz2",
                executable="rviz2",
                output="log",
                arguments=["-d", rviz_full_config],
                parameters=[{'use_sim_time': use_sim_time}],
            )

    # 统一的节点列表
    # robot_state_publisher 和 joint_state_broadcaster 由 create_controller_manager_nodes 创建
    nodes = [
        *controller_manager_nodes,
    ]

    # Add head controller spawners if any were detected
    nodes.extend(head_controller_spawners)

    # Add body controller spawners if any were detected
    nodes.extend(body_controller_spawners)

    # Add RViz if enabled
    if rviz_node:
        nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    # Command-line arguments
    robot_name_arg = DeclareLaunchArgument(
        "robot",
        default_value="fa_w1",
        description="Robot name (fa_w1, etc.)"
    )

    robot_type_arg = DeclareLaunchArgument(
        "arms",
        default_value="",
        description="Arm topology passed to xacro (robot-specific). Leave empty for the xacro default."
    )

    hardware_arg = DeclareLaunchArgument(
        "hardware",
        default_value="mock_components",
        description="Hardware type: 'isaac' for Isaac simulation, 'mock_components' for mock components"
    )

    enable_head_arg = DeclareLaunchArgument(
        'enable_head',
        default_value='true',
        description='Enable head controllers'
    )

    enable_body_arg = DeclareLaunchArgument(
        'enable_body',
        default_value='true',
        description='Enable body controllers'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz visualization'
    )

    return LaunchDescription([
        robot_name_arg,
        robot_type_arg,
        hardware_arg,
        enable_head_arg,
        enable_body_arg,
        use_rviz_arg,
        OpaqueFunction(function=launch_setup),
    ])

