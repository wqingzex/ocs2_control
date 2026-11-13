"""
Launch file for Wheel Humanoid MPC system.

Launches:
- MPC node
- Dummy simulation node
- Target publisher node
- RViz for visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # Package directories
    pkg_whole_body = get_package_share_directory('whole_body_mpc')
    pkg_whole_body_ros = get_package_share_directory('whole_body_mpc_nodes')
    
    # Launch arguments
    task_file_arg = DeclareLaunchArgument(
        'task_file',
        default_value=os.path.join(pkg_whole_body, 'config', 'task.info'),
        description='Path to task configuration file'
    )
    
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='',
        description='Path to robot URDF file (required)'
    )
    
    library_folder_arg = DeclareLaunchArgument(
        'library_folder',
        default_value='/tmp/whole_body_mpc',
        description='Path for auto-generated libraries'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz visualization'
    )
    
    use_dummy_arg = DeclareLaunchArgument(
        'use_dummy',
        default_value='true',
        description='Use dummy simulation instead of real robot'
    )
    
    # Node parameters
    common_params = {
        'task_file': LaunchConfiguration('task_file'),
        'urdf_file': LaunchConfiguration('urdf_file'),
        'library_folder': LaunchConfiguration('library_folder'),
    }
    
    # MPC Node
    mpc_node = Node(
        package='whole_body_mpc_nodes',
        executable='whole_body_mpc',
        name='whole_body_mpc',
        output='screen',
        parameters=[common_params],
    )
    
    # Dummy Node (for testing)
    dummy_node = Node(
        package='whole_body_mpc_nodes',
        executable='whole_body_dummy',
        name='whole_body_dummy',
        output='screen',
        parameters=[{
            **common_params,
            'mrt_frequency': 100.0,
        }],
        condition=IfCondition(LaunchConfiguration('use_dummy')),
    )
    
    # Target Publisher Node
    target_node = Node(
        package='whole_body_mpc_nodes',
        executable='whole_body_target',
        name='whole_body_target',
        output='screen',
    )
    
    # RViz
    rviz_config = os.path.join(pkg_whole_body_ros, 'config', 'whole_body.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )
    
    return LaunchDescription([
        # Arguments
        task_file_arg,
        urdf_file_arg,
        library_folder_arg,
        launch_rviz_arg,
        use_dummy_arg,
        
        # Nodes
        mpc_node,
        dummy_node,
        target_node,
        rviz_node,
    ])

