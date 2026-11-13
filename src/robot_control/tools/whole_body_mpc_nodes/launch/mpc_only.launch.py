"""
Launch file for Wheel Humanoid MPC node only.
Use this when connecting to real robot or external simulation.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_whole_body = get_package_share_directory('whole_body_mpc')
    
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
    
    # MPC Node
    mpc_node = Node(
        package='whole_body_mpc_nodes',
        executable='whole_body_mpc',
        name='whole_body_mpc',
        output='screen',
        parameters=[{
            'task_file': LaunchConfiguration('task_file'),
            'urdf_file': LaunchConfiguration('urdf_file'),
            'library_folder': LaunchConfiguration('library_folder'),
        }],
    )
    
    return LaunchDescription([
        task_file_arg,
        urdf_file_arg,
        library_folder_arg,
        mpc_node,
    ])

