"""
Launch file for Target Publisher node only.
Use this for interactive target setting with RViz.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Target Publisher Node with interactive markers
    target_node = Node(
        package='whole_body_mpc_nodes',
        executable='whole_body_target',
        name='whole_body_target',
        output='screen',
    )
    
    return LaunchDescription([
        target_node,
    ])

