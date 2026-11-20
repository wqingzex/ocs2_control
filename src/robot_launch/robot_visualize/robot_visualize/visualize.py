"""RViz visualization launch helpers: xacro -> robot_state_publisher + joint_state_publisher_gui + rviz2."""

import os
import re

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_helpers import create_rmw_zenohd_node
from launch_helpers.robot.profile import (
    _cli_launch_value,
    create_eef_side_launch_arguments,
    resolve_end_effectors,
)


def get_default_type_from_xacro(xacro_file):
    """
    从 xacro 文件中读取 type 参数的默认值
    
    Args:
        xacro_file (str): xacro 文件路径
        
    Returns:
        str: 默认的 type 值，如果找不到则返回 None
    """
    with open(xacro_file, 'r') as f:  # an unreadable file raises
        content = f.read()
    # 查找 xacro:arg name="type" default="..." 的模式
    match = re.search(r'xacro:arg\s+name=["\']type["\']\s+default=["\']([^"\']+)["\']', content)
    return match.group(1) if match else None


def process_xacro(robot_name, xacro_filename="robot.xacro", **kwargs):
    """
    通用的 xacro 处理函数
    
    Args:
        robot_name (str): 机器人名称
        xacro_filename (str): xacro 文件名 (默认: "robot.xacro")
        **kwargs: 传递给 xacro 的映射参数 (如 type, collider, direction 等)
        
    Returns:
        str: 处理后的 URDF XML 字符串
    """
    package_description = robot_name + "_description"
    pkg_path = get_package_share_directory(package_description)
    xacro_file = os.path.join(pkg_path, 'xacro', xacro_filename)
    
    # 构建 mappings 字典，过滤空值
    mappings = {k: v for k, v in kwargs.items() if v and str(v).strip()}
    
    # 特殊处理：如果 type 参数为空，尝试从 xacro 文件读取默认值
    if 'type' not in mappings:
        default_type = get_default_type_from_xacro(xacro_file)
        if default_type:
            mappings['type'] = default_type
    
    # 根据 mappings 是否为空来决定如何处理 xacro 文件
    if mappings:
        robot_description_config = xacro.process_file(xacro_file, mappings=mappings)
    else:
        robot_description_config = xacro.process_file(xacro_file)
    
    return robot_description_config.toxml()


def create_visualization_nodes(robot_description, rviz_config_file):
    """
    创建通用的可视化节点
    
    Args:
        robot_description (str): 机器人描述 XML
        rviz_config_file (str): RViz 配置文件路径
        
    Returns:
        list: Node 对象列表
    """
    return [
        # rviz2 executable hosts multiple rclcpp nodes; avoid name= remaps to one name.
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=["-d", rviz_config_file]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'publish_frequency': 100.0,
                    'use_tf_static': True,
                    'robot_description': robot_description
                }
            ],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            output='screen',
        )
    ]


def create_common_launch_arguments():
    """
    创建通用的 launch 参数
    
    Returns:
        list: DeclareLaunchArgument 对象列表
    """
    return [
        DeclareLaunchArgument(
            'type',
            default_value='',
            description="A component xacro's own model key (e.g. hand.launch.py); empty = the xacro default"
        ),
        DeclareLaunchArgument(
            'arms',
            default_value='',
            description='Which arms of m6_ccs: dual | left | right (empty = dual)'
        ),
        *create_eef_side_launch_arguments(),
        DeclareLaunchArgument(
            'collider',
            default_value='',
            description='Collider type parameter for xacro (empty means no collider parameter passed to xacro)'
        ),
        DeclareLaunchArgument(
            'direction',
            default_value='',
            description='Direction parameter for xacro (empty means no direction parameter passed to xacro)'
        ),
        DeclareLaunchArgument(
            'skin',
            default_value='',
            description='Xacro skin key for visual meshes (robot-specific). Empty means no skin parameter passed to xacro.',
        ),
        DeclareLaunchArgument(
            'variant',
            default_value='',
            description='Platform sub-variant (robot-specific): e.g. m6_ccs variant:=desktop for booth layout, '
            'fa_w2 head variant:=wheel_V1.0 for the old narrow head limits (default: wide). Empty means no variant passed to xacro.',
        ),
    ]


def build_visualization_xacro_mappings(robot_name: str, launch_configurations: dict) -> dict:
    """Xacro mappings for RViz-only launches (no ros2_control / robot_profile)."""
    mappings = {}
    # type: a component xacro's own model key (hand.launch.py hand:=xhand1 type:=xhand1); arms: m6_ccs topology
    for key in ("collider", "direction", "arms", "type"):
        value = _cli_launch_value(launch_configurations, key)
        if value:
            mappings[key] = value

    left, right = resolve_end_effectors(launch_configurations, None)
    if left:
        mappings["left_end_effector"] = left
    if right:
        mappings["right_end_effector"] = right
    return mappings


def create_visualization_launch_description(
    robot_param_name='robot',
    robot_default_value='go1',
    xacro_filename='robot.xacro',
    rviz_config_name='urdf.rviz',
    additional_args=None
):
    """
    创建通用的可视化 launch 描述
    
    Args:
        robot_param_name (str): 机器人参数名称
        robot_default_value (str): 机器人默认值
        xacro_filename (str): xacro 文件名
        rviz_config_name (str): RViz 配置文件名
        additional_args (list): 额外的参数列表
        
    Returns:
        LaunchDescription: 配置好的 launch 描述
    """
    def launch_setup(context, *args, **kwargs):
        robot_value = context.launch_configurations[robot_param_name]
        mappings = build_visualization_xacro_mappings(
            robot_value, context.launch_configurations
        )

        package_description = robot_value + "_description"
        pkg_path = get_package_share_directory(package_description)
        xacro_file = os.path.join(pkg_path, "xacro", xacro_filename)
        if "type" not in mappings:
            default_type = get_default_type_from_xacro(xacro_file)
            if default_type:
                mappings["type"] = default_type

        robot_description = xacro.process_file(xacro_file, mappings=mappings).toxml()
        
        # 获取 RViz 配置文件
        rviz_config_file = os.path.join(
            get_package_share_directory("robot_visualize"),
            "config", "rviz", rviz_config_name
        )
        
        # 创建可视化节点
        nodes = create_visualization_nodes(robot_description, rviz_config_file)
        
        # 如果使用rmw_zenoh_cpp，自动添加rmw_zenohd节点
        rmw_zenohd_node = create_rmw_zenohd_node()
        if rmw_zenohd_node is not None:
            nodes.insert(0, rmw_zenohd_node)  # 将rmw_zenohd放在最前面，确保先启动
        
        return nodes

    # 构建参数列表
    args = [
        DeclareLaunchArgument(
            robot_param_name,
            default_value=robot_default_value,
            description=f'{robot_param_name} name to visualize'
        )
    ]
    
    # 添加通用参数
    args.extend(create_common_launch_arguments())
    # 添加额外参数
    if additional_args:
        args.extend(additional_args)
    
    # 添加 opaque function
    args.append(OpaqueFunction(function=launch_setup))
    
    return LaunchDescription(args)
