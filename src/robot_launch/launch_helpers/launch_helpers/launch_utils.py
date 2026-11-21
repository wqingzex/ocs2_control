"""
Launch helpers shared by every launch entry point: rmw_zenoh router and launch-mode arguments.
"""

import os
import socket
import subprocess

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def is_rmw_zenoh():
    """
    检测当前使用的ROS2中间件是否为rmw_zenoh_cpp
    
    Returns:
        bool: 如果使用rmw_zenoh_cpp则返回True，否则返回False
    """
    rmw_implementation = os.environ.get('RMW_IMPLEMENTATION', '')
    return rmw_implementation == 'rmw_zenoh_cpp'


def is_zenoh_router_running():
    """Check for a local Zenoh process or a listener on the default endpoint."""
    try:
        # 按进程名查找可用的路由，排除启动命令本身和退出后的僵尸进程。
        result = subprocess.run(
            ['ps', '-C', 'rmw_zenohd,zenohd', '-o', 'stat='],
            capture_output=True,
            text=True
        )
        if any(state[0] not in 'ZX' for state in result.stdout.split()):
            return True
    except OSError as exc:
        print(f"[WARNING] Failed to check for Zenoh router processes: {exc}")

    if os.environ.get('ZENOH_ROUTER_CONFIG_URI') or 'listen/endpoints' in os.environ.get('ZENOH_CONFIG_OVERRIDE', ''):
        return False

    # Host-networked containers can reach the host's router across PID namespaces.
    try:
        with socket.create_connection(('localhost', 7447), timeout=0.25):
            return True
    except OSError:
        return False


def create_rmw_zenohd_node():
    """
    创建rmw_zenohd节点（当使用rmw_zenoh_cpp中间件时）
    如果系统中已经有zenoh router在运行，则不会重复启动
    
    Returns:
        Node: rmw_zenohd节点对象，如果不需要或已存在则返回None
    """
    if not is_rmw_zenoh():
        return None
    
    # 检查是否已经有zenoh router在运行
    if is_zenoh_router_running():
        print("[INFO] Zenoh router is already running, skipping rmw_zenohd node creation")
        return None
    
    return Node(
        package='rmw_zenoh_cpp',
        executable='rmw_zenohd',
        name='rmw_zenohd',
        output='screen',
    )


def parse_launch_mode(context):
    """
    解析launch_mode配置
    
    Args:
        context: Launch context对象
        
    Returns:
        tuple: (launch_mode, rviz_only, use_rviz)
            - launch_mode: 'full', 'control_only', or 'rviz_only'
            - rviz_only: bool, 是否为rviz_only模式
            - use_rviz: bool, 是否需要启动rviz
    """
    launch_mode = context.launch_configurations.get('launch_mode', 'full').lower()
    rviz_only = launch_mode == 'rviz_only'
    use_rviz = launch_mode in ['full', 'rviz_only']
    
    return launch_mode, rviz_only, use_rviz


def create_launch_mode_arguments():
    """
    创建launch_mode相关的参数声明
    
    Returns:
        list: DeclareLaunchArgument对象列表
    """
    return [
        DeclareLaunchArgument(
            'launch_mode',
            default_value='full',
            description="Launch mode: 'full' (rviz + control, default), 'control_only' (no rviz), or 'rviz_only' (only rviz for remote visualization)"
        ),
    ]
