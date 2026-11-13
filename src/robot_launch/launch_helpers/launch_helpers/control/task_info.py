"""OCS2 task .info parsing and teleop_command parameter preparation."""

import os
import re

from ament_index_python.packages import get_package_share_directory



def parse_task_info(task_file_path):
    """
    解析 task.info 文件，提取 dual_arm_mode 和 control_base_frame 信息。
    
    Args:
        task_file_path (str): task.info 文件路径
        
    Returns:
        tuple: (dual_arm_mode, control_base_frame, marker_fixed_frame)
            - dual_arm_mode (bool): 是否为双臂模式
            - control_base_frame (str): 控制基坐标系ID
            - marker_fixed_frame (str or None): marker固定坐标系，None表示由teleop_command.yaml决定

    Example:
        >>> dual_arm, frame, marker_frame = parse_task_info('/path/to/task.info')
        >>> print(f"Dual arm: {dual_arm}, Frame: {frame}, Marker frame: {marker_frame}")
    """
    dual_arm_mode = False
    control_base_frame = "world"
    marker_fixed_frame = None

    if not os.path.exists(task_file_path):
        print(f"[WARN] Task file not found: {task_file_path}")
        return dual_arm_mode, control_base_frame, marker_fixed_frame

    try:
        with open(task_file_path, 'r') as file:
            content = file.read()

        # 检查是否有 dualArmMode 配置
        dual_arm_match = re.search(r'dualArmMode\s+true', content)
        if dual_arm_match:
            dual_arm_mode = True
            print(f"[INFO] Detected dual arm mode from task file")

        # 检查是否有 eeFrame1（双臂机器人的第二个末端执行器）
        ee_frame1_match = re.search(r'eeFrame1\s+"([^"]+)"', content)
        if ee_frame1_match:
            dual_arm_mode = True
            print(f"[INFO] Detected dual arm mode from eeFrame1: {ee_frame1_match.group(1)}")

        # 提取 baseFrame
        base_frame_match = re.search(r'baseFrame\s+"([^"]+)"', content)
        if base_frame_match:
            control_base_frame = base_frame_match.group(1)
            print(f"[INFO] Detected base frame: {control_base_frame}")

        # 轮式底盘模式（manipulatorModelType=1）下，OCS2目标和EE均在world坐标系下
        # marker也必须锚定在world，否则底盘移动时marker跟着漂移
        model_type_match = re.search(r'manipulatorModelType\s+(\d+)', content)
        if model_type_match and int(model_type_match.group(1)) == 1:
            control_base_frame = "world"
            marker_fixed_frame = "world"
            print(f"[INFO] Wheel-based mode detected, overriding control_base_frame and marker_fixed_frame to 'world'")

        print(f"[INFO] Parsed task file - dual_arm_mode: {dual_arm_mode}, control_base_frame: {control_base_frame}, marker_fixed_frame: {marker_fixed_frame}")

    except Exception as e:
        print(f"[ERROR] Failed to parse task file {task_file_path}: {e}")

    return dual_arm_mode, control_base_frame, marker_fixed_frame


def prepare_teleop_command_parameters(
    task_file_path,
    config_file_path=None,
    marker_fixed_frame=None,
    hand_controllers=None,
):
    """
    准备 TeleopCommand 节点的参数。
    
    这个函数处理所有参数准备逻辑，包括：
    - 解析 task.info 文件
    - 查找配置文件路径
    - 构建参数字典
    
    Args:
        task_file_path (str): task.info 文件路径
        config_file_path (str, optional): 显式指定的配置文件路径
        marker_fixed_frame (str, optional): marker 固定坐标系。如果为 None，则使用配置文件中的值或节点默认值 'base_link'
        hand_controllers (list, optional): 手部/夹爪控制器名称列表
        
    Note:
        enable_head_control is now configured via YAML config file, not via function parameter
        
    Returns:
        list: 节点参数列表，可以直接传递给 Node 的 parameters 参数
        
    Example:
        >>> params = prepare_teleop_command_parameters(
        ...     task_file_path='/path/to/task.info',
        ...     hand_controllers=['left_hand_controller', 'right_hand_controller']
        ... )
        >>> node = Node(package='teleop_command', executable='teleop_command_node', parameters=params)
    """
    if not task_file_path:
        print(f"[ERROR] No task_file provided")
        return []
    
    # 解析 task.info 文件
    dual_arm_mode, control_base_frame, auto_marker_fixed_frame = parse_task_info(task_file_path)

    # 确定配置文件路径（优先级：显式指定的config_file > task_file同目录 > 默认配置）
    config_path = None
    
    # 如果显式指定了配置文件路径，优先使用
    if config_file_path:
        if os.path.exists(config_file_path):
            config_path = config_file_path
            print(f"[INFO] Using explicitly specified config file: {config_path}")
        else:
            print(f"[WARN] Specified config file not found: {config_file_path}, falling back to default search")
    
    # 如果还没有找到，检查 task_file 同目录下是否有 teleop_command.yaml
    if not config_path:
        task_file_dir = os.path.dirname(task_file_path)
        task_dir_config = os.path.join(task_file_dir, 'teleop_command.yaml')
        if os.path.exists(task_dir_config):
            config_path = task_dir_config
            print(f"[INFO] Using config file from task file directory: {config_path}")
    
    # 如果还没有找到，使用默认配置文件
    if not config_path:
        try:
            package_dir = get_package_share_directory('teleop_command')
            default_config = os.path.join(package_dir, 'config', 'default.yaml')
            if os.path.exists(default_config):
                config_path = default_config
                print(f"[INFO] Using default config file: {config_path}")
            else:
                print(f"[WARN] Default config file not found: {default_config}, using default parameters")
        except Exception as e:
            print(f"[WARN] Failed to get default config file: {e}")
    
    # 构建参数列表：先加载 YAML 配置，然后用 launch 参数覆盖
    parameters = []
    if config_path:
        parameters.append(config_path)
    
    # 必须覆盖的参数（从 task_file 解析或必需参数）
    override_params = {
        'dual_arm_mode': dual_arm_mode,
        'control_base_frame': control_base_frame
    }
    
    # marker_fixed_frame 优先级：函数参数 > task.info自动检测 > teleop_command.yaml
    effective_marker_fixed_frame = marker_fixed_frame if marker_fixed_frame is not None else auto_marker_fixed_frame
    if effective_marker_fixed_frame is not None:
        override_params['marker_fixed_frame'] = effective_marker_fixed_frame
    
    # 添加 hand_controllers 参数（如果提供）
    if hand_controllers:
        override_params['hand_controllers'] = hand_controllers

    parameters.append(override_params)
    
    return parameters
