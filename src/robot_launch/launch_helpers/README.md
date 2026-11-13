# launch_helpers

启动用的共享 Python 库：根据机器人名、末端类型和硬件，生成 URDF、拼出控制器配置、创建 `ros2_control` 节点。**只有库，没有 launch 文件**，不依赖任何应用包。

## 作用

`robot_bringup` 的 launch 文件和 `robot_visualize` 调用它，避免每个 launch 各写一遍"找包、读 xacro、拼参数"。

## 使用

```python
from launch_helpers import (
    create_controller_manager_nodes, get_ros2_control_robot_description, build_planning_urdf_launch_params)

nodes = create_controller_manager_nodes(
    robot_name="fiveages_w2", robot_type="linkerhand_l6", hardware="mock_components",
    launch_configurations=context.launch_configurations)        # robot_state_publisher、ros2_control_node、spawner
```

机器人按约定从 `<robot>_description` 包里取 xacro 和配置；`robot.local.yaml` 的 `xacro:` / `hardware:` 段和命令行参数一起拼成 xacro 参数。

## 模块

| 模块 | 内容 |
|---|---|
| `robot/config.py` | 加载机器人的 `ros2_control` 配置：类型、变体、`control.patch` 叠加 |
| `robot/profile.py` | 解析 `robot.local.yaml`、launch 参数，生成 xacro 参数（`build_xacro_mappings`） |
| `robot/urdf.py` | 从 xacro 生成 URDF：控制视图、规划视图（见下） |
| `control/compose.py` | 按左右末端类型合成控制器配置，注册表见 `config/eef_control_registry.yaml` |
| `control/manager.py` | 创建 `robot_state_publisher`、`ros2_control_node`、`joint_state_broadcaster` |
| `control/spawners.py` | 检测 URDF 里有哪些控制器并生成 spawner |
| `control/task_info.py` | 解析 OCS2 `.info`、准备 `teleop_command` 参数 |
| `launch_utils.py` | rmw_zenoh 路由、launch 模式参数 |

## 两份 URDF

同一份 xacro 按用途生成两份：

| 视图 | 入口 | 特点 |
|---|---|---|
| 控制（`get_ros2_control_robot_description`） | `xacro/ros2_control/robot.xacro` | 带硬件插件和关节接口，手指、底盘关节保持可动；字符串，进程内缓存 |
| 规划（`build_planning_urdf_launch_params`） | `xacro/robot.xacro` | 冻结手指和底盘关节，MPC 状态只含腰、臂、头；写成文件，路径给控制器 |

规划 URDF 的文件名是内容哈希：xacro 没变路径不变，改了自然是新文件。目录在每用户私有位置，依次取 `OCS2_PLANNING_URDF_DIR`、`$XDG_CACHE_HOME`、`~/.ros/ocs2_cache`，最后是 `/tmp/ocs2_ros2-<uid>`；目录不属于当前用户或其他人可写时拒绝使用。

## 注意

- `hardware:=gz` 会直接报错，Gazebo 已不支持。
- 测试：`pytest test`，需要 ROS 环境。
