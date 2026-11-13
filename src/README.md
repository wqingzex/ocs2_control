# src

源码，按职责分八个目录。包目录名与 ROS 包名一致；我们自己的分组目录统一加 `robot_` 前缀，`third_party` 是外部代码，不加。

| 目录 | 内容 |
|---|---|
| [`robot_control/`](robot_control/README.md) | 控制栈：控制器插件（臂、全身、关节、末端）、控制器核心、共用库、消息 |
| [`robot_launch/`](robot_launch/README.md) | 启动：URDF 生成、控制器拼装、整机启动文件、导航、RViz 看模型 |
| [`robot_hmi/`](robot_hmi/README.md) | 人机界面：RViz 面板、末端目标指令（遥操作和推理都用） |
| [`robot_description/`](robot_description/README.md) | 机器人模型：xacro、`ros2_control` 配置、OCS2 配置 |
| [`robot_hardware/`](robot_hardware/README.md) | 硬件插件（`hardware_interface::SystemInterface`），包平铺：Marvin、JX 电机、Modbus 末端、话题桥接、Rokae，通信层单独放在 `comm/` |
| [`robot_payload/`](robot_payload/README.md) | 本体外接装置：相机、手的配置、启动和设置 |
| [`robot_teleop/`](robot_teleop/README.md) | 遥操作：XR 头显、手柄、键盘、模块管理、Web 控制台 |
| `third_party/` | 子模块：`ocs2_ros2`（固定在 `ae75126f0`）、`OrbbecSDK_ROS2`；以及 `kh_ucanfd_sdk`（腰部 USB-CANFD 驱动） |

数据流：

```
输入（头显、手柄、RViz）→ robot_hmi → robot_control（控制器插件）→ robot_hardware 硬件插件 → 真机
                                                 ↑
                          robot_description（模型和配置）、robot_launch（启动时生成 URDF、拼装控制器）
```
