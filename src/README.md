# src

源码，按职责分九个目录。包目录名与 ROS 包名一致；我们自己的分组目录统一加 `robot_` 前缀，`third_party` 是外部代码，不加。

| 目录 | 内容 |
|---|---|
| [`robot_control/`](robot_control/README.md) | 控制栈：末端目标节点（遥操作和推理都用）、控制器插件（臂、全身、关节、末端）、控制器核心、共用库、消息 |
| [`robot_launch/`](robot_launch/README.md) | 启动：URDF 生成、控制器拼装、整机启动文件、导航、RViz 看模型 |
| [`robot_hmi/`](robot_hmi/README.md) | 人机界面：RViz 面板，遥操作应用（模块管理、Web 控制台、录制、诊断） |
| [`robot_description/`](robot_description/README.md) | 机器人模型：xacro、`ros2_control` 配置、OCS2 配置 |
| [`robot_hardware/`](robot_hardware/README.md) | 硬件插件（`hardware_interface::SystemInterface`），包平铺：Marvin、JX 电机、Modbus 末端、话题桥接、Rokae，通信层单独放在 `comm/` |
| [`robot_setup/`](robot_setup/README.md) | 配置：生成和改写 `robot.local.yaml`，确认相机、末端、头显（`robot_config`、`camera_setup`、`end_effector_setup`、`xr_setup`） |
| [`robot_teleop/`](robot_teleop/README.md) | 输入设备本身：XR 头显、手柄、键盘、DexCap（节点和协议） |
| [`robot_utils/`](robot_utils/README.md) | 运行和观测的基础设施，一个包：进程管理（单元、监管器、运行状态文件，Python）；日志模式、链路健康统计、飞行记录器（C++ 头文件）；故障码和离线分析工具（Python）。不依赖设备或控制包 |
| `third_party/` | 外部代码和厂家 SDK，见 [`third_party/README.md`](third_party/README.md)：子模块 `ocs2_ros2`、`OrbbecSDK_ROS2`、天机 SDK 338 / 343、`dexcap-sdk-cpp`；直接放入的 `kh_ucanfd_sdk`、`wuji-sdk-c` |

数据流：

```
输入（头显、手柄、RViz）→ target_generator（目标）→ 控制器插件（都在 robot_control）→ robot_hardware 硬件插件 → 真机
                                                 ↑
                          robot_description（模型和配置）、robot_launch（启动时生成 URDF、拼装控制器）
```
