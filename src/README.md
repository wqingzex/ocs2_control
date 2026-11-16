# src

源码，按职责分九个目录。包目录名与 ROS 包名一致；我们自己的分组目录统一加 `robot_` 前缀，`third_party` 是外部代码，不加。

| 目录 | 内容 |
|---|---|
| [`robot_control/`](robot_control/README.md) | 控制栈：控制器插件（臂、全身、关节、末端）、控制器核心、共用库、消息 |
| [`robot_launch/`](robot_launch/README.md) | 启动：URDF 生成、控制器拼装、整机启动文件、导航、RViz 看模型 |
| [`robot_hmi/`](robot_hmi/README.md) | 人机界面：RViz 面板、末端目标指令（遥操作和推理都用），遥操作应用（模块管理、Web 控制台、录制、诊断） |
| [`robot_description/`](robot_description/README.md) | 机器人模型：xacro、`ros2_control` 配置、OCS2 配置 |
| [`robot_hardware/`](robot_hardware/README.md) | 硬件插件（`hardware_interface::SystemInterface`），包平铺：Marvin、JX 电机、Modbus 末端、话题桥接、Rokae，通信层单独放在 `comm/` |
| [`robot_payload/`](robot_payload/README.md) | 外部设备，一个设备一个包：相机、手（`w2_payload/`），XR 头显、手柄、键盘（`input/`） |
| [`robot_runtime/`](robot_runtime/README.md) | 进程管理：单元、监管器（启动、停止、监控、日志）、运行状态文件，不认识具体设备 |
| [`robot_log/`](robot_log/README.md) | 日志基础设施：日志模式、链路健康统计、飞行记录器（C++ 头文件），故障码和离线分析工具（Python） |
| `third_party/` | 外部代码和厂家 SDK，见 [`third_party/README.md`](third_party/README.md)：子模块 `ocs2_ros2`、`OrbbecSDK_ROS2`、天机 SDK 338 / 343；直接放入的 `kh_ucanfd_sdk`、`wuji-sdk-c`、`dexcap-sdk-cpp` |

数据流：

```
输入（头显、手柄、RViz）→ robot_hmi → robot_control（控制器插件）→ robot_hardware 硬件插件 → 真机
                                                 ↑
                          robot_description（模型和配置）、robot_launch（启动时生成 URDF、拼装控制器）
```
