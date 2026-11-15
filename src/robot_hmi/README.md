# robot_hmi

给人用的控制界面：RViz 面板、末端目标指令、遥操作应用。依赖控制核心，不被控制核心依赖。

| 形式 | 包 | 作用 | 谁用 |
|---|---|---|---|
| RViz | [`control_rviz_panels`](control_rviz_panels/README.md) | RViz 面板：状态机切换、夹爪、关节、全身控制状态 | 遥操作、推理 |
| 输入汇合 | [`teleop_command`](teleop_command/README.md) | 末端目标：RViz 交互 marker、XR 头显/手柄输入、键盘手柄输入 | 遥操作、推理 |
| Web | [`teleop_app`](teleop_app/README.md) | 遥操作应用：整套模块的启动与监控、Web 控制台（前端在 `teleop_app/web/`，不是 ROS 包）、录制回放、诊断 | 遥操作 |

输入设备（XR 头显、手柄、键盘）在 [`robot_payload/input`](../robot_payload/input/README.md)，进程管理在 [`robot_runtime`](../robot_runtime/README.md)。使用说明见 [`docs/使用手册.md`](../../docs/使用手册.md)。

消息定义 `robot_control_msgs` 在 [`robot_control/`](../robot_control/robot_control_msgs/README.md)。
