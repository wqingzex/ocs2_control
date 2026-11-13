# robot_hmi

给人用的控制界面：RViz 面板和末端目标指令。遥操作、推理都会用到，依赖控制核心，不被控制核心依赖。

| 包 | 作用 |
|---|---|
| [`control_rviz_panels`](control_rviz_panels/README.md) | RViz 面板：状态机切换、夹爪、关节、全身控制状态 |
| [`teleop_command`](teleop_command/README.md) | 末端目标：RViz 交互 marker、VR 头显/手柄输入、键盘手柄输入 |

消息定义 `robot_control_msgs` 在 [`robot_control/`](../robot_control/robot_control_msgs/README.md)。
