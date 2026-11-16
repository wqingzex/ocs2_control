# robot_hmi

给人用的控制界面：RViz 面板、遥操作应用。依赖控制核心，不被控制核心依赖。

末端目标节点 [`target_generator`](../robot_control/target_generator/README.md)（RViz 交互 marker、XR、键盘手柄输入 → 目标位姿）原来在这里，现在属于控制栈，在 `robot_control/`：它依赖控制器核心，被启动文件依赖，不是给人用的界面。

| 形式 | 包 | 作用 | 谁用 |
|---|---|---|---|
| RViz | [`control_rviz_panels`](control_rviz_panels/README.md) | RViz 面板：状态机切换、夹爪、关节、全身控制状态 | 遥操作、推理 |
| Web | [`robot_app`](robot_app/README.md) | 遥操作应用：整套模块的启动与监控、Web 控制台（前端在 `robot_app/web/`，不是 ROS 包）、录制回放、诊断 | 遥操作 |

输入设备（XR 头显、手柄、键盘）在 [`robot_payload/input`](../robot_payload/input/README.md)，进程管理在 [`robot_utils`](../robot_utils/README.md)。使用说明见 [`docs/使用手册.md`](../../docs/使用手册.md)。

消息定义 `robot_control_msgs` 在 [`robot_control/`](../robot_control/robot_control_msgs/README.md)。
