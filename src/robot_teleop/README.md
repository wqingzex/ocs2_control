# teleop

遥操作应用：整套模块的启动与监控、Web 控制台、录制、诊断。输入设备（XR 头显、手柄、键盘）在 [`robot_payload/input`](../robot_payload/input/README.md)，输入转成目标位姿的部分在 [`robot_hmi/teleop_command`](../robot_hmi/teleop_command/README.md)。

| 包 | 作用 |
|---|---|
| [`teleop_app`](teleop_app/README.md) | 模块管理（启动、停止、日志）、Web 控制台、录制回放、诊断、头显配对 |
| [`web`](web/README.md) | Web 控制台的前端页面（不是 ROS 包，由 `teleop_app` 提供服务） |

使用说明见 [`docs/VR遥操作.md`](../../docs/VR遥操作.md)。
