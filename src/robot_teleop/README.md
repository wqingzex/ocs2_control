# teleop

遥操作：VR 头显和手柄、键盘怎么变成控制指令，以及整套模块的启动与 Web 控制台。

| 包 | 作用 |
|---|---|
| [`fa_teleop`](fa_teleop/README.md) | XR 头显桥接、模块管理（启动、停止、日志）、Web 控制台 |
| [`arms_teleop`](arms_teleop/README.md) | 手柄、键盘遥操作 |

输入转成目标位姿的部分在 [`robot_hmi/teleop_command`](../robot_hmi/teleop_command/README.md)。使用说明见 [`docs/VR遥操作.md`](../../docs/VR遥操作.md)。
