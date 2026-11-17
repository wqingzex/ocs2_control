# robot_teleop

操作员的输入设备，把输入转成机器人的控制指令，由 [`target_generator`](../robot_control/target_generator/README.md) 接收。名字里不带机器人型号。

| 包 | 输入 | 内容 |
|---|---|---|
| [`xr_teleop`](xr_teleop/README.md) | Pico 头显和手柄（XRoboToolkit） | 桥节点 `xr_bridge_node`（发布 `/teleop/*`）、网卡枚举、无头显联调 `xr_stub.py` |
| [`joystick_teleop`](joystick_teleop/README.md) | 游戏手柄 | 手柄节点、按键映射 |
| [`keyboard_teleop`](keyboard_teleop/README.md) | 键盘 | 键盘节点 |
| [`dexcap_teleop`](dexcap_teleop/README.md) | DexCap V4 外骨骼和手套 | 设备节点（`/dexcap/*`）、臂关节映射 `arm_joint_mapper`（直接发 `/arm_controller/joint_target_position`，不经 `target_generator`）、手套到 L6 / 夹爪的映射 |

这里只放设备本身（节点和协议）；怎么启动在 [`robot_launch`](../robot_launch/README.md)（`robot_startup/launch/` 的 `joystick.launch.py`、`dexcap.launch.py`，单元定义在 `launch_helpers/devices/`），头显配对在 [`robot_setup/xr_setup`](../robot_setup/xr_setup/README.md)。

同一时刻只运行一种输入，它们发布同样的话题。头显配对（`./scripts/setup.sh xr`）在 `xr_teleop`：它用 [`robot_utils`](../robot_utils/README.md) 里的进程管理 启动桥和 PC Service，不依赖 [`robot_app`](../robot_hmi/robot_app/README.md)。
