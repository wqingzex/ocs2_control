# robot_setup

**配置**：生成和改写 `config/robot.local.yaml`，确认设备（相机、末端、头显），由 `./scripts/setup.sh` 调用。一个设备一个 ROS 包，加上它们共用的配置框架。

| 包 | 作用 |
|---|---|
| [`robot_config`](robot_config/README.md) | 机器人配置读取与校验、配置改写和确认记录、设置窗口框架、负载辨识 |
| [`camera_setup`](camera_setup/README.md) | Orbbec 相机：扫描、相机设置窗口、预览 |
| [`end_effector_setup`](end_effector_setup/README.md) | 末端（灵巧手、夹爪）：串口探测、开合测试、末端设置窗口 |
| [`xr_setup`](xr_setup/README.md) | XR 头显配对：选网卡、等头显连上、保存头显 ID |
| [`dyn_setup`](dyn_setup/README.md) | 天机臂工具负载辨识向导（`ros2 run dyn_setup tool_dyn_identify_wizard`） |

依赖方向：各 `*_setup` 包依赖 `robot_config`，`robot_config` 不引用它们；`camera_setup`、`xr_setup` 还依赖 `launch_helpers`（预览和配对要临时启动设备），反过来不依赖。

每个 `*_setup` 包的模块命名一样：`discovery.py` 找设备，`session.py` 是配置步骤（也是 `python3 -m` 的入口）。

目录 `robot_setup/` 本身不是包：colcon 找到一个包之后不会再往它里面找，所以这些包并列放在这里。

启动在 [`robot_launch`](../robot_launch/README.md)，输入设备本身在 [`robot_teleop`](../robot_teleop/README.md)。
