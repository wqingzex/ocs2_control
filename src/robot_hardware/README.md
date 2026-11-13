# robot_hardware

`ros2_control` 硬件驱动：把控制器的关节指令发到真机，把状态读回来。设备驱动包平铺在这一层，不按设备或按通信分子目录；通信层单独放在 `comm/`：一个设备可以同时用几种通信（机械臂本体走 SDK、工具口走 RS485；灵巧手走 Modbus 或串口），按设备或按通信分目录都会有放不下的情况。包名自己说明用途，下表“用到的通信”只是说明，不是配置。

| 包 | 对象 | 用到的通信 | W2 / M6 是否使用 |
|---|---|---|---|
| [`marvin_hardware`](marvin_hardware/README.md) | 天机 Marvin 臂，及臂体上的末端 | `sdk`、`modbus`（臂体 RS485 工具口） | 是：双臂 |
| [`rokae_hardware`](rokae_hardware/README.md) | 珞石 Rokae 臂 | `sdk` | 否（用于 Rokae 机型） |
| [`juxie_hardware`](juxie_hardware/README.md) | JX 关节模组（腰、头） | `can` | 是：腰、头 |
| [`modbus_hardware`](modbus_hardware/README.md) | 外接 USB / RS485 的夹爪、灵巧手、力传感器 | `modbus`、`serial` | 是：外接末端 |
| [`gripper_hardware_common`](gripper_hardware_common/README.md) | 夹爪驱动共用的寄存器表、单位换算、`GripperBase` | （设备层，无通信） | 被上面的驱动使用 |
| [`topic_based_hardware`](topic_based_hardware/README.md) | 仿真：通过话题收发，对接 Isaac 等 | `topic` | 是：Isaac 仿真 |
| [`comm/`](comm/README.md) 下的 `robot_comm_core`、`robot_comm_can`、`robot_comm_serial`、`robot_comm_modbus` | 通信层（见下） | — | 被驱动使用 |

驱动本身不单独启动：`xacro/ros2_control/robot.xacro` 把它们写进 URDF 的 `<ros2_control>`，`ros2_control_node` 加载。哪台机器人用哪个驱动，看对应 `*_description` 包。

## 通信层 `comm/`

只有通信，没有设备：CAN、串口、Modbus 的接口和实现，放在子目录 [`comm/`](comm/README.md)。设备驱动不直接碰 socket 和串口，而是拿这里的接口，设备协议和通信分开，协议能脱离硬件做单元测试。包列表、怎么新加一个 CAN / 串口设备见 [`comm/README.md`](comm/README.md)。
