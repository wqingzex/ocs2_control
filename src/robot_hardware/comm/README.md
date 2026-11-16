# comm

通信层：只有通信，没有设备。设备驱动（上一层的各个驱动包）不直接碰 socket 和串口，而是拿这里的接口，设备协议和通信分开，协议能脱离硬件做单元测试。一个设备用几种通信都行（比如机械臂本体 CAN、工具口 485），各取所需。

| 包 | 内容 |
|---|---|
| [`robot_comm_core`](robot_comm_core/README.md) | 接口和共用的小工具，不依赖 ROS：`CanBus`、`CanInterfaceConfigurator`、`CanConfig`（所有 CAN 驱动共用同一组参数）、`SerialPort`、`ModbusMaster`、`writeAll`、接口名校验 |
| [`robot_comm_can`](robot_comm_can/README.md) | CAN 的实现：Linux SocketCAN 的 `SocketCanBus`、测试用的 `MockCanBus`、两种接口配置器 |
| `robot_comm_serial` | 串口的实现：POSIX 串口 `PosixSerialPort`（termios、非阻塞）、USB 转串口低延迟设置 `configureUsbLatency`、测试用的 `MockSerialPort` |
| `robot_comm_modbus` | Modbus 的实现：libmodbus 的 `LibmodbusRtuMaster`（从原来的 `ModbusRtuCommunicator` 搬来）、测试用的 `MockModbusMaster` |

Marvin 臂体口（天机 SDK 的异步 485）没有接到 `ModbusMaster`，原因见 [`docs/待办.md`](../../../docs/待办.md)。

## 新加一个 CAN 设备

末端执行器（灵巧手、夹爪）放进 [`end_effector_hardware`](../end_effector_hardware/README.md) 的 `hands/<型号>/` 或 `grippers/`；关节模组等整机设备各自一个包（如 `juxie_hardware`）。

1. 参数用同一组名字，设备前缀加在前面：`<设备>_can_interface`、`<设备>_can_bitrate`、`<设备>_can_data_bitrate`、`<设备>_can_restart_ms`、`<设备>_can_manage_interface`，用 `robot_comm::parseCanConfig(info.hardware_parameters, "<设备>_", 默认值, config, error)` 读，非法值会带着参数名报错。
2. 驱动持有 `std::unique_ptr<robot_comm::CanBus>`，`open(config)` 之前用 `robot_comm::makeConfigurator(config)` 配接口；测试里用 `MockCanBus` 注入。
3. 帧的构造和解析写成纯函数（参考 `juxie_hardware/juxie_protocol.h`），单元测试里直接喂 `CanFrame`。
4. 在 [`robot_hardware/README.md`](../README.md) 的表里写上它用到的通信，并在 `package.xml` 里依赖 `robot_comm_can`。

## 新加一个串口设备

驱动持有 `std::unique_ptr<robot_comm::SerialPort>`（默认 `PosixSerialPort`），用 `SerialConfig` 打开；`open()` 之后用 `takeNotices()` 取出低延迟设置等提示再写日志（参考 `end_effector_hardware/communicator/serial_notices.h`）；测试里注入 `MockSerialPort`，或者用 pty 测真实的 `PosixSerialPort`。在 `package.xml` 里依赖 `robot_comm_serial`。
