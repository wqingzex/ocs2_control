# end_effector_hardware

外接末端执行器的 ros2_control 驱动：夹爪、灵巧手、六轴力传感器，按设备分目录。原来的 `modbus_hardware`（RS485 / Modbus 上的末端）和 `can_hardware`（CAN 上的末端）合并而来。通信只用 [`comm/`](../comm/README.md)（`robot_comm_serial`、`robot_comm_modbus`、`robot_comm_can`），每个设备的协议在本包；夹爪和灵巧手的协议层（寄存器表、单位换算、指令组装、`GripperBase`）在 `include/end_effector_hardware/gripper_protocol/`，命名空间 `gripper_protocol`，导出成纯头文件目标 `end_effector_hardware::gripper_protocol`：[`marvin_hardware`](../marvin_hardware/README.md) 经臂体 RS485 透传驱动同样的设备，只链接这个目标，不链接插件库。夹爪只支持 Jodell RG75。

接在臂体 485 口上的末端走 [`marvin_hardware`](../marvin_hardware/README.md)；Wuji Hand 2（以太网、厂家 SDK）走 [`wuji_hardware`](../wuji_hardware/README.md)。

## 插件

| 插件 | 设备 | 接线 | 备注 |
|---|---|---|---|
| `GripperModbusHardware` | Jodell RG75 | USB-RS485（Modbus） | `gripper_type:=jodell`（默认，也是唯一支持的类型） |
| `LinkerHandRs485Hardware` | LinkerHand L6 | USB-RS485（Modbus） | `hand_type:=linkerhand_l6`，6 关节；左手 `0x28`、右手 `0x27` |
| `LinkerHandCanHardware` | LinkerHand L6 | USB-CAN（经典 CAN 1 Mbit/s） | `hand_type:=linkerhand_l6`；左手 `0x28`、右手 `0x27`；一只手一个 system |
| `XHand1Rs485Hardware` | XHand1 | USB-RS485（专用协议，3 Mbps） | 12 关节 |
| `Kwr75ForceTorqueSensor` | KWR75 六轴力传感器 | USB-RS485 | `type="sensor"`，默认 2.5 Mbps；发布 `/left_arm_external_wrench`、`/right_arm_external_wrench` |
| `Kwr75Sensor`（新，未上真机） | KWR75 六轴力传感器 | 任一 `robot_comm` bus：`usb_rs485://<设备>?baud=`、`arm_rs485://<臂 IP>/<left\|right>/port2` | `type="sensor"`；每次 `read()` 取上一帧回复、发下一次查询（0x48），不等待；`offline_after_ms`（默认 100）内没有好帧就输出 0。xacro 用 `left_ft_bus` / `right_ft_bus` 接，`left_ft_broadcaster` / `right_ft_broadcaster` 发布 `/<side>_ft_broadcaster/wrench` |
| `EndEffectorHardware`（新，未上真机） | Jodell RG75、LinkerHand L6、XHand1 | 任一 `robot_comm` bus：`usb_rs485://<设备>?baud=`、`arm_rs485://<臂 IP>/<left\|right>/port1` | `device`、`side`、`bus`；`io_mode` async（默认）/ sync；`torque` / `velocity` 也可运行时改。xacro 里 `end_effector_driver: bus` 时启用。在控制循环里异步初始化，接在臂上时可以先于臂激活，臂连上后才开始。`offline_after_ms` 内没有好回复就报离线（HW-004，默认 RG75 / L6 2000 ms、XHand1 200 ms）；RG75 离线后重新激活，应答后保持当时的位置，L6 恢复后继续写位置。XHand1（`device=xhand1`，`usb_rs485://...?baud=3000000`）：先发 0 力矩帧直到手回复，再保持当前位置；每次交换发指令并读回状态，力矩上限 = `torque_limit` × `torque`（xacro 里 0.8）；200 ms 没回复就退回 0 力矩帧，恢复后保持手当时的位置；帧格式和旧 `XHand1Rs485Hardware` 逐字节一致（`test_xhand1_parity`）；只导出 position 命令接口 |

本仓库的描述包只带 LinkerHand L6、XHand1、Wuji Hand 2，其余型号需要对应的 `*_description` 包。

## 目录

```text
include/end_effector_hardware/   src/ 同样分目录
├─ common/serial_notices.h       串口打开时的提示（低延迟设置等）转成日志
├─ grippers/                     GripperModbusHardware 和各品牌夹爪
├─ hands/
│  ├─ dexterous_hand_base.h, dexterous_hand_wrapper_template.h
│  ├─ linkerhand/                RS485 与 CAN 两种驱动；CAN 帧协议在 linkerhand_can_protocol.h（纯函数）
│  ├─ xhand1/
└─ sensors/kwr75/
```

## 使用

通过 xacro 宏接线，不要手写整段关节：`m6_ccs_description` 的 `external_ee_systems` 按 `left/right_end_effector_bus` 选 system（`usb_rs485`：RS485，`usb_can`：CAN）。在 `robot.local.yaml` 里配：

```yaml
robot:
  end_effector: linkerhand_l6
  end_effector_bus: usb_rs485   # usb_rs485（USB-RS485）| usb_can（USB-CAN）
hardware:
  usb_left_port: /dev/ttyUSB0          # usb 时
  usb_right_port: /dev/ttyUSB2
  # left_ee_can_interface: can1        # usb_can 时；不能用腰、头的 body_can_interface
  # right_ee_can_interface: can2
```

单手调试：

```bash
ros2 launch robot_startup hand.launch.py hand:=linkerhand type:=l6 hardware:=real direction:=1
```

用户要在 `dialout` 组里才能读写串口。

## 常用硬件参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `serial_port` | `/dev/ttyUSB0` | 串口（RS485 插件） |
| `baudrate` | 各插件不同（多为 115200） | 波特率 |
| `hand_side` | | `left` / `right` |
| `slave_id` | 各插件不同 | 从站号 |
| `left_tool_torque` `left_tool_velocity`（右同） | `1.0` | LinkerHand 力矩、速度比例 `[0, 1]`，运行时可改 |
| `read_feedback` | `true` | 读回实际位置；关闭则显示命令位置 |
| `response_timeout_ms` `byte_timeout_ms` | `20` `5` | Modbus 超时 |
| `can_interface` `can_bitrate` `can_restart_ms` `can_manage_interface` | `can0`、`1000000`、`100`、`true` | CAN 插件；所有 CAN 驱动共用的参数（`robot_comm::parseCanConfig`）。`can_manage_interface:=true` 时驱动执行 `sudo ip link set ...`，免密规则由 `./scripts/setup.sh host` 写入 |
| `read_tactile` `tactile_period_ms` `tactile_timeout_ms` | `false`、`20`、`100` | LinkerHand CAN 触觉，发布到 `/<型号>_hand/<side>/tactile/<finger>` |

各插件的全部参数见源码 `src/`。

LinkerHand CAN 驱动的总线读写在单独的线程里，`read()` / `write()` 只交换快照；第一次读到位置之前不下发指令，避免启动时手跳到 URDF 初值；每个周期都重发最新目标（发送成功不代表手收到了）。

## 位置单位

| 末端 | ros2_control 接口 | 设备 |
|---|---|---|
| Jodell RG75 | 0（闭）~ 1（开） | 0–255 |
| LinkerHand | 弧度（0 ~ 关节上限） | 0–255（255 为张开） |
| XHand1 | 弧度 | 专用协议 |

## 测试

```bash
colcon test --packages-select end_effector_hardware
```

`test_jodell_gripper`；`test_linkerhand_can_protocol`（帧的构造和解析、单位换算）；`test_linkerhand_can_hardware`（驱动跑在 `MockCanBus` 上：参数检查、反馈、指令、限幅、总线错误）；`test_bus_probe`（总线测量跑在 `MockBus` 上：往返时延、限速、设备不应答）。

## 总线测量

`./scripts/start.sh bus_probe` 按 `robot.local.yaml` 里的末端（`linkerhand_l6`、`xhand1`，USB-RS485）让设备自己的状态机在总线上跑，量往返时延、丢帧、请求频率和断档。**要先停运控**（串口只能一个进程用）。

```bash
./scripts/start.sh bus_probe --side left --duration 10          # 不限速：看这条总线能到多快
./scripts/start.sh bus_probe --cycle-hz 100 --io-mode sync      # 按 100 Hz 控制周期，sync 方式
```

设备收到的就是运控启动时的那些帧（保持当前位置），不下发运动指令，开始前要输入 YES（`--yes` 跳过）。XHand1（USB-RS485，3 Mbps）实测约 100 Hz、每次收发约 10 ms，`async` 和 `sync` 一样，两只手同时跑也一样，没有超时和断档（2026-11-18，真机）；手每次回复带 1920 字节触觉数据，传输占了大部分。`--csv 文件` 写出每次往返的毫秒数。程序在 `bus_probe`（`ros2 run end_effector_hardware bus_probe --help`）。臂末端 485 口（`arm_rs485`）的总线只在臂驱动运行时才有，还不支持。

## 编译与扩展

依赖 `libmodbus-dev`、`robot_comm_*`。新增夹爪：继承 `ModbusGripperBase` 实现协议，在 `GripperModbusHardware::on_init()` 按 `gripper_type` 实例化，并在 `end_effector_hardware.xml` 注册。新增一种 CAN 手：协议写成纯函数放在 `hands/<型号>/`，驱动持有 `robot_comm::CanBus`（测试里注入 `MockCanBus`），参照 `linkerhand`。

