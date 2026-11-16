# end_effector_hardware

外接末端执行器的 ros2_control 驱动：夹爪、灵巧手、六轴力传感器，按设备分目录。原来的 `modbus_hardware`（RS485 / Modbus 上的末端）和 `can_hardware`（CAN 上的末端）合并而来。通信只用 [`comm/`](../comm/README.md)（`robot_comm_serial`、`robot_comm_modbus`、`robot_comm_can`），每个设备的协议在本包；夹爪的寄存器表和单位换算在 [`gripper_hardware_common`](../gripper_hardware_common/README.md)（`marvin_hardware` 也用它）。

接在臂体 485 口上的末端走 [`marvin_hardware`](../marvin_hardware/README.md)；Wuji Hand 2（以太网、厂家 SDK）走 [`wuji_hardware`](../wuji_hardware/README.md)。

## 插件

| 插件 | 设备 | 接线 | 备注 |
|---|---|---|---|
| `GripperModbusHardware` | ChangingTek AG2F90、Jodell | USB-RS485（Modbus） | `gripper_type:=changingtek` / `jodell`，`variant:=90c` / `90d` |
| `LinkerHandRs485Hardware` | LinkerHand O6 / O7 / L6 | USB-RS485（Modbus） | 按关节数和 `hand_type` 区分；左手 `0x28`、右手 `0x27` |
| `LinkerHandCanHardware` | LinkerHand L6 | USB-CAN（经典 CAN 1 Mbit/s） | `hand_type:=linkerhand_l6`；左手 `0x28`、右手 `0x27`；一只手一个 system |
| `XHand1Rs485Hardware` | XHand1 | USB-RS485（专用协议，3 Mbps） | 12 关节 |
| `InspireRs485Hardware` | Inspire RH56（E2 / F2） | USB-RS485（Modbus） | 6 关节 |
| `FreedomRs485Hardware` | Freedom V1 / V2 | USB-RS485（专用协议） | `protocol_version:=auto` / `freedomv1` / `freedomv2` |
| `TheoHandRs485Hardware` | TheoHand STD16A | USB-RS485（Modbus） | 16 关节，115200 8N1 |
| `Kwr75ForceTorqueSensor` | KWR75 六轴力传感器 | USB-RS485 | `type="sensor"`，默认 2.5 Mbps；发布 `/left_arm_external_wrench`、`/right_arm_external_wrench` |

本仓库的描述包只带 LinkerHand L6、XHand1、Wuji Hand 2，其余型号需要对应的 `*_description` 包。

## 目录

```text
include/end_effector_hardware/   src/ 同样分目录
├─ common/serial_notices.h       串口打开时的提示（低延迟设置等）转成日志
├─ grippers/                     GripperModbusHardware 和各品牌夹爪
├─ hands/
│  ├─ dexterous_hand_base.h, dexterous_hand_wrapper_template.h
│  ├─ linkerhand/                RS485 与 CAN 两种驱动；CAN 帧协议在 linkerhand_can_protocol.h（纯函数）
│  ├─ xhand1/  inspire/  freedom/  theo/
└─ sensors/kwr75/
```

## 使用

通过 xacro 宏接线，不要手写整段关节：`m6_ccs_description` 的 `external_ee_systems` 按 `ee_drive_type` 选 system（`usb`：RS485，`usb_can`：CAN）。在 `robot.local.yaml` 里配：

```yaml
robot:
  hand: linkerhand_l6
  hand_drive: usb            # usb（USB-RS485）| usb_can（USB-CAN）
hardware:
  usb_left_port: /dev/ttyUSB0          # usb 时
  usb_right_port: /dev/ttyUSB2
  # left_ee_can_interface: can1        # usb_can 时；不能用腰、头的 body_can_interface
  # right_ee_can_interface: can2
```

单手调试：

```bash
ros2 launch robot_bringup hand.launch.py hand:=linkerhand type:=l6 hardware:=real direction:=1
```

用户要在 `dialout` 组里才能读写串口。

## 常用硬件参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `serial_port` | `/dev/ttyUSB0` | 串口（RS485 插件） |
| `baudrate` | 各插件不同（多为 115200） | 波特率 |
| `hand_side` | | `left` / `right` |
| `slave_id` | 各插件不同 | 从站号；Inspire 为 `auto`（左 2、右 1） |
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
| ChangingTek 夹爪 | 0（闭）~ 1（开） | 0–9000 |
| LinkerHand | 弧度（0 ~ 关节上限） | 0–255（255 为张开） |
| Inspire、Freedom、XHand1 | 弧度 | 各协议自定义 |

## 测试

```bash
colcon test --packages-select end_effector_hardware
```

`test_jodell_gripper`；`test_linkerhand_can_protocol`（帧的构造和解析、单位换算）；`test_linkerhand_can_hardware`（驱动跑在 `MockCanBus` 上：参数检查、反馈、指令、限幅、总线错误）。

## 编译与扩展

依赖 `libmodbus-dev`、`gripper_hardware_common`、`robot_comm_*`。新增夹爪：继承 `ModbusGripperBase` 实现协议，在 `GripperModbusHardware::on_init()` 按 `gripper_type` 实例化，并在 `end_effector_hardware.xml` 注册。新增一种 CAN 手：协议写成纯函数放在 `hands/<型号>/`，驱动持有 `robot_comm::CanBus`（测试里注入 `MockCanBus`），参照 `linkerhand`。

已知待办：Inspire 协议在本包和 Marvin 里各有一份实现；`InspireRs485Hardware` 的关节上限写死为 RH56E2。
