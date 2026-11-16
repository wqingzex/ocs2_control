# can_hardware

接在 CAN 总线上的末端执行器驱动，和 [`modbus_hardware`](../modbus_hardware/README.md)（接在 RS485 / Modbus 上的末端）对称。通信用 [`comm/robot_comm_can`](../comm/README.md)（`SocketCanBus`），每个设备的帧协议是本包里的纯函数，可以不接硬件做单元测试。

## 插件

| 插件 | 产品 | 备注 |
|---|---|---|
| `LinkerHandCanHardware` | LinkerHand L6 | 经典 CAN 1 Mbit/s；`hand_type:=linkerhand_l6`；左手 `0x28`、右手 `0x27`；一只手一个 system |

协议和型号表在 `include/can_hardware/hands/linkerhand/linkerhand_can_protocol.h`。O6 / O7 / L10 用同一套协议，加一行型号配置和对应的描述包即可接入；XHand1 走 CAN 需要厂家的 CAN 协议，目前仍走 RS485（`modbus_hardware/XHand1RS485Hardware`）。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `hand_type` | （必填） | 选型号，目前只有 `linkerhand_l6` |
| `hand_side` | `right` | `left` / `right`，决定 CAN ID |
| `can_id` | 按 `hand_side` | 覆盖 CAN ID |
| `can_interface` `can_bitrate` `can_restart_ms` `can_manage_interface` | `can0`、`1000000`、`100`、`true` | 所有 CAN 驱动共用的参数（`robot_comm::parseCanConfig`）；`can_manage_interface:=true` 时驱动执行 `sudo ip link set ...`，免密规则由 `./scripts/setup.sh host` 写入 |
| `read_feedback` | `true` | 50 Hz 读位置；第一次读到位置前不下发指令，避免启动时手跳到 URDF 初值 |
| `read_tactile` | `false` | 读触觉矩阵，发布到 `/<型号>_hand/<side>/tactile/<finger>`（`UInt8MultiArray`） |
| `tactile_period_ms` `tactile_timeout_ms` `send_initial_command` | `20`、`100`、`false` | |

节点参数 `<side>_tool_torque`、`<side>_tool_velocity`（0~1）设置关节力矩和速度上限，运行时可改，每次改动发两遍限幅帧。

总线读写在单独的线程里，`read()` / `write()` 只交换快照。每个控制周期都重发最新目标：发送成功不代表手收到了，跳过不变的指令可能让手停在丢掉的那一帧上。

## 使用

通过 xacro 宏接线：`linkerhand_description` 的 `linkerhand_usb_can_side_system`，由 `m6_ccs_description` 的 `external_ee_systems` 在 `ee_drive_type:=usb_can` 时调用。接口名在 `robot.local.yaml` 的 `hardware.left_ee_can_interface` / `right_ee_can_interface`。

## 测试

```bash
colcon test --packages-select can_hardware
```

`test_linkerhand_can_protocol`：帧的构造和解析、单位换算；`test_linkerhand_can_hardware`：驱动跑在 `MockCanBus` 上，覆盖参数检查、反馈、指令、限幅、总线错误。
