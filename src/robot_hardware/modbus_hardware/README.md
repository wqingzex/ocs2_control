# modbus_hardware

外接 USB 串口（Modbus / RS485）的末端执行器驱动：夹爪、灵巧手、六轴力传感器。

## 作用

`hardware:=real_usb` 时，W2 / M6 的末端手走这里（URDF 里的 `external_ee_systems`），串口默认左 `/dev/ttyUSB0`、右 `/dev/ttyUSB2`。臂体上的 RS485 末端走 [`marvin_hardware`](../marvin_hardware/README.md)。

## 插件

| 插件 | 产品 | 备注 |
|---|---|---|
| `ModbusHardware` | ChangingTek AG2F90 | `gripper_type:=changingtek`，`variant:=90c` / `90d` |
| `DexterousHandHardware` | LinkerHand O6 / O7 / L6 | 按关节数和 `hand_type` 区分；左手 `0x28`、右手 `0x27`；拒绝 Inspire / Freedom |
| `InspireHandHardware` | Inspire RH56（E2 / F2） | 6 关节，Modbus RTU |
| `FreedomRS485Hardware` | Freedom V1 / V2 | `protocol_version:=auto` / `freedomv1` / `freedomv2` |
| `XHand1RS485Hardware` | XHand1 | 12 关节，专用 RS485，默认 3 Mbps |
| `TheoHandModbusHardware` | TheoHand STD16A | 16 关节，115200 8N1 |
| `Kwr75ForceTorqueSensor` | KWR75 六轴力传感器 | `type="sensor"`，默认 2.5 Mbps；发布 `/left_arm_external_wrench`、`/right_arm_external_wrench` |

本仓库的描述包只带 LinkerHand L6 和 XHand1，其余型号需要对应的 `*_description` 包。

## 使用

通过 xacro 宏接线，不要手写整段关节：`robot.xacro` 里的 `external_ee_systems` / `side_usb_hand_system`（`m6_ccs_description`）。串口在 `robot.local.yaml` 的 `hardware:` 里配：

```yaml
hardware:
  usb_left_port: /dev/ttyUSB0
  usb_right_port: /dev/ttyUSB2
```

单手调试：

```bash
sudo usermod -a -G dialout $USER          # 或 sudo chmod 666 /dev/ttyUSB0
ros2 launch robot_bringup hand.launch.py hand:=linkerhand type:=l6 hardware:=real direction:=1
```

## 常用硬件参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `serial_port` | `/dev/ttyUSB0` | 串口 |
| `baudrate` | 各插件不同（多为 115200） | 波特率 |
| `hand_side` | | `left` / `right` |
| `slave_id` | 各插件不同 | 从站号；Inspire 为 `auto`（左 2、右 1） |
| `left_tool_torque` `left_tool_velocity`（右同） | `1.0` | LinkerHand 力矩、速度比例 `[0, 1]` |
| `read_feedback` | `true` | 读回实际位置；关闭则显示命令位置 |
| `response_timeout_ms` `byte_timeout_ms` | `20` `5` | Modbus 超时 |

各插件的全部参数见源码 `src/`。

## 位置单位

| 末端 | ros2_control 接口 | 设备 |
|---|---|---|
| ChangingTek 夹爪 | 0（闭）~ 1（开） | 0–9000 |
| LinkerHand O6 / O7 | 0 ~ 1 | 0–255 |
| Inspire、Freedom、XHand1 | 弧度 | 各协议自定义 |

## 编译与扩展

依赖 `libmodbus-dev`、`gripper_hardware_common`。新增夹爪：继承 `ModbusGripperBase` 实现协议，在 `ModbusHardware::on_init()` 按 `gripper_type` 实例化，并在 `modbus_hardware.xml` 注册；新增 `SystemInterface` 则另建插件类。

已知待办：Inspire 协议在本包和 Marvin 里各有一份实现；`InspireHandHardware` 的关节上限写死为 RH56E2。
