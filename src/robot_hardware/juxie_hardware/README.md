# juxie_hardware

JX 电机的 `ros2_control` 驱动，CAN FD 通信，循环同步位置（CSP）模式。插件 `juxie_hardware/JxHardware`。

结构：`ros2_control` 这一侧在 `jx_hardware.cpp`；CAN 链路是注入的 `robot_comm::CanBus`（真机是 `SocketCanBus`，见 [`robot_comm_can`](../comm/robot_comm_can/README.md)）；帧的构造、反馈解析、限速限加速、错误码表是纯函数，在 `juxie_protocol.{h,cpp}`，不依赖 ROS 和 socket，单元测试直接跑。

## 作用

fa_w2 的腰（`body_joint1~4`）和头（`head_joint1~2`）用它，URDF 里是独立的 `juxie_body` system。最多 8 个电机，1 Mbps 仲裁段、5 Mbps 数据段。

## 使用

由机器人 xacro 配置，fa_w2 见 `fa_w2_description/xacro/ros2_control/robot.xacro`。参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `can_interface` | `can0` | CAN 接口名（1 到 15 个字母、数字、`_`、`.`、`-`，不能以 `-` 开头；它会进 shell 命令和 sudoers 规则，所以做了校验） |
| `can_bitrate`、`can_data_bitrate` | `1000000`、`5000000` | 仲裁段和数据段速率。Juxie 需要 CAN FD，数据段速率必须大于 0。**改成别的值需要同步改 sudoers 规则**（见下） |
| `can_restart_ms` | `100` | bus-off 后自动恢复，0 关闭 |
| `can_manage_interface` | `true` | `true`：驱动自己执行 `sudo ip link set …`；`false`：接口已经配好（systemd-networkd 等），驱动不动它 |
| `stats_log_period_s` | `0` | 大于 0 时每隔这么多秒打印一次控制线程和 CAN 的统计，0 只在驱动停止时打印一次（见“稳定性统计”） |
| `motor_ids` | 必填 | 电机 ID，如 `1,2,3`，顺序必须与 URDF 关节顺序一致 |
| `control_period_ms` | `1` | 控制周期 |
| `max_position_step_nct` | `25` | 每周期最大位置增量，防跃迁 |
| `max_position_accel_nct` | `5` | 相邻两帧速度增量的最大变化，加减速兜底 |

接口：命令 `position`（rad）；状态 `position`（rad）、`velocity`（rad/s）。电机角度范围 ±π。

## 稳定性统计

驱动会数自己的运行情况，用来在真机上判断速度和稳定性。在 fa_w2 上把 `config/robot.local.yaml` 的 `hardware:` 下 `body_can_stats_period_s` 设成比如 `10`，每 10 秒打印一行（也可以直接在驱动参数里写 `stats_log_period_s`）：

```
Control loop stats (10.0 s): 5000 cycles, wake-up lateness mean 52.1 us / max 180.4 us, 0 late (> half a period),
0 failed sends; 5000 reads, feedback missing in 0.000 % of motor-reads; CAN tx 5001 (errors 0), rx 30000 (errors 0)
```

| 项 | 含义 | 正常 |
|---|---|---|
| wake-up lateness | 控制线程醒来比预定时刻晚多少，反映调度抖动 | 均值几十 µs；最大值远小于控制周期 |
| late | 晚了超过半个周期的次数 | 0 |
| failed sends | 多控帧没发出去（发送队列满、接口掉线） | 0 |
| feedback missing | 一次 `read()` 里没收到反馈的电机占比，反映总线丢帧或电机掉线 | 接近 0 % |
| CAN tx / rx errors | `SocketCanBus` 的发送、接收失败计数 | 0 |

驱动停止时再打印一次总计（`Control thread stopped, totals`）。电机报故障时会发布 `/Base_HardwareError`。

## CAN 接口权限

驱动要执行 `ip link set <接口> up type can bitrate 1000000 dbitrate 5000000 restart-ms 100 fd on`（接口默认 `can0`，由 `can_interface` 决定）。免密码的做法是运行 `./scripts/setup.sh host`，它会写入对应的 sudoers 规则，接口名取自环境变量 `CAN_IFACE`，没有的话取 `config/robot.local.yaml` 的 `robot.body.interface.can`，再没有就是 `can0`；手动配置时命令必须与驱动执行的完全一致，不能用通配符。如果接口已经由别处配好，设 `can_manage_interface: false`，驱动就不再执行这条命令。`robot_comm_can` 的测试会检查驱动的命令和 `setup.sh` 的规则是同一段文字。

## 排查

| 现象 | 检查 |
|---|---|
| CAN 初始化失败 | 接口名是否正确、适配器是否支持 CAN FD |
| 电机无响应 | 电机 ID、CAN 总线连接 |
| 权限错误 | sudoers 规则（见上） |
