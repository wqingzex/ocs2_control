# robot_comm_can

CAN / CAN FD 的实现，接口见 [`robot_comm_core`](../robot_comm_core/README.md)。

| 类 | 作用 |
|---|---|
| `SocketCanBus` | Linux SocketCAN（`CAN_RAW`）。非阻塞；`CanConfig::fd()` 时打开 CAN FD 帧；有发送和接收的成功、失败计数。`CanFrame::fd` 的帧按 `canfd_frame` 写，其余按 `can_frame` 写 |
| `MockCanBus` | 内存里的总线，给单元测试用：`pushReceived` 排队要收的帧，`sentFrames` 读发出去的帧，`setSendHook` 让测试在发送的同一线程里立刻回应（和设备模型组成闭环），`setOpenFails` / `setSendFails` / `setReceiveFails` 强制失败 |
| `SudoIpConfigurator` | 用 `sudo ip link set …` 配接口，命令和驱动一直以来执行的一字不差；`buildCommand` 是纯函数，便于测试和打印 |
| `NoopConfigurator` | 什么也不做，接口已经由别处配好时用（`can_manage_interface: false`） |
| `makeConfigurator(config)` | 按 `config.manage_interface` 在上面两个里选一个 |

**命令和 sudoers 规则保持同步**：`scripts/setup.sh host` 写的 sudoers 规则里有一段 `up type can bitrate 1000000 dbitrate 5000000 restart-ms 100 fd on`，驱动执行的命令必须和它一字不差。测试 `sudoers_rule_matches_the_driver_command` 会检查 `setup.sh` 里确实有这一段，测试 `TheJuxieCommandIsExactlyWhatTheDriverAlwaysRan` 检查驱动生成的命令。改其中一边会让测试失败。

## 在真机上测速度和稳定性

`SocketCanBus` 在容器里和开发机上没法测真实收发（没有 CAN 接口）。真机上用驱动自带的统计，见 [`juxie_hardware` 的“稳定性统计”](../../juxie_hardware/README.md#稳定性统计)。
