# robot_comm_core

通信接口，header-only，不依赖 ROS。

| 头文件 | 内容 |
|---|---|
| `robot_comm/can.h` | `CanConfig`（接口名、速率、是否由驱动配接口）和 `parseCanConfig`（从硬件参数读，非法值带参数名报错）；`CanFrame`（和 SocketCAN 无关，标志位是独立字段）；抽象类 `CanBus`（`open`、`close`、`send`、`receive`、`waitReadable`，收发都不阻塞，实时周期里可以用）和 `CanInterfaceConfigurator`；`CanStats` |
| `robot_comm/interface_name.h` | `isValidInterfaceName`：接口名要进 `sudo ip link …` 的 shell 命令和 sudoers 规则，只放行 1 到 15 个 `[A-Za-z0-9_.-]`，不能以 `-` 开头 |

实现见 [`robot_comm_can`](../robot_comm_can/README.md)。
