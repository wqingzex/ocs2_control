# robot_comm_core

通信接口，不依赖 ROS。CAN 部分 header-only；bus 部分在共享库 `robot_comm_bus` 里（同一进程的所有插件共用一个 `BusHub`）。

| 头文件 | 内容 |
|---|---|
| `robot_comm/can.h` | `CanConfig`（接口名、速率、是否由驱动配接口）和 `parseCanConfig`（从硬件参数读，非法值带参数名报错）；`CanFrame`（和 SocketCAN 无关，标志位是独立字段）；抽象类 `CanBus`（`open`、`close`、`send`、`receive`、`waitReadable`，收发都不阻塞，实时周期里可以用）和 `CanInterfaceConfigurator`；`CanStats` |
| `robot_comm/interface_name.h` | `isValidInterfaceName`：接口名要进 `sudo ip link …` 的 shell 命令和 sudoers 规则，只放行 1 到 15 个 `[A-Za-z0-9_.-]`，不能以 `-` 开头 |

实现见 [`robot_comm_can`](../robot_comm_can/README.md)。

## bus：多个设备共用一条半双工线路

| 头文件 | 内容 |
|---|---|
| `robot_comm/bus.h` | `Bus`：一条线路，同一时间只有一个请求在途；`submit` / `poll` 异步，`transact` 同步。`ReplySpec` 说明怎么认回复（`frame_length`、`max_size`、`timeout`、`inter_byte_gap`，或 `expect_reply = false` 不等回复） |
| `robot_comm/bus_name.h` | bus 名：`usb_rs485://<设备>?baud=…`、`usb_serial://…`、`arm_rs485://<臂 IP>/<left\|right>/<port1\|port2>`、`arm_canfd://<臂 IP>/<side>`；`parseBusName` 校验 |
| `robot_comm/queued_bus.h` | `QueuedBus`：各客户端按提交顺序排队，每个客户端最多一个请求，只拿得到自己的完成通知；在泵这条线的线程里调 `transact` 会被拒绝（它永远等不到回复） |
| `robot_comm/bus_hub.h` | `BusHub`：按名字取 bus，同一条线路给同一个 bus；USB 的由工厂打开，最后一个客户端释放时关闭；臂上的由臂驱动 `provide`。`BusClient` 是设备手里的那一份 |
| `robot_comm/mock_bus.h` | `MockBus`：测试用，`setResponder` 模拟设备 |

失败时的行为：

- **bus 不可用**（臂没连上、断开）：排队的和在途的请求都以 `Error` 结束，新请求被拒绝；臂重新连上后恢复。
- **bus 被撤下或替换**（臂驱动重新初始化）：客户端在旧 bus 上的在途请求，下一次 `poll()` 拿到一条 `Error` 完成通知，不会一直等下去。
- **USB 串口读写出错**（拔掉转接头）：bus 变成 `Failed`，IO 线程每秒尝试重开一次（`SerialBus::setReopenInterval`），重开成功就恢复就绪。
