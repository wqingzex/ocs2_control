# bus_bench

比较一个末端在不同通信设置下的表现：对每一组「总线 URI × `io_mode` × 控制器更新率」起一次驱动，采样 `/joint_states`，报告实际发布频率、间隔抖动、断档和漂移；加 `--move` 还会让一个关节走一个小阶跃，量读回跟上需要多久。

在容器里运行（要先编译，`install/` 里有 `end_effector_hardware`）：

```bash
./docker/run.sh tools/bus_bench/bus_bench.sh --help
```

## 用法

```bash
# 没有硬件：mock 硬件，查控制器本身能不能按请求的频率发布
./docker/run.sh tools/bus_bench/bus_bench.sh --mock --rates 100,200,500,1000

# 真机，手不动：更新率 × io_mode 扫描（总线 URI 用 by-id，ttyUSB 编号插拔后会变）
./docker/run.sh tools/bus_bench/bus_bench.sh --side left \
  --bus "usb_rs485:///dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A9BJTKUI-if00-port0?baud=3000000" \
  --rates 50,100,200,500 --io-modes async,sync

# 比较不同总线设置：重复 --bus（比如 low_latency 开关）
./docker/run.sh tools/bus_bench/bus_bench.sh --side left \
  --bus "usb_rs485://<设备>?baud=3000000" --bus "usb_rs485://<设备>?baud=3000000&low_latency=0"

# 加响应时延：手会动（一个关节走 0.2 rad 的阶跃再回来），连接前会要求输入 YES
./docker/run.sh tools/bus_bench/bus_bench.sh --side left --bus "<URI>" --move
```

结果打印成表，同时写 `build/bus_bench/results.csv`（`--out` 可改），每个配置的驱动日志在同一目录。

| 列 | 含义 |
|---|---|
| `Hz` | `/joint_states` 实际发布频率，和请求的更新率比较 |
| `p50ms` `p99ms` `maxms` | 相邻两条状态的间隔，抖动看 p99 和 max |
| `gaps` | 间隔超过 `--max-gap-ms`（默认 100 ms）的次数 |
| `drift` | 采样期间每个关节的最大减最小（手不动时应接近传感器噪声）|
| `faults` | 驱动日志里 `HW-00x`、`Deactivating`、`[ERROR]` 的行数 |
| `t10ms` `t50ms` `t90ms` `err` | `--move`：读回走完阶跃 10%、50%、90% 所用时间，和结束时离起点的差 |

## 看数字时要注意

- **响应时延包含轨迹控制器自己的 40 ms 阶跃斜坡**，所以比较不同总线或模式时看差值，不要看绝对值。mock 硬件给出的是基线（100 Hz 约 15 / 35 / 45 ms）。
- **`Hz` 低于请求的更新率**，说明驱动或总线跟不上那个速率；mock 能到而真机不能，限制就在驱动或总线。
- 更新率写在控制器的 yaml 里，不是另外传的参数。

## 范围和限制

- 目前只支持 `xhand1`（`rig.xacro` 里的关节和 URDF 来自 `xhand1_description`），换设备要在 `rig.xacro` 和 `bus_bench.py` 的 `DEVICES` 里加一项。总线只支持 `usb_rs485` / `usb_serial`；`arm_rs485` 的总线由臂驱动提供，不在这个装置里。
- 通信限制在本机（UDP、仅 localhost、独立域，见 `bus_bench.sh`），不会发到机器人网络。
- 启动前会检查：已有 `ros2_control_node` 或 `robot_state_publisher` 在跑、设备被别的进程占着，都会拒绝。
- `--move` 之外的运行不给手下发任何运动指令；手会保持当前位置（驱动先发 0 力矩探测帧，手回复后保持位置）。
- 臂末端 485 口的阶段 0 测量是另一个工具：`marvin_hardware` 的 `arm_rs485_bench`（直接用天机 SDK，见 `docs/待办.md`）。
