# robot_log

机器人软件的两种日志模式，只有头文件，只依赖 rclcpp。

| 模式 | 含义 |
|---|---|
| `normal` | INFO 及以上，长期开着（默认） |
| `debug` | 我们自己的组件降到 DEBUG，ROS 内部吵闹的记录器仍保持 INFO |

**模式在一次运行里是固定的，运行中不切换**：写在配置的 `log.mode`，supervisor 把它通过环境变量 `ROBOT_LOG_MODE` 交给它启动的每个进程，各组件初始化时读一次。要换模式，改配置重启。直接用 `ros2 launch` 起、没有这个环境变量的进程，是 normal。

## 在组件里用

```cpp
#include <robot_log/log_mode.hpp>

robot_log::applyLogMode();   // 初始化时调用一次；返回这次运行是不是 debug
```

debug 时：先把 `noisyRosLoggers()` 里那几个 ROS 内部记录器固定在 INFO（每个进程只做一次），再把进程的**默认日志级别**设成 DEBUG。我们自己的节点记录器和库里用 `rclcpp::get_logger("名字")` 建的记录器没有显式条目，自动跟着默认级别走，所以不用维护名单。normal 时什么都不动，`--ros-args --log-level` 给的级别照常有效。

## 为什么是这个做法

- **不能只设默认级别**：仿真里实测，默认级别设成 DEBUG 后，第一分钟 `controller_manager`（9.4 万行）和 `rcl`（4.9 万行，每收一条消息一行）就写了 14 万行、20 MB，我们自己的软件才几十行。所以吵闹的 ROS 内部记录器要先固定在 INFO。
- **不能逐个设我们自己的记录器**：`rcutils_logging_set_logger_level` 把级别放进一张表，官方标注“Thread-Safe: No”。控制器是在 `ros2_control_node` 已经运行、别的线程在打日志时才加载的，这时往表里新增条目可能触发扩容，而别的线程正在读它。现在写这张表的只有“固定吵闹记录器”那一批，在进程第一次调用时做一次；其余只改默认级别这一个整数。

`noisyRosLoggers()` 是靠实测得出的，如果以后发现别的 ROS 内部记录器也在刷屏，加到那个列表里。

## 飞行记录器（`robot_log/flight.hpp`）

每个模块在 `/dev/shm/robot_flight/<模块>.ring` 里有一个环形缓冲，记最近一两分钟**它自己收到和发出的东西**（时间戳和数值）。写一条记录约 40 ns，不加锁、不分配，实时线程里也能用。

缓冲是共享内存里的文件，**进程崩溃后文件还在**（重启或断电才会丢）；模块重新启动时，旧文件改名为 `<模块>.ring.prev`。出事时由监视器把整个目录复制到 `logs/incidents/<事故号>/`，不需要模块自己配合，哪怕模块已经崩了也能复制。文件格式写在 `flight.hpp` 头部注释里，解码在 `python/robot_log_tools/flight.py`（`python3 -m robot_log_tools.flight list/csv/snapshot`）。

```cpp
auto flight = robot_log::FlightRecorder::create("arm_controller");   // 失败返回 nullptr，模块照常运行
if (flight) flight->record(kCtlTargetIn, positions.data(), positions.size(), /*aux=*/joint_count);
```

一条记录最多 28 个 double；超过的被截断。

## 出问题后怎么收集和分析（`./scripts/start.sh diag`）

**没有常驻进程**。环在共享内存里，进程崩了文件还在；日志在磁盘上；出错后人工执行一条命令：

```
./scripts/start.sh diag [--minutes N] [原因]     # 默认汇总出错前 10 分钟
```

它做的事（`python/robot_log_tools/evidence.py`，由 `fa_teleop_runtime/diag.py` 打包）：
1. 取各单元日志里最近 N 分钟的行（日志每行带时间；没有时间的，如 PC Service，取最后 300 行）。
2. 统计这段时间里出现的**故障码**（停机类在前，每个码按单元给次数、首次和最后一次时间）。
3. 统计这段时间里的 **`[HEALTH]` 行**（每个组件：周期是否迟到、速率范围、最长间隔，有 WARN 的窗口会标出）。
4. 拷贝各模块的**飞行记录环**，按墙钟合并分析（`analyze.py`）：哪个模块停止写入、数据流里的空洞、控制周期迟到、关节命令跳变、各跳延迟，最早的一条就是该先看的地方。
5. 加上事件时间线、运行状态、当前配置、机器状态（时钟、内存、磁盘、网络计数、内核消息）。

结果是 `logs/diag/diag_<时间>.tar.gz`，旁边有同样内容的 `diag_<时间>.report.txt`，报告也会直接打印。控制台的"导出诊断包"按钮做同一件事。

**日志保存**：每个单元只保留最新的 20 份运行日志（`events.log` 超过 5 MB 滚动一次）。按总大小设上限、自动清理的方案先不做（记在 `docs/待办.md`）。

## 离线工具（`python/robot_log_tools`）

不依赖 ROS，只读文件：`fault_codes`（故障码表的加载、检查、生成 `docs/故障码.md`）、`flight`（解码环、导出 CSV、拷贝）、`analyze`（合并分析环，也可单独用：`python3 -m robot_log_tools.analyze <环目录>`）、`evidence`（汇总日志窗口、故障码、健康和分析）。两份表在 `config/`（`fault_codes.yaml`、`flight_types.yaml`）。源码目录里直接 `python3 -m pytest src/robot_log/test` 就能测（`test/conftest.py` 加了路径）；装好后 `import robot_log_tools`。

## Python 里怎么记日志（`robot_log_tools.log`）

Python 单元用它，格式和 C++ 一致：`[WARN] [2026-11-13 10:00:00.123] [名字]: [XR-006] 消息`。

```python
from robot_log_tools import log
logger = log.get_logger("my_unit")           # 同名只建一次；ROBOT_LOG_MODE=debug 才显示 DEBUG
logger.info("watching %s", path)
log.fault(logger, "XR-006", "frames stopped %d ms", 300)   # 带故障码；码不在表里也照常写，只加一句提示
```

Python 的健康统计：`log.RateStats()` 在每次事件时 `record()`（任意线程），`log.HealthReporter(logger, "名字", stats).start()` 每个窗口（normal 10 秒、debug 1 秒）写一行 `[HEALTH] 名字 window=10.0s count=.. rate_hz=.. gap_max_ms=..`，格式和 C++ 的 `health.hpp` 完全一样（测试里对着同一串字面量）；最长间隔超过 `gap_warn_ms` 时升为 WARN 并带故障码。链路上需要健康行的几跳（控制周期、XR、teleop_command、硬件读写）都是 C++，所以现在**还没有 Python 单元在用它**。命令行工具直接给人看的输出仍然用 `print`，不是日志。
