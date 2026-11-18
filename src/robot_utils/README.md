# robot_utils

运行和观测机器人软件用的基础设施，一个包，所有其他包都可以依赖它，它不依赖任何设备或控制包。**只放这两类东西**：运行（进程管理）和观测（日志、故障码、黑匣子、诊断）；设备和控制代码不放这里。

| 目录 | 内容 | 使用 |
|---|---|---|
| `include/robot_log/` | C++ 头文件：日志模式、健康统计、黑匣子；只有头文件，只依赖 rclcpp | `#include <robot_log/log_mode.hpp>`，CMake 里 `find_package(robot_utils)` |
| `python/robot_runtime/` | 进程管理：单元、监督器、状态文件 | `import robot_runtime` |
| `python/robot_log_tools/` | 日志和诊断：`logger`（Python 日志、健康统计）、`error_code`（故障码表）、`blackbox`（黑匣子解码）、`diagnose`（出事后的分析和证据汇总） | `import robot_log_tools` |
| `config/` | `fault_codes.yaml`、`blackbox_types.yaml` | |

Python 模块名和 C++ 头文件路径没有随包名变化（合并前是 `robot_log` 和 `robot_runtime` 两个包），所以代码里的 `import` 和 `#include` 不用改，包名只是构建单元的名字。两个 Python 包互相独立，`robot_runtime` 不引用 `robot_log_tools`（事件时间线 `EventLog` 和旧日志清理在 `supervisor.py` 里，`test/test_boundaries.py` 检查依赖方向）；只有 `fault_codes.yaml` 里有一条出处指向 `supervisor.py`。

测试：`python3 -m pytest src/robot_utils/test` 测 Python（`test/conftest.py` 加了路径），`colcon test --packages-select robot_utils` 跑全部（含 C++）。

# 运行：进程管理（`python/robot_runtime`）

把一次运行拆成若干**单元**（一个进程组、一份日志），按依赖顺序启动，监控，停止。不认识任何设备，也不认识具体机型。

| 文件 | 内容 |
|---|---|
| `units.py` | `Unit`（命令、依赖、健康话题、就绪日志、分组）、`UnitSet`（启动和停止顺序）、`Driver`（不是单个进程的单元，如启动后自己 fork 并退出的守护进程） |
| `supervisor.py` | `Supervisor`：启动、停止（SIGINT，再 SIGTERM，再 SIGKILL）、状态、日志尾部、异常退出记录；状态文件 `run/<模块>.json` 重启后仍有效 |
| `processes.py` | 状态文件读写、进程是否存活（防 PID 复用）、等日志出现某段文字；`OwnedDrivers`（设置窗口自己拉起并清理的子进程） |
| `events.py` | 每个单元只留最新的 20 份日志 |

**谁提供单元**：设备的启动定义统一放在 `robot_launch`：`launch_helpers/devices/camera.py`（相机）、`launch_helpers/devices/xr.py`（PC Service、XR 桥）；手柄在 `joystick_teleop.units`；`robot_app.build_units` 汇总，运控和 RViz 由 `robot_app` 构造。依赖单向：`robot_launch`、`robot_setup`、设备包和 `robot_app` 依赖本包，本包不依赖它们（所以进程管理不能并进 `robot_app`：它们都要用 `Unit` 和 `Supervisor`，会形成循环依赖）。

# 观测：日志、故障码、黑匣子、诊断

## 日志模式（`include/robot_log/log_mode.hpp`）

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

## 黑匣子（`robot_log/blackbox.hpp`）

每个模块在 `/dev/shm/robot_blackbox/<模块>.ring` 里有一个环形缓冲，记最近一两分钟**它自己收到和发出的东西**（时间戳和数值）。写一条记录约 40 ns，不加锁、不分配，实时线程里也能用。

缓冲是共享内存里的文件，**进程崩溃后文件还在**（重启或断电才会丢）；模块重新启动时，旧文件改名为 `<模块>.ring.prev`。出事时由监视器把整个目录复制到 `logs/incidents/<事故号>/`，不需要模块自己配合，哪怕模块已经崩了也能复制。文件格式写在 `blackbox.hpp` 头部注释里，解码在 `python/robot_log_tools/blackbox.py`（`python3 -m robot_log_tools.blackbox list/csv/snapshot`）。

```cpp
auto blackbox = robot_log::BlackboxRecorder::create("arm_controller");   // 失败返回 nullptr，模块照常运行
if (blackbox) blackbox->record(kCtlTargetIn, positions.data(), positions.size(), /*aux=*/joint_count);
```

一条记录最多 28 个 double；超过的被截断。

## 出问题后怎么收集和分析（`./scripts/start.sh diag`）

**没有常驻进程**。环在共享内存里，进程崩了文件还在；日志在磁盘上；出错后人工执行一条命令：

```
./scripts/start.sh diag [--minutes N] [原因]     # 默认汇总出错前 10 分钟
```

它做的事（`python/robot_log_tools/diagnose.py`，由 `robot_app/diag.py` 打包）：
1. 取各单元日志里最近 N 分钟的行（日志每行带时间；没有时间的，如 PC Service，取最后 300 行）。
2. 统计这段时间里出现的**故障码**（停机类在前，每个码按单元给次数、首次和最后一次时间）。
3. 统计这段时间里的 **`[HEALTH]` 行**（每个组件：周期是否迟到、速率范围、最长间隔，有 WARN 的窗口会标出）。
4. 拷贝各模块的**黑匣子环**，按墙钟合并分析（`diagnose.py` 的 `report`）：哪个模块停止写入、数据流里的空洞、控制周期迟到、关节命令跳变、各跳延迟，最早的一条就是该先看的地方。
5. 加上事件时间线、运行状态、当前配置、机器状态（时钟、内存、磁盘、网络计数、内核消息）。

结果是 `logs/diag/diag_<时间>.tar.gz`，旁边有同样内容的 `diag_<时间>.report.txt`，报告也会直接打印。控制台的"导出诊断包"按钮做同一件事。

**日志保存**：每个单元只保留最新的 20 份运行日志（`events.log` 超过 5 MB 滚动一次）。按总大小设上限、自动清理的方案先不做（记在 `docs/待办.md`）。

## 离线工具（`python/robot_log_tools`）

不依赖 ROS，只读文件：`error_code`（故障码表的加载、检查、生成 `docs/故障码.md`：`python3 -m robot_log_tools.error_code --markdown`）、`blackbox`（解码环、导出 CSV、拷贝：`python3 -m robot_log_tools.blackbox list|csv|snapshot`）、`diagnose`（合并分析环，也可单独用：`python3 -m robot_log_tools.diagnose <环目录>`；汇总日志窗口、故障码、健康和分析）。`logger` 是进程运行时用的日志库，见下一节。两份表在 `config/`（`fault_codes.yaml`、`blackbox_types.yaml`）。源码目录里直接 `python3 -m pytest src/robot_utils/test` 就能测（`test/conftest.py` 加了路径）；装好后 `import robot_log_tools`。找 `config/` 下的表：先看源码树里 `python/` 旁边，再看安装的 share 目录，两处都没有就报错（写明查了哪两处）。

## Python 里怎么记日志（`robot_log_tools.logger`）

Python 单元用它，格式和 C++ 一致：`[WARN] [2026-11-13 10:00:00.123] [名字]: [XR-006] 消息`。

```python
from robot_log_tools import logger
log = logger.get_logger("my_unit")           # 同名只建一次；ROBOT_LOG_MODE=debug 才显示 DEBUG
log.info("watching %s", path)
logger.fault(log, "XR-006", "frames stopped %d ms", 300)   # 带故障码；码不在表里也照常写，只加一句提示
```

Python 的健康统计：`logger.RateStats()` 在每次事件时 `record()`（任意线程），`logger.HealthReporter(log, "名字", stats).start()` 每个窗口（normal 10 秒、debug 1 秒）写一行 `[HEALTH] 名字 window=10.0s count=.. rate_hz=.. gap_max_ms=..`，格式和 C++ 的 `health.hpp` 完全一样（测试里对着同一串字面量）；最长间隔超过 `gap_warn_ms` 时升为 WARN 并带故障码。链路上需要健康行的几跳（控制周期、XR、target_generator、硬件读写）都是 C++，所以现在**还没有 Python 单元在用它**。命令行工具直接给人看的输出仍然用 `print`，不是日志。

## 写日志的规则

- C++ 用 `RCLCPP_*`，不要 `std::cerr`、`std::cout`、`printf`；没有节点用 `rclcpp::get_logger("组件名")`。Python 单元用上面的 `robot_log_tools.logger`。命令行工具给人看的输出可以直接打印。
- 级别：ERROR 动作失败或东西坏了；WARN 劣化迹象还能继续；INFO 状态变化和启动信息，**一次事件一行**；DEBUG 排查细节。不要把正常情况打成 WARN。
- **控制周期（`update`/`read`/`write`/`run`）、MPC 线程、每帧回调里不许直接打**：状态变化时打一次；要持续提示就限频 `RCLCPP_WARN_THROTTLE(logger, clock, 5000, ...)`（没有节点时钟用 `steadyLogClock()`）；不要用 `static bool warned`；实时线程里不做耗时格式化或 I/O。
- **故障码**：[`docs/故障码.md`](../../docs/故障码.md) 里登记的故障，日志行以 `[码]` 开头。停机类（S1/S2）每次都记不限频；监控类（M）可限频，但出现和恢复每次都记；健康行只在异常时加码。加新故障或改文案：改 `config/fault_codes.yaml`，再运行 `python3 -m robot_log_tools.error_code --markdown > docs/故障码.md`；`test_error_code.py` 检查登记的文案和前缀还在代码里、文档和表一致。
- 消息写清**什么失败了、为什么、怎么处理**，带上关节序号、数值和阈值、文件名；一条日志一件事；要能 `grep`，不靠表情符号和颜色。
- 格式：supervisor 启动的模块统一 `[级别] [年-月-日 时:分:秒.毫秒] [节点名]: 消息`。

## 三种数据和记录点

| 数据 | 开销 | 去向 | 回答什么 |
|---|---|---|---|
| 日志行（含 `[码]`） | 低，只在事件时写（稳态主要是健康行） | `logs/<单元>/<时间>.log` | 出了什么事 |
| `[HEALTH]` 行，每窗口一行 | 每次记录约 10 ns | 同上，诊断时按窗口汇总 | 这一环平时健不健康 |
| 黑匣子环，每帧 / 每周期 | 每条约 40 ns | `/dev/shm/robot_blackbox/<模块>.ring` | 出事前每一环实际收发了什么数值 |

健康行格式 `[HEALTH] <组件> window=10.0s key=value ...`：控制器 `cycles`、`period_avg_ms`、`period_max_ms`、`late`、`duration_*`（迟到升 WARN 带 `[CTL-011]`）；XR bridge `count`、`rate_hz`、`gap_max_ms`、`received_*`、`parse_fail`、`connected`（间隔超过 `health_gap_warn_ms`=100 升 WARN 带 `[XR-006]`）；target_generator 目标速率和 XR 输入速率；Marvin 和 Juxie 的 `read`/`write`（带 `[HW-013]`）。

| 环节 | 记录点 |
|---|---|
| xr_bridge | 101 收到的头显帧、102 发布 |
| target_generator | 201 XR 位姿输入、202 目标输出 |
| 各控制器 | 301 收到的关节目标（MOVEJ；MPC 的末端目标话题还没记）、302 实测关节、303 命令、304 状态机切换 |
| Marvin / Juxie | 401 读到的关节位置、402 写出的命令 |

## 设计要点

- 采集（C++ header-only）只依赖 rclcpp 和标准库，业务包只通过头文件使用；实时线程里只做原子计数、写环、读时钟，不加锁、不分配、不打日志。
- 分析（`python/robot_log_tools`）不依赖 ROS，只读文件，事后在任何机器上都能分析。
- **只记录，不改变停机行为**：新增的故障条件先只打码，要升级为停机单独征得同意。
- 单一来源，另一处由测试保证一致：故障码 `fault_codes.yaml`（每条有 `source{file,match}` 锚点）；记录类型 `blackbox_types.yaml` ↔ `blackbox_types.hpp`；环格式 `blackbox.hpp` ↔ `blackbox.py`。
- 阈值是经验值，没用真机校准；**全部只在仿真里验证过**，真机项目见 [`docs/待办.md`](../../docs/待办.md)。

## 仿真里测到的

`mock_components`、容器里，分体 / 全身 / 双臂三种入口各测 HOLD 和 OCS2 的 60 秒窗口：

- **日志量很小**：稳态里几乎全是 `[HEALTH]` 行；normal 最多约 6 KB/分钟，debug 约 50 KB/分钟，都远低于预算（normal ≤ 100 KB/分钟）。一次完整启动加关闭约 450–540 行。
- **CPU 没有因日志和记录而变**：与加健康行、环之前的基线一致，在噪声内；CPU 几乎都是 MPC 计算。
- **内存多约 3–11 MB**：每个控制器一个 4 MB 的环，写满前逐页占用。
- **故障注入**：停住控制进程 1.5 秒 → 报告里有 `CTL-011`、健康行 `period max 1501 ms`、所有控制器的环在同一时刻断 1.5 秒；`kill -9` 控制进程 → 所有控制器报 `silent`。XR 帧中断没法在仿真里做。

测量脚本已删除（不是运行所需），要重测得重写。
