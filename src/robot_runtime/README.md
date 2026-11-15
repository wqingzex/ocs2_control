# robot_runtime

进程管理：把一次运行拆成若干**单元**（一个进程组、一份日志），按依赖顺序启动，监控，停止。不认识任何设备，也不认识具体机型。

## 组成

| 文件 | 内容 |
|---|---|
| `units.py` | `Unit`（命令、依赖、健康话题、就绪日志、分组）、`UnitSet`（启动和停止顺序）、`Driver`（不是单个进程的单元，如启动后自己 fork 并退出的守护进程） |
| `supervisor.py` | `Supervisor`：启动、停止（SIGINT，再 SIGTERM，再 SIGKILL）、状态、日志尾部、异常退出记录；状态文件 `run/<模块>.json` 重启后仍有效 |
| `processes.py` | 状态文件读写、进程是否存活（防 PID 复用）、等日志出现某段文字；`OwnedDrivers`（设置窗口自己拉起并清理的子进程） |
| `events.py` | 每个单元只留最新的 20 份日志 |

## 谁提供单元

各设备包登记自己的单元，`teleop_app.build_units` 汇总：`w2_camera.units`（相机）、`xr_teleop.units`（PC Service、XR 桥）、`joystick_teleop.units`（手柄）；运控和 RViz 由 `teleop_app` 构造。

依赖单向：设备包和 `teleop_app` 依赖 `robot_runtime`，`robot_runtime` 只依赖 [`robot_log`](../robot_log/README.md) 的事件时间线。

## 测试

`pytest test`。
