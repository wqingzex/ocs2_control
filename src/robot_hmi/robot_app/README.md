# robot_app

遥操作应用：运控、相机、头显、手柄等**模块的统一启停与监控**（进程管理在 `robot_utils`），Web 控制台，录制回放和诊断。全部由 `config/robot.local.yaml` 驱动。输入设备在 [`robot_teleop`](../../robot_teleop/README.md)。

## 作用

日常使用 `./scripts/start.sh` 或双击仓库根目录的 `robot_control_suite控制台`，背后就是这个包。每个模块是独立进程、独立日志，出问题直接看对应模块：

| 模块 | 内容 |
|---|---|
| `pc_service` | XRoboToolkit PC Service（[`xr_teleop`](../../robot_teleop/xr_teleop/README.md) 提供，只停止自己启动的） |
| `xr` | [`xr_teleop`](../../robot_teleop/xr_teleop/README.md) 的 `xr_bridge` 节点，把头显和手柄数据发布成 `/teleop/*` |
| `control` | 运控 launch（控制器 + `target_generator`，不带 RViz） |
| `camera_head` `camera_left_wrist` `camera_right_wrist` | 每台相机一个驱动进程 |
| `joystick` `rviz` | 按需启动，不随"全部启动"带起 |

## 使用

```bash
./scripts/start.sh                      # 前台启动全部，Ctrl+C 停止（运控先切 HOLD）
./scripts/start.sh up|down|restart <模块>
./scripts/start.sh status --hz          # 状态、PID、话题频率
./scripts/start.sh logs xr -f           # logs/<模块>/latest.log
./scripts/start.sh check                # 只校验配置，打印将要启动的内容
./scripts/start.sh diag                 # 打包日志、时间线、配置，发给开发者
```

`--set 段.字段=值` 只对本次运行覆盖配置，文件不变。详细用法、按键、坐标对齐、录制回放、故障排查见 [`docs/使用手册.md`](../../../docs/使用手册.md)。

## 组成

| 路径 | 内容 |
|---|---|
| `src/` | C++：`console_launcher`（生成 `robot_control_suite控制台`） |
| `robot_app/` | Python：配置加载与校验、构造运控和 RViz 单元并汇总各设备包的单元（`units`）、命令行（`units_cli`）、Web 控制台（`console/`，FastAPI）、导航、录制、诊断。进程管理在 [`robot_utils`](../../robot_utils/README.md)，头显配对在 [`xr_teleop`](../../robot_teleop/xr_teleop/README.md) |
| `launch/teleop.launch.py` | 旧的"一个 launch 起全部" |

## 配置

本机配置是仓库根目录的 `config/robot.local.yaml`（`./scripts/setup.sh config` 按入口从 [`robot_config` 的模板](../../robot_setup/robot_config/robot_config/templates/base.yaml) 生成）。本包管 `robot`（`entry`: `whole_body`（默认）/ `split_body` / `dual_arm`，手、硬件）、`xr`（桥接、PC Service、网卡、头显 ID）、`cameras`（三台相机序列号和旋转）等段；`platform`、`hardware`、`control` 段由 ROS 启动文件读取。启动前校验：

```bash
python3 -m robot_app.check config/robot.local.yaml <工作区根目录>
```

## 测试

`pytest test`（需要 ROS 环境）。`test_console_launcher.py` 要先编译本包生成 `robot_control_suite控制台`，否则报错。
