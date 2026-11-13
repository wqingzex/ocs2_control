# w2_payload

W2 本体外接装置的公共部分：相机、左右手的配置、启动和设置，以及机器人配置文件的读取框架。遥操作和推理程序共用，**不依赖** `fa_teleop`、头显桥接或 PC Service。

## 作用

`./scripts/setup.sh` 的相机、手、负载辨识、检查这几步就是这个包；`./scripts/start.sh` 启动相机也用它。配置写入同一份 `config/robot.local.yaml`，关闭窗口或 Ctrl+C 保留原配置。

## 使用

```bash
./scripts/setup.sh cam      # 弹窗：按实时画面分配头部、左右腕，设置旋转并逐项确认
./scripts/setup.sh hand     # 弹窗：选 usb / 485 接线，用"开合测试"核对左右手
./scripts/setup.sh dyn      # 负载辨识
./scripts/setup.sh check    # 检查配置是否完整
```

左右以机器人自身方向为准。手指测试只在你点击并确认后才执行。确认记录绑定当前设备映射，映射变化后需要重新确认。

相机窗口发现旧的采集进程时，会列出设备和 PID，询问是否停止并重启预览；只有明确确认才停止，运控进程不在清理范围内。容器里看不到的宿主机进程需要先在宿主机上停。

不用遥操作也能启动相机：

```bash
ros2 launch w2_payload cameras.launch.py config:=/绝对路径/config/robot.local.yaml
```

推理程序读同一份配置：

```python
from w2_payload.config import load
from w2_payload.devices.cameras import camera_specs

cameras = camera_specs(load("config/robot.local.yaml"))
```

## 组成

依赖方向从下到上，`runtime` 和 `config` 不依赖其他层（`test/test_boundaries.py` 检查）：

| 模块 | 内容 |
|---|---|
| `runtime.py` | `run/<模块>.json` 状态文件、进程识别、setup 自己拉起的子进程 |
| `config/` | 配置框架与校验（`robot`、`cameras` 两段）、配置编辑 |
| `devices/` | Orbbec USB 扫描、相机规格与启动、旧采集进程识别、手的串口探测 |
| `setup/` | 设置步骤、Qt 窗口、确认记录、负载辨识、配置检查 |

其他包的配置段通过 `Extension` 接入：`fa_teleop` 用它加入 `xr`、`joystick`、`navigation` 段和 `robot.entry` 的取值检查。不带 `Extension` 调用 `load` 时其他包的段被忽略，所以推理程序读同一份文件不会报错。

`hardware.usb_left_port` 等设备参数仍由机器人的 `robot_profile` 机制传给运控和手驱动，这里不另存设备映射。XR 配对在遥操作包里，由 `./scripts/setup.sh xr` 调用。

## 测试

`pytest test`（需要 ROS 环境）。窗口测试需要 `python3-pyqt5`，没装时这几个会报错。
