# robot_config

W2 外接装置的公共部分：机器人配置文件的读取、设置窗口的框架、负载辨识。相机在 [`camera`](../camera_setup/README.md)，手在 [`end_effector`](../end_effector_setup/README.md)。遥操作和推理程序共用，**不依赖** `robot_app`、头显桥接或 PC Service。

## 作用

`./scripts/setup.sh` 的负载辨识（`dyn`）和检查（`check`）两步是这个包，相机、手两步由各自的包提供，配置写入同一份 `config/robot.local.yaml`。

```bash
./scripts/setup.sh dyn      # 负载辨识（--reset 恢复默认负载）
./scripts/setup.sh check    # 检查配置是否完整
```

推理程序读同一份配置：

```python
from robot_config.config import load
from launch_helpers.devices.camera import camera_specs

cameras = camera_specs(load("config/robot.local.yaml"))
```

## 组成

依赖方向从下到上（`test/test_boundaries.py` 检查每个文件允许引用哪些文件）：

| 文件 | 内容 |
|---|---|
| `config.py` | 模板（`templates/`，键的清单）、加载和校验、设备映射字段（相机和手的确认由哪些字段决定）；不依赖包内其他文件 |
| `config_edit.py` | 保留注释地改写配置；确认记录和指纹，映射变化后要重新确认 |
| `device_setup.py` | 设备配置的公共框架：会话基类、Qt 窗口外壳、终端输入输出、各步骤的命令行入口 |
| `config_status.py` | 各步骤是否需要配置，配置检查（`check`） |
| `sync.py` | `setup.sh config`：按入口生成配置，补缺的键 |
| `load_identify.py` | 负载辨识（`setup.sh dyn`） |
| `testing.py` | 测试用：由模板得到完整配置 `full_config` |

`sync`、`config_status`、`load_identify` 是 `setup.sh` 直接 `python3 -m` 调用的命令入口，所以各占一个文件。

设备包提供会话（草稿、校验、保存）和面板（窗口里的卡片），窗口框架不认识具体设备。

其他包的配置段通过 `Extension` 接入：`robot_app` 用它加入 `xr`、`joystick`、`navigation` 段和 `robot.entry` 的取值检查。不带 `Extension` 调用 `load` 时其他包的段被忽略，所以推理程序读同一份文件不会报错。

`robot.end_effector.interface.left.port` 等设备参数仍由机器人的 `robot_profile` 机制传给运控和手驱动，这里不另存设备映射。XR 配对在遥操作包里，由 `./scripts/setup.sh xr` 调用。

## 测试

`pytest test`（需要 ROS 环境）。窗口测试需要 `python3-pyqt5`。
