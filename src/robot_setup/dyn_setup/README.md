# dyn_setup

天机臂（Marvin）的**工具负载辨识向导**。向导本身是一个 ROS 可执行文件；怎么调用它、结果怎么写回 `robot.local.yaml` 在 [`robot_config`](../robot_config/README.md)（`./scripts/setup.sh dyn`）。

```bash
./scripts/setup.sh dyn           # 逐臂询问，辨识完成后确认写入 robot.end_effector.dyn_param
./scripts/start.sh dyn-identify  # 只运行向导（交互式）
ros2 run dyn_setup tool_dyn_identify_wizard
```

## 组成

| 文件 | 内容 |
|---|---|
| `scripts/tool_dyn_identify_wizard.py` | 向导：连接机械臂、采集空载和带载数据、辨识，结果写到 `<工作目录>/logs/dyn_identify/` |

向导用 `marvin_hardware` 装好的 `SDK_PYTHON`（源码树里运行时用 `third_party/TJ_FX_ROBOT_CONTRL_SDK_<固件>`），所以依赖 `marvin_hardware`，反过来不依赖。
