# config

本机配置。

| 文件 | 作用 |
|---|---|
| `robot.local.yaml` | 这台机器的配置：启动什么、硬件参数。`scripts/start.sh`、`scripts/setup.sh`、遥操作和推理都读它。**不进 git**，每台机器各自一份；新机器上 `./scripts/setup.sh` 会从模板生成 |
| `robot.template.yaml` | 模板，复制成 `robot.local.yaml` 后修改，只写与默认值不同的项 |
| `ros_env.sh` | 可选，不进 git：`RMW_IMPLEMENTATION`、`ROS_DOMAIN_ID`、Zenoh 配置；双击启动的控制台也会读 |

## robot.local.yaml

分两部分：

- 上半部分是**启动什么**：`robot`（机型入口、手、硬件）、`xr`、`cameras` 等。字段说明和默认值见 [`teleop.yaml`](../src/robot_hmi/teleop_app/config/teleop.yaml)，启动前用 `./scripts/start.sh check` 校验。
- 下半部分是**这台机器的硬件参数**：`platform`、`defaults`、`hardware`、`control`，由 ROS 启动文件读取，字段见 [`fiveages_w2_description`](../src/robot_description/fiveages_w2_description/README.md) 和 [`m6_ccs_description`](../src/robot_description/component/m6_ccs_description/README.md)。

首次使用用 `./scripts/setup.sh cam` 和 `./scripts/setup.sh hand` 弹窗确认相机和左右手，确认结果自动写回这个文件；天机臂 SDK 版本（`hardware.marvin_firmware`：338 或 343）由 `./scripts/setup.sh marvin` 选择并写回，编译时使用。临时改某项只对本次有效：`./scripts/start.sh --set robot.hardware=isaac`。
