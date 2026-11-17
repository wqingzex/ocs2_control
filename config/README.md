# config

本机配置。

| 文件 | 作用 |
|---|---|
| `robot.local.yaml` | 这台机器的配置：启动什么、硬件参数。`scripts/start.sh`、`scripts/setup.sh`、遥操作和推理都读它。**不进 git**，每台机器各自一份；新机器上 `./scripts/setup.sh` 会从模板生成 |
| `robot.template.yaml` | 模板，复制成 `robot.local.yaml` 后修改，只写与默认值不同的项 |
| `ros_env.sh` | 可选，不进 git：`RMW_IMPLEMENTATION`、`ROS_DOMAIN_ID`、Zenoh 配置；双击启动的控制台也会读 |

## robot.local.yaml

分两部分，模板里每组有编号标题，每个键标了 `[必改]`（新机器上要确认）、`[可选]`（默认值够用，要改才取消注释）、`[自动写入]`（`setup.sh` 或网页控制台写回，不用手填）：

- **A 部分 启动什么**：① 机型与运行方式（`robot.entry`、`robot.hardware`）② 末端执行器（`robot.end_effector`、`robot.end_effector_bus`）③ 遥操作输入 `xr` ④ 相机 `cameras` ⑤ 录制 `record`。字段说明和默认值见 [`teleop.yaml`](../src/robot_hmi/robot_app/config/teleop.yaml)，启动前用 `./scripts/start.sh check` 校验。
- **B 部分 这台机器的硬件参数**，由 ROS 启动文件读取：⑥ 底盘与臂控制模式 `platform` ⑦ 天机臂（`hardware` 里的 SDK 版本、关节速度和加速度）⑧ 腰、头 CAN ⑨ 末端接口（串口、CAN）⑩ 末端负载覆盖 `defaults` ⑪ 控制器参数覆盖 `control.patch`。字段见 [`fa_w2_description`](../src/robot_description/fa_w2_description/README.md) 和 [`m6_ccs_description`](../src/robot_description/component/m6_ccs_description/README.md)。

分组只是模板里的排版，键路径没有变，已有的 `robot.local.yaml` 不用迁移。

首次使用用 `./scripts/setup.sh cam` 和 `./scripts/setup.sh hand` 弹窗确认相机和左右手，确认结果自动写回这个文件；天机臂 SDK 版本（`hardware.marvin_firmware`：338 或 343）由 `./scripts/setup.sh marvin` 选择并写回，编译时使用。临时改某项只对本次有效：`./scripts/start.sh --set robot.hardware=isaac`。
