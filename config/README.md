# config

本机文件，**不进 git**，每台机器各自一份。

| 文件 | 作用 |
|---|---|
| `robot.local.yaml` | 这台机器的配置：机器、控制、遥操作、相机。`setup.sh`、`start.sh`、遥操作和推理都读它 |
| `ros_env.sh` | 可选：`RMW_IMPLEMENTATION`、`ROS_DOMAIN_ID`、Zenoh 配置；双击启动的控制台也会读 |

## robot.local.yaml

`./scripts/setup.sh config` 生成：没有文件就按 `--entry whole_body|split_body|dual_arm` 从模板写一份；已有就补缺的键，保留值和注释，原文件备份为 `.bak`。

模板在 [`robot_config/templates/`](../src/robot_setup/robot_config/robot_config/templates/)：`base.yaml` 是带注释的完整模板，每个入口一个补丁文件。**模板就是键的清单**：

- 文件必须有模板的每个键，缺了报"缺少 xxx"；模板里没有的键报"未知字段"。程序不补默认值。
- 值写 `null` 表示没设置，由机器人描述包的默认值决定。
- 模板改了键之后，每台机器运行一次 `./scripts/setup.sh config`。

分组：`robot`（机器：底盘、腰头、臂、末端，每个部件先写 `type` 再写 `interface`）、`control`、`xr` 和 `joystick`（遥操作输入）、`cameras`（每台相机各有自己的成像参数）、`log` / `record`（运行时）、`setup`（确认记录，自动写入）。各字段含义见模板里的注释，常用项见[使用手册第 5 节](../docs/使用手册.md)。

首次使用：`./scripts/setup.sh`。之后 `setup.sh cam` 和 `setup.sh ee` 弹窗确认相机和左右手，结果自动写回；`setup.sh marvin` 选天机臂 SDK 版本。临时改某项只对本次有效：`./scripts/start.sh --set robot.hardware=isaac`（设备映射不能这样改）。
