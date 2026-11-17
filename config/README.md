# config

本机配置。

| 文件 | 作用 |
|---|---|
| `robot.local.yaml` | 这台机器的配置：启动什么、硬件参数。`scripts/start.sh`、`scripts/setup.sh`、遥操作和推理都读它。**不进 git**，每台机器各自一份；新机器上 `./scripts/setup.sh` 会从模板生成 |
| `robot.template.yaml` | 模板，复制成 `robot.local.yaml` 后修改，只写与默认值不同的项 |
| `ros_env.sh` | 可选，不进 git：`RMW_IMPLEMENTATION`、`ROS_DOMAIN_ID`、Zenoh 配置；双击启动的控制台也会读 |

## robot.local.yaml

模板里每个键标了 `[必改]`（新机器上要确认）、`[可选]`（默认值够用，要改才取消注释）、`[自动写入]`（`setup.sh` 或网页控制台写回，不用手填）。结构：

- **`robot`：机器**，每个部件先写**是什么**（`type`），再写**怎么连**（`interface`）：

  | 部件 | `type` | `interface` |
  |---|---|---|
  | `base` 底盘 | `linkhou_s2` / `linkhou_s2_v2`（另有 `joints_movable`） | 无 |
  | `body` 腰、头 | `juxie` | `can`（CAN FD 接口）、`stats_period_s` |
  | `arm` 臂 | `marvin` | `ip`（控制器以太网地址）、`firmware`（SDK 版本 338 / 343） |
  | `end_effector` 末端 | `linkerhand_l6` / `xhand1` / `wuji_hand2` / `jodell_rg75` / `none` | `bus`，以及左右手各自的 `port`（by-id 串口）/ `can` / `channel`；另有 `dyn_param`（每侧工具负载） |

  另外有 `entry`（控制入口）、`hardware`（运行方式 `real` / `mock_components` / `isaac`，下面各部件的 `interface` 只在真机生效）、`variant`。
- **`control`：控制参数**：`arm_mode`、`limits`（关节最大速度和加速度）、`patch`（对 ros2_control 控制器参数的覆盖，如 home 位姿、`info_file_name`）。
- **`xr` / `cameras` / `record`**：遥操作与采集。

字段说明和默认值见 [`teleop.yaml`](../src/robot_hmi/robot_app/config/teleop.yaml)，启动前用 `./scripts/start.sh check` 校验；`robot` 和 `control` 里的值由 ROS 启动文件转成 xacro 参数（名字没变），各机型的参数见 [`fa_w2_description`](../src/robot_description/fa_w2_description/README.md) 和 [`m6_ccs_description`](../src/robot_description/component/m6_ccs_description/README.md)。旧布局的 `platform`、`hardware`、`defaults` 三段和 `robot.end_effector_bus` 等旧字段已删除，写了会报"未知字段"。

首次使用用 `./scripts/setup.sh cam` 和 `./scripts/setup.sh hand` 弹窗确认相机和左右手，确认结果自动写回这个文件；天机臂 SDK 版本（`robot.arm.interface.firmware`：338 或 343）由 `./scripts/setup.sh marvin` 选择并写回，编译时使用。临时改某项只对本次有效：`./scripts/start.sh --set robot.hardware=isaac`（手的串口等设备映射不能这样改）。
