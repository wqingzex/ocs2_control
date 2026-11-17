# config

本机文件。**不进 git**，每台机器各自一份。

| 文件 | 作用 |
|---|---|
| `robot.local.yaml` | 这台机器的配置：什么臂、底盘、末端，各自怎么连，控制参数，遥操作和相机。`scripts/start.sh`、`scripts/setup.sh`、遥操作和推理都读它 |
| `ros_env.sh` | 可选：`RMW_IMPLEMENTATION`、`ROS_DOMAIN_ID`、Zenoh 配置；双击启动的控制台也会读 |

## robot.local.yaml

用 `./scripts/setup.sh config` 生成：没有文件时按入口（`--entry whole_body|split_body|dual_arm`，默认 `whole_body`）从模板写一份；已有文件时把缺的键补上，保留你的值和注释，原文件备份为 `robot.local.yaml.bak`。

模板在 [`src/robot_setup/robot_config/robot_config/templates/`](../src/robot_setup/robot_config/robot_config/templates/)：`base.yaml` 是完整的基础模板（带注释），每个入口一个补丁文件（只写和基础不同的值，比如 `dual_arm.yaml`）。**模板就是键的清单**：

- 文件里必须有模板的每一个键，没有的启动时报"缺少 xxx（运行 `./scripts/setup.sh config`）"；不在模板里的键报"未知字段"。程序不会替你补默认值。
- 值写 `null` 表示没设置，由机器人描述包自己的默认值决定。模板里的值就是各机型描述包现在的默认值。
- 模板改了键（新增、改名）之后，每台机器运行一次 `./scripts/setup.sh config`。

结构：

- **`robot`：机器**，每个部件先写**是什么**（`type`），再写**怎么连**（`interface`）：

  | 部件 | `type` | `interface` |
  |---|---|---|
  | `base` 底盘 | `linkhou_s2` / `linkhou_s2_v2`（另有 `joints_movable`） | 无 |
  | `body` 腰、头 | `juxie` | `can`（CAN FD 接口）、`stats_period_s` |
  | `arm` 臂 | `marvin` | `ip`（控制器以太网地址）、`firmware`（SDK 版本 338 / 343） |
  | `end_effector` 末端 | `linkerhand_l6` / `xhand1` / `wuji_hand2` / `jodell_rg75` / `none` | `bus`，以及左右手各自的 `port`（by-id 串口）/ `can` / `channel`；另有 `dyn_param`（每侧工具负载） |

  另外有 `entry`（控制入口）、`hardware`（运行方式 `real` / `mock_components` / `isaac`，下面各部件的 `interface` 只在真机生效）、`variant`。
- **`control`：控制参数**：`arm_mode`、`limits`（关节最大速度和加速度）、`patch`（对 ros2_control 控制器参数的覆盖，如 home 位姿、`info_file_name`，内容不检查键）。
- **`setup`**：设备确认记录，由 `setup.sh` 和网页控制台写入，不用手填。
- **`xr` / `joystick`**：遥操作输入设备。**`cameras`**：感知，每台相机（`head`、`left_wrist`、`right_wrist`）各有自己的 `white_balance`、`ae_max_exposure`、`exposure`、`gain`（`exposure` 和 `gain` 都写 `-1` 是自动曝光，都写数值是固定曝光；头部默认自动，腕部默认固定）。**`log` / `record` / `navigation`**：运行时。

`robot` 和 `control` 里的值由 ROS 启动文件转成 xacro 参数（名字没变），各机型的参数见 [`fa_w2_description`](../src/robot_description/fa_w2_description/README.md) 和 [`m6_ccs_description`](../src/robot_description/component/m6_ccs_description/README.md)。旧布局的 `platform`、`hardware`、`defaults` 三段和 `robot.end_effector_bus` 等旧字段已删除，写了会报"未知字段"。

首次使用：`./scripts/setup.sh`（完整初始化，第一步就是生成配置），之后 `./scripts/setup.sh cam` 和 `./scripts/setup.sh ee` 弹窗确认相机和左右手，确认结果自动写回这个文件；天机臂 SDK 版本（`robot.arm.interface.firmware`：338 或 343）由 `./scripts/setup.sh marvin` 选择并写回，编译时使用。临时改某项只对本次有效：`./scripts/start.sh --set robot.hardware=isaac`（手的串口等设备映射不能这样改）。
