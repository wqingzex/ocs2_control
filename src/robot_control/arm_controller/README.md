# arm_controller

双臂 MPC 的 `ros2_control` 控制器插件 `arm_controller/ArmController`：分体、双臂模式用。

## 结构

```
ArmController                 本包：运动学求解器、MoveL / MoveC action、运动学服务、MPC 位姿目标同步
  └─ MpcFsmController         （mpc_core）在 FsmController 上加 MPC 组件、OCS2 状态、可视化线程、MoveJ 限位与自碰撞
       └─ FsmController       （robot_controller_core）生命周期、HOME / HOLD / MOVEJ 状态、参数、腰部话题
            └─ FsmControllerBase  （robot_controller_core）接口加载、状态切换流程、fsm_command / fsm_state
```

MPC 任务本身在 [`arm_mpc`](../lib/mpc/arm_mpc)；共用的运行时在 [`mpc_core`](../lib/mpc/mpc_core/README.md)。

## 使用

整机由 `robot_launch/robot_startup` 的启动文件起（`./scripts/start.sh` 按 `robot.entry` 选）。在机器人描述包的 `config/ros2_control/*.yaml` 里声明（fa_w2 见 `fa_w2_description/config/ros2_control/common.yaml`）：

```yaml
controller_manager:
  ros__parameters:
    arm_controller: {type: arm_controller/ArmController}
arm_controller:
  ros__parameters:
    update_rate: 500
    mpc_frequency: 100
    robot_name: m6_ccs            # 规划模型
    info_package: fa_w2_description   # 可选，.info 所在的包；不写 = <robot_name>_description
    info_file_name: dual_arm_eef  # <包>/config/ocs2/<name>.info，默认 dual_arm_eef
    joints: [left_joint1, ...]    # 顺序和数量必须与 .info 的状态一致
```

## 状态机

状态 HOME、HOLD、MOVEJ、OCS2。`/fsm_command`（`std_msgs/Int32`）：`1` HOME、`2` HOLD、`3` OCS2、`4` MOVEJ。激活后默认 HOLD；MoveL / MoveC 必须先进入 MOVEJ。HOME 内，`switch_command_base`（默认 100）加 `n` 切换到第 `n` 个构型（`home_1`、`home_2` …）。HOME 只响应 `2`，HOLD 的 `3` 进 OCS2。发布共用话题 `fsm_state`。

## 参数

| 参数 | 说明 |
|---|---|
| `robot_name` | 机器人名，决定规划 URDF；不写 `info_package` 时 `.info` 也从 `<robot_name>_description` 取 |
| `info_package` | `.info` 所在的包（如整机用部件臂规划时放在整机包里），默认空 |
| `info_file_name` | `<包>/config/ocs2/<name>.info`，默认 `dual_arm_eef`；命名见 [fa_w2_description](../../robot_description/fa_w2_description/README.md#info-命名) |
| `mpc_frequency` | MPC 求解频率 |
| `recompile_libraries` | 重编 CppAD 库；也可由 `.info` 的 `model_settings.recompileLibraries` 覆盖，Isaac 默认不重编 |
| `thread_priority` `cpu_affinity` | MPC 工作线程的实时优先级和绑核 |
| `rt_timing_enabled` `traj_record_enabled` `traj_record_dir` | 实时周期计时、轨迹录制 |
| `joints` `command_interfaces` `state_interfaces` `update_rate` | 关节和硬件接口 |
| `home_*` `movej_*` `hold_*` | 各状态的时长、插值（`tanh` / `linear` / `none`）、阈值 |
| `movel_*` | MoveL 的速度、加速度、加加速度 |
| `waist_lifting_enabled` 等 `waist_*` | 腰部升降参数 |

规划用的 URDF 由 `launch_helpers` 从 xacro 生成并写成文件，经 `planning_urdf_path` 传入，缺少时启动报错。控制模式按硬件接口自动选：只有 `position` 是位置模式，`position / velocity / effort / kp / kd` 齐全是 MIX（带重力补偿）。

## 话题

以实例名为前缀，例如 `/arm_controller/...`；除特别注明，只在 MOVEJ 状态生效。

| 话题 | 类型 | 说明 |
|---|---|---|
| `joint_target_position` | `Float64MultiArray` | 关节目标（另有 `/body` `/head` `/left` `/right` 子话题） |
| `joint_target_trajectory` | `JointTrajectory` | 多路点轨迹 |
| `waist_lifting_command` `waist_turning_command` | `Float64` | 腰升降 / 转向速度因子 |

腰的升降还可通过 `robot_control_msgs/action/WaistLiftingPose` 调用。
