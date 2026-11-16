# whole_body_controller

轮式人形全身 MPC 的 `ros2_control` 控制器插件 `whole_body_controller/WholeBodyController`：双臂、腰、头、底盘一起优化。

## 结构

```
WholeBodyController           本包：MPC 的 home 参考、世界 TF、底盘 TF 探测、人形模式重置
  └─ MpcFsmController         （mpc_core）在 FsmController 上加 MPC 组件、OCS2 状态、可视化线程、MoveJ 限位与自碰撞
       └─ FsmController       （robot_controller_core）生命周期、HOME / HOLD / MOVEJ 状态、参数、腰部话题
            └─ FsmControllerBase  （robot_controller_core）接口加载、状态切换流程、fsm_command / fsm_state
```

MPC 任务本身在 [`whole_body_mpc`](../lib/mpc/whole_body_mpc/README.md)；共用的运行时在 [`mpc_core`](../lib/mpc/mpc_core/README.md)。

## 使用

整机由 `robot_launch/robot_bringup` 的 `whole_body.launch.py` 起。在机器人描述包的 `config/ros2_control/*.yaml` 里声明（fiveages_w2 见 `fiveages_w2_description/config/ros2_control/common.yaml`）：

```yaml
controller_manager:
  ros__parameters:
    whole_body_controller: {type: whole_body_controller/WholeBodyController}
whole_body_controller:
  ros__parameters:
    update_rate: 500
    mpc_frequency: 100
    robot_name: fiveages_w2
    info_file_name: whole_body_fixed_eef_l6   # 默认值；可选值见 fiveages_w2_description README
    joints: [...]
```

状态机、通用参数（`robot_name`、`info_package`、`info_file_name`、`mpc_frequency`、线程、`home_*`、`movej_*`、`waist_*` 等）和 [`arm_controller`](../arm_controller/README.md) 相同。

## 全身专有

参数：`fsm_default_humanoid_mode`（默认 `HM_DUAL_IDLE`）、`humanoid_mode_topic`（默认 `mode_command`）、`cppad_library_subdir`、`head_frame`、`body_pose_from_current_state`。模式命令、预设和能力门控见 [`whole_body_mpc`](../lib/mpc/whole_body_mpc/README.md)；`.info` 的主要段有 `model_information`、`bodyRelative`、`waistLock`、`bimanualCoupling`、`endEffector`、`selfCollision`、`inputCost`、`initialModeSchedule`（用 `HM_*` 名）、`ddp` / `mpc`。

| 话题 | 类型 | 说明 |
|---|---|---|
| `waist_lifting_pose_relative` `waist_lifting_pose_absolute` `waist_phi_command` | | 腰位姿 |
| `whole_body_capabilities` `current_state` | `WholeBodyCapability` `WholeBodyCurrentState` | 能力门控和当前模式，RViz 面板订阅 |
| `mode_command` | `std_msgs/String` | 模式切换 |

```bash
ros2 topic pub --once /mode_command std_msgs/msg/String "data: 'BODY_LOCK'"            # 局部命令
ros2 topic pub --once /mode_command std_msgs/msg/String "data: 'PRESET:HM_DUAL_IDLE'"   # 整体预设
```
