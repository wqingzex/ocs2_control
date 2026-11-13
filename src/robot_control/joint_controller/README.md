# joint_controller

不带 MPC 的关节空间控制器插件 `joint_controller/JointController`：腰、头、身体关节的 HOME / HOLD / MOVEJ 插值。灵巧手和夹爪控制器（[`end_effector_controller`](../end_effector_controller/README.md)）继承它。

## 结构

```
JointController               本包：单数组关节目标、URDF 限位、独立 TF 监听
  └─ FsmController            （robot_controller_core）生命周期、HOME / HOLD / MOVEJ 状态、参数、腰部话题
       └─ FsmControllerBase   （robot_controller_core）接口加载、状态切换流程、fsm_command / fsm_state
```

不依赖 OCS2 和 `mpc_core`：MPC 控制器的骨架 `MpcFsmController`（`mpc_core`）也是在 `FsmController` 上加 MPC。

## 使用

```yaml
controller_manager:
  ros__parameters:
    body_joint_controller: {type: joint_controller/JointController}
body_joint_controller:
  ros__parameters:
    joints: [body_joint1, body_joint2, body_joint3, body_joint4]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    home_1: [0.0, 0.0, 0.0, 0.0]  # 必填，最多 home_10
```

## 状态机

状态 HOME、HOLD、MOVEJ。`/fsm_command`（`std_msgs/Int32`）：`1` HOME、`2` HOLD、`3` 或 `4` MOVEJ。激活后默认 HOLD；HOME 响应 `2/3/4`，HOLD 的 `3/4` 进 MOVEJ。HOME 内，`switch_command_base`（默认 100）加 `n` 切换到第 `n` 个构型。不发布共用话题 `fsm_state`（避免和 MPC 控制器、RViz 面板混在一起）。

## 参数和话题

参数：`joints`、`command_interfaces`、`state_interfaces`、`update_rate`、`home_*`、`movej_*`、`hold_*`、`waist_*`，含义同 [`arm_controller`](../arm_controller/README.md)。

| 话题 | 类型 | 说明 |
|---|---|---|
| `{实例名}/joint_target_position` | `Float64MultiArray` | 所有关节的目标，只在 MOVEJ 生效 |
| `joint_target_trajectory` | `JointTrajectory` | 多路点轨迹 |
| `waist_*` | | 腰位姿、升降、转向（有腰时） |
