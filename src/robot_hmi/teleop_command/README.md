# teleop_command

给机械臂、头、腰设置**末端目标**的节点：RViz 里拖交互 marker，或用 VR 头显手柄、键盘手柄输入，输出目标位姿给控制器。

## 作用

位于输入和控制器之间。输入来源：RViz 交互 marker、VR（`VRInputHandler`，订阅 `/teleop/*`）、键盘手柄（`/teleop/control_input`）。输出：`left_target`、`right_target`，由控制器里的 `PoseBasedReferenceManager` 转成 MPC 目标。VR 同时负责握把离合（按住跟随、松开停止）和状态机切换。

## 使用

不单独启动，由控制器的 launch 带起，参数由 `launch_helpers` 自动准备（从 task `.info` 读 `dual_arm_mode`、`control_base_frame`，检测 `hand_controllers`）：

```bash
./scripts/start.sh                                         # W2：运控 + VR
ros2 launch robot_bringup dual_arm.launch.py robot:=m6_ccs
```

在 RViz 里加 `InteractiveMarkers` 显示，Fixed Frame 设为 `marker_fixed_frame`，拖 marker 即发目标。VR 的使用和按键见 [`docs/VR遥操作.md`](../../../docs/VR遥操作.md)。

## 配置

先找 task `.info` 同目录的 `teleop_command.yaml`，没有则用本包 `config/default.yaml`。

| 参数 | 默认 | 含义 |
|---|---|---|
| `enable_vr` | yaml 中 `false`，代码默认 `true` | 是否处理 VR 输入 |
| `vr_control_scheme` | `legacy` | VR 按键方案：`legacy` 或 `simple`（握把按住跟随，扳机比例开合） |
| `vr_align_head_yaw` | `false` | 按头显朝向对齐 VR 坐标 |
| `vr_pose_scale` | `1.0` | 手柄位姿缩放，建议 0.75~1.5 |
| `vr_thumbstick_linear_scale` / `_angular_scale` | `0.005` / `0.05` | 摇杆每步的线、角位移 |
| `linear_scale` / `angular_scale` | `0.005` / `0.05` | 键盘手柄的缩放 |
| `reference_link` | `head_link2` | 头部 marker 的参考连杆 |
| `marker_fixed_frame` | `base_link` | marker 的固定坐标系 |

## 接口

| 方向 | 话题 | 说明 |
|---|---|---|
| 发布 | `left_target` `right_target`（及 `/stamped`） | 目标位姿 |
| 发布 | `/fsm_command` | VR 触发的状态机切换 |
| 订阅 | `left_current_pose` `right_current_pose` | 当前位姿，用来更新 marker |
| 订阅 | `/teleop/control_input` | 键盘手柄输入（`Inputs`） |
| 订阅 | `/teleop/head_pose` `/teleop/left_ee_pose` `/teleop/right_ee_pose` `/teleop/controller_state` `/teleop/thumbstick_axes` `/teleop/trigger_values` | 来自 `teleop_app` 的 XR 桥接 |
