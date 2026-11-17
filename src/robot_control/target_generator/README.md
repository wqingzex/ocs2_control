# target_generator

给机械臂、头、腰设置**末端目标**的节点：RViz 里拖交互 marker，或用 XR 头显手柄、键盘手柄输入，输出目标位姿给控制器。

**原名 `teleop_command`**（原在 `robot_hmi/`）。改名后变了的地方：节点名 `/teleop_command` → `/target_generator`（RViz 里 `Interactive Markers Namespace` 要改成 `/target_generator`），机型旁边的 `teleop_command.yaml` → `target_generator.yaml`，启动参数 `enable_teleop_command` → `enable_target_generator`，飞行记录和健康日志里的模块名同步改了。话题 `left_target`、`right_target`、`/fsm_command`、`/teleop/*` 没变。

## 作用

位于输入和控制器之间。输入来源：RViz 交互 marker、XR（`XrInputHandler`，订阅 `/teleop/*`）、键盘手柄（`/teleop/control_input`）。输出：`left_target`、`right_target`，由控制器里的 `PoseBasedReferenceManager` 转成 MPC 目标。XR 同时负责状态机切换和跟随的开始/停止（右摇杆按下），见 [`XR_INPUT_HANDLER_README.md`](XR_INPUT_HANDLER_README.md)。

## 使用

不单独启动，由控制器的 launch 带起，参数由 `launch_helpers` 自动准备（从 task `.info` 读 `dual_arm_mode`、`control_base_frame`，检测 `hand_controllers`）：

```bash
./scripts/start.sh                                         # 运控 + XR
ros2 launch robot_startup dual_arm.launch.py robot:=m6_ccs
```

在 RViz 里加 `InteractiveMarkers` 显示，Fixed Frame 设为 `marker_fixed_frame`，拖 marker 即发目标。XR 的使用和按键见 [`docs/使用手册.md`](../../../docs/使用手册.md)。

## 配置

先找 task `.info` 同目录的 `target_generator.yaml`，没有则用本包 `config/default.yaml`。

| 参数 | 默认 | 含义 |
|---|---|---|
| `enable_xr` | yaml 中 `false`，代码默认 `true` | 是否处理 XR 输入 |
| `xr_align_head_yaw` | `false` | 按头显朝向对齐 XR 坐标 |
| `xr_pose_scale` | `1.0` | 手柄位姿缩放，建议 0.75~1.5 |
| `linear_scale` / `angular_scale` | `0.005` / `0.05` | 键盘手柄的缩放 |
| `reference_link` | `head_link2` | 头部 marker 的参考连杆 |
| `marker_fixed_frame` | `base_link` | marker 的固定坐标系 |

## 接口

| 方向 | 话题 | 说明 |
|---|---|---|
| 发布 | `left_target` `right_target`（及 `/stamped`） | 目标位姿 |
| 发布 | `/fsm_command` | XR 触发的状态机切换 |
| 订阅 | `left_current_pose` `right_current_pose` | 当前位姿，用来更新 marker |
| 订阅 | `/teleop/control_input` | 键盘手柄输入（`Inputs`） |
| 订阅 | `/teleop/head_pose` `/teleop/left_ee_pose` `/teleop/right_ee_pose` `/teleop/controller_state` `/teleop/trigger_values` | 来自 `robot_app` 的 XR 桥接 |
