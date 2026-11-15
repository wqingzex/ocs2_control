# XrInputHandler

`teleop_command_node` 里处理 XR 手柄的部分：把 `xr_teleop` 桥节点发来的手柄位姿、按键和扳机，变成状态机命令、双臂目标位姿和手的开合。按键表见 [`docs/使用手册.md`](../../../docs/使用手册.md) 第 6.1 节。

## 启用

只有图里存在名为 `/xr_target_node` 的节点时才处理 XR 输入（桥节点和 `xr_stub.py` 都用这个名字）。节点消失后自动停用并打 `[TEL-001]`。

## 按键

事件码来自 `/teleop/controller_state`（`std_msgs/Int32`，每帧都发，没按键时是 0），定义在 `xr_teleop/button_event_encoder.hpp`：

| 码 | 按键 | 处理 |
|---|---|---|
| 1 | 右 A | 状态机前进：HOME → HOLD → OCS2 |
| 2 | 左 X | 状态机后退：OCS2 / MOVEJ → HOLD → HOME；HOME 下切换 HOME ↔ REST |
| 3 | 右摇杆按下 | 只在 OCS2：STORAGE → UPDATE（记录基准，双臂开始跟随），UPDATE → STORAGE（停止） |

其他值一律忽略。当前 FSM 状态来自 `teleop_command_node` 转发的 `/fsm_command`（`fsmCommandCallback`）；离开 OCS2 时自动回到 STORAGE，再进 OCS2 要重新按右摇杆才跟随。

## 跟随（UPDATE）

进入 UPDATE 时记下手柄位姿和机器人末端当前位姿作为基准。之后每帧：

```
目标位置 = 机器人基准位置 + 位移坐标系⁻¹ × (手柄当前位置 − 手柄基准位置) × xr_pose_scale
目标姿态 = (手柄基准姿态⁻¹ × 手柄当前姿态) × 机器人基准姿态
```

位移坐标系默认是进入 UPDATE 时手柄的姿态；`xr_align_head_yaw: true` 时改用头显的水平朝向（操作员正前方 = 机器人 +X）。目标的位置变化超过 1 cm 或姿态变化超过 0.005 rad 才发布，避免刷屏。

## 扳机

`/teleop/trigger_values`（linear.x 左、angular.x 右，0 松开 ~ 1 按到底）按比例映射到 `/<控制器>/target_percent`（1 张开 ~ 0 闭合）。控制器来自 `hand_controllers` 参数，名字里带 `left` / `right` 的分别对应左右扳机；灵巧手和夹爪都支持 `target_percent`。

## 话题

| 方向 | 话题 | 说明 |
|---|---|---|
| 订阅 | `/teleop/left_ee_pose` `/teleop/right_ee_pose` `/teleop/head_pose` | 手柄和头显位姿 |
| 订阅 | `/teleop/controller_state` | 按键事件码 |
| 订阅 | `/teleop/trigger_values` | 扳机按下深度 |
| 发布 | `left_target` `right_target` | 跟随时的目标位姿 |
| 发布 | `/fsm_command` | 状态机命令 |
| 发布 | `/<控制器>/target_percent` | 手 / 夹爪开合 |

## 参数（`teleop_command.yaml`）

| 参数 | 默认 | 含义 |
|---|---|---|
| `enable_xr` | `true`（`config/default.yaml` 里为 `false`） | 是否创建 XrInputHandler |
| `xr_pose_scale` | `1.0` | 手柄位移缩放，建议 0.75~1.5 |
| `xr_align_head_yaw` | `false` | 按头显朝向解释位移 |

## 测试

`test/test_xr_input_handler.cpp`：在一个进程里起处理器和一个 `xr_target_node`，发按键、位姿和扳机，检查 `/fsm_command`、`left_target` 和 `target_percent`。

## 停用的代码

`XrInputHandler.cpp` 的 `processButtonEvent` 里保留着旧方案"握把 + 扳机"在线校准 `xr_pose_scale` 的代码（已注释掉，没有按键触发）。需要在线调比例时再评估恢复。
