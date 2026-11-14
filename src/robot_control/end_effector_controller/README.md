# end_effector_controller

末端执行器的控制器插件，都是关节空间状态机（`robot_controller_core` 的 HOME / HOLD / MOVEJ）加开合命令：

| 插件 | 做什么 |
|---|---|
| `end_effector_controller/HandController` | 灵巧手：关节插值，加开合命令 |
| `end_effector_controller/GripperController` | 夹爪：单关节，开合命令，关闭时带力反馈 |

`EndEffectorController`（`src/EndEffector.h`、`EndEffectorController.cpp`）直接继承 `robot_controller_core::FsmController`，和 `joint_controller` 没有继承关系；开合行为是它持有的 `EndEffector`（`HandEndEffector.cpp`、`GripperEndEffector.cpp`），控制器把钩子转给它。

测试：`test/gripper_mock/run.sh` 用单关节 `mock_components` 夹爪测开合、百分比、HOLD、解除、重新激活保持位置、HOME 和力反馈。

## 灵巧手：HandController

`left_hand_controller` 这类实例用，开合只在 MOVEJ 状态生效，不切状态。两个构型取自 HOME 的 `home_N`：

| 话题 | 类型 | 说明 |
|---|---|---|
| `target_command` | `Int32`（0 / 1） | `0` 闭合、`1` 张开 |
| `target_percent` | `Float64`（0~1） | 在闭合和张开构型之间逐关节线性插值，`0` 闭合、`1` 张开 |

| 参数 | 默认 | 说明 |
|---|---|---|
| `target_command_close_config` | `1` | 闭合用第几个 HOME 构型（索引从 0 起，即 `home_2`） |
| `target_command_open_config` | `0` | 张开用第几个（`home_1`） |
| `target_command_hold_joints` | `[]` | 开合时保持当前位置不动的关节 |

## 夹爪：GripperController

一个实例管一个夹爪关节（`joints` 只能写一个）。闭合和张开位置从 `/robot_description` 自动解析：关节限位里离 `initial_value` 近的一端是“闭合”。和其他实例一样走状态机，命令只在 MOVEJ 生效；HOLD 停在当前位置，HOME 回到 `home_1`（没写 `home_1` 时用硬件的 `initial_value`）。

```yaml
controller_manager:
  ros__parameters:
    left_gripper_controller: {type: end_effector_controller/GripperController}
left_gripper_controller:
  ros__parameters:
    joints: [left_gripper_joint]
    command_interfaces: [position]
    state_interfaces: [position, effort]   # 不用力反馈时只写 position，并把 use_effort_interface 设为 false
    use_effort_interface: true
    force_threshold: 0.15
    force_feedback_ratio: 0.3
    movej_interpolation_type: none         # 目标直接写给硬件；要平滑改成 linear / tanh 并设 movej_duration
```

| 话题 | 类型 | 说明 |
|---|---|---|
| `{joint}/position_command` | `Float64` | 直接位置，无力反馈（相对名，跟随命名空间） |
| `target_command` | `Int32`（0 / 1） | `0` 闭合（带力反馈）、`1` 张开 |
| `target_percent` | `Float64`（0~1） | `0` 闭合、`1` 张开；朝闭合方向运动时带力反馈 |

| 参数 | 默认 | 说明 |
|---|---|---|
| `use_effort_interface` | `true` | 用 effort 状态接口做力反馈；为 `true` 但没有 effort 接口时报错并关掉力反馈 |
| `force_threshold` | `0.1` | 触发力反馈的阈值 |
| `force_feedback_ratio` | `0.5` | 触发后继续推进剩余距离的比例：`0` 停在当前位置，`1` 继续到原目标 |

力反馈只在朝闭合方向、本次命令还没触发过时生效：`|effort| > force_threshold` 时，新目标 = 当前位置 + (原目标 − 当前位置) × `force_feedback_ratio`。
