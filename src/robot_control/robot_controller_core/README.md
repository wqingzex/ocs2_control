# robot_controller_core

控制器核心：有限状态机（HOME / HOLD / MOVEJ）和 FSM 控制器基类。运动学、重力补偿、限位在 [`robot_dynamics`](../lib/robot_dynamics/README.md)，轨迹管理在 [`trajectory_manager`](../lib/trajectory_manager/README.md)。

## 作用

`robot_controller`（所有 MPC 和关节实例）共用这里的状态实现，`teleop_command` 和 RViz 面板用它的 FSM 命令工具。新增一种控制器时，复用这里的状态，只写自己特有的状态。

## 内容

| 组件 | 功能 |
|---|---|
| `FSMState` / `FSMStateTransitionValidator` | 状态机基类与转换校验 |
| `StateHome` | 回预设构型（最多 10 个），平滑插值，可在构型间切换 |
| `StateHold` | 保持当前关节位置，可选重力补偿 |
| `StateMoveJ` | 关节空间运动：单目标、轨迹、MoveL / MoveC（服务与 action 在此提供），用 `trajectory_manager` 和 `robot_dynamics` |
| `FsmControllerBase` / `WaistCommands` | FSM 控制器基类、腰部指令 |
| `FSMCommandPublisher` | 发布 `/fsm_command` |

控制模式按硬件接口自动判定：只有 `position` 命令接口为 **POSITION**（重力补偿不生效）；同时有 `position / velocity / effort / kp / kd` 为 **MIX**（输出补偿力矩）。

`/fsm_command` 命令值：`1` HOME、`2` HOLD、`3` 控制器的主状态、`4` MOVEJ；大于等于 `switch_command_base` 的值在 HOME 内切换构型。

## 使用

在控制器包里链接：

```cmake
find_package(robot_controller_core REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC robot_controller_core::robot_controller_core)
```

```xml
<depend>robot_controller_core</depend>
```

构造 `StateHome` / `StateHold` / `StateMoveJ` 时传入 `CtrlInterfaces`，用法见 `robot_controller` 的 `MpcFsmController`。
