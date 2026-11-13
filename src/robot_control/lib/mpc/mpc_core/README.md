# mpc_core

OCS2 控制器（`arm_controller`、`whole_body_controller`）共用的部分：目标管理、MPC 时序、可视化。

## 内容

| 组件 | 功能 |
|---|---|
| `PoseBasedReferenceManager` | 订阅末端目标并转成 MPC 的 `TargetTrajectories`；提供圆弧、路径服务 |
| `OCS2ReferenceTargetContext` | 目标管理需要的上下文 |
| `MpcExecutionParams` | 由控制频率和 `mpc_frequency` 算出 MPC 周期与线程休眠 |
| `OCS2PinocchioVisualizer` | 在 RViz 里显示 Pinocchio 模型与自碰撞 |

## 接口（`PoseBasedReferenceManager`）

| 方向 | 名称 | 说明 |
|---|---|---|
| 订阅 | `left_target` `right_target`（及 `/stamped`）、`body_target`、`dual_target/stamped` | 末端、躯干、双臂目标位姿 |
| 订阅 | `target_path` | 路径目标 |
| 服务 | `execute_left_circle` `execute_right_circle` `execute_path` | 圆弧、路径（类型见 `robot_control_msgs`） |
| 发布 | `left_current_target` `right_current_target` | 当前生效的目标 |

目标位姿通常来自 `teleop_command`。

## 使用

在控制器包里链接：

```cmake
find_package(mpc_core REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC mpc_core::mpc_core)
```

依赖 OCS2（`ocs2_ros2` 子模块）和 Pinocchio。
