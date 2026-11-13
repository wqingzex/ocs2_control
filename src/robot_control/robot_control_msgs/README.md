# robot_control_msgs

控制栈共用的接口定义，只有 `.msg` / `.srv` / `.action`，没有代码。

## 作用

控制器、RViz 面板、遥操作、目标管理之间的话题与服务契约都在这里。要新增或修改接口，改这里再重新编译依赖它的包。

## 内容

| 类型 | 名称 | 用途 |
|---|---|---|
| msg | `Inputs` | 键盘/手柄的位姿增量输入（`control_input` 话题） |
| msg | `WholeBodyCapability` | 全身控制器支持哪些约束（躯干竖直、腰锁定、头耦合、双臂耦合等） |
| msg | `WholeBodyCurrentState` | 全身控制器当前的底盘、躯干等状态；RViz 面板据此生成身体模式下拉框 |
| msg | `LinearMessage` `CircleMessage` `JointWaypoint` | 直线、圆弧、关节路点，作为下面服务和 action 的字段 |
| srv / action | `ExecuteLinear` `MovecUseIK` `JointTrajectory` | MoveL、MoveC、关节轨迹（由 `StateMoveJ` 提供） |
| srv | `ExecuteCircle` `ExecutePath` | 圆弧、路径目标（由 `PoseBasedReferenceManager` 提供） |
| srv | `KinematicsService` | 运动学查询（由 `arm_controller` 提供） |
| srv | `CartesianPath` | 仓库内没有使用者 |

## 使用

```bash
ros2 interface show robot_control_msgs/msg/WholeBodyCurrentState
ros2 action send_goal /arm_controller/execute_linear robot_control_msgs/action/ExecuteLinear "{...}"
```

仓库内没有这些服务和 action 的客户端，它们是给外部程序和命令行调用的入口。
