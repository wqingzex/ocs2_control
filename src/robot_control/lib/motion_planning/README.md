# motion_planning

轨迹规划库：关节、笛卡尔直线、圆弧的插补，速度规划，以及 W2 腰部的运动学。

## 作用

被 `robot_controller_core`（MoveJ / MoveL / MoveC 的插补）和 `mpc_core` 使用。

依赖 Eigen 和 rclcpp：不再是不依赖 ROS 的纯库，日志统一走 rclcpp 的日志接口（`RCLCPP_ERROR_STREAM(planning::logger(), ...)`，头文件 `motion_planning/common/logger.h`），输出会进节点日志并带级别。

## 内容

| 模块 | 内容 |
|---|---|
| `common` | `JointVector`、四元数与旋转矩阵、轨迹点与约束参数（`TrajectPoint`、`TrajectoryParameter`）、混合状态 |
| `kinematics` | `FiveAgesW2FK` / `FiveAgesW2IK`：5 轴腰部的正、逆运动学 |
| `velocity_planner` | `Doubles` 七段速度曲线，及起止速度、加速度为零的特化版本 |
| `path_planner` | `moveJ` 关节插补、`moveL` 直线（含姿态 slerp）、`CircularCurver` 圆弧、`SpeedJ` 速度插补、`BezierCurve` 五阶贝塞尔、`SmoothCurveOfMultiJointsUsingBlending` 多段混合转接；基类 `Planner` 负责时间缩放与运行循环 |

## 使用

```cmake
find_package(motion_planning REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC motion_planning::motion_planning)
```

规划器的用法见 `robot_controller_core` 里的 `StateMoveJ` 和 `CartesianTrajectoryManager`。
