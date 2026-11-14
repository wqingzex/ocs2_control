# robot_dynamics

控制器共用的机器人模型库，建立在 Pinocchio 之上，不含状态机。

| 内容 | 文件 |
|---|---|
| 运动学（FK / IK，臂的 DLS / BFGS 求解，运动学服务） | `Kinematics.h`、`AngleUtils.h` |
| 重力补偿 | `GravityCompensation.h` |
| 关节限位（从 URDF 读取）、头部耦合限位、伺服滤波 | `JointLimitsManager.h`、`HeadCouplingLimits.h`、`JointServoFilter.h` |

头文件用 `#include <robot_dynamics/Kinematics.h>`，命名空间是 `robot_dynamics`。被 `trajectory_manager`、`robot_controller_core`、两个 MPC 任务包和 `robot_controller` 使用。
