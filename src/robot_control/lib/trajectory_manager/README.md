# trajectory_manager

MOVEJ 状态用的轨迹管理，依赖 `robot_dynamics`（运动学、限位）和 `motion_planning`（路径、速度规划）。

| 内容 | 文件 |
|---|---|
| 关节轨迹（moveJ、带参数轨迹、速度/加速度限制） | `JointTrajectoryManager.h`、`Interpolation.h` |
| 笛卡尔轨迹（moveL、圆弧，调用 IK） | `CartesianTrajectoryManager.h` |
| 腰升降规划 | `WaistLiftingPlanner.h` |
| 停止规划、轨迹记录 | `JointSpeedStopPlanner.h`、`TrajectoryRecorder.h` |

头文件用 `#include <trajectory_manager/JointTrajectoryManager.h>`，命名空间是 `trajectory_manager`。`motion_planning` 现在是必需依赖。
