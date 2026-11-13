# lib

控制器共用的库，被 `robot_control` 下的控制器和 `robot_hmi` 下的工具链接，自身不是可执行程序。

| 位置 | 作用 |
|---|---|
| [`motion_planning`](motion_planning/README.md) | 轨迹规划库：moveJ / moveL / 圆弧 / 速度规划，依赖 Eigen 和 rclcpp（日志） |
| [`robot_dynamics`](robot_dynamics/README.md) | 机器人模型：运动学（FK/IK）、重力补偿、关节限位，建立在 Pinocchio 之上 |
| [`trajectory_manager`](trajectory_manager/README.md) | 关节、笛卡尔轨迹管理，腰升降规划，依赖 `robot_dynamics` 和 `motion_planning` |
| [`mpc/`](mpc/README.md) | OCS2 MPC：共用运行时 `mpc_core`、双臂 `arm_mpc`、全身 `whole_body_mpc` |
