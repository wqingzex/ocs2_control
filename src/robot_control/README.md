# robot_control

双臂和 fiveages_w2 的控制栈：从末端目标到关节指令。控制器都是 ros2_control 的 `ControllerInterface` 插件，由 `controller_manager` 加载。

```
目标（robot_hmi 的 teleop_command：RViz marker / XR / 手柄）
   → 控制器（MPC 或关节插值，ros2_control 插件）
   → src/robot_hardware 里的硬件插件 → 真机
```

| 位置 | 内容 |
|---|---|
| [`arm_controller/`](arm_controller/README.md) | 控制器插件：双臂 MPC |
| [`whole_body_controller/`](whole_body_controller/README.md) | 控制器插件：全身 MPC（轮式人形） |
| [`joint_controller/`](joint_controller/README.md) | 控制器插件：腰、头、身体的关节空间控制（不含 MPC） |
| [`end_effector_controller/`](end_effector_controller/README.md) | 控制器插件：灵巧手 `HandController`、夹爪 `GripperController` |
| [`robot_controller_core/`](robot_controller_core/README.md) | 控制器核心：状态机（HOME/HOLD/MOVEJ）和 FSM 控制器基类 |
| [`robot_control_msgs/`](robot_control_msgs/README.md) | 控制器对外的消息、服务、action 定义 |
| [`tools/`](tools/README.md) | 开发用工具：`whole_body_mpc_nodes`，不接硬件单独跑全身 MPC |
| [`lib/`](lib/README.md) | 非插件的库：`motion_planning`、`robot_dynamics`、`trajectory_manager`，OCS2 的 MPC（`lib/mpc/`） |

启动文件在 [`../robot_launch`](../robot_launch/README.md)，RViz 面板和目标指令在 [`../robot_hmi`](../robot_hmi/README.md)。

## 使用

日常通过仓库根目录的 `./scripts/start.sh` 启动，不需要直接操作这里。只编译某个包：

```bash
colcon build --packages-up-to arm_controller
```

整个工作区用 `./scripts/setup.sh build`。依赖：OCS2（`src/third_party/ocs2_ros2` 子模块）、Pinocchio。

## 约定

包名与目录名一致。控制器插件各占一个包，直接放在本目录，和 `robot_hardware` 下的硬件插件对称；被多个控制器共用的代码放核心或 `lib/`。
