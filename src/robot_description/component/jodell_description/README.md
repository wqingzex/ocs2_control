# jodell_description

Jodell（钧舵）夹爪的描述：**RG75-300**（平行夹爪），本仓库唯一支持的夹爪。来自真机部署包 `fa-deploy-ws` 的 `robot-descriptions-common/gripper/jodell_description`，fa_w2 的部署预设里左手默认就是 RG75。

| 文件 | 内容 |
|---|---|
| `xacro/RG75-300.xacro` | RG75 宏 `RG75-300`：`name` 前缀、`use_flange`（带法兰）、`pad`（指尖垫：默认 / FiveAges / leapmotor / heavy_carry / ptc）、`kinematic_only`（规划用，不带可动手指） |
| `xacro/ros2_control/RG75-300.xacro` | 仿真用的接口宏 `rg75_interfaces`：`<name>_gripper_joint` 的 position 命令，position / velocity / effort 状态 |
| `xacro/ros2_control/side_systems.xacro` | USB（Modbus）接线的 `ros2_control` 系统 |

夹爪关节名固定为 `<side>_gripper_joint`（`left_gripper_joint`、`right_gripper_joint`），对应的控制器是 `end_effector_controller/GripperController` 实例（见 `robot_control/end_effector_controller/README.md`）。

## 在整机上用

fa_w2 和 M6 的类型解析认 `rg75`：

```bash
ros2 launch robot_startup split_body.launch.py robot:=fa_w2 hardware:=mock_components end_effector:=jodell_rg75
ros2 launch robot_startup dual_arm.launch.py  robot:=m6_ccs      hardware:=mock_components end_effector:=jodell_rg75
```

真机上夹爪走 Marvin 工具端口（`left_end_effector_bus: arm_rs485`）。夹爪的标定负载（`dyn_param`）已经写在 `m6_ccs_description/xacro/eefs.xacro` 里。真机上的动作没有验证过。
