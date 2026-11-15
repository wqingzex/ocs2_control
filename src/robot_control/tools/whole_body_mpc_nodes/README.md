# whole_body_mpc_nodes

`whole_body_mpc` 的独立 ROS 测试环境：MPC 节点、无硬件仿真节点、目标发布节点。

## 作用

不接真机、不经过 `ros2_control`，单独验证 MPC 问题和模式切换。实际控制 fiveages_w2 用 `whole_body_controller`，不用这里的节点。

## 使用

```bash
ros2 launch whole_body_mpc_nodes whole_body.launch.py urdf_file:=/path/to/robot.urdf
```

拖 RViz 里的交互 marker 设目标（10 Hz 发布），切模式：

```bash
ros2 topic pub /mode_command std_msgs/String "data: 'DUAL_COUPLED'" --once
ros2 topic pub /left_arm_target geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 0.5, y: 0.3, z: 1.0}, orientation: {w: 1.0}}}" --once
```

其他 launch：`mpc_only.launch.py`、`target_only.launch.py`。

## 节点

| 节点 | 作用 | 主要接口 |
|---|---|---|
| `whole_body_mpc` | MPC 控制器；参数 `task_file`、`urdf_file`、`library_folder` | 订阅 `whole_body_mpc_target`、`whole_body_mode_schedule`；发布 `whole_body_mpc_policy` |
| `whole_body_dummy` | 无硬件仿真；参数同上加 `mrt_frequency`（默认 100 Hz） | 订阅 MPC 策略；发布状态、TF、marker |
| `whole_body_target` | 交互 marker 目标发布 | 订阅 `left_arm_target` `right_arm_target` `body_target`（`PoseStamped`）、`mode_command`（`String`）；发布目标轨迹（21 维）和模式表 |

底盘开关在 `.info` 的 `model_information.enableMobileBase`：`true` 为轮式底盘（状态 3 + N_arm，输入 2 + N_arm），`false` 为固定基座（N_arm）。
