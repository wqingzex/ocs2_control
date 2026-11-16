# keyboard_teleop

键盘遥操作：把按键转成 `/teleop/control_input`（机械臂位姿增量）、`/cmd_vel`（底盘）和腰的命令。不发 `/fsm_command`。

## 作用

不需要头显和手柄时的遥操作入口，输出由 `target_generator` 接收。不要同时运行键盘和手柄（[`joystick_teleop`](../joystick_teleop/README.md)），它们发布同样的话题。

## 使用

键盘（要在**独立的交互终端**里运行，需要真实的 TTY）：

```bash
ros2 run keyboard_teleop keyboard_teleop
```

| 按键 | 动作 |
|---|---|
| `WASD` | 平移（机械臂模式发 `Inputs` 的 x/y，底盘模式发 `cmd_vel` 线速度） |
| `I` `K` | z（仅机械臂） |
| `J` `L` | yaw / 偏航角速度 |
| 方向键 | roll / pitch（仅机械臂） |
| `N` | 切换机械臂 / 底盘 |
| `M` | 切换镜像（底盘忽略） |
| `-` `+` | 速度档 1~10，默认 3 |
| 空格 | 切换当前臂 |
| 回车 | 夹爪开合 |

键盘参数：`update_rate`（20 Hz）、`movement_key_stale_ms`（120）、`discrete_key_stale_ms`（320，必须大于内核按键重复延迟约 250 ms，否则切换键会触发两次）、`chassis.linear_scale`（0.25）、`chassis.angular_scale`（0.5）、`mirror_movement`、`show_status_line`。

## 话题

| 方向 | 话题 | 类型 |
|---|---|---|
| 发布 | `/teleop/control_input` | `robot_control_msgs/Inputs` |
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` |
| 发布 | `/body_joint_controller/waist_lifting_command` `waist_turning_command` | `Float64`（恒发 0） |
