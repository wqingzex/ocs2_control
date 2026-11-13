# arms_teleop

手柄和键盘遥操作：把输入转成 `control_input`（机械臂位姿增量）、`/cmd_vel`（底盘）和腰的命令。

## 作用

不需要头显时的遥操作入口，输出由 `teleop_command` 接收。手柄栈还会发 `/fsm_command`，键盘不发。Web 控制台的"手柄遥操"卡片或 `./scripts/start.sh up joystick` 启动的就是手柄这一路。

## 使用

手柄：

```bash
ros2 launch arms_teleop joystick_teleop.launch.py                  # 默认 auto
ros2 launch arms_teleop joystick_teleop.launch.py joy_dev:=/dev/input/js1
ros2 run joy joy_enumerate_devices                                 # 列出手柄
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `joy_driver` | `auto` | `auto` / `game_controller` / `joy`：`auto` 在 SDL 能识别（`Mapped: true`）时用 `game_controller_node`，否则用 `joy_node` |
| `joy_dev` | `/dev/input/js0` | 手柄设备 |
| `config` | `auto` | `config/joy_mapping/` 下的映射文件名；`auto` 为 `generic_gamepad`（已识别）或 `default` |

未识别的手柄：`ros2 topic echo /joy` 记下按键下标，在 `config/joy_mapping/` 新建 yaml，用 `config:=<名字>` 指定。手柄设备也可保存在 `robot.local.yaml` 的 `joystick.device`（空为自动）。

键盘（要在**独立的交互终端**里运行，需要真实的 TTY）：

```bash
ros2 run arms_teleop keyboard_teleop
```

不要同时运行手柄和键盘，它们发布同样的话题。

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
| 发布 | `control_input` | `robot_control_msgs/Inputs` |
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` |
| 发布 | `/body_joint_controller/waist_lifting_command` `waist_turning_command` | `Float64`（键盘恒发 0） |
| 发布 | `/fsm_command` | `Int32`（仅手柄） |
| 订阅 | `joy` | `sensor_msgs/Joy`（仅手柄） |
