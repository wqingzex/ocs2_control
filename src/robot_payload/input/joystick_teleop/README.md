# joystick_teleop

手柄遥操作：把手柄输入转成 `/teleop/control_input`（机械臂位姿增量）、`/cmd_vel`（底盘）、腰的命令和 `/fsm_command`。

## 作用

不需要头显时的遥操作入口，输出由 `teleop_command` 接收。Web 控制台的"手柄遥操"卡片或 `./scripts/start.sh up joystick` 启动的就是这个包。不要同时运行手柄和键盘（[`keyboard_teleop`](../keyboard_teleop/README.md)），它们发布同样的话题。

## 使用

手柄：

```bash
ros2 launch joystick_teleop joystick_teleop.launch.py                  # 默认 auto
ros2 launch joystick_teleop joystick_teleop.launch.py joy_dev:=/dev/input/js1
ros2 run joy joy_enumerate_devices                                 # 列出手柄
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `joy_driver` | `auto` | `auto` / `game_controller` / `joy`：`auto` 在 SDL 能识别（`Mapped: true`）时用 `game_controller_node`，否则用 `joy_node` |
| `joy_dev` | `/dev/input/js0` | 手柄设备 |
| `config` | `auto` | `config/joy_mapping/` 下的映射文件名；`auto` 为 `generic_gamepad`（已识别）或 `default` |

未识别的手柄：`ros2 topic echo /joy` 记下按键下标，在 `config/joy_mapping/` 新建 yaml，用 `config:=<名字>` 指定。手柄设备也可保存在 `robot.local.yaml` 的 `joystick.device`（空为自动）。

## 话题

| 方向 | 话题 | 类型 |
|---|---|---|
| 发布 | `/teleop/control_input` | `robot_control_msgs/Inputs` |
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` |
| 发布 | `/body_joint_controller/waist_lifting_command` `waist_turning_command` | `Float64` |
| 发布 | `/fsm_command` | `Int32` |
| 订阅 | `joy` | `sensor_msgs/Joy` |
