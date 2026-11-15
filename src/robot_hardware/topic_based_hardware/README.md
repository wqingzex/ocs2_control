# topic_based_hardware

通过 ROS 2 话题收发关节指令和状态的硬件接口。插件 `topic_based_hardware/TopicBasedSystem`。

## 作用

对接不在 `ros2_control` 里的另一端，双臂和 fiveages_w2 用它连 **Isaac Sim**：控制器的指令发到话题，仿真器的关节状态从话题读回。上游项目的副本，支持 position、velocity、effort 接口。

## 使用

由 `m6_ccs_description/xacro/ros2_control/simulation_hardware.xacro` 在 `hardware:=isaac` 时写入：

```xml
<hardware>
  <plugin>topic_based_hardware/TopicBasedSystem</plugin>
  <param name="joint_commands_topic">/isaac/joint_command</param>
  <param name="joint_states_topic">/isaac/joint_states</param>
  <param name="initialize_commands_from_state">true</param>
</hardware>
```

| 参数 | 说明 |
|---|---|
| `joint_commands_topic` | 指令发布到哪个话题 |
| `joint_states_topic` | 从哪个话题读关节状态 |
| `initialize_commands_from_state` | `true`：用收到的第一帧状态初始化指令，启动时不会突然跳动，**真机和对接外部系统时应保持 `true`**；`false`：用配置里的 `initial_value`，只适合仿真或确知初始位置 |

启动后驱动按上述参数自动初始化并在日志里记录用了哪种方式。
