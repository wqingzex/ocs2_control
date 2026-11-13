# rokae_hardware

珞石 Rokae 机械臂的 `ros2_control` 驱动（xCoreSDK）。插件 `rokae_hardware/RokaeHardware`。本仓库的 W2 / M6 不使用，用于 Rokae 机型。

## 使用

接口：命令 `position`；状态 `position`、`velocity`、`effort`。测试环境 Ubuntu 24.04 + ROS 2 Jazzy。

```xml
<hardware>
  <plugin>rokae_hardware/RokaeHardware</plugin>
  <param name="arm_ip">192.168.0.160</param>
  <param name="local_ip">192.168.0.233</param>
  <param name="gripper_type">ag2f90_c</param>
</hardware>
```

## 末端工具（经机器人末端 RS485）

URDF 里加一个名字含 `gripper` 或 `hand` 的关节，用 `gripper_type` 选协议：

| `gripper_type` | 产品 | 说明 |
|---|---|---|
| `ag2f90_c` / `ag2f90_d`（及 `changingtek90c/d` 等别名） | ChangingTek AG2F90 | 位置 0（闭）~ 1（开）；速度、力矩寄存器 `0x0104`、`0x0105` |
| `jodell` | Jodell RG75 | 位置是开口物理距离（0 ~ 0.038372 m） |
| `linkerhand_l6`（或 `l6`） | LinkerHand L6 | 左 `0x28`、右 `0x27`；可选 `hand_slave_id`、`hand_configure_xpanel`、`hand_read_interval_ms` |

常用参数：`gripper_slave_id`、`gripper_read_interval_ms`；力矩和速度用归一化的 `left_tool_torque` / `left_tool_velocity`（右同，`0~1`），可用 `ros2 param set` 运行时修改；`gripper_configure_xpanel=false` 保持控制器当前的末端 24V 和 RS485 设置，只有机型和固件支持时才设为 `true`。未连接的夹爪状态保持 0。

## 负载

初始化实时运动控制器时按 `setLoad()` 设置负载，默认全零。辨识后显式配置（可只给部分分量，其余为零）：

```xml
<param name="load_mass">1.2874</param>
<param name="load_cog">[-0.030145, -0.010147, 0.14785]</param>
<param name="load_inertia">[0.0, 0.0, 0.0]</param>
```
