# marvin_hardware

天机 Marvin 机械臂（M6 等）的 `ros2_control` 驱动，通过 Marvin SDK 连接控制器，只驱动臂本体；臂末端的手、夹爪和力传感器是各自独立的组件（见 [`end_effector_hardware`](../end_effector_hardware/README.md)，经本包提供的法兰总线通信）。插件 `marvin_hardware/MarvinHardware`。

## 作用

fa_w2 和 M6 的双臂用它（URDF 里的 `arms`）。双臂时左臂走 A 通道，右臂走 B 通道。末端执行器不走本包的关节接口。

## 使用

由 xacro 配置，宏在 `m6_ccs_description/xacro/ros2_control/marvin_hardware.xacro`。`robot.local.yaml` 的 `hardware:` 段经 xacro 传入（其中 `arm_ctrl_mode` 对应插件的 `ctrl_mode`），插件参数：

| 参数 | 说明 |
|---|---|
| `arm_type` | `left` / `right` / `dual` |
| `device_ip` | 控制器 IP |
| `ctrl_mode` | 初始控制模式 |
| `max_joint_speed` / `max_joint_acceleration` | 关节限速 |
| `left_dyn_param` / `right_dyn_param` | 末端负载动力学，10 维（`setup.sh dyn` 辨识） |
| `left_kine_param` / `right_kine_param` | 末端负载运动学，6 维 |

接口：命令 `position`；状态 `position`、`velocity`、`effort`。

## 法兰总线

每条臂的法兰口登记成 `robot_comm` bus，供 `end_effector_hardware` 的 `EndEffectorHardware`（RG75、L6）和 `Kwr75Sensor` 使用（见 `m6_ccs_description` 的 `external_ee_systems`、`ft_sensor_systems`）。代码在 `comm/`：`sdk_channels` 是 SDK 通道收发函数的薄封装（左臂 A、右臂 B），`arm_bus` 把它们变成 bus。

| bus | SDK 通道 |
|---|---|
| `arm_rs485://<device_ip>/<left\|right>/port1` | COM1 |
| `arm_rs485://<device_ip>/<left\|right>/port2` | COM2 |
| `arm_canfd://<device_ip>/<left\|right>` | CANFD |

一条臂只有一个 SDK 收发缓冲，三个口共用一个在途请求，轮流发。`write()` 末尾发出下一帧，`read()` 开头取回复；臂连上之前和断开之后，这些 bus 不可用，挂在上面的请求都以错误结束。

## 运行期调参

启动时硬件参数同步声明为节点参数，可用 `ros2 param set /<硬件节点> <参数> <值>` 修改（节点名用 `ros2 param list` 查）。

| 参数 | 说明 |
|---|---|
| `ctrl_mode` | `POSITION` / `JOINT_IMPEDANCE` / `CART_IMPEDANCE` / `POWER_OFF` |
| `joint_k_gains` `joint_d_gains` | 7 维关节阻抗 |
| `cart_k_gains` `cart_d_gains` | 7 维笛卡尔阻抗 |
| `left_brake_release` `right_brake_release` | 松闸，仅 `POWER_OFF` 下可用 |

**松闸**：先停控制器，再把硬件组件设为 `inactive`，之后才能松闸；恢复前先抱闸。

```bash
ros2 control switch_controllers --deactivate whole_body_controller
ros2 control set_hardware_component_state <SystemName> inactive
ros2 param set /<硬件节点> left_brake_release true      # 手动调整后改回 false
ros2 control set_hardware_component_state <SystemName> active
ros2 param set /<硬件节点> ctrl_mode POSITION
ros2 control switch_controllers --activate whole_body_controller
```

## 编译

Marvin SDK 按控制器固件分成两个子模块（上游 `cynthia-you/TJ_FX_ROBOT_CONTRL_SDK`，`.gitmodules` 里都设了 `shallow = true`，只拉固定的那一个提交）：`src/third_party/TJ_FX_ROBOT_CONTRL_SDK_343`（固定在 `02440e8`，固件 100343 及以上，用 `contrlSDK100343`）和 `src/third_party/TJ_FX_ROBOT_CONTRL_SDK_338`（固定在 `1fa6cc6`，固件 338，用 `contrlSDK`）。克隆后 `git submodule update --init --depth 1 src/third_party/TJ_FX_ROBOT_CONTRL_SDK_343 src/third_party/TJ_FX_ROBOT_CONTRL_SDK_338`（`./scripts/setup.sh deps` 会做）。用 `-DMARVIN_FIRMWARE=343|338` 选（默认 343），要和控制器固件一致，装错时连接后读不到控制器版本，激活后立即急停抱闸。`./scripts/setup.sh` 的 `marvin` 步骤询问版本并写入配置的 `robot.arm.interface.firmware`，`./scripts/setup.sh build` 按它传 `-DMARVIN_FIRMWARE`。`libKine.so` 缺失或不是 Linux ELF 时编译会自动用 g++ 重建（338 子模块里带的是 Windows DLL，会被重建）。`examples/` 里是不经过 `ros2_control` 的夹爪、灵巧手 SDK 调试程序。夹爪和灵巧手的寄存器表、单位换算用 `end_effector_hardware` 导出的纯头文件目标 `end_effector_hardware::gripper_protocol`（不链接它的插件库）。
