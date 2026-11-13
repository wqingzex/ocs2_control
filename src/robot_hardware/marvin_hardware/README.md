# marvin_hardware

天机 Marvin 机械臂（M6 等）的 `ros2_control` 驱动，通过 Marvin SDK 连接控制器，并驱动臂体上的末端工具。插件 `marvin_hardware/MarvinHardware`。

## 作用

W2、M6 的双臂用它（URDF 里的 `M6_CCS_System`）。双臂时左臂走 A 通道，右臂走 B 通道。独立 USB 转接的末端不走本包，用 [`modbus_hardware`](../modbus_hardware/README.md)。

## 使用

由 xacro 配置，宏在 `m6_ccs_description/xacro/ros2_control/marvin_hardware.xacro`。`robot.local.yaml` 的 `hardware:` 段经 xacro 传入（其中 `arm_ctrl_mode` 对应插件的 `ctrl_mode`），插件参数：

| 参数 | 说明 |
|---|---|
| `arm_type` | `left` / `right` / `dual` |
| `device_ip` | 控制器 IP |
| `left_ee_type` / `right_ee_type` | 末端类型键，见下 |
| `left_ee_channel` / `right_ee_channel` | `2` 臂体 RS485（默认），`1` CAN / CAN FD |
| `ctrl_mode` | 初始控制模式 |
| `max_joint_speed` / `max_joint_acceleration` | 关节限速 |
| `left_dyn_param` / `right_dyn_param` | 工具动力学，10 维 |
| `left_kine_param` / `right_kine_param` | 工具运动学，6 维 |

接口：命令 `position`；状态 `position`、`velocity`、`effort`。

## 末端类型

按 `left_ee_type` / `right_ee_type`（大写归一化）创建；未知夹爪回退 `JDGripper`，未知灵巧手回退 O7。

| 类型键 | 产品 | 总线 |
|---|---|---|
| `RG75` / `JDGRIPPER`、`ERG32` | Jodell RG75、ERG32-150 | RS485 |
| `AG2F90_C` `AG2F90_D` `AG2F120S` `AG2F120S_D` | ChangingTek 夹爪 | RS485 |
| `linkerhand_o7` `linkerhand_o6` `linkerhand_l6` | LinkerHand | RS485 |
| `freedom_v1` `freedom_v2`、`inspire_e2` `inspire_f2` | Freedom、Inspire RH56 | RS485；`ee_channel=1` 时 CAN / CAN FD |

本仓库的描述包只带 `linkerhand_l6` 和 `xhand1`（后者不走本包）。

## 运行期调参

启动时硬件参数同步声明为节点参数，可用 `ros2 param set /<硬件节点> <参数> <值>` 修改（节点名用 `ros2 param list` 查）。

| 参数 | 说明 |
|---|---|
| `ctrl_mode` | `POSITION` / `JOINT_IMPEDANCE` / `CART_IMPEDANCE` / `POWER_OFF` |
| `joint_k_gains` `joint_d_gains` | 7 维关节阻抗 |
| `cart_k_gains` `cart_d_gains` | 7 维笛卡尔阻抗 |
| `left_tool_torque` `left_tool_velocity`（右同） | 末端归一化力矩、速度 |
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

Marvin SDK 是子模块 `src/third_party/TJ_FX_ROBOT_CONTRL_SDK`（上游 `cynthia-you/TJ_FX_ROBOT_CONTRL_SDK`，固定在 `02440e8`），克隆后需 `git submodule update --init src/third_party/TJ_FX_ROBOT_CONTRL_SDK`。只支持底层控制系统 100343（43 版固件）及以上，使用其中的 `contrlSDK100343`，不支持 43 以前的固件。`libKine.so` 缺失时编译会自动重建。`examples/` 里是不经过 `ros2_control` 的夹爪、灵巧手 SDK 调试程序。夹爪的位置换算复用 `gripper_hardware_common`。
