# gripper_hardware_common

夹爪硬件驱动共用的工具，**只有头文件**。

## 作用

`end_effector_hardware` 和 `marvin_hardware` 都要驱动不同品牌的夹爪，位置换算、Modbus 寄存器、重复指令过滤这些逻辑放在这里，避免每个驱动各抄一份。

## 内容

| 类 | 功能 |
|---|---|
| `PositionConverter` | 归一化位置（0 闭合、1 张开）与厂商位置互转：Changingtek 90（0–9000）、Jodell（0–255） |
| `ModbusConfig` | Changingtek 90、Jodell 的寄存器地址、功能码、从站地址等常量 |
| `CommandChangeDetector` | 指令变化小于阈值（默认 1%）时不重复发送 |
| `ReadFrequencyController` | 每 N 个周期读一次，降低 Modbus 通信量（默认 4） |
| `JodellCommandBuilder` | 组装 Jodell 的 3 寄存器指令（位置、力矩、速度） |

## 使用

```cmake
find_package(gripper_hardware_common REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC gripper_hardware_common::gripper_hardware_common)
```

```xml
<depend>gripper_hardware_common</depend>
```
