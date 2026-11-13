# control_rviz_panels

RViz 面板插件：在 RViz 里切换控制器状态、控制夹爪和关节、查看全身控制状态。

## 作用

给 `arm_controller` 和 `whole_body_controller` 提供图形界面。面板只是客户端：发命令、显示控制器上报的状态，下拉框的选项由控制器通过 `WholeBodyCapability` 决定。

## 面板

| 面板 | 功能 |
|---|---|
| `FsmPanel` | 状态机切换 HOME / HOLD / OCS2，只显示当前状态下可用的按钮；全身控制时带身体模式下拉框，切换身体模式也在这里 |
| `GripperControlPanel` | 夹爪开合、目标百分比 |
| `JointControlPanel` | 在关节目标模式下调整关节位置，腰的升降和转动 |
| `WholeBodyCapabilityPanel` | 全身控制器支持的约束一览 |
| `TargetErrorDisplay`（RViz 的 Display，不是面板） | 左右臂目标与实际末端位姿的误差显示 |

状态转换：HOME → HOLD；HOLD → OCS2 或 HOME；OCS2 → HOLD。启动时处于 HOLD。

## 使用

```bash
ros2 run rviz2 rviz2
```

`Panels` → `Add New Panel`，选择 `control_rviz_panels/<面板名>`。控制器要先启动。

## 接口

| 方向 | 话题 | 类型 |
|---|---|---|
| 发布 | `/fsm_command` | `std_msgs/Int32`（1 HOME、2 HOLD、3 OCS2、4 MOVEJ） |
| 发布 | `mode_command` | `std_msgs/String`（身体模式） |
| 发布 | `<控制器>/target_percent` 等 | 夹爪、腰的命令 |
| 订阅 | `/fsm_command` | 同步其他来源（遥操作、VR）切换的状态 |
| 订阅 | WBC 能力与状态话题 | `WholeBodyCapability`、`WholeBodyCurrentState` |
