# XrInputHandler

`target_generator_node` 里处理 XR 手柄的部分：把 `xr_teleop` 桥节点发来的手柄位姿、按键和扳机，变成状态机命令、双臂目标位姿和手的开合。按键表见 [`docs/使用手册.md`](../../../docs/使用手册.md) 第 6.1 节。

## 输入和跟随

XR 输入来自 `/teleop/*` 话题（桥节点或 `xr_stub.py` 发布）。

开始跟随前的"输入可信"检查（`XrInputHandler` 的每臂输入状态）：该手柄 `xr_guard.stale_ms`（默认 200 ms）内有新位姿、已收到机器人当前末端位姿，两项都满足才开始跟随，否则打 `[TEL-003]`。跟随中输入停了，手臂停在最后一个目标上，没有看门狗和定时器（底盘遥控有，见下）。单帧的位置突跳由守卫丢弃（`[TEL-002]`，参数见 `config/default.yaml` 的 `xr_guard.*`）。每个通过守卫的帧都发布，不按距离量化。

## 按键

事件码来自 `/teleop/controller_state`（`std_msgs/Int32`，每帧都发，没按键时是 0），定义在 `xr_teleop/button_event_encoder.hpp`：

| 码 | 按键 | 处理 |
|---|---|---|
| 1 | 右 A | 状态机前进：HOME → HOLD → OCS2 |
| 2 | 左 X | 状态机后退：OCS2 / MOVEJ → HOLD → HOME；HOME 下切换 HOME ↔ REST |
| 3 | 右摇杆按下 | 只在 OCS2：STORAGE → UPDATE（记录基准，双臂开始跟随），UPDATE → STORAGE（停止）；底盘遥控开着时忽略 |
| 4 | 左摇杆按下 | 底盘遥控 开 ↔ 关（见下一节），开时手臂和头部停止跟随 |

其他值一律忽略。当前 FSM 状态来自 `target_generator_node` 转发的 `/fsm_command`（`fsmCommandCallback`）；离开 OCS2 时自动回到 STORAGE，再进 OCS2 要重新按右摇杆才跟随。

## 跟随（UPDATE）

进入 UPDATE 时记下手柄位姿和机器人末端当前位姿作为基准。之后每帧：

```
目标位置 = 机器人基准位置 + 位移坐标系⁻¹ × (手柄当前位置 − 手柄基准位置) × xr_pose_scale
目标姿态 = (手柄基准姿态⁻¹ × 手柄当前姿态) × 机器人基准姿态
```

位移和转动都在进入 UPDATE 时手柄的姿态坐标系里算，所以往前推的方向取决于按键时手柄指向哪里；头显位姿不参与控制。每个通过守卫的帧都发布。

## 扳机

`/teleop/trigger_values`（linear.x 左、angular.x 右，0 松开 ~ 1 按到底）按比例映射到 `/<控制器>/target_percent`（1 张开 ~ 0 闭合）。控制器来自 `hand_controllers` 参数，名字里带 `left` / `right` 的分别对应左右扳机；灵巧手和夹爪都支持 `target_percent`。

## 头部跟随

没有按键。手臂在跟随（UPDATE）、全身控制器的头部任务是 `HEAD_TRACKING`、已收到 `head_current_pose` 三条同时满足时，开始头部跟随：

```
头部目标位置 = 机器人头基准位置 + (头显当前位置 − 头显基准位置)
头部目标姿态 = 头显当前姿态 × 头显基准姿态⁻¹ × 机器人头基准姿态
```

基准在条件第一次满足时的第一帧头显上取。任何一条不满足就停止并丢掉基准，再满足时重新取，不会跳。目标过守卫（和手臂共用 `xr_guard.*`，`[TEL-002]`），头显位姿含 NaN 或零四元数的帧丢弃。头显位姿和机器人头位姿都在模型的 base 坐标系里，不做坐标系转换（和手臂不同：手臂的位移在手柄基准姿态的坐标系里算，头部直接加）。纯函数在 `head_follow.h`，`test/test_head_follow.cpp` 单测。

## 底盘遥控

**默认关**（`xr_base: false`）：关着时不创建 `base_velocity_target` 的发布者、不订阅摇杆、没有看门狗，左摇杆按下只记一条 INFO，手臂照常跟随，误按没有任何效果。要用时在 `target_generator` 配置里改成 `true`。

左摇杆按下开 / 关。开着时 `/teleop/thumbstick_axes`（`Twist`：`linear.x/y` 左摇杆，`angular.x/y` 右摇杆，前推是负数）的每一帧都变成底盘速度**参考**发到 `base_velocity_target`（相对名，和 `left_target` 一样落在控制器命名空间下）：

```
linear.x  = −左摇杆y × linear_scale      前进
linear.y  = −左摇杆x × linear_scale      向左平移（摇杆往右推 → 向右）
angular.z = −右摇杆x × angular_scale     逆时针为正（摇杆往右推 → 顺时针）
```

每个轴先限幅到 [−1, 1]；一帧里有非有限值，整帧按 0 发。摇杆回中就持续发 0。

**这个速度不直接给底盘**：全身控制器（MPC）订阅 `base_velocity_target`，在每次求解前把它写进目标轨迹的底盘输入（`SwitchedHumanoidReferenceManager::modifyReferences`），底盘输入代价把底盘拉向这个速度，和手臂跟踪、速度限幅一起折中，最后由 MPC 往 `/base_velocity_command` 发速度。所以这是**偏好**，推满摇杆不保证达到 `linear_scale`；要更“听话”就调小 `.info` 里底盘输入的 R。MPC 侧 `base_velocity_target_timeout_sec`（默认 0.2 s）内没有新帧就当 0。

**什么时候底盘才会动**：底盘必须被解锁（`BASE_*` 模式），而解锁要有外部 `world→base` TF，没有时底盘被硬锁，参考速度没有效果。去掉对 TF 的依赖见 `docs/待办.md`。

**能开**（`[TEL-004]`）：在 OCS2、200 ms 内有摇杆帧。**立刻发零速度参考并退出**：再按一次左摇杆、离开 OCS2、摇杆帧停了超过 `xr_chassis.stale_ms`（50 ms 一次的看门狗，头显断线和桥掉了都不会再有零帧）。退出后要重新按左摇杆。进入时如果手臂在跟随，先 `stopFollowing()`（头部一起停）。

没做：腰的升降 / 旋转（`waist_*_command` 只在 `body_joint_controller` 的 MoveJ 里响应，OCS2 里没作用）；节点自己挂掉时 MPC 侧超时后参考回到 0，但底盘驱动本身的指令超时在仓库外，没核实。

## 话题

| 方向 | 话题 | 说明 |
|---|---|---|
| 订阅 | `/teleop/thumbstick_axes` | 摇杆帧，只用于底盘遥控 |
| 发布 | `base_velocity_target` | 底盘速度参考（`Twist`），底盘遥控开着时每帧都发，退出时发 0；全身控制器订阅它 |
| 订阅 | `/teleop/head_pose` | 头显位姿，只用于头部跟随 |
| 订阅 | `head_current_pose` | 机器人头当前位姿（模型有 `headFrame` 才发） |
| 订阅 | `/whole_body_controller/current_state` | 只用 `head_state` |
| 发布 | `head_target` | 头部目标位姿（队列长度 1） |
| 订阅 | `/teleop/left_ee_pose` `/teleop/right_ee_pose` | 手柄位姿 |
| 订阅 | `/teleop/controller_state` | 按键事件码 |
| 订阅 | `/teleop/trigger_values` | 扳机按下深度 |
| 发布 | `left_target` `right_target` | 跟随时的目标位姿 |
| 发布 | `/fsm_command` | 状态机命令 |
| 发布 | `/<控制器>/target_percent` | 手 / 夹爪开合 |

## 参数（`target_generator.yaml`）

| 参数 | 默认 | 含义 |
|---|---|---|
| `enable_xr` | `true`（`config/default.yaml` 里为 `false`） | 是否创建 XrInputHandler |
| `xr_pose_scale` | `1.0` | 手柄位移缩放，建议 0.75~1.5 |
| `xr_base` | `false` | 底盘遥控总开关；关着时没有发布者，左摇杆按键无效 |
| `xr_chassis.linear_scale` | `0.1` | 底盘遥控，左摇杆推满的速度（m/s），占位值 |
| `xr_chassis.angular_scale` | `0.2` | 底盘遥控，右摇杆推满的转速（rad/s），占位值 |
| `xr_chassis.stale_ms` | `200` | 底盘遥控：摇杆帧超过这么久没来就停底盘并退出，也是开启时"摇杆输入要多新"，占位值 |

## 测试

`test/test_xr_input_handler.cpp`：在一个进程里起处理器和一个 `xr_target_node`，发按键、位姿和扳机，检查 `/fsm_command`、`left_target` 和 `target_percent`；头部用例再发 `/teleop/head_pose`、`head_current_pose` 和 `WholeBodyCurrentState`，检查 `head_target`。

**这些测试会发布 `/fsm_command`、`*_target`、`target_percent`、`head_target`，容器里有真机或仿真在运行时必须隔离 ROS 域再跑**，否则会直接发给它：`ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST colcon test --packages-select target_generator`，先 `ros2 node list --no-daemon --spin-time 4` 确认看不到任何节点。

## 停用的代码

`XrInputHandler.cpp` 的 `processButtonEvent` 里保留着旧方案"握把 + 扳机"在线校准 `xr_pose_scale` 的代码（已注释掉，没有按键触发）。需要在线调比例时再评估恢复。
