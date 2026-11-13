# whole_body_mpc

轮式人形双臂的 MPC 问题定义（`WholeBodyInterface`），复用 `ocs2_mobile_manipulator` 的动力学、运动学和约束。

## 作用

`whole_body_controller` 用它构造全身控制的 MPC。提供：运行时切换的**模式**（躯干约束、双臂耦合、底盘锁定、单臂关闭）、双臂协同约束、躯干跟踪。

## 使用

```cpp
#include <whole_body_mpc/WholeBodyInterface.h>
using namespace ocs2::whole_body_mpc;

WholeBodyInterface interface(taskFile, libraryFolder, urdfFile);
auto refManager = interface.getReferenceManagerPtr();

// 运行时切换模式
refManager->setModeSchedule(ModeSchedule({t + 0.5}, {Modes::DUAL_COUPLED, Modes::DUAL_INDEPENDENT}));

// 设置目标（14 维或 21 维，见下）
refManager->setTargetTrajectories(targets);
```

问题的参数（模型坐标系、初始状态、`initialModeSchedule.modeSequence`、约束权重、DDP/MPC）在 `config/task.info`；W2 实际使用的 `.info` 在 `fiveages_w2_description/config/ocs2/`。

## 模式

模式编码为一个 `size_t`。配置文件和 `PRESET:` / `FULL:` 命令只认下面 6 个预设（`HM_*`）：

| 预设 | 含义 |
|---|---|
| `HM_LEFT_ARM` / `HM_RIGHT_ARM` | 只动左 / 右臂（底盘、躯干锁定） |
| `HM_DUAL_IDLE` | 默认安全模式：双臂，躯干和底盘锁定，互相独立 |
| `HM_DUAL_RELATIVE_FIXED_BASE` | 双臂，躯干相对约束（竖直），底盘锁定 |
| `HM_DUAL_BODY_TRACKING` | 双臂，躯干跟踪目标，底盘可动 |
| `HM_DUAL_RELATIVE_MOBILE_BASE` | 双臂，躯干相对约束，底盘可动 |

其他组合（如双臂耦合）用 C++ 的 `Modes::*` 或下面的局部命令。

**躯干约束 `BodyConstraintMode`**：`FREE` 自由；`RELATIVE` 软约束躯干相对底盘的 roll、pitch、x、y；`TRACKING` 跟踪目标中的躯干位姿；`LOCKED` 躯干关节速度锁定。**双臂 `BimanualMode`**：`INDEPENDENT` 各跟各的目标；`COUPLED` 满足相对位姿约束。

**WBC 的 `mode_command` 话题**（字符串，空格转下划线）：默认是局部更新，没提到的字段保持不变。

| 类别 | 关键字 |
|---|---|
| 底盘 | `BASE_LOCK` `BASE_UNLOCK` |
| 躯干 | `BODY_FREE` `BODY_RELATIVE` `BODY_TRACKING` `BODY_LOCK` `BODY_UNLOCK` |
| 双臂 | `ARMS_COUPLED` `ARMS_INDEPENDENT` |
| 单臂 | `LEFT_ARM_ENABLE` `LEFT_ARM_DISABLE` `RIGHT_ARM_ENABLE` `RIGHT_ARM_DISABLE`（关闭的臂不跟踪，关节速度锁定） |

加前缀 `PRESET:` 或 `FULL:` 加 `HM_*` 名，整体替换为该预设。

## 能力门控

可用的模式由 `.info` 在构造时决定，请求不支持的模式会被拒绝，初始模式自动降级：

| 能力 | 配置 | 缺失时 |
|---|---|---|
| 移动底盘 | `manipulatorModelType = 1` | 底盘解锁降为锁定 |
| 躯干相对 | `bodyRelative.activate` | 不可用 `RELATIVE` |
| 腰锁定 | `waistLock.activate = true` | `LOCKED` 降为 `FREE` |
| 双臂耦合 | `bimanualCoupling.activate = true` | `COUPLED` 降为 `INDEPENDENT` |
| 躯干跟踪 | `bodyTrackingEE.activate = true` | 不可用 `TRACKING` |

## 目标格式

14 维（未启用躯干跟踪）：左臂位置 `[0:3]`、姿态四元数 `[3:7]`，右臂位置 `[7:10]`、姿态 `[10:14]`。21 维（`bodyTrackingEE.activate = true`）：前 14 维相同，再加躯干位置 `[14:17]`、姿态 `[17:21]`。
