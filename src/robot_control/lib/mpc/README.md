# mpc

OCS2 的 MPC 部分，建立在 `ocs2_mobile_manipulator` 之上。

| 包 | 作用 |
|---|---|
| [`mpc_core`](mpc_core/README.md) | 共用运行时：MPC 任务基类、目标位姿管理、实时计时、可视化，被下面两个任务包继承 |
| `arm_mpc` | 双臂 / 单臂 MPC 任务：OCS2 接口、观测、策略、复位 |
| [`whole_body_mpc`](whole_body_mpc/README.md) | 轮式人形全身 MPC：模式系统、约束、`WholeBodyInterface` |

实际控制时，由控制器插件使用 `ArmMpcTask` / `WholeBodyMpcTask`，不经过 [`tools/whole_body_mpc_nodes`](../../tools/whole_body_mpc_nodes/README.md)（独立测试用的节点）。
