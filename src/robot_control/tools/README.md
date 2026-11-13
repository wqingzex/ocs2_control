# tools

开发和调试用的独立程序，运行时不依赖它们，也不被别的包链接。

| 位置 | 作用 |
|---|---|
| [`whole_body_mpc_nodes`](whole_body_mpc_nodes/README.md) | 不接硬件、不经过 `ros2_control`，单独跑全身 MPC：MPC 节点、无硬件仿真、目标发布，用来验证 MPC 问题和模式切换 |
