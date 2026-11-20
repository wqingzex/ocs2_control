# docs

| 文档 | 内容 |
|---|---|
| [`使用手册.md`](使用手册.md) | 首次部署、日常启动、网页控制台（启动与功能）、配置、遥操作（XR 头显按键、坐标对齐、无头显联调）、相机、录制回放、故障排查、收集日志、真机逐步验证 |
| [`robot_control_suite控制原理.md`](robot_control_suite控制原理.md) | 运控原理：启动流程、状态机、MPC / MRT、控制器与驱动的调用关系、数值逆解（DLS / BFGS）、与 Marvin SDK 的关系 |
| [`故障码.md`](故障码.md) | 故障码：每个故障属于停机类还是监控类、系统现在怎么处理、先查什么（由 `src/robot_utils/config/fault_codes.yaml` 生成，不要手改） |
| [`待办.md`](待办.md) | 还没做、没验证、没查实的事，按设备和主题分节 |
| `images/` | 各包 README 引用的图片 |

日志的写法、记录点和设计见 [`src/robot_utils/README.md`](../src/robot_utils/README.md)。各目录自己的说明在该目录的 `README.md`，入口见根目录 [`README.md`](../README.md)。
