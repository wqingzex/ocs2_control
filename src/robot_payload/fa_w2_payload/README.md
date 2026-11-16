# fa_w2_payload

W2 本体上外接的装置。一个设备一个 ROS 包，加上它们共用的配置框架：

| 目录 | 包名 | 作用 |
|---|---|---|
| [`common/`](common/README.md) | `fa_w2_payload` | 公共：机器人配置读取与校验、配置改写和确认记录、设置窗口框架、进程状态、负载辨识 |
| [`camera/`](camera/README.md) | `camera` | Orbbec 相机：扫描、驱动启动、画面与帧率、相机配置窗口 |
| [`end_effector/`](end_effector/README.md) | `end_effector` | 灵巧手：串口探测、开合测试、手配置窗口 |

依赖方向：`camera`、`end_effector` 依赖 `fa_w2_payload`（目录 `common/`），`fa_w2_payload` 不引用它们。

目录 `fa_w2_payload/` 本身不是包：colcon 找到一个包之后不会再往它里面找，所以三个包并列放在这里。
