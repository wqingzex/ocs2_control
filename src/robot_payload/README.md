# payload

和外部设备打交道的包，一个设备一个包，包里放该设备需要的全部东西（扫描、配置窗口、数据转换节点）。

| 目录 | 内容 |
|---|---|
| [`w2_payload/`](w2_payload/README.md) | W2 机器人上的设备：Orbbec 相机、LinkerHand 灵巧手，以及它们共用的配置框架 |
| [`input/`](input/README.md) | 操作员的输入设备：XR 头显、手柄、键盘，与机器人型号无关 |

遥操作应用（[`teleop_app`](../robot_teleop/teleop_app/README.md)）和推理程序共用这里，这里不依赖它们。
