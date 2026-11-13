# payload

机器人本体上外接的装置：相机、左右手（末端）。包括它们的配置、启动、设置窗口和负载辨识。

| 包 | 作用 |
|---|---|
| [`w2_payload`](w2_payload/README.md) | 机器人配置框架、Orbbec 相机的扫描与启动、相机和手的配置窗口、负载辨识 |

遥操作（[`fa_teleop`](../robot_teleop/fa_teleop/README.md)）和推理程序共用这里，这里不依赖它们。
