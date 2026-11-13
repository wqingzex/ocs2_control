# sensor_models

传感器的三维网格（`.glb`），供各机器人的 xacro 引用，只有数据。

| 目录 | 模型 |
|---|---|
| `livox/` | `mid360` |
| `orbbec/` | `336`、`336L` |
| `robosense/` | `airy` |
| `ultrasonic/` | 超声传感器 |
| `linkhou/` | `ds51` |

在 xacro 里用 `$(find sensor_models)/meshes/<目录>/<文件>` 引用。
