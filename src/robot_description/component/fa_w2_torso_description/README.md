# fa_w2_torso_description

fa_w2 的胸部平台，带前向 Orbbec 相机。

## 作用

`xacro/torso.xacro` 定义宏 `torso`，网格在 `meshes/torso/`，相机模型来自 [`sensor_models`](../sensor_models/README.md)。由 [`fa_w2_body_description`](../fa_w2_body_description/README.md) 引用。

整机在 [`fa_w2_description`](../../fa_w2_description/README.md) 里组装；单独看这个部件：

```bash
ros2 launch robot_visualize component.launch.py robot:=fa_w2_torso
```

| 参数 | 含义 |
|---|---|
| `variant` | 外观变体 |
| `collider` | `simple`（盒体、圆柱）或 `convex`（网格） |

