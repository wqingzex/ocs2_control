# w2_head_description

W2 的头（颈部两个关节和 Orbbec 相机）。

## 作用

`xacro/head.xacro` 定义宏 `head`，网格在 `meshes/head/`，相机模型来自 [`sensor_models`](../sensor_models/README.md)。

整机在 [`fiveages_w2_description`](../../fiveages_w2_description/README.md) 里组装；单独看这个部件：

```bash
ros2 launch robot_visualize component.launch.py robot:=w2_head variant:=stand
```

| 参数 | 含义 |
|---|---|
| `variant` | `stand` / `v1.5` 扩展头的关节限位（偏航 ±1.3963 rad，俯仰 −0.4014 ~ 0.5236 rad） |
| `collider` | `simple`（盒体、圆柱）或 `convex`（网格） |

