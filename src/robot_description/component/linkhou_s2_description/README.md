# linkhou_s2_description

W2 的轮式底盘 Linkhou S2，两个型号：`linkhou_s2`（旧款）和 `linkhou_s2_v2`（默认）。

## 作用

`xacro/linkhou_s2.xacro`、`xacro/linkhou_s2_v2.xacro` 各定义宏 `linkhou_s2`；网格在 `meshes/`，雷达和超声波模型来自 [`sensor_models`](../sensor_models/README.md)。

整机在 [`fiveages_w2_description`](../../fiveages_w2_description/README.md) 里组装；单独看这个部件：

```bash
ros2 launch robot_visualize component.launch.py robot:=linkhou_s2 chassis:=linkhou_s2    # 旧款；默认是 linkhou_s2_v2
```

| 参数 | 含义 |
|---|---|
| `chassis` | `linkhou_s2_v2`（默认）或 `linkhou_s2` |
| `chassis_joints_movable` | 底盘关节是否可动，默认 `true`，静态预览可设 `false` |
| `collider` | `simple`（盒体、圆柱）或 `convex`（网格） |

`variant:=white` 引用的 `meshes/linkhou_s2_v2/white/` 目录目前没有，用这个外观会找不到网格。
