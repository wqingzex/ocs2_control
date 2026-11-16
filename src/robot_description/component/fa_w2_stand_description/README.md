# fa_w2_stand_description

fa_w2 的升降立柱（4 段）。

## 作用

`xacro/stand.xacro` 定义宏 `stand`，网格在 `meshes/stand/`。整机目前不使用立柱，只用来单独查看。

整机在 [`fa_w2_description`](../../fa_w2_description/README.md) 里组装；单独看这个部件：

```bash
ros2 launch robot_visualize component.launch.py robot:=fa_w2_stand
```

| 参数 | 含义 |
|---|---|
| `variant` | `v0` 锁定立柱升降 |
| `collider` | `simple`（盒体、圆柱）或 `convex`（网格） |

