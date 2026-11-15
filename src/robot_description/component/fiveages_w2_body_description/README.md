# fiveages_w2_body_description

fiveages_w2 的移动腰段（躯干 4 个关节）。

## 作用

`xacro/body.xacro` 定义宏 `body`，网格在 `meshes/body/`，顶部接 [`fiveages_w2_torso_description`](../fiveages_w2_torso_description/README.md) 的胸部平台。

整机在 [`fiveages_w2_description`](../../fiveages_w2_description/README.md) 里组装；单独看这个部件：

```bash
ros2 launch robot_visualize component.launch.py robot:=fiveages_w2_body
```

| 参数 | 含义 |
|---|---|
| `variant` | 外观变体 |
| `collider` | `simple`（盒体、圆柱）或 `convex`（网格） |

