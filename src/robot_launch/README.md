# robot_launch

启动相关的包：把"哪台机器人、什么硬件、什么末端"变成 URDF、控制器配置和节点。

| 包 | 作用 |
|---|---|
| [`launch_helpers`](launch_helpers/README.md) | 纯 Python 库：机器人和配置解析、URDF 生成、控制器拼装，不含 launch 文件 |
| [`robot_visualize`](robot_visualize/README.md) | 在 RViz 里看模型、拖关节，不带控制器 |
| [`robot_navigation`](robot_navigation/README.md) | Nav2 导航启动，以及地图、参数的查找 |

整机启动文件在 `robot_bringup`（分体、全身、双臂），它们调用 `launch_helpers`。
