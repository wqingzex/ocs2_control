# robot_launch

**所有启动**都在这里：把"哪台机器人、什么硬件、什么末端"变成 URDF、控制器配置和节点，也包括相机、头显、手柄这些设备怎么起。设备本身在 `robot_hardware` / `robot_teleop`，设备的配置在 `robot_setup`。

| 包 | 作用 |
|---|---|
| [`launch_helpers`](launch_helpers/README.md) | 纯 Python 库：机器人和配置解析、URDF 生成、控制器拼装、设备启动参数和单元（`devices/`），不含 launch 文件 |
| [`robot_visualize`](robot_visualize/README.md) | 在 RViz 里看模型、拖关节，不带控制器 |
| [`robot_navigation`](robot_navigation/README.md) | Nav2 导航启动，以及地图、参数的查找 |

所有启动文件在 `robot_bringup`：整机（分体、全身、双臂），以及设备：`cameras.launch.py`（相机）、`joystick.launch.py`（手柄）、`dexcap.launch.py`（DexCap，需要先编译 `dexcap_teleop`），它们调用 `launch_helpers`。
