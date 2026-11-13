# description

机器人模型：URDF（xacro）、`ros2_control` 配置、OCS2 配置。**只放数据，不放启动逻辑**；启动由 `robot_launch/` 和各控制器的 launch 完成，它们按约定从 `<robot>_description` 包里取文件。

| 位置 | 内容 |
|---|---|
| [`fiveages_w2_description`](fiveages_w2_description/README.md) | W2 整机：组装、控制器配置、OCS2 `.info`、导航地图和 Nav2 参数 |
| [`component/`](component/README.md) | 可复用部件：臂、灵巧手、底盘、立柱、躯干、头、传感器，每个部件一个包 |

整机包直接放在本目录，部件包都放在 `component/` 里。

## 约定

- 机器人名 `foo` 对应包 `foo_description`，约定的文件：`xacro/robot.xacro`（运动学）、`xacro/ros2_control/robot.xacro`（加硬件）、`config/ros2_control/`、`config/ocs2/`。
- 同一份 xacro 会生成两份 URDF：控制用（手指、底盘可动）和 MPC 规划用（冻结），见 [`launch_helpers`](../robot_launch/launch_helpers/README.md)。
- 末端目前支持灵巧手 `linkerhand_l6`、`xhand1`，夹爪 `rg75`（Jodell RG75-300，用启动参数 `type:=rg75`；`config/robot.local.yaml` 的 `robot.hand` 还不识别它）。`changingtek_description` 里的 AG2F90 / AG2F120S 夹爪描述也在，但整机的类型解析还没接。
- 看模型：`ros2 launch robot_visualize humanoid.launch.py robot:=fiveages_w2`，详见 [`robot_visualize`](../robot_launch/robot_visualize/README.md)。
