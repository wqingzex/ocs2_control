# wuji_description

舞肌 **Wuji Hand 2**（Beta 2）的 URDF 和 `ros2_control` 描述，20 个关节。从 ctrl 的 `robot-descriptions-common/dexhands/wuji_description` 搬来；插件名、控制器类型已经对齐本仓库（`wuji_hardware/WujiHand2Hardware`、`end_effector_controller/HandController`）。几何参考官方 [wuji-description](https://github.com/wuji-technology/wuji-description)。

## 使用

`direction:=1` 为左手（默认），`-1` 为右手。

```bash
ros2 launch robot_visualize hand.launch.py hand:=wuji type:=hand2                    # 看模型
ros2 launch robot_bringup hand.launch.py hand:=wuji type:=hand2 direction:=1         # mock 控制
```

整机上用 `robot.end_effector: wuji_hand2`、`robot.end_effector_bus: ethernet`；真机走以太网（`wuji_hardware`，扫描后按 `hand_side` 认左右，不写 IP），见 [wuji_hardware](../../../robot_hardware/wuji_hardware/README.md)。装到臂上的方式：

```xml
<xacro:WujiHand2 name="${side}_hand" direction="${1 if side == 'left' else -1}" use_mount="true"/>
```

| 文件 | 作用 |
|---|---|
| `xacro/hand2.xacro` | 模型（`WujiHand2` 宏） |
| `xacro/ros2_control/hand2.xacro` | 关节接口（`wuji_hand2_interfaces`） |
| `xacro/ros2_control/side_systems.xacro` | 整机用的以太网 system（`wuji_hand2_ethernet_side_system`，默认 `rw_rate` 200 Hz） |
| `config/ros2_control/templates/hand2.side.yaml` | 整机的手控制器模板（`launch_helpers` 的 `eef_control_registry.yaml` 指向它） |
| `config/ros2_control/hand2.yaml`、`hands2.yaml` | 单手 / 双手台架的控制器配置 |

## 关节

单手台架不带前缀；装在臂上时 `name:=left_hand` / `right_hand`，关节名为 `left_hand_<关节>`。

| Finger | Joints |
|--------|--------|
| Thumb | `thumb_cmc_flex`, `thumb_cmc_abd`, `thumb_mcp`, `thumb_ip` |
| Index | `index_finger_mcp_flex`, `index_finger_mcp_abd`, `index_finger_pip`, `index_finger_dip` |
| Middle | `middle_finger_mcp_flex`, `middle_finger_mcp_abd`, `middle_finger_pip`, `middle_finger_dip` |
| Ring | `ring_mcp_flex`, `ring_mcp_abd`, `ring_pip`, `ring_dip` |
| Pinky | `pinky_mcp_flex`, `pinky_mcp_abd`, `pinky_pip`, `pinky_dip` |

无名指去掉了官方名里的 `finger`（`ring_finger_*` → `ring_*`），避免 RViz 关节面板把它当成右手；SDK / 固件的关节顺序不变。

指尖 / 指尖传感器是固定坐标系，只用于 TF，不受控。

## 坐标系

| Link | Role |
|------|------|
| `hand_base` | External wrist for fa_w2 / dexhands; **+Z = finger extension** (same idea as LinkerHand O6) |
| `wrist` | Official beta2 wrist mesh + finger roots; fingers extend in **−Z** of this frame |
| `wrist_align` | Fixed `R_x(π)` mapping official −Z onto `hand_base` +Z |

Joint names, order, and limits are unchanged — simulation controllers and a future HardwareInterface still use joint-space angles (no sign flip for +Z). When mounting on an arm, calibrate `*_hand_base_joint` against this **+Z** `hand_base`.

- `direction:=1` left (default), `direction:=-1` right
- Single geometric tree + `direction` formulas (joint origins / axes)
- **Mesh:** left `.glb` + `scale="1 ${direction} 1"` (same mesh format family as LinkerHand / BrainCo). Index/middle/ring share `digit_*`; pinky keeps official `l_pinky_proximal` / `proximal_abd` / `middle` (distal + tip_sensor still share `digit_*`).
- `thumb_mcp` / `pinky_mcp_flex`: forced mirror for joint-space control (~1–3 mm fingertip FK vs official right URDF)

