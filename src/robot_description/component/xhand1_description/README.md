# xhand1_description

XHand1 灵巧手的 URDF 和 `ros2_control` 描述。原始文件见 `src/XHAND1_URDF_ver 1.3`。

## 使用

`direction:=1` 为左手（默认），`-1` 为右手。

```bash
ros2 launch robot_visualize hand.launch.py hand:=xhand1                                       # 看模型
ros2 launch robot_startup hand.launch.py hand:=xhand1 type:=xhand1 direction:=1      # mock 控制
```

整机上用 `end_effector:=xhand1`，真机只支持外接 USB（RS485，`end_effector_hardware`），见 [m6_ccs](../m6_ccs_description/README.md#末端执行器)。
