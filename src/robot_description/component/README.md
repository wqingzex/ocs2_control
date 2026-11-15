# component

机器人的可复用部件，**每个部件一个包，全部同级**；整机在上一级（如 [`fiveages_w2_description`](../fiveages_w2_description/README.md)）里组装。

| 包 | 内容 |
|---|---|
| [`m6_ccs_description`](m6_ccs_description/README.md) | 天机 M6-CCS 双臂；末端接线与共享的 `ros2_control` 宏 |
| [`linkerhand_description`](linkerhand_description/README.md) | LinkerHand L6 灵巧手 |
| [`xhand1_description`](xhand1_description/README.md) | XHand1 灵巧手 |
| [`linkhou_s2_description`](linkhou_s2_description/README.md) | fiveages_w2 底盘 Linkhou S2（两个型号） |
| [`fiveages_w2_stand_description`](fiveages_w2_stand_description/README.md) | fiveages_w2 升降立柱 |
| [`fiveages_w2_body_description`](fiveages_w2_body_description/README.md) | fiveages_w2 移动腰段 |
| [`fiveages_w2_torso_description`](fiveages_w2_torso_description/README.md) | fiveages_w2 胸部平台 |
| [`fiveages_w2_head_description`](fiveages_w2_head_description/README.md) | fiveages_w2 头 |
| [`sensor_models`](sensor_models/README.md) | 传感器网格 |

单独看某个部件：`ros2 launch robot_visualize component.launch.py robot:=<包名去掉 _description>`，部件参数见各包 README。
