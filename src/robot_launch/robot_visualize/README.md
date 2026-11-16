# robot_visualize

在 RViz 里看机器人模型、拖关节，不带控制器和硬件。

## 作用

检查 xacro 改动、看新末端的外观。启动 `robot_state_publisher`、`joint_state_publisher_gui`、`rviz2`。机器人和末端按 `robot:=<名字>`、`type:=<末端>` 指定，模型取自 `<名字>_description`。

## 使用

```bash
ros2 launch robot_visualize humanoid.launch.py    robot:=fa_w2 end_effector:=linkerhand_l6
ros2 launch robot_visualize manipulator.launch.py robot:=m6_ccs arms:=dual
ros2 launch robot_visualize hand.launch.py        hand:=xhand1
ros2 launch robot_visualize component.launch.py   robot:=fa_w2_body
ros2 launch robot_visualize visualize.launch.py   robot:=<名字>
```

常用参数：`type`、`left_end_effector` / `right_end_effector`（左右不同末端）、`collider`、`direction`、`skin`、`variant`、`chassis`。

`robot_app` 的"只看模型"也是调用这里的 `humanoid.launch.py` / `manipulator.launch.py`。RViz 配置在 `config/rviz/`，公共函数在 `robot_visualize/visualize.py`。
