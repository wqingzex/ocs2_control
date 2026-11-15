# robot_navigation

Nav2 导航启动，以及地图和 Nav2 参数的查找。

## 作用

`./scripts/start.sh nav` 用它：起 `map_server`、Nav2、RViz，配合 Isaac Sim 的真值定位和激光。

## 使用

```bash
ros2 launch robot_navigation navigation_isaac_gt.launch.py robot:=fiveages_w2
ros2 launch robot_navigation navigation_isaac_gt.launch.py robot:=fiveages_w2 map:=siemens_task2
ros2 launch robot_navigation navigation_isaac_gt.launch.py robot:=fiveages_w2 nav2_profile:=map_only
```

`map` 不填用找到的第一张；指定的地图找不到、或一张地图都没有时，启动报错并列出搜索过的包，不会悄悄换地图。

## 地图和参数从哪里找

地图在各包的 `config/maps/`，Nav2 参数在 `config/nav2/`，按顺序找：

1. 启动参数 `map_config_package` / `nav_config_package`
2. `<robot>_description/config/navigation.yaml` 里写的同名项
3. `<robot>_description`
4. 本包（只带通用的 `config/nav2/nav2_params_isaac_gt.yaml`，不带地图）

fiveages_w2 的地图和参数在 `fiveages_w2_description`。`nav2_profile:=map_only` 取 `nav2_params_isaac_gt_map_only.yaml`。
