# changingtek_description

Changingtek（长坤）夹爪的描述：AG2F90-C、AG2F120S。来自真机部署包 `fa-deploy-ws` 的 `robot-descriptions-common/gripper/changingtek_description`。

目前整机的类型解析**只接了 Jodell RG75**（见 `jodell_description`）；这个包留在这里，是因为 `xhand1_description` 的法兰（`use_flange`）用到了它的网格 `meshes/AG2F90-C/visual/flange.glb`。要在整机上用 AG2F90 / AG2F120S，需要在 `m6_ccs_description/xacro/eef_type_resolver.xacro`、`eefs.xacro` 和 `ros2_control/end_effector_interfaces.xacro` 里照 RG75 的写法加上对应的类型。

| 文件 | 内容 |
|---|---|
| `xacro/AG2F90-C.xacro`、`xacro/AG2F120S.xacro` | 夹爪宏（`use_flange`、`pad`、`kinematic_only`） |
| `xacro/ros2_control/*.xacro` | 仿真接口宏和 USB（Modbus）接线的 `ros2_control` 系统 |
