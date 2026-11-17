# xr_teleop

XR 头显（Pico + XRoboToolkit PC Service）的遥操作输入：把头显和手柄数据发布成 `/teleop/*`，由 `target_generator` 的 `XrInputHandler` 接收。

## 组成

| 路径 | 内容 |
|---|---|
| `src/`、`include/xr_teleop/` | C++：`xr_bridge`（组件，节点 `/xr_target_node`）、SDK 客户端、位姿转换、按键编码 |
| `xr_teleop/xr_network.py` | 列出可用网卡和 IPv4 地址，配对和 `start.sh check` 用 |
| `xr_teleop/config.py` | 配置的 `xr` 段：默认值和校验 |
| `scripts/xr_stub.py` | 无头显联调，模拟 XR 数据：`ros2 run xr_teleop xr_stub.py [--script steps.json] [--silent]` |

这个包只放头显本身（桥节点和协议）。PC Service 的起停、单元定义和就绪判断在 [`launch_helpers/devices/xr.py`](../../robot_launch/launch_helpers/launch_helpers/devices/xr.py)，头显配对在 [`robot_setup/xr_setup`](../../robot_setup/xr_setup/README.md)。

编译 `xr_bridge` 需要 PC Service 的 SDK（`/opt/apps/roboticsservice/SDK`，或设置 `XRT_SDK_DIR`）；没有时只编译核心库和测试。用法、配对、故障排查见 [`docs/使用手册.md`](../../../docs/使用手册.md)。

## 测试

`colcon test --packages-select xr_teleop`。位姿和摇杆、扳机、握把模拟量的单元测试用 `test/data/*.csv` 作为与旧 Python 实现逐帧一致的基准；按键编码（右 A = 1，左 X = 2，右摇杆按下 = 3，见 `docs/使用手册.md` 第 6.1 节）有单独的用例。
