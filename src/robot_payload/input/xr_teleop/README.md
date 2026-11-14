# xr_teleop

XR 头显（Pico + XRoboToolkit PC Service）的遥操作输入：把头显和手柄数据发布成 `/teleop/*`，由 `teleop_command` 的 `VRInputHandler` 接收。

## 组成

| 路径 | 内容 |
|---|---|
| `src/`、`include/xr_teleop/` | C++：`xr_bridge`（组件，节点 `/xr_target_node`）、SDK 客户端、位姿转换、按键编码 |
| `xr_teleop/pc_service.py` | 启停 PC Service（只停止自己启动的） |
| `xr_teleop/xr_network.py` | 列出可用网卡和 IPv4 地址，供配对时选择 |
| `xr_teleop/pairing.py` | 头显配对（`./scripts/setup.sh xr`）：选网卡、启动 PC Service 和桥、等头显连上、保存头显 ID |
| `xr_teleop/units.py` | 本设备的单元：PC Service（守护进程，由 `Driver` 管理）和 `xr` 桥，由 `teleop_app` 汇总 |
| `xr_teleop/config.py` | 配置的 `xr` 段：默认值和校验 |
| `xr_teleop/probe.py` | 只订阅 `/teleop/connection_status` 和 `/teleop/device_id` 的小探针，配对用 |
| `scripts/xr_stub.py` | 无头显联调，模拟 XR 数据：`ros2 run xr_teleop xr_stub.py [--script steps.json] [--silent]` |

编译 `xr_bridge` 需要 PC Service 的 SDK（`/opt/apps/roboticsservice/SDK`，或设置 `XRT_SDK_DIR`）；没有时只编译核心库和测试。用法、配对、故障排查见 [`docs/VR遥操作.md`](../../../../docs/VR遥操作.md)。

## 测试

`colcon test --packages-select xr_teleop`。C++ 位姿和按键的单元测试用 `test/data/*.csv` 作为与旧 Python 实现逐帧一致的基准。
