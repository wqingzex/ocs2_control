# xr_setup

XR 头显的**配对**（`./scripts/setup.sh xr`）：选网卡、起 PC Service 和桥、等头显连上、保存头显 ID。头显本身（桥节点、协议）在 [`robot_teleop/xr_teleop`](../../robot_teleop/xr_teleop/README.md)，怎么**启动**在 `robot_launch`。

```bash
./scripts/setup.sh xr      # 提示在 Pico 的 XRoboToolkit App 里选的 PC Service 地址，等头显连上后询问是否绑定
```

## 组成

| 文件 | 内容 |
|---|---|
| `discovery.py` | **找设备**：监听头显连接状态和设备 ID 的探测节点 `XrProbe`、等追踪数据 `wait_for_headset` |
| `session.py` | **配置步骤**：选网卡、更换绑定、保存 ID、状态查询 `xr_status`、`setup.sh xr` 入口 |

启动用到的 PC Service 起停、单元定义和就绪判断在 [`launch_helpers/devices/xr.py`](../../robot_launch/launch_helpers/launch_helpers/devices/xr.py)，网卡枚举在 `xr_teleop.xr_network`。`xr_setup` 依赖它们，反过来不依赖。
