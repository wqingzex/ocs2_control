# wuji_hardware

舞肌 **Wuji Hand 2** 的 ros2_control 硬件接口（以太网），基于官方 C SDK `libwuji_sdk_c`。从 ctrl 的 `wuji_ros2_control` 搬来，改了包名和插件名，去掉了依赖旧启动包的两个独立 launch。

插件名：`wuji_hardware/WujiHand2Hardware`

## 数据接口

20 个关节，顺序拇指 → 小指（与 `wuji_description` 一致）。只导出 **position 命令**；发给手的 `joint_command` 里 velocity / effort 填 0。

| 方向 | 接口 | 单位 |
|------|------|------|
| 命令 | `position` | rad |
| 状态 | `position` | rad |
| 状态 | `velocity` | rad/s |
| 状态 | `effort` | 固件上报值 |

`mit_kp` / `mit_kd` / `effort_limit` 可以热调，走的是硬件节点参数（`ros2 param set` / rqt），见下文「硬件接口参数」。

## SDK

用 `src/third_party/wuji-sdk-c/` 里固定版本的官方发布包（`wuji-sdk-c-<版本>-<x86_64|aarch64>-linux-gnu.tar.gz`），CMake 配置时解到编译目录，不联网；运行时库装在插件旁边，靠 `$ORIGIN` 找到。换版本：把新的两个发布包放进 `third_party/wuji-sdk-c/`，改 `CMakeLists.txt` 的 `WUJI_SDK_VERSION`；临时用本机别的 SDK：`--cmake-args -DWUJI_SDK_ROOT=<含 include/ 和 lib/ 的目录>`。

## 使用

和其他末端一样，在 `robot.local.yaml` 里选：

```yaml
robot:
  end_effector: wuji_hand2
  end_effector_bus: ethernet
```

URDF 里的 system 由 `wuji_description` 的 `wuji_hand2_ethernet_side_system` 生成，经 `m6_ccs_description` 的 `external_ee_systems` 接进整机和双臂。

## 设备连接（真机）

手是静态 IP，不出 DHCP。出厂左手 `192.168.1.110`、右手 `192.168.1.111`。工控机网卡须与手同一网段（改过 IP 则以当前网段为准）。ROS 按 `hand_side` 扫描认边，**不要**把 IP / SN 写进 yaml。

### 改到 192.168.2.x 网段

出厂是 `192.168.1.110` / `.111`（掩码 `255.255.255.0`）。整机上天机臂默认 `192.168.1.190`，已占用 `192.168.1.x`；手若也留在 1. 网段，会和臂抢同一网段（工控机两块网卡也不能都配成 `192.168.1.x`）。现场把手改到 `192.168.2.x`，对应网卡走 2. 网段。

yaml **仍然不写 IP**；网卡与手同一网段后继续扫描 + `hand_side`。不要把手设成 `.0` 或 `.255`。先停掉运控（`./scripts/start.sh` 启动的控制器）。

**推荐：Wuji Studio**

1. 网卡先留在 `192.168.1.x`，打开 Studio，连上手。
2. **Device Tools → Modify Device IP**：左 `192.168.2.110`、右 `192.168.2.111`。设备会重启。
3. 把网卡改到 `192.168.2.x`（不要和手撞 IP），Studio 或 `wuji devices` 能扫到即可。
4. 之后照常启动运控，不用改 description。

**无 Studio：官方 Python SDK**（须先在 `192.168.1.x` 连上；`set` 后必须 `reboot` 才生效）

```python
from wuji_sdk import SdkManager, Handedness
manager = SdkManager.instance()
hand = manager.connect(handedness=Handedness.Left, device_name="left")  # 右手用 Right
hand.ip().set("192.168.2.110")  # 右手 .111
hand.reboot()
manager.disconnect_all()
```

等约 8 秒，把网卡改到 `192.168.2.x`，再 `wuji devices` 确认。详见官方 [ip（SET/GET）](https://docs.wuji.tech/docs/zh/wuji-hand/latest/sdk-reference/)。

要同时查看 SN 和 `IP:端口`（调试用，不要填进 yaml）：

```python
from wuji_sdk import SdkManager
manager = SdkManager.instance()
for d in manager.scan():
    print(f"SN={d.sn}  type={d.device_type}  address={d.address}")
```

也可 `pip install wuji-cli` 后执行 `wuji devices` / `wuji ping`。只扫描、不要再 `connect()`。

**与官方 SDK 真机遥操互斥：** 不要和官方 retarget / 真机遥操例程（如 `examples/python/retargeting/1.teleop_real.py`）同时跑本包 HI，避免双写 `joint_command`。真机发送失败会 disable 电机并退出，重新启动运控即可。

### 硬件接口参数

启动时用 URDF 种子：`mit_kp=3.0`、`mit_kd=0.05`、`effort_limit=1.5`。运行中用 `ros2 param set` / rqt 改对应硬件节点（名字与 URDF `<ros2_control name="...">` 一致）。不要写进 `robot.local.yaml`；重启后回到种子。

```bash
# 整机左手示例；先 ros2 node list 确认
ros2 param set /wuji_hand2_left_system mit_kp 4.0
ros2 param set /wuji_hand2_left_system mit_kd 0.08
ros2 param set /wuji_hand2_left_system effort_limit 1.5
```

rqt：Configuration → Parameter，选该硬件节点。

- 单手台架：`/wuji_hand_system`
- 双手台架 / 整机：`/wuji_hand2_left_system`、`/wuji_hand2_right_system`

改的是固件 MIT / 电流限，不是 `joint_command` 里的 kp/kd。上层仍只发 position。热调不写回 xacro，重启回到 URDF 种子 `3.0` / `0.05` / `1.5`。ROS 侧只拒 NaN/Inf/负数；固件 ceiling 未公布，超限 SET 失败并保持旧值。

| 参数 | 初值 | 来源 |
|------|------|------|
| `hand_side` | left / right | URDF（扫描认边） |
| `mit_kp` / `mit_kd` | `3.0` / `0.05` | URDF 种子；运行中 rqt / `ros2 param set` |
| `effort_limit` | `1.5` | URDF 种子；运行中同样热调 |

