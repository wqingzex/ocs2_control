# dexcap_teleop

DexCap V4 外骨骼和手套作为遥操作输入：设备节点发布原始关节角，臂映射把外骨骼的左右臂转成双臂 14 个关节目标，手套映射驱动灵巧手或夹爪。从 ctrl 的 `dexcap_ros2_control` 和 `teleop_joint_mapper` 合并而来，后者现在是本包的 `arm_joint_mapper` 节点；话题和控制器名已对齐本仓库。

SDK 在 [`src/third_party/dexcap-sdk-cpp`](../../../third_party/README.md)（预编译，**只有 x86_64**，Jetson 上设备节点无法使用）。

## 节点

| 节点 | 输入 | 输出 |
|---|---|---|
| `dexcap_upper_body_node` | 外骨骼串口（默认 `/dev/ttyACM0`） | `/dexcap/waist`、`/dexcap/left_arm`、`/dexcap/right_arm`（`JointState`，默认 1000 Hz 上限） |
| `dexcap_glove_node` | 手套串口（默认左 `/dev/ttyUSB1`、右 `/dev/ttyUSB0`） | `/dexcap/<side>_hand`（21 个关节，40 Hz） |
| `arm_joint_mapper` | `/dexcap/left_arm`、`/dexcap/right_arm`、`/joint_states`、`/fsm_state` | `/arm_controller/joint_target_position`（14 维） |
| `dexcap_hand_mapper` | `/dexcap/<side>_hand` | `/<side>_hand_controller/joint_target_position`（LinkerHand L6，6 维） |
| `dexcap_gripper_mapper` | `/dexcap/<side>_hand`（捏合距离） | `/<side>_gripper_controller/target_percent` |

节点只在 SDK 数据时间戳变化时发布，不重复发旧帧。

## 启动

先用 `split_body` 或 `dual_arm` 启动运控（`whole_body` 下手臂归 `whole_body_controller` 管，臂映射不起作用），再启动 DexCap：

```bash
ros2 launch dexcap_teleop dexcap.launch.py                                    # 外骨骼 + 两只手套，臂映射
ros2 launch dexcap_teleop dexcap.launch.py left_eef:=hand right_eef:=hand     # 手套驱动 L6
ros2 launch dexcap_teleop dexcap.launch.py left_eef:=gripper right_eef:=gripper
ros2 launch dexcap_teleop dexcap.launch.py arm_mapper:=false                  # 只要原始 /dexcap/* 话题
ros2 launch dexcap_teleop dexcap.launch.py start_upper_body:=false \
    left_hand_serial_port:=/dev/ttyUSB0 right_hand_serial_port:=/dev/ttyUSB1  # 只开手套，端口按本次枚举
```

launch 启动设备节点前会检查串口是否存在、是否可读写，失败时直接报错。两只手套错峰 0.5 s 启动（SDK 同时连接两台设备会识别错乱）；日志里看到 `left glove polling configured at 40.0 Hz`、`right glove ...` 才表示两侧都在采样。

### 臂映射

`arm_joint_mapper` 把外骨骼关节按 `config/arm_joint_mapper.yaml` 的 `profiles.tianji.*`（符号、`π/2` 偏移、关节范围）映射成 M6-CCS 双臂的 14 个关节，做 `2π` 周期修正、Ruckig 插值，**只在 FSM 处于 MOVEJ（`4`）时**发布。某个关节落不进范围时只有这个关节回退到上一次的合法值。`/joint_states` 里要有左右臂 14 个关节，用来做插值起点。

插值已经在这里做了，臂控制器要透传，在 `robot.local.yaml` 里：

```yaml
control:
  patch:
    arm_controller:
      ros__parameters:
        movej_interpolation_type: none
```

### 手套映射到 L6

`dexcap_hand_mapper` 原本是给 LinkerHand O6 写的；O6 和 L6 的 6 个主动关节同名，这里默认值和 `config/hand_mapper_<side>.yaml` 的 `target_at_one` 已改成 L6 的关节上限（`0.99, 1.39, 1.26 ×4`），手套侧的标定值照搬。**映射还没在真机 L6 上验证过。**

## 串口

默认外骨骼 `/dev/ttyACM0`、左手套 `/dev/ttyUSB1`、右手套 `/dev/ttyUSB0`。两只 V4 手套的 USB ID 可能相同，`/dev/serial/by-id` 分不出左右，重新插拔后 `ttyUSB0` / `ttyUSB1` 也可能互换，启动前先看一下：`ls -l /dev/ttyACM* /dev/ttyUSB*`。一个设备不能被两个进程同时打开。

用户要在宿主机的 `dialout` 组里（`sudo usermod -aG dialout "$USER"`，重新登录）。在容器里运行时，容器内 `dialout` 的 GID 要和宿主机一致（`getent group dialout`、`ls -ln /dev/ttyACM0` 对比），否则组名相同也读写不了；不要用 `chmod 666`。

## 验证

```bash
ros2 topic hz /dexcap/left_arm
ros2 topic hz /dexcap/left_hand
ros2 topic echo /fsm_state
ros2 topic hz /arm_controller/joint_target_position
```

有输入但没有输出时检查：FSM 是否在 MOVEJ；`/joint_states` 里是否有 14 个臂关节；DexCap 输入是否超时（`input_timeout` 0.2 s）。
