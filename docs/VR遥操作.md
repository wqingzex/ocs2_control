# W2 VR 遥操作

Pico 头显 + XRoboToolkit App 遥操作天机 M6-CCS 双臂（末端 LinkerHand L6 / XHand1），同时启动 Orbbec 头部、腕部相机。全部代码在本仓库，不再依赖 fa-py-libraries。

## 1. 概览

```
Pico 头显（XRoboToolkit App）
   │ WiFi
   ▼
XRoboToolkit PC Service（RoboticsServiceProcess，本机）
   │ 本机 gRPC（127.0.0.1:60061）
   ▼
xr_teleop xr_bridge_node（C++，节点名 /xr_target_node）
   │ /teleop/head_pose /teleop/left_ee_pose /teleop/right_ee_pose /teleop/controller_state
   │ /teleop/thumbstick_axes /teleop/trigger_values /teleop/grip_values /teleop/connection_status
   ▼
teleop_command / VRInputHandler ──/left_target /right_target──▶ arm_controller（MPC）

Orbbec 335Lg（头）+ 305 ×2（腕）──▶ /camera_{head,left_wrist,right_wrist}/color/image_raw/compressed
```

`./scripts/start.sh` 在命令行按 `config/robot.local.yaml` 依次拉起：PC Service（未运行时）→ 运控 launch（含 teleop_command）→ xr_bridge → 相机，Ctrl+C 停止。双击 `W2控制台` 则打开网页控制台，在页面启停模块。每台相机是独立进程，某台相机故障不影响 VR 和运控。

与旧链路（fa-py-libraries）相比：

| 旧 | 新 |
|---|---|
| Python XRTargetNode + uv 环境 | C++ `xr_bridge_node`，直接调用 PC Service SDK |
| 网卡交互选择、按回车开始 | 无交互；按头显帧时间戳发布 |
| 断线后继续发最后一帧 | 300 ms 无新帧即停发，`/teleop/connection_status=false`，机械臂保持最后目标；PC Service 断开 3 s 后自动重连 |
| 相机由 camera_setup 脚本启动 | `cameras.launch.py`，参数与话题名不变 |
| 多个仓库、交互菜单 | 一个仓库、`./scripts/start.sh` 读配置 |

`/xr/*` 话题、消息类型、事件码与旧 Python 节点完全一致（测试见第 9 节）。

## 2. 首次部署：`./scripts/setup.sh`

```bash
git clone --recurse-submodules --shallow-submodules git@github.com:wqingzex/ocs2_control.git && cd ocs2_control
./scripts/setup.sh                 # 完整初始化；相机和手弹出本地配置窗口
./scripts/setup.sh cam             # 只重新扫描、配置相机；失败重试无需重做依赖和编译
./scripts/setup.sh cam --from      # 从相机步骤继续：相机 → 手 → XR 配对 → 最终检查
./scripts/setup.sh hand            # 只配置手
./scripts/setup.sh xr              # 只做头显配对验证
./scripts/setup.sh dyn             # 负载辨识：逐臂询问（默认不辨识），确认后写入 defaults.left/right_dyn_param
./scripts/setup.sh dyn --reset     # 恢复默认负载（清空两个覆盖值，旧值记在 logs/dyn_identify/）
./scripts/setup.sh --force         # 完整流程里也重新配置已配置的步骤
```

**有默认值或已配置的步骤不再询问**：完整流程开头先列出每步状态，只执行"需配置"的步骤：

| 步骤 | 不用配置的情况 |
|---|---|
| 相机 | 相机映射有确认记录且没变，或 `cameras.enable: false` |
| 手 | 左右手有确认记录且没变，或 `robot.hand: none` |
| 负载辨识 | 仿真；没有末端；`linkerhand_l6`（内置辨识值）；左右臂都已填写。`xhand1` 没有内置值，使用驱动默认值 0，要更准再运行 `./scripts/setup.sh dyn`。只填了一侧会提示补全 |
| XR | `xr.enable: false` 或已保存头显 ID |

单步命令（`./scripts/setup.sh cam`、`./scripts/setup.sh dyn` 等）永远重新配置。负载辨识向导的结果写入 `logs/dyn_identify/result_A.json`（A 左臂、B 右臂），写入配置前会显示新旧值并确认；`./scripts/start.sh dyn-identify` 仍是只显示结果、不写配置的原向导。

任意单步命令后加 `--from`，会按固定初始化顺序执行该步骤及后面的全部步骤。例如 `./scripts/setup.sh build-workspace --from` 从编译继续到最终检查；不加 `--from` 时仍只执行指定步骤。使用 `--from` 时只能指定一个起始步骤，相机失败时仍立即退出。

容器和物理机都执行完整七步向导，包括相机分配、手的接线和串口配置、XR 头显配对验证。容器仅跳过物理机的 udev、CAN 操作。相机或 XR 在配置中关闭时，也会询问是否启用并配置；只有明确选择不配置时才保持关闭。相机未发现或扫描失败时立即报错退出，不继续进入手和 XR 步骤。头显未连上会记为“未完成”，最终返回失败状态，不能当作已经配置成功。容器中使用真实相机和头显仍需要相应的设备访问及网络连通。无需记住初始化的分步命令；原分步命令仍兼容维护脚本。

| 步骤 | 做什么 |
|---|---|
| `deps` | 子模块 + `rosdep install`（`rosdep update` 网络超时会自动重试） |
| `host` | `./scripts/setup.sh host`：Orbbec udev 规则、腰部 CAN 免密 sudo、PC Service deb（需要 sudo） |
| `build-workspace` | 增量编译当前仓库的整个 ROS 2 工作区：运控、硬件驱动、机器人模型、相机、XR 和控制台；复用已有结果，跳过 ocs2_ros2 里用不到的示例包。只编译，不执行设备配置、配对或启动运控。`teleop_app` 会生成双击入口 `W2控制台`。`--minimal` 可缩小为 W2 必需的包及其依赖 |
| `cam` | 扫描后弹出本地窗口，显示各设备实时画面、型号、序列号与端口；按画面分配头部 / 左腕 / 右腕，可停用、交换左右腕、设置 0/90/180/270° 旋转。所选画面均正常且逐项确认位置、方向后，才保存序列号、端口与旋转 |
| `hand` | 弹出本地窗口选择 L6 / XHand1、usb / 485 接线和左右设备；优先使用稳定串口路径。分别点击“开合测试”、核对实物并确认左右后保存。开合测试由用户点击并确认后执行，不启动整机运控 |
| `xr` | 默认选择有线网卡，也可选 Wi-Fi；显示所选网卡的 IPv4 地址。临时启动 PC Service + xr_bridge，不限时等待头显追踪数据及设备 ID，确认绑定后写入 `xr.device_id`；Ctrl+C 可取消，结束后只停止自己启动的进程 |
| `check` | 只做最终配置检查，不启动模块 |

相机和手的初始化由 `setup.sh` 直接弹出 Qt 窗口完成，无需打开 Web 控制台。左右以机器人自身方向为准。相机窗口可通过遮挡左右腕镜头核对位置；旋转应用于预览与最终发布的画面。检测到旧采集进程时会列出设备与 PID，询问是否停止；明确确认后只停止对应的相机进程，并自动重启预览。设备占用或无最新画面时不能确认保存。容器不可见的宿主机采集进程需先在宿主机停止，不更改进程隔离或停止运控。

这些通用功能位于 `src/robot_payload` 下：`w2_payload/` 里的 `common`（包名 `w2_payload`）、`w2_camera`、`w2_hand` 三个包，遥操作与推理共用配置、设备识别和相机 launch，不依赖 `teleop_app`。推理可使用 `w2_payload.config.load` 读取同一份配置，并用 `ros2 launch w2_payload cameras.launch.py config:=...` 启动相机。`teleop_app` 只依赖 `w2_payload`（在 package.xml 中声明），`w2_payload` 不知道 `teleop_app`；`xr / joystick / navigation` 配置段由 `teleop_app` 管理，XR 配对仍在遥操作模块。

手部只在点击“开合测试”并确认后才连接设备并动作：食指、中指、无名指、小指慢速运动到行程约一半（L6 寄存器 255 为张开、130 为半握；已经偏握时改为张开），再回到测试前的姿态，拇指不动；任何一步出错都会先写回原姿态。XHand1 的反馈请求使用与现有驱动一致的零力矩、禁用模式帧。485 模式还需确认原运控已停止。485 的左右臂接口固定，接反时调整实物接线。关闭窗口或 Ctrl+C 时释放本次使用的资源，保留旧配置；保存时再次读取左右手反馈（只读，不动作），断连不会写入确认结果。左右手都完成开合测试后才允许保存。

确认结果保存在同一文件的 `setup.cameras` / `setup.hand` 中，并绑定设备映射。修改序列号、旋转、接线或串口后，旧确认失效，`./scripts/setup.sh check` 会显示需要重新确认。`--yes` 只能复用已有确认的配置，不能替代首次画面或实物确认。物理机需桌面显示，容器需映射显示与对应 USB / 串口设备。

- `setup.sh host` 只放行 `ip link set can0 down/up ...` 两条命令（`/etc/sudoers.d/fa-w2-can`，写入前用 `visudo` 校验）。arm64 无桌面环境时可用 `XRT_PCS_HEADLESS=1 ./scripts/setup.sh host`。
- **xr_bridge 在编译时需要 PC Service 的 SDK**（`/opt/apps/roboticsservice/SDK`，或设置 `XRT_SDK_DIR`）。没装 PC Service 时 `xr_teleop` 仍能编译，但不会生成 `xr_bridge_node`，启动时会报错提示。
- 头显端：安装 XRoboToolkit App，与机器人处于可互通的局域网（电脑可用网线或 Wi-Fi），在 App 里选择机器人地址连接 PC Service。

XR 的 `network_mode` 默认是 `ethernet`（网线），也支持 `wifi`。这里指电脑/机器人接入局域网的网卡；Pico 与其处于可互通的网络即可，电脑可以插网线、Pico 使用 Wi-Fi。初始化保存 `network_interface` 和头显 `device_id`，IP 按当前网卡重新读取。PC Service 自身负责局域网服务广播，头显 App 首次仍需选择电脑，配对以收到有效头部或手柄追踪数据及设备 ID 为准。头显端的连接及数据发送入口以所装 XRoboToolkit App 版本为准，企业版 PICO 的设备型号本身不能确定 App 界面。公开 SDK 没有按 ID 强制连接或唤醒头显的接口。

[官方示例界面](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/blob/main/Docs/ui.png)展示了 Tracking 中的 Head、Controller 以及 Data & Control 下的 Send。配对时至少勾选 Head 或 Controller 一项，并开启 **Data & Control 区域的 Send**，向电脑发送追踪数据；本次无需开启 Data Collection 的采集选项。Controller 是手柄追踪，Hand 是裸手追踪。若实际 App 没有这些选项，应先核对 App 名称、版本和界面，避免套用示例按钮路径。

之后 `start.sh` 自动启动 PC Service 和桥接节点，监听设备事件，只接收绑定 ID 的追踪数据；其他头显不会接管控制。断线后保持目标，等待同一头显恢复。`/teleop/device_id` 发布当前接收数据的头显 ID。更换头显需要在 XR 初始化中选择重新绑定。

容器桥接网络下，容器 IP 和发现广播不一定能从 Pico 访问；需要另外配置可达的服务网络。ROS 的 `ROS_LOCALHOST_ONLY` 只影响 ROS 通信，不会替代 Pico 到 PC Service 的网络连接。

Linux 上的 XR 验证容器可使用 Docker `--network host`，使 PC Service 使用物理机的局域网地址和广播。保持独立的 `ROS_DOMAIN_ID`，启动前确认 PC Service 的 gRPC/设备连接端口未被本机其他服务占用。主机网络模式不使用 Docker `-p` 端口映射，网页控制台应选择空闲端口。此方式不修改物理机的网络配置或防火墙。

## 3. 日常启动与配置

### 3.1 启动（`./scripts/start.sh`）

每个部分是一个**独立进程、独立日志**的模块，出问题直接看对应模块：

| 模块 | 内容 | 依赖 |
|---|---|---|
| `pc_service` | XRoboToolkit PC Service（只停止自己启动的） | — |
| `control` | 运控 launch（控制器 + teleop_command，`launch_mode:=control_only`，不带 RViz） | — |
| `joystick` | 手柄遥操作（`joystick_teleop`）；**按需**，和 rviz 一样不会被"全部启动"带起，也不会自动启动运控 | `control` |
| `rviz` | RViz（同一运控 launch 的 `launch_mode:=rviz_only`，带 FSM/手/关节面板）；**按需**：`up` 和"全部启动"不启动它，需要时单独 `./scripts/start.sh up rviz` 或在控制台点启动；窗口出现在机器人屏幕上 | `control` |
| `xr` | xr_bridge_node（节点名 `/xr_target_node`） | `pc_service` |
| `camera_head` / `camera_left_wrist` / `camera_right_wrist` | 每台相机一个驱动进程 | — |

```bash
./scripts/start.sh                         # 前台启动全部模块，Ctrl+C 停止（运控先切 HOLD）
```

以下分步命令仅用于脚本和终端维护，日常使用不需要：

```bash
./scripts/start.sh config/other.yaml        # 用其他配置文件前台启动模块
./scripts/start.sh up [模块...]            # 后台启动，自动先启动依赖
./scripts/start.sh down [模块...]          # 停止；停一个模块会先停依赖它的模块
./scripts/start.sh restart camera_head
./scripts/start.sh status --hz             # 状态、PID、运行时长、话题频率、日志里最近一条错误
./scripts/start.sh logs xr -f              # 日志：logs/<模块>/latest.log（每次启动一个带时间戳的文件）
./scripts/start.sh check [config]          # 只校验并打印将要启动的内容
./scripts/start.sh -c config/other.yaml status   # 用其他配置文件执行子命令
```

**停止运控时**：FSM 不在 HOLD（例如 OCS2）会先询问，确认后发 `/fsm_command 2`、等 FSM 回到 HOLD 再停；5 s 内没回到 HOLD 会再次询问。`--yes` 跳过询问（仍然先切 HOLD）。模块状态保存在 `run/`，`start.sh` 退出或控制台重启都不影响已启动的模块。

旧的"一个 launch 起全部"仍可用：`ros2 launch teleop_app teleop.launch.py config:=config/robot.local.yaml`。

### 3.1.1 导航、负载辨识、手柄遥操作

```bash
./scripts/start.sh nav --list                  # 列出可用地图（默认地图带标记）
./scripts/start.sh nav [--map ID] [--map-only] # 前台启动导航，Ctrl+C 停止；不写 --map 用配置里的或默认地图
./scripts/start.sh dyn-identify [--reuse-noload]   # 机械臂负载辨识向导（交互式，需要终端；数据和日志在 logs/dyn_identify/）
```

- 导航需要 `nav2_bringup`（`./scripts/setup.sh deps` 会装；缺少时命令会直接提示）。
- 负载辨识：`--reuse-noload` 复用上次空载数据，直接做带载测试；不加会询问。向导会反复提问，所以只能在终端里运行。
- **手柄遥操作**在 Web 控制台的"手柄遥操"卡片里：选手柄（自动 / jsN）→ 启动；需要运控已在运行；也可以 `./scripts/start.sh up joystick`。

### 3.2 Web 控制台

```bash
# 日常双击仓库根目录的 W2控制台，即可打开网页界面。
./scripts/start.sh console --open  # 对应的终端命令（可选）
./scripts/start.sh console start|stop|restart|status   # 后台启动 / 停止 / 重启 / 查看（默认端口 8765）
# 如需指定端口：./scripts/start.sh console --open --port 8765
# 让其他电脑能访问（见下面“网络”）：./scripts/start.sh console --open --host 0.0.0.0
```

双击打开（不用敲命令）：

- **机器人本机**：正常编译后，直接双击**仓库根目录的 `W2控制台`**。控制台没运行就在后台启动（日志 `logs/console/`），然后用默认浏览器打开控制台；已经在运行就直接打开。页面里的“全部启动”按钮用于启动运控、VR 和相机。
- `W2控制台` 是原生可执行文件，随 `teleop_app` 正常编译自动生成，并具有执行权限；x86_64 和 ARM64 机器各自编译生成本机架构的同名文件，无需安装桌面快捷方式。它根据自身位置调用 `start.sh console --open`，可随整个仓库移动，不能单独移出仓库。该文件属于编译产物，不提交到 Git。
- **4090 / 其他电脑**：控制台默认只监听本机，要先在机器人上用 `--host 0.0.0.0` 启动（或用下面的 SSH 端口转发）。启动后浏览器直接打开 `http://机器人IP:8765/`（页面顶部"其他电脑访问"里也列了各网卡的地址），可以存书签。机器人防火墙要放行 8765 端口。

- **控制模式**：HOLD / HOME / OCS2 / MOVEJ / 切换姿态（Home ↔ Rest），可切换的规则与 RViz 的 FSM 面板相同（HOLD 下可切 HOME / OCS2 / MOVEJ，其他状态只能回 HOLD，切换姿态只在 HOME 下可用），服务端会再校验一次；HOME 和切换姿态会先弹窗确认。
- **打开 / 关闭 RViz**：在控制模式区域。RViz 是独立模块（`launch_mode:=rviz_only`），窗口出现在机器人屏幕上；它只订阅话题、面板只在点按钮时才发命令，打开或关闭都不影响遥操作和推理控制。运控没启动时按钮不可用。
- 模块列表：启动 / 停止 / 重启 / 日志，状态每秒刷新；顶部显示 FSM 与 VR 连接；"全部启动 / 全部停止"。
- **停止运控**：FSM 不在 HOLD 时弹窗确认，确认后先切 HOLD 再停；切不到 HOLD 会拒绝停止。
- 相机设置：扫描设备（先停掉运行中的相机）→ 为头部 / 左腕 / 右腕选择序列号和旋转 → 保存到配置（保留注释）；可一键对调左右腕。
- 预览：运行中的相机直接显示画面（compressed 话题的 JPEG 原样转发，不解码，每路 ≤10 Hz，页面隐藏时停止拉流）。
- 网络：默认只监听本机（`127.0.0.1`）。没有登录、明文 HTTP，而且能切换 HOME / OCS2 / MOVEJ 让机械臂运动。从 4090 访问推荐 SSH 端口转发：`ssh -L 8765:127.0.0.1:8765 <机器人>`，再打开 `http://127.0.0.1:8765/`；确实要直接访问，用 `--host 0.0.0.0` 启动，并且只在机器人内网使用。
- 安全：没有登录，也没有 token。控制台默认只监听本机，这是唯一的访问限制；用 `--host 0.0.0.0` 启动后，同一网络里能访问端口的人都能操作机器人，需要时用防火墙限制来源。关闭控制台不会停止已启动的模块。

配置文件是 `config/robot.local.yaml`（模板 `config/robot.template.yaml`），遥操作和推理共用：上半部分（`robot` / `xr` / `cameras` / `joystick` / `navigation`）决定启动什么，下半部分（`platform` / `defaults` / `hardware` / `control`）是这台机器的硬件参数，由 ROS 启动文件读取。启动部分只需写与默认值不同的字段，完整字段与默认值见 `src/robot_teleop/teleop_app/config/teleop.yaml`。启动前会做校验：未知字段、类型错误、xhand1 + 天机 485、相机序列号为空等都会直接报错，并指出是哪个字段。

| 字段 | 默认 | 说明 |
|---|---|---|
| `robot.entry` | `whole_body` | `whole_body` W2 全身（WBC）/ `split_body` W2 分体 / `dual_arm` M6 双臂 |
| `robot.hand` | `linkerhand_l6` | `none` / `linkerhand_l6` / `xhand1` |
| `robot.hand_drive` | `usb` | 接线方式：`usb`（485 转 USB 接电脑）/ `485`（接机械臂 485 接口）；xhand1 只能 `usb` |
| `robot.hardware` | `real` | `real` / `mock_components` / `isaac` |
| `xr.enable` / `xr.start_pc_service` | `true` / `true` | 启动 xr_bridge / 自动启动 PC Service（退出时只关闭自己启动的） |
| `xr.frame_timeout_ms` | `300` | 多久没有新帧判为断线 |
| `xr.control_scheme` | `legacy` | 按键方案，见第 5 节（旧字段名 `button_profile` 仍可用） |
| `xr.network_mode` | `ethernet` | `ethernet`（默认网线）/ `wifi`（无线），初始化选择电脑的局域网网卡 |
| `xr.network_interface` | `""` | 初始化保存的网卡名，IPv4 地址每次重新读取 |
| `xr.device_id` | `""` | 首次成功接收追踪数据后保存的头显 ID；日常启动只接受该设备 |
| `cameras.*` | 30 Hz，三台都开 | 按序列号绑定；`rotation` 取 0/90/180/270；`usb_port` 可选 |
| `joystick.device` | `""` | 手柄设备：空 = 自动，或 `/dev/input/jsN`（在控制台页面里选择会写回这里） |
| `navigation.*` | `robot: fiveages_w2`，`map: ""`，`map_only: false` | 导航：地图留空用默认地图；`map_only: true` 只用静态地图；`map_config_package` / `nav_config_package` 可选 |

## 4. 坐标对齐（取代"建立房间"）

控制只用增量：进入跟随时记录手柄与机械臂的基准位姿，之后把手柄位移映射到机械臂上。Pico 开机时"建立房间"确定的原点不影响增量，**真正影响的是方向**：

- 旧行为（`vr_align_head_yaw: false`）：位移按"进入跟随时手柄的姿态"解释，手柄握得倾斜时，往前推手臂会斜着走；方向还会随房间朝向变化。
- **头显 yaw 对齐**（`vr_align_head_yaw: true`）：进入跟随（legacy 按右摇杆 / simple 按下握把）时读取头显的水平朝向，**操作员面朝的方向 = 机器人 +X**，与房间原点和手柄倾角都无关。只改位移方向，姿态映射不变。

开关在机器人的 `teleop_command.yaml`（W2：`src/robot_description/fiveages_w2_description/config/ocs2/teleop_command.yaml`），默认关闭。**真机首次开启前先在 mock/Isaac 验证方向，并调低 `vr_pose_scale`。**

## 5. 按键

两处设置要一致：`config/robot.local.yaml` 的 `xr.control_scheme` 和 `teleop_command.yaml` 的 `vr_control_scheme`（`./scripts/start.sh check` 会提示不一致）。默认都是 `legacy`。

### 5.1 simple（10 个事件）

| 按键 | 功能 | 事件 |
|---|---|---|
| 右 A | HOLD ↔ OCS2 | 11 |
| 左 X | HOME / HOLD（HOME 下切换 REST） | 12 |
| 左 / 右握把**按住** | 对应手臂跟随；按下时重新记录基准（和头显朝向），松开即停。**进入 OCS2 时已经按着的握把不跟随**，要先松开再按（避免一进 OCS2 就动） | —（读 `/teleop/thumbstick_axes` z） |
| 左 / 右扳机 | 手按比例开合 | —（`/teleop/trigger_values`） |
| 左 Y + 右 B | 原事件 15（保持不变） | 15 |
| 左 Y + 右 A | 进入 MOVEJ（录制） | 54 |
| 右 B（单按） | 录制开始 / 停止，交替 | 50 / 51 |
| 左 Y 长按 0.8 s | 删除上一条录制 | 53 |
| 右摇杆按下 | 人工干预标记 | 52 |

镜像、摇杆微调、底盘切换在 simple 方案中不可用。录制相关事件码与 legacy 相同，外部录制程序无需修改。

### 5.2 legacy（与旧 xr_target_node 一致）

组合键在最后一个键的上升沿触发；握把、摇杆、Y、B 单独使用时在**松开**时触发，且若本次按下期间参与过组合键则不再触发。

| 事件 | 按键 | 事件 | 按键 |
|---|---|---|---|
| 2 / 5 | 左 / 右握把单击（摇杆模式切换） | 13 / 14 | 左 / 右握把 + 同侧扳机 |
| 3 / 6 | 左 Y / 右 B 单击（暂停 / 恢复） | 15 | 左 Y + 右 B |
| 4 | 右摇杆单击（STORAGE ↔ UPDATE） | 16 | 双握把 + 左 Y + 右 B |
| 7 | 左摇杆单击（镜像） | 20 | 双摇杆同时按下 |
| 9 / 10 | 左 / 右扳机 | 21–24 / 25–28 | 左 / 右握把 + 同侧摇杆 上下左右 |
| 11 / 12 | 右 A / 左 X（状态机） | 29 | 左握把 + 左摇杆按下 |
| 50 / 51 | 左握把 + 右 A / 右 B | 52 | 右握把 + 右摇杆按下 |
| 53 | 左握把 + 左 Y | 54 | 左 Y + 右 A |

## 6. 相机

| 位置 | 型号 | 驱动 launch | 话题 |
|---|---|---|---|
| 头 | Gemini 335Lg | `gemini_330_series.launch.py`，自动曝光 | `/camera_head/color/image_raw/compressed` |
| 左 / 右腕 | Gemini 305 | `gemini305.launch.py`，固定曝光 50 / 增益 16 | `/camera_{left,right}_wrist/color/image_raw/compressed` |

参数移植自 `fiveages-vla/camera_setup`（dev @04398b9c）：640×480 YUYV，关闭深度、红外、点云、TF 和 diagnostics。305 有两只彩色眼，旋转 90/270 时用左眼、否则用右眼，统一重映射到 `<name>/color`，话题名与旋转无关。USB 2 下 60 Hz 可能超带宽，默认 30 Hz。只想单独开相机：`ros2 launch teleop_app cameras.launch.py config:=config/robot.local.yaml`。

## 7. 录制与回放

```bash
./scripts/start.sh record [名称]                       # 只录关节：/joint_states、/fsm_state，保存到 bags/
./scripts/start.sh record 名称 --topics collect        # 数采：关节 + 已启用的相机压缩图像
./scripts/start.sh record 名称 --topics xr             # 原 XR 录制：/xr/*、目标与当前位姿、/fsm_state
./scripts/start.sh record 名称 --topics /a,/b          # 自定义话题
./scripts/start.sh bags                                # 列出录包：时长、大小、消息数、话题
./scripts/start.sh bags --delete 名称 --yes            # 删除（只能删录包目录下的直接子目录）
./scripts/start.sh play 名称 [--rate 0.5] [--loop] [--paused] [--start 30]
./scripts/start.sh play 名称 --xr                      # 把 /xr/* 回放给运控，同时运行 xr_stub --silent 提供 /xr_target_node
```

- 录包目录默认 `<工作区>/bags`，可用 `record.dir` 改；`record.extra_topics` 给每次录制追加话题。
- 录包名只能用字母、数字、`_ . -`；不写名称时自动用 `话题集_时间`。同名录包不会被覆盖。
- 磁盘剩余不足 2 GB 时拒绝开始录制。相机图像体积大，数采前先看剩余空间。
- 关节录包回放会重新发布 `/joint_states` 和 `/fsm_state`，用于绘图、核对数据；运控运行时不要回放它们（会和控制器自己发布的话题冲突）。

## 8. 无头显联调（xr_stub）

`VRInputHandler` 只有在 `/xr_target_node` 节点存在时才启用 VR，`xr_stub.py` 用同名节点发布 `/xr/*`：

```bash
./scripts/start.sh config/mock.yaml            # 前台启动仿真运控；配置使用 mock_components，关闭 xr 和 cameras
ros2 run xr_teleop xr_stub.py --script steps.json
```

时间线示例（秒，从 stub 启动算起；未写的字段保持不变）：

```json
[{"at": 9.0, "head_yaw_deg": 90},
 {"at": 10.0, "event": 11},
 {"at": 12.0, "left_grip": true},
 {"at": 13.0, "left_delta": [0.0, 0.1, 0.0]},
 {"at": 15.0, "left_grip": false}]
```

## 9. 测试

```bash
colcon build --packages-select xr_teleop teleop_app teleop_command --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select xr_teleop teleop_app teleop_command && colcon test-result --verbose
```

- **与旧 Python 节点逐帧一致**：`xr_teleop/test/data/*.csv` 由旧实现（fa-py-libraries `xr_target_node.py` @041159c，经 `XroboToolkitWrapper`）生成。按键 6000 帧覆盖全部 29 个事件码；位姿 300 帧，含 NaN / 零四元数的回退情况。
- 配置校验、launch 参数映射、相机参数（pytest）；simple 按键方案；JSON 解析；头显 yaw 对齐（`teleop_command/test/test_vr_alignment.cpp`）。
- 模块管理：依赖顺序、PID 复用识别、SIGINT→SIGTERM→SIGKILL、停运控前切 HOLD 的询问流程；初始化步骤（相机扫描解析、角色建议、写回配置保留注释、手与串口）；Web 控制台 API（token、409 确认流程、相机分配、MJPEG 流、静态文件越权）。
- docker mock 端到端：`./scripts/start.sh up/status --hz/restart/logs/down`（OCS2 下停运控先发 HOLD，无残留进程）；`./scripts/setup.sh --yes cameras hand vr summary`；控制台 uvicorn + rclpy（2.5 s 收到 25 帧 MJPEG，即 10 Hz 限速）。
- 已在 docker（ROS 2 Jazzy）中做过 mock 端到端验证：simple + 对齐、操作员朝 +90° 时，往正前方推 0.1 m，`/left_target` 沿机器人 +X 移动 0.1 m，松开握把后保持不动；legacy 映射与改动前一致。

## 10. 故障排查

| 现象 | 处理 |
|---|---|
| 不知道是哪个部分出问题 | `./scripts/start.sh status --hz` 看哪个模块停止 / 频率为 0 / 有最近错误，再 `./scripts/start.sh logs <模块>` |
| `xr_bridge_node is not built` | 未找到 PC Service SDK：`./scripts/setup.sh host`，然后 `./scripts/setup.sh build-workspace` |
| `RoboticsService did not start` | 桌面版 PC Service 需要 `DISPLAY`；无桌面用 headless 版，或手动运行 `/opt/apps/roboticsservice/runService.sh` |
| `/teleop/connection_status` 一直为 false | App 未连上 PC Service（同一网段、IP 正确）；`logs/xr/latest.log` 中 `SDK 事件:` 行显示服务端 / 头显事件 |
| 有 `/xr/*` 数据但手臂不动 | 状态机需在 OCS2（右 A）；legacy 需按右摇杆进入 UPDATE，simple 需按住握把；检查两处方案设置是否一致 |
| 相机 `Usb pal is not exist` / 打不开 | udev 规则未装或未重新插拔；序列号是否正确 |
| 腰部 CAN 初始化失败 | 未配置免密 sudo：`./scripts/setup.sh host` |

## 11. 尚未在真机验证

以下只做过编译、单元测试和 mock 验证，上真机前需逐项确认：

- PC Service + App + `xr_bridge_node` 的真实数据链路（开发机未运行 PC Service：只验证了节点启动、SDK 向 `127.0.0.1:60061` 发起连接、连接不上时每 3 s 重新初始化）；
- 头显 yaw 对齐与 simple 握把离合在真机上的手感，以及 `vr_pose_scale` 的取值；
- 三台 Orbbec 相机在 USB / GMSL 下实际出图（容器中没有 USB 设备）；
- Isaac 仿真入口（容器中没有 Isaac，topic_based 硬件因收不到 `/isaac/joint_states` 无法激活）；
- 腰部 CAN `restart-ms 100` 在真机上的效果；
- Web 控制台从另一台机器（4090）访问：容器中只验证了本机回环地址，跨机器需确认防火墙放行 8765 端口。

另外，`cgal5_colcon` 在编译时会从 GitHub 下载 CGAL 5.3 源码包，**离线环境无法编译**（原有问题，未改动）；网络不稳定时重跑 `./scripts/setup.sh build-workspace` 即可。
