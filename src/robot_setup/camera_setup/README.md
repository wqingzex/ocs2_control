# camera_setup

Orbbec 相机的**配置**：头部 Gemini 335Lg，左右腕 Gemini 305。依赖 [`robot_config`](../robot_config/README.md) 的配置框架。相机怎么**启动**在 `robot_launch`（见下）。

```bash
./scripts/setup.sh cam      # 弹窗：按实时画面分配头部、左右腕，设置旋转，显示帧率，逐项确认
```

旋转角度保存到 `cameras.<位置>.rotation`，启动时按它生效。左右以机器人自身方向为准。

窗口发现旧的采集进程时，会列出设备和 PID，询问是否停止并重启预览；只有明确确认才停止，运控进程不在清理范围内。容器里看不到的宿主机进程需要先在宿主机上停。

多台相机要顺序启动：驱动用共享内存里的互斥锁串行初始化设备，只重试 5 次。`start.sh up` 和设置窗口的预览都等每台驱动日志里出现 `Initialize device cost` 后再起下一台。

## 组成

| 文件 | 内容 |
|---|---|
| `discovery.py` | **找设备**：USB 扫描、位置推荐、旧采集进程识别、画面订阅与帧率 |
| `session.py` | **配置步骤**：配置会话、预览、窗口卡片、`setup.sh cam` 入口 |

## 启动相机（在 robot_launch）

| 内容 | 位置 |
|---|---|
| 配置转驱动参数 `camera_specs`、单元定义 `camera_units` | [`launch_helpers/devices/camera.py`](../../robot_launch/launch_helpers/launch_helpers/devices/camera.py) |
| 启动文件 | `ros2 launch robot_startup cameras.launch.py config:=/绝对路径/config/robot.local.yaml [only:=camera_head]` |

设置窗口的预览会调用上面的启动文件，所以 `camera_setup` 依赖 `launch_helpers` 和 `robot_startup`，反过来不依赖。
