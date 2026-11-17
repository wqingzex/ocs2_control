# camera

W2 的 Orbbec 相机：头部 Gemini 335Lg，左右腕 Gemini 305。依赖 [`robot_config`](../robot_config/README.md) 的配置框架。

```bash
./scripts/setup.sh cam      # 弹窗：按实时画面分配头部、左右腕，设置旋转，显示帧率，逐项确认
ros2 launch camera_setup cameras.launch.py config:=/绝对路径/config/robot.local.yaml [only:=camera_head]
```

旋转角度保存到 `cameras.<位置>.rotation`，启动时按它生效。左右以机器人自身方向为准。

窗口发现旧的采集进程时，会列出设备和 PID，询问是否停止并重启预览；只有明确确认才停止，运控进程不在清理范围内。容器里看不到的宿主机进程需要先在宿主机上停。

多台相机要顺序启动：驱动用共享内存里的互斥锁串行初始化设备，只重试 5 次。`start.sh up` 和设置窗口的预览都等每台驱动日志里出现 `Initialize device cost` 后再起下一台。

## 组成

| 文件 | 内容 |
|---|---|
| `camera.py` | USB 扫描、相机规格与驱动参数、旧采集进程识别、画面订阅与帧率 |
| `camera_setup.py` | 相机配置会话、预览、窗口卡片、`setup.sh cam` 入口 |
| `launch/cameras.launch.py` | 启动 `orbbec_camera` 驱动，每台一个进程 |

`camera.py` 不依赖 Qt 和配置改写，`robot_app` 启动相机只用它。
