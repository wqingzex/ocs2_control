# scripts

仓库的两个入口脚本，都从仓库根目录运行：`./scripts/setup.sh`、`./scripts/start.sh`。脚本里的 `help` 是权威说明。

## setup.sh：初始化

依次完成依赖、主机 / SDK、编译、相机配置、手配置、负载辨识、XR 配对、最终检查。已配置（有确认记录）或默认值可用的步骤自动跳过，开头列出每步状态。

```bash
./scripts/setup.sh                 # 完整初始化
./scripts/setup.sh cam             # 单步重试：cam 相机、hand 手、xr 头显配对、dyn 负载辨识、deps 依赖、host 主机 / SDK、check 检查
./scripts/setup.sh cam --from      # 从相机步骤继续到最后，其他步骤同样支持 --from
./scripts/setup.sh build-workspace # 增量编译整个工作区，不做设备配置
```

选项：`--force` 全部重新配置，`--yes` 使用建议值，`--minimal` 只编译 W2 必需的包，`-c` 指定配置文件，`dyn --reset` 恢复默认负载。相机和手会弹出本地窗口，确认后写入 `config/robot.local.yaml`；容器里不执行 udev 和 CAN 操作。

## start.sh：启动和日常操作

```bash
./scripts/start.sh                 # 按配置启动运控、VR、相机，Ctrl+C 停止（运控先切 HOLD）
./scripts/start.sh sim | isaac     # 硬件改为 mock_components / Isaac
./scripts/start.sh viz             # 只在 RViz 看模型，带关节滑块，不启动控制
./scripts/start.sh up|down|restart|status|logs [模块]   # 按模块管理，status --hz 看话题频率
./scripts/start.sh nav [--list] [--map ID] [--map-only]  # 导航
./scripts/start.sh dyn-identify    # 机械臂负载辨识向导，需要终端
./scripts/start.sh console         # Web 控制台
./scripts/start.sh diag            # 打包日志、事件时间线、配置，出问题时发给开发者
./scripts/start.sh record [名称] [--topics joint|xr|collect|/a,/b]   # 录制，默认只录关节
./scripts/start.sh play 名称 [--rate 0.5] [--loop] [--paused] [--start 秒] [--xr]
./scripts/start.sh bags [--delete 名称 --yes]            # 列出、删除录包
./scripts/start.sh check | build   # 只校验配置 / 编译
```

选项，命令前后都可以：`-c 配置文件`，`--set 字段=值`（只对本次运行有效，如 `--set robot.hardware=isaac`），`--entry split_body|dual_arm|whole_body`，`--hand none|linkerhand_l6|xhand1`，`--hand-drive usb|485`，`--rviz`（同时启动 RViz）。

模块、日志位置和排查方法见 [`docs/VR遥操作.md`](../docs/VR遥操作.md)；背后的实现在 [`fa_teleop`](../src/robot_teleop/fa_teleop/README.md) 和 [`w2_payload`](../src/robot_payload/w2_payload/README.md)。
