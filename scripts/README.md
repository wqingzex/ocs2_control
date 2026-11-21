# scripts

仓库的两个入口脚本，都从仓库根目录运行：`./scripts/setup.sh`、`./scripts/start.sh`。脚本里的 `help` 是权威说明。

## setup.sh：初始化

先在本机配置选择组件，再装依赖、准备 SDK、编译，最后确认设备。安装和编译读取所选配置，按控制入口、硬件模式、末端、相机和遥操作输入选择依赖。已确认或默认值可用的设备步骤会跳过。

```bash
./scripts/setup.sh                 # 完整初始化
./scripts/setup.sh clean           # 按编号选择清理 build/、install/ 或本机配置
./scripts/setup.sh config          # 生成或补全 config/robot.local.yaml（--entry 选入口；完整初始化的第 1 步，模板加了键之后单独跑它补键）
./scripts/setup.sh cam             # 单步重试：home HOME 位姿、cam 相机、ee 末端、teleop 遥操作输入、dyn 负载辨识、deps 依赖、host 主机 / SDK、marvin 天机臂 SDK 版本、build 编译、check 检查
./scripts/setup.sh cam --from      # 从相机步骤继续到最后，其他步骤同样支持 --from
./scripts/setup.sh marvin          # 选择天机臂 SDK 版本（338 / 343，要和臂的控制器固件一致），写入配置
./scripts/setup.sh build           # 按本机配置增量编译
```

完整向导和 `--from` 在每一步开始前询问是否执行。已配置或有可用默认值时，回车跳过，输入 `y` 可以重新配置。单步命令直接进入指定步骤。`--force` 让向导每步默认执行，`--yes` 自动采用建议选项，`-c` 指定配置文件，`dyn --reset` 恢复默认负载。相机和手会弹出本地窗口，确认后写入 `config/robot.local.yaml`。容器里的主机配置步骤只准备所需的 XR 组件。

`clean` 支持空格分隔的多选，直接回车取消。选择本机配置时，当前文件会移到旁边的 `.before-clean.*` 备份。日志会保留。清理后可以用 `./scripts/setup.sh build` 重编译。

## start.sh：启动和日常操作

```bash
./scripts/start.sh                 # 按配置启动运控、XR、相机，Ctrl+C 停止（运控先切 HOLD）
./scripts/start.sh sim | isaac     # 硬件改为 mock_components / Isaac
./scripts/start.sh viz             # 只在 RViz 看模型，带关节滑块，不启动控制
./scripts/start.sh up|down|restart|status|logs [模块]   # 按模块管理，status --hz 看话题频率；up control 等控制器全部激活（失败会报错）并默认带 RViz（--no-rviz 不带）；up xr 等头显发来数据，没连上给警告
./scripts/start.sh dyn-identify    # 机械臂负载辨识向导，需要终端
./scripts/start.sh bus_probe [--side left|right] [--duration S] [--cycle-hz N]   # 测末端 USB 总线的往返时延、丢帧、请求频率；先停运控
./scripts/start.sh console         # Web 控制台，前台运行（Ctrl+C 停止）
./scripts/start.sh console start|stop|restart|status [--port N]   # 后台启动 / 停止 / 重启 / 查看，默认端口 8765
./scripts/start.sh console --open  # 后台启动并用浏览器打开（双击 robot_control_suite控制台 用的就是它）
./scripts/start.sh diag            # 打包日志、事件时间线、配置，出问题时发给开发者
./scripts/start.sh record [名称] [--topics joint|xr|collect|/a,/b]   # 录制，默认只录关节
./scripts/start.sh play 名称 [--rate 0.5] [--loop] [--paused] [--start 秒] [--xr]
./scripts/start.sh bags [--delete 名称 --yes]            # 列出、删除录包
./scripts/start.sh check | build   # 只校验配置 / 编译
```

选项，命令前后都可以：`-c 配置文件`，`--set 字段=值`（只对本次运行有效，如 `--set robot.hardware=isaac`），`--entry whole_body|split_body|dual_arm`，`--end-effector none|linkerhand_l6|xhand1|wuji_hand2|jodell_rg75`，`--end-effector-bus usb_rs485|arm_rs485|usb_can|ethernet`，`--rviz`（同时启动 RViz）。

## nudge.py：真机验证用的小幅微调

```bash
./scripts/nudge.py show                  # 头、腰关节位置和状态机状态
./scripts/nudge.py fsm movej|hold        # 状态机 MOVEJ（4）/ HOLD（2）
./scripts/nudge.py head 1 0.05           # 头关节 1（或 2）相对当前位置动 0.05 rad，等到位
./scripts/nudge.py waist 0.05            # 转腰 body_joint4，只在 MOVEJ 下执行
```

增量不超过 0.1 rad，用于 [`docs/使用手册.md`](../docs/使用手册.md)；需要容器里的 ROS 环境。

模块、日志位置和排查方法见 [`docs/使用手册.md`](../docs/使用手册.md)；背后的实现在 [`robot_app`](../src/robot_hmi/robot_app/README.md) 和 [`robot_config`](../src/robot_setup/robot_config/README.md)。
