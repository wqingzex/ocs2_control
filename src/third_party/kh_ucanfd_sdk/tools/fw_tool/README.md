# KCAN 固件工具

KCAN-USB 设备固件升级工具套件，支持固件版本查询、升级、APP 切换等功能。

## 功能特性

- **固件版本查询**: 获取当前固件版本和编译日期
- **终端电阻控制**: 控制 120Ω 终端电阻的开关
- **APP 切换**: 切换到指定地址的 APP 或下一个 APP 槽位
- **交互式菜单**: 提供友好的交互式操作界面
- **自动化升级**: 一键完成固件升级全流程

## 编译

```bash
# 普通编译
make

# 静态链接版本
make static

# 清理
make clean
```

## 安装

```bash
sudo make install
```

默认安装到 `/usr/local/bin/`。

安装后常用入口（`/usr/local/bin`）：
- `kcan_fw_tool`：工厂模式工具（固件/终端/APP 切换）
- `kcan_fw_upgrade`：Python 自动升级器（包含 `lsfw` 查询、`auto` 自动升级、以及 `fw**.enc` 直接安装）
- `kcan_fw_upgrade_bash`：Bash 升级包装脚本（兼容入口，内部会调用 `kcan_fw_upgrade` 完成实际逻辑）

## 使用方法

### 命令行模式

```bash
# 查看帮助
kcan_fw_tool -h

# 获取固件版本
kcan_fw_tool can0 version

# 获取终端电阻状态
kcan_fw_tool can0 term-get

# 开启终端电阻
kcan_fw_tool can0 term-on

# 关闭终端电阻
kcan_fw_tool can0 term-off

# 切换到指定 APP
kcan_fw_tool can0 switch-app -a 0x10000

# 切换到下一个 APP
kcan_fw_tool can0 next-app
```

### 交互式菜单模式

```bash
kcan_fw_tool can0 menu
```

这将显示一个交互式菜单，可以方便地选择各种操作。

### 详细输出

添加 `-v` 或 `--verbose` 选项可以显示详细的调试信息：

```bash
kcan_fw_tool can0 version -v
```

### 跳过确认

添加 `-y` 或 `--yes` 选项跳过确认提示（用于脚本自动化）：

```bash
kcan_fw_tool can0 switch-app -a 0x10000 -y
```

## 典型使用流程

### 查询设备信息

```bash
# 获取固件版本
kcan_fw_tool can0 version

# 获取终端电阻状态
kcan_fw_tool can0 term-get
```

### 固件升级

#### 自动升级（推荐）

使用自动化升级脚本可以一键完成整个升级流程：

```bash
# 默认（kcan_fw_upgrade）
sudo kcan_fw_upgrade can0 firmware.enc

# 默认自动升级（读取 tools/fw_images/fw_manifest.json，按设备类型/当前 FW/Boot 判断是否需要升级）
sudo kcan_fw_upgrade can0 auto

# Python 版本
sudo ./kcan_fw_upgrade.py can0 firmware.enc

# Python 自动升级
sudo ./kcan_fw_upgrade.py can0 auto

# 带详细输出
sudo kcan_fw_upgrade can0 firmware.enc -v

# 设备已在升级模式时（跳过 Bootloader 步骤）
sudo kcan_fw_upgrade can0 firmware.enc --skip-bootloader

# 如果你在 `tools/fw_tool/` release 目录下运行，并且想强制走 bash wrapper：
# （release 目录下入口文件名是 `kcan_fw_upgrade.sh`）
# sudo ./kcan_fw_upgrade.sh can0 firmware.enc

# 如果你在系统已安装后执行，并且想强制走 bash wrapper：
# sudo kcan_fw_upgrade_bash can0 firmware.enc
```

#### 已安装固件查询（lsfw）
```bash
# 列出系统已安装的固件镜像及其适配信息
sudo kcan_fw_upgrade lsfw

# 按通配符筛选（例如仅列出 8.5.x）
sudo kcan_fw_upgrade lsfw KH-UCANFD-*.enc

# 如果你是在源码/ release 目录下运行，也可以使用：
# sudo ./kcan_fw_upgrade.sh lsfw
```

#### 仅给固件“名称/通配符”安装（无需关心路径）
```bash
# 下面的固件 token 会自动从 /usr/lib/firmware 查找并解析为实际文件路径
sudo kcan_fw_upgrade can0 KH-UCANFD-*.enc
sudo kcan_fw_upgrade can0 fw**.enc

# 如果你是在源码/ release 目录下运行，也可以使用：
# sudo ./kcan_fw_upgrade.sh can0 KH-UCANFD-*.enc
```

自动化脚本会自动：
1. 进入 Bootloader 模式
2. 识别 USB 存储设备
3. 挂载设备并拷贝固件文件
4. 等待设备重启
5. 验证升级后的固件版本

当固件参数使用 `auto` 时，上述流程前会先：
 - 读取设备当前 FW/BL 版本
 - 通过 USB idProduct 推断设备通道数（FD/FDPRO/X4）
 - 结合 `tools/fw_images/fw_manifest.json` 选择“最新且兼容”的固件
 - 若已是最新固件则直接退出，不触发升级

#### 手动升级

```bash
# 1. 通过自动化脚本使设备进入升级模式

# 2. 设备进入升级模式后变为 USB 存储设备
#    挂载并拷贝 .enc 固件文件到设备

# 3. 升级完成后设备会自动重启
```

### APP 切换

```bash
# 切换到指定地址的 APP
kcan_fw_tool can0 switch-app -a 0x80000

# 或切换到下一个 APP 槽位
kcan_fw_tool can0 next-app
```

## 注意事项

1. **权限要求**: 需要 root 权限或适当的 CAN 接口权限
2. **接口状态**: 工具会自动管理 CAN 接口的 up/down 状态
3. **设备重启**: `switch-app`、`next-app` 命令会触发设备重启
4. **超时处理**: 如果命令超时，工具会自动重试

## 故障排除

### 工具无法正常工作

1. 检查 CAN 接口是否存在：
   ```bash
   ip link show can0
   ```

2. 检查驱动是否正确加载：
   ```bash
   dmesg | grep -i kcan
   ```

### 命令无响应

1. 使用 `-v` 选项查看详细日志
2. 检查 CAN 接口状态：
   ```bash
   ip -details link show can0
   ```

## 文件结构

```
fw_tool/
├── Makefile                # 构建脚本
├── README.md               # 本文档
├── kcan_fw_tool.c          # 主程序源码
├── kcan_fw_upgrade.sh      # 自动升级脚本 (Bash)
└── kcan_fw_upgrade.py      # 自动升级脚本 (Python)
```

## 许可证

Copyright (C) 2022-2026 Chengdu Kunhong Electronic Technology Co., Ltd. All rights reserved.
