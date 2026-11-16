# third_party

外部代码和厂家 SDK，统一放在这里，不改动内容。子模块固定在某个提交；没有可用仓库的 SDK 直接放入。

| 目录 | 内容 | 方式 | 版本 | 用到它的包 |
|---|---|---|---|---|
| `ocs2_ros2` | OCS2（legubiao/ocs2_ros2） | 子模块 | `ae75126` | `robot_control/lib/mpc` |
| `OrbbecSDK_ROS2` | Orbbec 相机 ROS 2 驱动 | 子模块 | `cb24b00` | `w2_payload` 的相机 |
| `TJ_FX_ROBOT_CONTRL_SDK_338` / `_343` | 天机 Marvin 臂 SDK（38 版 / 43 版及以上固件） | 子模块 | `1fa6cc6` / `02440e8` | `marvin_hardware` |
| `kh_ucanfd_sdk` | 腰部 USB-CANFD 适配器驱动 | 直接放入 | — | 宿主机内核模块（`scripts/setup.sh`） |
| `wuji-sdk-c` | 舞肌 Wuji Hand 2 C SDK（`libwuji_sdk_c`，MIT） | 直接放入官方发布包 | `2026.8.31` | `wuji_hardware` |

子模块初始化：`git submodule update --init --recursive --depth 1`。

## wuji-sdk-c

官方 [wuji-technology/wuji-sdk](https://github.com/wuji-technology/wuji-sdk) 的发布包，只有预编译库，没有源码仓库可做子模块，所以直接放 `x86_64` 和 `aarch64` 两个 `tar.gz`（`v2026.8.31`，即 `wuji_hardware` 驱动最后一次修改时的最新版）。`wuji_hardware` 的 CMake 在配置时解压对应架构的那一个，不联网。

换版本：下载新版本的 `wuji-sdk-c-<版本>-x86_64-linux-gnu.tar.gz` 和 `-aarch64-linux-gnu.tar.gz` 放进来、删掉旧的，改 `robot_hardware/wuji_hardware/CMakeLists.txt` 里的 `WUJI_SDK_VERSION`。
