# third_party

外部代码和厂家 SDK，统一放在这里，不改动内容。子模块固定在某个提交；没有可用仓库的 SDK 直接放入。

| 目录 | 内容 | 方式 | 版本 | 用到它的包 |
|---|---|---|---|---|
| `ocs2_ros2` | OCS2（legubiao/ocs2_ros2） | 子模块 | `ae75126` | `robot_control/lib/mpc` |
| `OrbbecSDK_ROS2` | Orbbec 相机 ROS 2 驱动 | 子模块 | `cb24b00` | `w2_payload` 的相机 |
| `TJ_FX_ROBOT_CONTRL_SDK_338` / `_343` | 天机 Marvin 臂 SDK（38 版 / 43 版及以上固件） | 子模块 | `1fa6cc6` / `02440e8` | `marvin_hardware` |
| `kh_ucanfd_sdk` | 腰部 USB-CANFD 适配器驱动 | 直接放入 | — | 宿主机内核模块（`scripts/setup.sh`） |
| `wuji-sdk-c` | 舞肌 Wuji Hand 2 C SDK（`libwuji_sdk_c`，MIT） | 直接放入官方发布包 | `2026.8.31` | `wuji_hardware` |
| `dexcap-sdk-cpp` | DexCap V4 C++ SDK（DexRobot/dexcap-sdk-cpp：`libDexCap` 及其自带的 libserial / libusb / libudev / yaml-cpp，**仅 x86_64**） | 子模块 | `fc6590e` | `dexcap_teleop` |
| `xCoreSDK-CPP` | 珞石 Rokae 臂 xCoreSDK C++（[RokaeRobot/xCoreSDK-CPP](https://github.com/RokaeRobot/xCoreSDK-CPP)，Apache-2.0；仓库只有头文件，预编译库在 GitHub Release） | 子模块 + Release 库 | `895c2e9`（v0.7.1） | `rokae_hardware` |

子模块初始化：`git submodule update --init --recursive --depth 1`。

## dexcap-sdk-cpp

上游 [DexRobot/dexcap-sdk-cpp](https://github.com/DexRobot/dexcap-sdk-cpp)，固定在 `fc6590e`（2026-07-07，"C interfaces updated"，Ubuntu 22 RC1）：这一版的 `libs/linux/` 和头文件与 `dexcap_teleop` 开发时用的拷贝逐字节一致。之后的上游提交把库挪到了 `libs/linux/x86_64/`、`aarch64/` 并换了 `libDexCap.so`，升级要改 `dexcap_teleop/CMakeLists.txt` 的 `DEXCAP_SDK_LIB_DIR` 并重新验证。

目录里没有要编译的东西（`examples/` 是要 Qt6 的厂家示例 `DexGloveDemo`，`urdf/` 是 ROS 1 catkin 包），`./scripts/setup.sh deps` / `build` 会在子模块根放 `COLCON_IGNORE`，colcon 整个跳过。头文件需要系统的 `libserial-dev`（`dexcap_teleop` 的 `package.xml` 已声明，`./scripts/setup.sh deps` 经 rosdep 安装）。

## wuji-sdk-c

官方 [wuji-technology/wuji-sdk](https://github.com/wuji-technology/wuji-sdk) 的 Release 附件（sha256 与官方一致）。该仓库本身只有示例和文档，库只在 Release 附件里，做成子模块拿不到库，所以直接放 `x86_64` 和 `aarch64` 两个 `tar.gz`（`v2026.8.31`，即 `wuji_hardware` 驱动最后一次修改时的最新版）。`wuji_hardware` 的 CMake 在配置时解压对应架构的那一个，不联网。

换版本：下载新版本的 `wuji-sdk-c-<版本>-x86_64-linux-gnu.tar.gz` 和 `-aarch64-linux-gnu.tar.gz` 放进来、删掉旧的，改 `robot_hardware/wuji_hardware/CMakeLists.txt` 里的 `WUJI_SDK_VERSION`。

## xCoreSDK-CPP

珞石官方仓库 [RokaeRobot/xCoreSDK-CPP](https://github.com/RokaeRobot/xCoreSDK-CPP)，固定在 `895c2e9`（tag `v0.7.1`）。仓库里只有头文件，**预编译库不在 git 里**，按平台放在 [Release v0.7.1](https://github.com/RokaeRobot/xCoreSDK-CPP/releases/tag/v0.7.1)（`xCoreSDK-0.7.1-linux-x86_64.tar.gz`、`-aarch64.tar.gz`）。`./scripts/setup.sh deps` 和 `build` 会下载、**校验 sha256**、解压到 `src/third_party/xCoreSDK-CPP/lib/Linux/<架构>/`（上游 `.gitignore` 已忽略，子模块不会变脏）。

换版本：子模块切到新 tag，同时改 `scripts/setup.sh` 的 `ROKAE_SDK_VERSION` 和两个 sha256（版本对不上时 `setup.sh` 会拒绝继续）。v0.7.0 的库已经从上游清掉，取不到。整个目录被 `COLCON_IGNORE`，`rokae_hardware` 按路径引用。
