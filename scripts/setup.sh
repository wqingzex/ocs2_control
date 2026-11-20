#!/usr/bin/env bash
# 初始化入口：直接 ./scripts/setup.sh，完整配置依赖、硬件、相机和 XR。
# 用法见 ./scripts/setup.sh help。结构：1 常量  2 通用工具  3 步骤  4 入口。
set -Eeuo pipefail

# ==================================================================================================
# 1. 常量
# ==================================================================================================
WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"   # 仓库根目录（脚本在 scripts/ 下）
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
CONFIG="${WS_DIR}/config/robot.local.yaml"
YES=""
MINIMAL=""
FORCE=""   # --force：完整流程里也重新配置已配置的步骤
RESET=""   # --reset：dyn 恢复默认负载
FULL_INIT=""
ENTRY=""      # --entry：新建配置时的控制入口

# 步骤的顺序和标题；序号 i/N 由这张表算出来。SOFT_STEPS 失败时只记录，不中断后面的步骤。
STEP_ORDER=(deps config host marvin build cameras ee dyn xr summary)
declare -A STEP_TITLE=(
  [deps]="依赖：子模块 + rosdep"
  [config]="本机配置 robot.local.yaml（没有就按入口从模板生成，缺的键补上）"
  [host]="主机配置：相机 udev、腰部 CAN 免密 sudo、XRoboToolkit PC Service"
  [marvin]="天机臂 SDK 版本（338 / 343，要和臂的控制器固件一致）"
  [build]="编译整个 ROS 2 工作区（运控、驱动、机器人模型、相机、XR、控制台）"
  [cameras]="相机"
  [ee]="末端（手、夹爪）"
  [dyn]="负载辨识（有默认值就不用配置）"
  [xr]="XR（头显）"
  [summary]="配置检查"
)
SOFT_STEPS=(ee dyn xr summary)

# ocs2_ros2（子模块，src/third_party/ocs2_ros2，固定在 ae75126）带着一批用不到的示例包：编译和 rosdep 都跳过。
OCS2_SKIP_PACKAGES=(
  ocs2_ballbot ocs2_ballbot_ros ocs2_ballbot_mpcnet
  ocs2_cartpole ocs2_cartpole_ros
  ocs2_double_integrator ocs2_double_integrator_ros
  ocs2_legged_robot ocs2_legged_robot_ros ocs2_legged_robot_mpcnet ocs2_legged_robot_raisim
  ocs2_quadrotor ocs2_quadrotor_ros
  ocs2_mpcnet_core ocs2_raisim_core
  ocs2_anymal_commands ocs2_anymal_loopshaping_mpc ocs2_anymal_models ocs2_anymal_mpc
  ocs2_quadruped_interface ocs2_quadruped_loopshaping_interface
  ocs2_switched_model_interface ocs2_switched_model_msgs segmented_planes_terrain_model
)
# 天机臂（Marvin）SDK 版本：config 的 robot.arm.interface.firmware，编译时传给 marvin_hardware 的 -DMARVIN_FIRMWARE。
MARVIN_FIRMWARES=(338 343)
MARVIN_FIRMWARE_DEFAULT=343   # 和 marvin_hardware/CMakeLists.txt 的默认值一致
# build --minimal 只编译这些包及其依赖。
MINIMAL_PACKAGES=(
  fa_w2_description juxie_hardware marvin_hardware end_effector_hardware
  keyboard_teleop xr_teleop topic_based_hardware robot_app camera_setup end_effector_setup
)

# ==================================================================================================
# 2. 通用工具
# ==================================================================================================
log() { printf '\n\033[1;32m=== %s\033[0m\n' "$*"; }
info() { printf '>>> %s\n' "$*"; }
warn() { printf '\033[1;33m[WARN] %s\033[0m\n' "$*" >&2; }
die() { printf '\033[1;31m[ERROR] %s\033[0m\n' "$*" >&2; exit 1; }
is_container() { [[ -f /.dockerenv || -f /run/.containerenv || -n "${container:-}" ]]; }
as_root() { if (( EUID == 0 )); then "$@"; else sudo "$@"; fi; }
step_command() {
  case "$1" in
    cameras) printf 'cam' ;;
    summary) printf 'check' ;;
    *) printf '%s' "$1" ;;
  esac
}

usage() {
  cat <<'EOF_USAGE'
初始化：./scripts/setup.sh
  依次完成：依赖、本机配置、主机 / SDK、天机臂 SDK 版本、编译、相机配置、手配置、负载辨识、XR 配对、最终检查。
  已配置（有确认记录）或默认值可用的步骤会跳过；--force 全部重新配置。
  相机和手会弹出本地配置窗口，确认后保存到 config/robot.local.yaml。
  容器内也保留完整向导；仅不执行物理机的 udev 和 CAN 操作。
单步重试：./scripts/setup.sh <步骤>，只执行指定步骤：
  config 本机配置：没有 config/robot.local.yaml 就按入口（--entry whole_body|split_body|dual_arm，默认 whole_body）从模板生成；
         已有就把缺的键补上（保留你的值和注释，原文件备份为 .bak）。程序不会替你补默认值，模板改了键之后每台机器跑一次。
  cam 相机；ee 末端（手、夹爪）；xr 头显配对；dyn 负载辨识（逐臂询问，默认不辨识；--reset 恢复默认负载）。
  build 编译整个 ROS 2 工作区：运控、驱动、机器人模型、相机、XR、控制台。
        使用增量编译，复用已有结果；不执行设备配置或配对，不启动运控。
  marvin 选择天机臂 SDK 版本（338 或 343，要和臂的控制器固件一致），写入配置，之后的编译都用它。
  deps 依赖；host 主机 / SDK；check 配置检查。
从某步继续：./scripts/setup.sh cam --from
  从相机开始，依次执行手、XR 配对和最终检查；其他步骤同样支持 --from。
可选参数：--yes 使用建议值；--force 重新配置已配置的步骤；--minimal 只编译遥操作必需的包；-c 指定配置文件；--entry 新建配置时的控制入口。
EOF_USAGE
}

# 加载 ROS（和本工作区），再读可选的 config/ros_env.sh（RMW / Domain ID / Zenoh，README 2.1）
source_ros() {
  local setup="/opt/ros/${ROS_DISTRO}/setup.bash"
  [[ -f "${setup}" ]] || die "未找到 ${setup}：请先安装 ROS 2 ${ROS_DISTRO}"
  set +u
  # shellcheck disable=SC1090
  source "${setup}"
  # shellcheck disable=SC1091
  [[ -f "${WS_DIR}/config/ros_env.sh" ]] && source "${WS_DIR}/config/ros_env.sh"
  if [[ "${1:-}" == "--workspace" ]]; then
    [[ -f "${WS_DIR}/install/setup.bash" ]] || die "工作区未编译：先运行 ./scripts/setup.sh"
    # shellcheck disable=SC1091
    source "${WS_DIR}/install/setup.bash"
  fi
  set -u
}

# ==================================================================================================
# 3. 步骤（每个步骤是 step_<名字>，成功返回 0）
# ==================================================================================================

# ---- deps -----------------------------------------------------------------------------------------
deps_rosdep_update() {
  local i
  for i in 1 2 3; do  # rosdep update 偶尔因网络超时失败
    rosdep update --rosdistro "${ROS_DISTRO}" && return 0
    warn "rosdep update 失败（第 ${i} 次），重试"
    sleep 3
  done
}

# 子模块里不是 ROS 包、却会被 colcon 扫到的目录，让 colcon 忽略：
#   MarvinPlatform_EN：PyInstaller 打包脚本，colcon 扫描时会执行它的 setup.py 并报错；
#   dexcap-sdk-cpp：整个忽略。里面只有预编译库，dexcap_teleop 按路径引用；examples/ 是厂家示例（CMake 项目
#   DexGloveDemo，要 Qt6 和 libbluetooth），urdf/ 是 ROS 1 catkin 包，都编不了。
#   xCoreSDK-CPP：整个忽略。珞石 SDK 的头文件和库只给 rokae_hardware 按路径引用，它自己的 CMake 工程（示例、Eigen）不能被 colcon 当成包去编。
# 子模块里的 COLCON_IGNORE 同时加入子模块的 info/exclude（写相对子模块根的路径），避免子模块显示为有未跟踪文件。
colcon_ignore_submodule_dirs() {
  local dir entry exclude
  for dir in "${WS_DIR}"/src/third_party/TJ_FX_ROBOT_CONTRL_SDK_*/MarvinPlatform_EN \
             "${WS_DIR}"/src/third_party/dexcap-sdk-cpp \
             "${WS_DIR}"/src/third_party/xCoreSDK-CPP; do
    [[ -d "${dir}" ]] || continue
    touch "${dir}/COLCON_IGNORE"
    entry="$(git -C "${dir}" rev-parse --show-prefix 2>/dev/null || true)COLCON_IGNORE"
    exclude="$(git -C "${dir}" rev-parse --path-format=absolute --git-path info/exclude 2>/dev/null || true)"
    if [[ -n "${exclude}" ]] && ! grep -qxF "${entry}" "${exclude}" 2>/dev/null; then
      mkdir -p "$(dirname "${exclude}")"
      echo "${entry}" >> "${exclude}"
    fi
  done
}

# 珞石 xCoreSDK：官方仓库（子模块 third_party/xCoreSDK-CPP，固定在发布 tag）不带预编译库，库按平台放在 GitHub Release 里。
# 版本要和子模块一致；下载的包先校验 sha256 再解压，解压到子模块的 lib/（上游 .gitignore 已忽略，子模块不会变脏）。
ROKAE_SDK_VERSION="0.7.1"
rokae_sdk_sha256() {
  case "$1" in
    x86_64) echo "ccc125c41f0f72712eb12835f80a5be5fa62429ca045bc5476fe1be224b36270" ;;
    aarch64) echo "78cc578b066c9edb2c015389a14bc5bdf1c3c69f6e8c1c7ee749dec568fc4518" ;;
    *) return 1 ;;
  esac
}
rokae_sdk_libs() {
  local dir="${WS_DIR}/src/third_party/xCoreSDK-CPP" arch pkg url tmpdir expected actual version
  arch="$(uname -m)"
  [[ -f "${dir}/CMakeLists.txt" ]] || die "缺少 ${dir}（先执行 ./scripts/setup.sh deps）"
  version="$(sed -n 's/^project *( *xCoreSDK VERSION \([0-9.]*\).*/\1/p' "${dir}/CMakeLists.txt" | head -1)"
  [[ "${version}" == "${ROKAE_SDK_VERSION}" ]] \
    || die "子模块 xCoreSDK-CPP 是 ${version:-未知版本}，setup.sh 里固定的库是 ${ROKAE_SDK_VERSION}：换版本要一起改 ROKAE_SDK_VERSION 和两个 sha256"
  if [[ -f "${dir}/lib/Linux/${arch}/libxCoreSDK.a" && -f "${dir}/lib/Linux/${arch}/libxMateModel.a" ]]; then
    info "珞石 xCoreSDK ${ROKAE_SDK_VERSION} 的库已就绪"; return
  fi
  expected="$(rokae_sdk_sha256 "${arch}")" || die "珞石 xCoreSDK 没有 ${arch} 的发布包"
  pkg="xCoreSDK-${ROKAE_SDK_VERSION}-linux-${arch}.tar.gz"
  url="https://github.com/RokaeRobot/xCoreSDK-CPP/releases/download/v${ROKAE_SDK_VERSION}/${pkg}"
  tmpdir="$(mktemp -d)"
  info "下载 ${pkg}"
  curl -fSL -o "${tmpdir}/${pkg}" "${url}" || { rm -rf "${tmpdir}"; die "下载失败：${url}"; }
  actual="$(sha256sum "${tmpdir}/${pkg}" | cut -d' ' -f1)"
  if [[ "${actual}" != "${expected}" ]]; then
    rm -rf "${tmpdir}"
    die "${pkg} 的 sha256 不对（期望 ${expected}，实际 ${actual}），未解压"
  fi
  tar -xzf "${tmpdir}/${pkg}" -C "${dir}"  # 包里的路径是 lib/Linux/<arch>/...
  rm -rf "${tmpdir}"
  info "珞石 xCoreSDK ${ROKAE_SDK_VERSION} 的库已解压到 ${dir}/lib/Linux/${arch}"
}

step_deps() {
  git -C "${WS_DIR}" submodule update --init --recursive --depth 1  # 浅克隆：只取固定的提交
  colcon_ignore_submodule_dirs
  rokae_sdk_libs
  source_ros
  command -v rosdep >/dev/null || die "未安装 rosdep：sudo apt install python3-rosdep"
  [[ -d /etc/ros/rosdep/sources.list.d ]] || as_root rosdep init
  deps_rosdep_update
  # 示例包不编译，它们的依赖（raisim、torch ...）也不让 rosdep 去装
  local paths=()
  if command -v colcon >/dev/null; then
    mapfile -t paths < <(cd "${WS_DIR}" && colcon list --paths-only --packages-skip "${OCS2_SKIP_PACKAGES[@]}")
  fi
  (( ${#paths[@]} )) || paths=("${WS_DIR}/src")
  (cd "${WS_DIR}" && rosdep install --from-paths "${paths[@]}" --ignore-src -r -y --rosdistro "${ROS_DISTRO}")
}

# ---- host -----------------------------------------------------------------------------------------
host_udev() {
  local src="${WS_DIR}/src/third_party/OrbbecSDK_ROS2/orbbec_camera/scripts/99-obsensor-libusb.rules"
  local dst="/etc/udev/rules.d/99-obsensor-libusb.rules"
  [[ -f "${src}" ]] || die "缺少 ${src}（先执行 ./scripts/setup.sh deps）"
  if [[ -f "${dst}" ]] && cmp -s "${src}" "${dst}"; then info "Orbbec udev 规则已安装"; return; fi
  info "安装 Orbbec udev 规则 -> ${dst}"
  sudo install -m 0644 "${src}" "${dst}"
  sudo udevadm control --reload-rules
  sudo udevadm trigger
  info "已安装；已插上的相机请重新插拔一次"
}

# 腰部 CAN 接口名：环境变量 CAN_IFACE > config/robot.local.yaml 的 robot.body.interface.can > can0。
# 名字会写进 sudoers 规则，必须是合法的接口名（和驱动 jx_hardware 的校验一致）。
body_can_iface() {
  local iface="${CAN_IFACE:-}"
  if [[ -z "${iface}" && -f "${CONFIG}" ]]; then
    iface="$(python3 - "${CONFIG}" <<'PY' 2>/dev/null || true
import sys, yaml
data = yaml.safe_load(open(sys.argv[1])) or {}
body = (data.get("robot") or {}).get("body") or {}
print(str((body.get("interface") or {}).get("can", "") or "").strip())
PY
)"
  fi
  iface="${iface:-can0}"
  [[ "${iface}" =~ ^[A-Za-z0-9_.][A-Za-z0-9_.-]{0,14}$ ]] || die "CAN 接口名不合法：'${iface}'（1 到 15 个字符，只能用字母、数字、_ . -，不能以 - 开头）"
  printf '%s' "${iface}"
}

# robot.end_effector.interface.bus 为 usb_can 时两只手的 CAN 接口（interface.left.can / right.can，
# 默认 can1 / can2），每行一个；其他接线方式不输出。名字同样要合法，且不能和腰部接口相同。
ee_can_ifaces() {
  [[ -f "${CONFIG}" ]] || return 0
  local body="$1" iface
  while IFS= read -r iface; do
    [[ -n "${iface}" ]] || continue
    [[ "${iface}" =~ ^[A-Za-z0-9_.][A-Za-z0-9_.-]{0,14}$ ]] || die "手的 CAN 接口名不合法：'${iface}'"
    [[ "${iface}" != "${body}" ]] || die "手的 CAN 接口 ${iface} 和腰部 body_can_interface 相同；手要用另一个 CAN 接口"
    printf '%s\n' "${iface}"
  done < <(python3 - "${CONFIG}" <<'PY' 2>/dev/null || true
import sys, yaml
data = yaml.safe_load(open(sys.argv[1])) or {}
robot = data.get("robot") or {}
eef = robot.get("end_effector") or {}
side = lambda name: (eef.get("interface") or {}).get(name) or {}
if (eef.get("type") or "linkerhand_l6") != "none" and (eef.get("interface") or {}).get("bus") == "usb_can":
    names = [str(side("left").get("can") or "can1").strip(), str(side("right").get("can") or "can2").strip()]
    print("\n".join(dict.fromkeys(names)))  # the two hands may share one interface
PY
)
}

# 只放行驱动实际执行的 `sudo ip link set <can> ...`，参数写死、不用通配符：
# 腰部 jx_hardware（CAN FD），以及 end_effector.interface.bus: usb_can 时手的 end_effector_hardware（经典 CAN 1 Mbit/s）
host_sudoers() {
  local iface file="/etc/sudoers.d/fa_w2-can" legacy
  for legacy in /etc/sudoers.d/fa-w2-can /etc/sudoers.d/fiveages_w2-can; do  # 旧文件名写入的同一条规则：删掉，只留新文件
    if sudo test -e "${legacy}"; then
      sudo rm -f "${legacy}"
      info "已删除旧的 ${legacy}"
    fi
  done
  iface="$(body_can_iface)"
  local up_args="up type can bitrate 1000000 dbitrate 5000000 restart-ms 100 fd on"
  local cmds=() ip rule tmp
  local ee_up_args="up type can bitrate 1000000 restart-ms 100" ee_ifaces ee_iface
  ee_ifaces="$(ee_can_ifaces "${iface}")" || exit 1
  for ip in /usr/sbin/ip /sbin/ip; do  # Ubuntu 上 /sbin 是 /usr/sbin 的软链接，两个路径都放行
    cmds+=("${ip} link set ${iface} down" "${ip} link set ${iface} ${up_args}")
    for ee_iface in ${ee_ifaces}; do
      cmds+=("${ip} link set ${ee_iface} down" "${ip} link set ${ee_iface} ${ee_up_args}")
    done
  done
  rule="$(id -un) ALL=(root) NOPASSWD: $(IFS=,; echo "${cmds[*]}" | sed 's/,/, /g')"
  if sudo test -f "${file}" && [[ "$(sudo cat "${file}")" == "${rule}" ]]; then  # 旧的通配符规则会被替换
    info "CAN 免密 sudo 规则已是最新（腰部 ${iface}${ee_ifaces:+，手 $(echo ${ee_ifaces})}）"
    return
  fi
  info "写入 ${file}"
  tmp="$(mktemp)"
  printf '%s\n' "${rule}" > "${tmp}"
  sudo visudo -cf "${tmp}" >/dev/null || { rm -f "${tmp}"; die "sudoers 规则校验失败，未写入"; }
  sudo install -m 0440 -o root -g root "${tmp}" "${file}"
  rm -f "${tmp}"
  info "已写入：${rule}"
}

host_pc_service_deb() {  # 按架构选 deb 文件名
  case "$(dpkg --print-architecture)" in
    amd64) printf 'XRoboToolkit_PC_Service_1.0.0_ubuntu_%s_amd64.deb' "$(. /etc/os-release && echo "${VERSION_ID}")" ;;
    arm64)
      if [[ "${XRT_PCS_HEADLESS:-0}" == "1" ]]; then printf 'XRoboToolkit-PC-Service-headless_1.0.0.0_arm64.deb'
      else printf 'XRoboToolkit-PC-Service_1.0.0.0_arm64.deb'; fi ;;
    *) die "不支持的架构: $(dpkg --print-architecture)" ;;
  esac
}

# 固定每个 deb 的 sha256：以 root 安装下载来的包之前先校验，release 附件被替换就会拒绝安装。
# 值取自 release v1.0.0 的附件摘要；ubuntu 24.04 amd64 的那个另外下载后自己算过，一致。
host_pc_service_sha256() {
  case "$1" in
    XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb) printf '61961067eb4b41f81ed7cae35f4690dbb0ddfefb329a12b24e0b90ebc46ada91' ;;
    XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb) printf 'bce661f0be0b8a246ceecb2e5f1675a81c26b834648dc7fdf23f8c0bfe2a5d19' ;;
    XRoboToolkit-PC-Service_1.0.0.0_arm64.deb) printf '2a6bc1088c77c363fb13030ed0fc8d0f048ec12bbd8ec61a68d2ebd48ed9f942' ;;
    XRoboToolkit-PC-Service-headless_1.0.0.0_arm64.deb) printf '532c605dfa1a02b05b7c285b856c91771c78623cded30ef5b16ea371de49ed5f' ;;
    *) return 1 ;;
  esac
}

host_pc_service() {
  local script="/opt/apps/roboticsservice/runService.sh"
  local release="https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0"
  local deb tmpdir expected actual
  if [[ -x "${script}" ]]; then info "XRoboToolkit PC Service 已安装（${script}）"; return; fi
  # PC Service 的 postinst 需要这两个桌面工具，精简容器通常未预装。
  if ! command -v xdg-icon-resource >/dev/null || ! command -v update-desktop-database >/dev/null; then
    as_root apt-get install -y --no-install-recommends xdg-utils desktop-file-utils
  fi
  deb="$(host_pc_service_deb)"
  tmpdir="$(mktemp -d)"
  info "下载 ${deb}"
  curl -fSL -o "${tmpdir}/${deb}" "${release}/${deb}" || die "下载失败：${release}/${deb}"
  expected="$(host_pc_service_sha256 "${deb}")" \
    || { rm -rf "${tmpdir}"; die "没有 ${deb} 的校验和（这个系统版本没有对应的安装包），未安装"; }
  actual="$(sha256sum "${tmpdir}/${deb}" | cut -d' ' -f1)"
  if [[ "${actual}" != "${expected}" ]]; then
    rm -rf "${tmpdir}"
    die "${deb} 的 sha256 不对（期望 ${expected}，实际 ${actual}），未安装"
  fi
  as_root apt-get install -y "${tmpdir}/${deb}"
  rm -rf "${tmpdir}"
  info "PC Service 已安装，SDK 已就绪"
}

step_host() {
  if is_container; then
    info "容器环境：仅准备 XR SDK，跳过 udev 和 CAN 配置"
  else
    host_udev
    host_sudoers
  fi
  host_pc_service
}

# ---- marvin：天机臂 SDK 版本 -------------------------------------------------------------------------
# 固件和 SDK 不配套时，驱动连上臂后读不到控制器版本，随即急停（marvin_hardware/CMakeLists.txt）。
is_marvin_firmware() { [[ " ${MARVIN_FIRMWARES[*]} " == *" $1 "* ]]; }

marvin_firmware_config() {  # 配置里选的版本；没选则为空；不是 338 / 343 时报错退出
  local value=""
  if [[ -f "${CONFIG}" ]]; then
    value="$(python3 - "${CONFIG}" <<'PY' 2>/dev/null || true
import sys, yaml
data = yaml.safe_load(open(sys.argv[1])) or {}
arm = (data.get("robot") or {}).get("arm") or {}
print(str((arm.get("interface") or {}).get("firmware", "") or "").strip())
PY
)"
  fi
  [[ -z "${value}" ]] || is_marvin_firmware "${value}" \
    || die "robot.arm.interface.firmware 只能是 338 或 343（${CONFIG} 里是 '${value}'）"
  printf '%s' "${value}"
}

marvin_firmware_built() {  # 已有编译结果用的版本（CMake 缓存）；没编译过则为空
  local cache="${WS_DIR}/build/marvin_hardware/CMakeCache.txt"
  [[ -f "${cache}" ]] && sed -n 's/^MARVIN_FIRMWARE:STRING=//p' "${cache}" | head -n 1
  return 0
}

marvin_firmware_suggestion() {  # 没选时的建议值：沿用已编译的版本，否则用 CMake 默认值
  local built
  built="$(marvin_firmware_built)"
  if is_marvin_firmware "${built}"; then printf '%s' "${built}"; else printf '%s' "${MARVIN_FIRMWARE_DEFAULT}"; fi
}

marvin_cmake_arg() {  # 编译参数；配置里没选时用建议值并提醒
  local value
  value="$(marvin_firmware_config)"
  if [[ -z "${value}" ]]; then
    value="$(marvin_firmware_suggestion)"
    warn "配置里没有选择天机臂 SDK 版本，这次用 ${value}；用 ./scripts/setup.sh marvin 选择并写入配置"
  fi
  printf -- '-DMARVIN_FIRMWARE=%s' "${value}"
}

step_marvin() {
  local current suggestion choice
  current="$(marvin_firmware_config)"
  suggestion="${current:-$(marvin_firmware_suggestion)}"
  if [[ -n "${YES}" ]]; then
    choice="${suggestion}"
  else
    info "天机臂 SDK 版本要和臂的控制器固件一致：338（38 版固件）或 343（43 版及以上）。"
    info "选错时驱动连上臂后读不到控制器版本，会立即急停。"
    read -r -p "选择 338 或 343（回车使用 ${suggestion}）：" choice \
      || die "没有读到输入；要使用建议值 ${suggestion}，运行 ./scripts/setup.sh marvin --yes"
    choice="${choice:-${suggestion}}"
  fi
  is_marvin_firmware "${choice}" || die "天机臂 SDK 版本只能是 338 或 343，输入的是 '${choice}'"
  PYTHONPATH="${SETUP_PYTHONPATH}:${PYTHONPATH:-}" python3 - "${CONFIG}" "${choice}" <<'PY'
import sys
from robot_config.config_file import set_fields
set_fields(sys.argv[1], "robot.arm.interface", {"firmware": int(sys.argv[2])})
PY
  info "已写入 robot.arm.interface.firmware: ${choice}（之后编译 marvin_hardware 都用这个版本）"
}

# ---- build ----------------------------------------------------------------------------------------
step_build() {
  colcon_ignore_submodule_dirs
  [[ -n "${MINIMAL}" ]] || rokae_sdk_libs  # rokae_hardware 要这些库；--minimal 不编它
  source_ros
  cd "${WS_DIR}"
  export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-8}"
  export MAKEFLAGS="${MAKEFLAGS:--j${CMAKE_BUILD_PARALLEL_LEVEL}}"
  local select=(--packages-skip "${OCS2_SKIP_PACKAGES[@]}") marvin_arg
  [[ -n "${MINIMAL}" ]] && select=(--packages-up-to "${MINIMAL_PACKAGES[@]}")  # 和完整编译用同样的安装方式
  marvin_arg="$(marvin_cmake_arg)"
  colcon build "${select[@]}" --parallel-workers "${COLCON_PARALLEL_WORKERS:-8}" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF "${marvin_arg}"
}

# ---- 相机 camera_setup、末端 end_effector_setup、负载辨识 robot_config；XR 配对由 xr_teleop 处理 ---------------------------------
SETUP_PYTHONPATH="${WS_DIR}/src/robot_setup/robot_config:${WS_DIR}/src/robot_setup/camera_setup:${WS_DIR}/src/robot_setup/end_effector_setup:${WS_DIR}/src/robot_setup/xr_setup"
SETUP_RUNTIME_PYTHONPATH="${WS_DIR}/src/robot_utils/python"
run_python_step() {
  source_ros --workspace
  local python_status=0 module command="" python_path="${SETUP_PYTHONPATH}"
  case "$1" in
    cameras)
      module=camera_setup.session
      python_path="${python_path}:${SETUP_RUNTIME_PYTHONPATH}" ;;
    ee) module=end_effector_setup.session ;;
    dyn) module=robot_config.setup_steps command=dyn ;;
    xr)
      module=xr_setup.session
      python_path="${python_path}:${WS_DIR}/src/robot_teleop/xr_teleop" ;;
  esac
  # shellcheck disable=SC2086
  PYTHONPATH="${python_path}:${PYTHONPATH:-}" \
    python3 -m "${module}" ${command} --config "${CONFIG}" --ws-root "${WS_DIR}" ${YES} ${RESET} $([[ "$1" == "xr" ]] && echo xr) || python_status=$?
  if (( python_status == 130 )); then exit 130; fi  # Ctrl+C cancels the full init, not only this step.
  return "${python_status}"
}
# 本机配置：不需要编译过的工作区，只要源码路径和 ruamel.yaml
step_config() {
  PYTHONPATH="${SETUP_PYTHONPATH}:${PYTHONPATH:-}" \
    python3 -m robot_config.setup_steps config --config "${CONFIG}" ${ENTRY:+--entry "${ENTRY}"} ${YES}
}
# 旧进程：每次进入相机/XR 配置前全部清掉（包括运行中的运控驱动），卡住的驱动会一直占着相机，旧的桥接节点会和新的抢头显数据。
# 相机：向导本身、所有相机驱动（含预览驱动和运控的 /camera_head 等），一台相机只能被一个驱动占用。
OLD_CAMERA_SETUP_PATTERNS=(
  '^python3 -m camera_setup\.session'
  'robot_startup cameras\.launch\.py'
  'orbbec_camera.*(gemini305|gemini_330_series)\.launch\.py'
  'component_container.*__ns:=/[A-Za-z0-9_]*camera_(head|left_wrist|right_wrist)'
)
# XR：向导本身、桥接节点、PC Service（配对时由向导自己重新起）。
OLD_XR_SETUP_PATTERNS=(
  '^python3 -m xr_setup\.session'
  'xr_teleop.*xr_bridge_node'
  'RoboticsServiceProcess'
)
old_setup_pids() {  # 参数：名字数组的名字
  local -n patterns="$1"
  local pattern
  for pattern in "${patterns[@]}"; do pgrep -f "${pattern}" || true; done | sort -un | grep -vx "$$" || true
}
stop_old_setup_processes() {  # 参数：中文名、模式数组的名字；先 INT（让驱动自己释放设备），不行再 TERM、KILL
  local label="$1" array="$2" pids step signal seconds
  pids="$(old_setup_pids "${array}" | tr '\n' ' ')"
  [[ -n "${pids// /}" ]] || return 0
  info "清理上次${label}配置残留的进程：${pids}"
  for step in "INT 5" "TERM 2" "KILL 1"; do
    signal="${step% *}" seconds="${step#* }"
    # shellcheck disable=SC2086
    kill -"${signal}" ${pids} 2>/dev/null || true
    for _ in $(seq 1 $((seconds * 10))); do
      pids="$(old_setup_pids "${array}" | tr '\n' ' ')"
      [[ -n "${pids// /}" ]] || return 0
      sleep 0.1
    done
  done
  warn "仍有${label}配置进程没能结束：${pids}"
}
stop_old_camera_setup() {
  local pids=()
  mapfile -t pids < <(old_setup_pids OLD_CAMERA_SETUP_PATTERNS)
  (( ${#pids[@]} )) || return 0
  info "清理上次相机配置残留的进程：${pids[*]}"
  if ! PYTHONPATH="${SETUP_RUNTIME_PYTHONPATH}:${PYTHONPATH:-}" python3 - "${pids[@]}" <<'PY'
import sys

from robot_runtime.processes import start_ticks, stop_processes

targets = {pid: start_ticks(pid) for pid in map(int, sys.argv[1:])}
try:
    stopped = stop_processes(lambda: targets)
except PermissionError:
    stopped = False
sys.exit(0 if stopped else 1)
PY
  then
    warn "仍有相机配置进程没能结束：${pids[*]}"
  fi
}
stop_old_xr_setup() { stop_old_setup_processes XR OLD_XR_SETUP_PATTERNS; }
step_cameras() { stop_old_camera_setup; run_python_step cameras; }
step_ee() { run_python_step ee; }
step_dyn() { run_python_step dyn; }
step_xr() { stop_old_xr_setup; run_python_step xr; }

# ---- 已配置或默认值可用的步骤不再配置 ---------------------------------------------------------------
# 完整流程里这样的步骤直接跳过；单步命令（如 ./scripts/setup.sh cam）和 --force 永远重新配置。
step_ready() {  # 返回 0 表示不用配置；原因写入 READY_REASON
  local status=0 python_path="${SETUP_PYTHONPATH}:${PYTHONPATH:-}"
  READY_REASON=""
  case "$1" in
    marvin)
      local firmware
      firmware="$(marvin_firmware_config)" || return 1   # 值不对：不跳过，由 step_marvin 报错
      [[ -n "${firmware}" ]] || return 1
      READY_REASON="已选择 ${firmware}" ;;
    config)
      READY_REASON="$(PYTHONPATH="${python_path}" python3 -m robot_config.setup_steps config --config "${CONFIG}" --status 2>/dev/null)" || status=$? ;;
    cameras|ee|dyn)
      READY_REASON="$(PYTHONPATH="${python_path}" python3 -m robot_config.setup_steps status --config "${CONFIG}" "$1" 2>/dev/null)" || status=$? ;;
    xr)
      READY_REASON="$(PYTHONPATH="${WS_DIR}/src/robot_teleop/xr_teleop:${python_path}" python3 -m xr_setup.session \
        --config "${CONFIG}" --ws-root "${WS_DIR}" --status xr 2>/dev/null)" || status=$? ;;
    *) return 1 ;;
  esac
  return "${status}"
}

# ---- summary --------------------------------------------------------------------------------------
step_summary() {
  source_ros --workspace
  PYTHONPATH="${SETUP_PYTHONPATH}:${PYTHONPATH:-}" \
    python3 -m robot_config.setup_steps check "${CONFIG}"
  local chosen built
  chosen="$(marvin_firmware_config)"
  built="$(marvin_firmware_built)"
  info "天机臂 SDK：配置 ${chosen:-未选择}，已编译 ${built:-未编译}"
  if [[ -n "${chosen}" && -n "${built}" && "${chosen}" != "${built}" ]]; then
    warn "天机臂 SDK 的配置和编译结果不一致：运行 ./scripts/setup.sh build 重新编译"
  fi
}

# ==================================================================================================
# 4. 入口
# ==================================================================================================
parse_args() {  # 结果：STEPS（要执行的步骤）、YES、MINIMAL、CONFIG
  STEPS=()
  FULL_INIT=""
  local from_step="" step_index first_step
  while (( $# )); do
    case "$1" in
      --yes|-y) YES="--yes" ;;
      --minimal) MINIMAL=1 ;;
      --from) from_step=1 ;;
      --force) FORCE=1 ;;
      --reset) RESET="--reset" ;;
      -c) [[ -n "${2:-}" ]] || die "-c 需要配置文件路径"; CONFIG="$2"; shift ;;
      --entry) [[ -n "${2:-}" ]] || die "--entry 需要取值（whole_body | split_body | dual_arm）"; ENTRY="$2"; shift ;;
      -h|--help|help) usage; exit 0 ;;
      cam) STEPS+=(cameras) ;;
      check) STEPS+=(summary) ;;
      *) [[ -n "${STEP_TITLE[$1]:-}" ]] || die "未知参数: $1（./scripts/setup.sh help 查看用法）"; STEPS+=("$1") ;;
    esac
    shift
  done
  if [[ -n "${from_step}" ]]; then
    (( ${#STEPS[@]} == 1 )) || die "--from 需要且只能指定一个起始步骤，例如 ./scripts/setup.sh cam --from"
    first_step="${STEPS[0]}"
    for step_index in "${!STEP_ORDER[@]}"; do
      if [[ "${STEP_ORDER[$step_index]}" == "${first_step}" ]]; then
        STEPS=("${STEP_ORDER[@]:step_index}")
        break
      fi
    done
    FULL_INIT=1
  elif (( ${#STEPS[@]} == 0 )); then
    STEPS=("${STEP_ORDER[@]}")
    FULL_INIT=1
  fi
  [[ -z "${RESET}" || " ${STEPS[*]} " == *" dyn "* ]] || die "--reset 只用于 dyn：./scripts/setup.sh dyn --reset"
  [[ "${CONFIG}" = /* ]] || CONFIG="${PWD}/${CONFIG}"
  [[ -f "${CONFIG}" || " ${STEPS[*]} " == *" config "* ]] \
    || die "配置文件不存在: ${CONFIG}（先运行 ./scripts/setup.sh config 生成）"
}

is_soft_step() {
  local s
  for s in "${SOFT_STEPS[@]}"; do [[ "$1" == "$s" ]] && return 0; done
  return 1
}

step_number() {  # 本次流程里的序号：i/N
  local i
  local order=("${STEPS[@]}")
  [[ -z "${FULL_INIT}" ]] || order=("${STEP_ORDER[@]}")
  for i in "${!order[@]}"; do
    [[ "${order[$i]}" == "$1" ]] && { printf '%d/%d' $((i + 1)) "${#order[@]}"; return; }
  done
}

run_step() {  # 结果记入 RESULTS；SOFT_STEPS 失败不中断
  local name="$1"
  log "$(step_number "${name}") ${STEP_TITLE[${name}]}"
  if [[ -n "${FULL_INIT}" && -z "${FORCE}" ]] && step_ready "${name}"; then
    info "跳过：${READY_REASON}（重新配置：./scripts/setup.sh $(step_command "${name}")）"
    RESULTS+=("$(step_command "${name}"): ${READY_REASON}，已跳过")
    return 0
  fi
  if ! is_soft_step "${name}"; then
    "step_${name}"  # 必须成功：失败时 set -e 直接中断
    RESULTS+=("$(step_command "${name}"): OK")
  elif "step_${name}"; then
    RESULTS+=("$(step_command "${name}"): OK")
  else
    RESULTS+=("$(step_command "${name}"): 未完成（处理后运行 ./scripts/setup.sh $(step_command "${name}") 重试）")
    INCOMPLETE+=("${STEP_TITLE[${name}]}")
  fi
}

main() {
  parse_args "$@"
  RESULTS=()
  INCOMPLETE=()
  local step
  for step in "${STEPS[@]}"; do run_step "${step}"; done
  log "初始化结果"
  printf '  %s\n' "${RESULTS[@]}"
  if (( ${#INCOMPLETE[@]} )); then
    warn "初始化尚未完成：${INCOMPLETE[*]}。请按上方单步命令重试，无需重新执行已完成的步骤"
    return 1
  fi
  if [[ -n "${FULL_INIT}" ]]; then
    printf '\n下一步：./scripts/start.sh\n'
  else
    info "所选步骤已完成"
  fi
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then main "$@"; fi
