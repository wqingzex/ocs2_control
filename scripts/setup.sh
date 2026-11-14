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

# 步骤的顺序和标题；序号 i/N 由这张表算出来。SOFT_STEPS 失败时只记录，不中断后面的步骤。
STEP_ORDER=(deps host build cameras hand dyn xr summary)
declare -A STEP_TITLE=(
  [deps]="依赖：子模块 + rosdep"
  [host]="主机配置：相机 udev、腰部 CAN 免密 sudo、XRoboToolkit PC Service"
  [build]="编译整个 ROS 2 工作区（运控、驱动、机器人模型、相机、XR、控制台）"
  [cameras]="相机"
  [hand]="手"
  [dyn]="负载辨识（有默认值就不用配置）"
  [xr]="XR（头显）"
  [summary]="配置检查"
)
SOFT_STEPS=(hand dyn xr summary)

# ocs2_ros2（子模块，src/third_party/ocs2_ros2，固定在 ae75126）带着一批 W2 用不到的示例包：编译和 rosdep 都跳过。
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
# build-workspace --minimal 只编译这些包及其依赖。
MINIMAL_PACKAGES=(
  fiveages_w2_description juxie_hardware marvin_hardware modbus_hardware
  joystick_teleop keyboard_teleop xr_teleop topic_based_hardware teleop_app w2_camera w2_hand
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
    build) printf 'build-workspace' ;;
    cameras) printf 'cam' ;;
    summary) printf 'check' ;;
    *) printf '%s' "$1" ;;
  esac
}

usage() {
  cat <<'EOF_USAGE'
初始化：./scripts/setup.sh
  依次完成：依赖、主机 / SDK、编译、相机配置、手配置、负载辨识、XR 配对、最终检查。
  已配置（有确认记录）或默认值可用的步骤会跳过；--force 全部重新配置。
  相机和手会弹出本地配置窗口，确认后保存到 config/robot.local.yaml。
  容器内也保留完整向导；仅不执行物理机的 udev 和 CAN 操作。
单步重试：./scripts/setup.sh <步骤>，只执行指定步骤：
  cam 相机；hand 手；xr 头显配对；dyn 负载辨识（逐臂询问，默认不辨识；--reset 恢复默认负载）。
  build-workspace 编译整个 ROS 2 工作区：运控、驱动、机器人模型、相机、XR、控制台。
        使用增量编译，复用已有结果；不执行设备配置或配对，不启动运控。
  deps 依赖；host 主机 / SDK；check 配置检查。
从某步继续：./scripts/setup.sh cam --from
  从相机开始，依次执行手、XR 配对和最终检查；其他步骤同样支持 --from。
可选参数：--yes 使用建议值；--force 重新配置已配置的步骤；--minimal 只编译 W2 必需的包；-c 指定配置文件。
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

# MarvinPlatform_EN 是 PyInstaller 打包脚本，不是 ROS 包；colcon 扫描时会执行它的 setup.py 并报错，所以让 colcon 忽略。
# 子模块里的 COLCON_IGNORE 同时加入子模块的 info/exclude，避免子模块显示为有未跟踪文件。
colcon_ignore_marvin_platform() {
  local dir exclude
  for dir in "${WS_DIR}"/src/third_party/TJ_FX_ROBOT_CONTRL_SDK_*/MarvinPlatform_EN; do
    [[ -d "${dir}" ]] || continue
    touch "${dir}/COLCON_IGNORE"
    exclude="$(git -C "${dir}" rev-parse --path-format=absolute --git-path info/exclude 2>/dev/null || true)"
    if [[ -n "${exclude}" ]] && ! grep -qxF 'MarvinPlatform_EN/COLCON_IGNORE' "${exclude}" 2>/dev/null; then
      mkdir -p "$(dirname "${exclude}")"
      echo 'MarvinPlatform_EN/COLCON_IGNORE' >> "${exclude}"
    fi
  done
}

step_deps() {
  git -C "${WS_DIR}" submodule update --init --recursive --depth 1  # 浅克隆：只取固定的提交
  colcon_ignore_marvin_platform
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

# 腰部 CAN 接口名：环境变量 CAN_IFACE > config/robot.local.yaml 的 hardware.body_can_interface > can0。
# 名字会写进 sudoers 规则，必须是合法的接口名（和驱动 jx_hardware 的校验一致）。
body_can_iface() {
  local iface="${CAN_IFACE:-}"
  if [[ -z "${iface}" && -f "${CONFIG}" ]]; then
    iface="$(python3 - "${CONFIG}" <<'PY' 2>/dev/null || true
import sys, yaml
data = yaml.safe_load(open(sys.argv[1])) or {}
print(str((data.get("hardware") or {}).get("body_can_interface", "") or "").strip())
PY
)"
  fi
  iface="${iface:-can0}"
  [[ "${iface}" =~ ^[A-Za-z0-9_.][A-Za-z0-9_.-]{0,14}$ ]] || die "CAN 接口名不合法：'${iface}'（1 到 15 个字符，只能用字母、数字、_ . -，不能以 - 开头）"
  printf '%s' "${iface}"
}

# 只放行 jx_hardware.cpp 实际执行的两条 `sudo ip link set <can> ...`，参数写死、不用通配符
host_sudoers() {
  local iface file="/etc/sudoers.d/fa-w2-can"
  iface="$(body_can_iface)"
  local up_args="up type can bitrate 1000000 dbitrate 5000000 restart-ms 100 fd on"
  local cmds=() ip rule tmp
  for ip in /usr/sbin/ip /sbin/ip; do  # Ubuntu 上 /sbin 是 /usr/sbin 的软链接，两个路径都放行
    cmds+=("${ip} link set ${iface} down" "${ip} link set ${iface} ${up_args}")
  done
  rule="$(id -un) ALL=(root) NOPASSWD: $(IFS=,; echo "${cmds[*]}" | sed 's/,/, /g')"
  if sudo test -f "${file}" && [[ "$(sudo cat "${file}")" == "${rule}" ]]; then  # 旧的通配符规则会被替换
    info "腰部 CAN（${iface}）免密 sudo 规则已是最新"
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

# ---- build ----------------------------------------------------------------------------------------
step_build() {
  colcon_ignore_marvin_platform
  source_ros
  cd "${WS_DIR}"
  export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-4}"
  export MAKEFLAGS="${MAKEFLAGS:--j${CMAKE_BUILD_PARALLEL_LEVEL}}"
  local select=(--packages-skip "${OCS2_SKIP_PACKAGES[@]}")
  [[ -n "${MINIMAL}" ]] && select=(--packages-up-to "${MINIMAL_PACKAGES[@]}")  # 和完整编译用同样的安装方式
  colcon build "${select[@]}" --parallel-workers "${COLCON_PARALLEL_WORKERS:-2}" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
}

# ---- 相机 w2_camera、手 w2_hand、负载辨识 w2_payload；XR 由遥操作模块处理 ---------------------------------
PAYLOAD_PYTHONPATH="${WS_DIR}/src/robot_payload/w2_payload/common:${WS_DIR}/src/robot_payload/w2_payload/w2_camera:${WS_DIR}/src/robot_payload/w2_payload/w2_hand"
run_python_step() {
  source_ros --workspace
  local python_status=0 module python_path="${PAYLOAD_PYTHONPATH}"
  case "$1" in
    cameras) module=w2_camera.camera_setup ;;
    hand) module=w2_hand.hand_setup ;;
    dyn) module=w2_payload.load_identify ;;
    xr)
      module=xr_teleop.pairing
      python_path="${python_path}:${WS_DIR}/src/robot_payload/input/xr_teleop" ;;
  esac
  # shellcheck disable=SC2086
  PYTHONPATH="${python_path}:${PYTHONPATH:-}" \
    python3 -m "${module}" --config "${CONFIG}" --ws-root "${WS_DIR}" ${YES} ${RESET} $([[ "$1" == "xr" ]] && echo xr) || python_status=$?
  if (( python_status == 130 )); then exit 130; fi  # Ctrl+C cancels the full init, not only this step.
  return "${python_status}"
}
step_cameras() { run_python_step cameras; }
step_hand() { run_python_step hand; }
step_dyn() { run_python_step dyn; }
step_xr() { run_python_step xr; }

# ---- 已配置或默认值可用的步骤不再配置 ---------------------------------------------------------------
# 完整流程里这样的步骤直接跳过；单步命令（如 ./scripts/setup.sh cam）和 --force 永远重新配置。
step_ready() {  # 返回 0 表示不用配置；原因写入 READY_REASON
  local status=0 python_path="${PAYLOAD_PYTHONPATH}:${PYTHONPATH:-}"
  READY_REASON=""
  case "$1" in
    cameras|hand|dyn)
      READY_REASON="$(PYTHONPATH="${python_path}" python3 -m w2_payload.config_status status --config "${CONFIG}" "$1" 2>/dev/null)" || status=$? ;;
    xr)
      READY_REASON="$(PYTHONPATH="${WS_DIR}/src/robot_payload/input/xr_teleop:${python_path}" python3 -m xr_teleop.pairing \
        --config "${CONFIG}" --ws-root "${WS_DIR}" --status xr 2>/dev/null)" || status=$? ;;
    *) return 1 ;;
  esac
  return "${status}"
}

# ---- summary --------------------------------------------------------------------------------------
step_summary() {
  source_ros --workspace
  PYTHONPATH="${PAYLOAD_PYTHONPATH}:${PYTHONPATH:-}" \
    python3 -m w2_payload.config_status check "${CONFIG}"
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
      -h|--help|help) usage; exit 0 ;;
      cam) STEPS+=(cameras) ;;
      check) STEPS+=(summary) ;;
      build-workspace) STEPS+=(build) ;;
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
  if [[ ! -f "${CONFIG}" && "${CONFIG}" == "${WS_DIR}/config/robot.local.yaml" ]]; then  # 新机器：先用模板
    cp "${WS_DIR}/config/robot.template.yaml" "${CONFIG}"
    info "已从 config/robot.template.yaml 生成 ${CONFIG}"
  fi
  [[ -f "${CONFIG}" ]] || die "配置文件不存在: ${CONFIG}"
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
