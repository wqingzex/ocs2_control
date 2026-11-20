#!/usr/bin/env bash
# robot_control_suite 命令行启动入口：读 config/robot.local.yaml，前台启动模块。
# 用法见 ./scripts/start.sh help。
set -Eeuo pipefail

usage() {
  cat <<'EOF_USAGE'
robot_control_suite 命令行入口（面向专业人员）
首次使用：先运行 ./scripts/setup.sh。

启动
  ./scripts/start.sh                      按配置启动运控、XR 和相机，Ctrl+C 停止
  ./scripts/start.sh sim | isaac          同上，硬件改为仿真（mock_components）/ Isaac
  ./scripts/start.sh viz                  只在 RViz 里看机器人模型（带关节滑块，不启动控制）
  ./scripts/start.sh up|down|restart|status|logs [模块]   按模块管理（status --hz 看话题频率）
  ./scripts/start.sh diag [--minutes N] [原因]           出问题后用：汇总出错前 N 分钟（默认 10）的各模块日志、故障码、健康情况，
                                                         和各模块最近收发的数据，指出最先出问题的一环；打成一个包，报告同时打印
  ./scripts/start.sh dyn-identify         负载辨识
  ./scripts/start.sh console              Web 控制台，前台运行（Ctrl+C 停止）
  ./scripts/start.sh console start|stop|restart|status [--port N] [--host H]
                                          后台启动 / 停止 / 重启 / 查看控制台（默认端口 8765）
  ./scripts/start.sh console --open       后台启动并用浏览器打开（双击 robot_control_suite控制台 用的就是它）
录制与回放
  ./scripts/start.sh record [名称] [--topics joint|xr|collect|/a,/b]   默认只录关节
  ./scripts/start.sh play 名称 [--rate 0.5] [--loop] [--paused] [--start 秒] [--xr]
  ./scripts/start.sh bags [--delete 名称 --yes]                        列出 / 删除录包
其他
  ./scripts/start.sh check | build
  ./scripts/start.sh bus_probe [--side left|right] [--duration S] [--cycle-hz N] [--io-mode async|sync]
                                          按配置里的末端测它的 USB 总线：往返时延、丢帧、请求频率（要先停运控；--help 看全部选项）

选项（命令前后都可以）
  -c 配置文件        --set 字段=值（本次运行有效，可重复，如 --set robot.hardware=isaac）
  --entry whole_body|split_body|dual_arm   --end-effector none|linkerhand_l6|xhand1|wuji_hand2|jodell_rg75
  --end-effector-bus usb_rs485|arm_rs485|usb_can|ethernet
  --rviz             同时启动 RViz（用于 start / sim / isaac / up）；up control 默认带 RViz，不要 RViz 用 up control --no-rviz
EOF_USAGE
}

WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"   # 仓库根目录（脚本在 scripts/ 下）
DEFAULT_CONFIG="${WS_DIR}/config/robot.local.yaml"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
die() { echo "[ERROR] $*" >&2; exit 1; }

source_ros() {
  local setup="/opt/ros/${ROS_DISTRO}/setup.bash"
  [[ -f "${setup}" ]] || die "未找到 ${setup}（请安装 ROS 2 ${ROS_DISTRO} 或设置 ROS_DISTRO）"
  set +u
  # shellcheck disable=SC1090
  source "${setup}"
  # shellcheck disable=SC1091
  [[ -f "${WS_DIR}/config/ros_env.sh" ]] && source "${WS_DIR}/config/ros_env.sh"  # RMW / Domain ID / Zenoh (README 2.1)
  if [[ "${1:-}" == "--workspace" ]]; then
    [[ -f "${WS_DIR}/install/setup.bash" ]] || die "工作区未编译：先运行 ./scripts/setup.sh"
    # shellcheck disable=SC1091
    source "${WS_DIR}/install/setup.bash"
  fi
  set -u
}

resolve_config() {
  local cfg="${1:-${DEFAULT_CONFIG}}"
  [[ "${cfg}" = /* ]] || cfg="${PWD}/${cfg}"
  [[ -f "${cfg}" ]] || die "配置文件不存在: ${cfg}"
  printf '%s' "${cfg}"
}

check_config() {
  python3 -m robot_app.check "$1" "${WS_DIR}" ${SET_ARGS[@]+"${SET_ARGS[@]}"} || die "配置校验失败（见上方提示）"
}

cmd_build() {
  exec "${WS_DIR}/scripts/setup.sh" build "$@"  # 包清单只在 setup.sh 里维护
}

cmd_check() {
  source_ros --workspace
  check_config "$(resolve_config "${1:-}")"
}

CONFIG_ARG=""
SET_ARGS=()   # --set 字段=值：只对本次运行生效，不写配置文件
WITH_ARGS=()  # --rviz：同时启动 RViz

units() {
  source_ros --workspace
  local cfg args=("$@")
  cfg="$(resolve_config "${CONFIG_ARG}")"
  if [[ "${1:-}" == "up" ]]; then args=(up ${WITH_ARGS[@]+"${WITH_ARGS[@]}"} "${@:2}"); fi
  exec python3 -m robot_app.units_cli --config "${cfg}" --ws-root "${WS_DIR}" \
    ${SET_ARGS[@]+"${SET_ARGS[@]}"} "${args[@]}"
}

cmd_start() {
  source_ros --workspace
  local cfg
  cfg="$(resolve_config "${CONFIG_ARG}")"
  check_config "${cfg}"
  exec python3 -m robot_app.units_cli --config "${cfg}" --ws-root "${WS_DIR}" \
    ${SET_ARGS[@]+"${SET_ARGS[@]}"} up --foreground ${WITH_ARGS[@]+"${WITH_ARGS[@]}"}
}

# 双击入口与终端共用本脚本；--open 无需终端，错误显示为桌面通知并保留日志。
console_notify() {
  printf '%s\n' "$1" >&2
  if command -v notify-send >/dev/null 2>&1; then
    notify-send "robot_control_suite 控制台" "$1" || true
  elif command -v zenity >/dev/null 2>&1; then
    zenity --error --title="robot_control_suite 控制台" --text="$1" || true
  fi
}

console_port_open() { (exec 3<>"/dev/tcp/127.0.0.1/$1") 2>/dev/null; }
# 端口上是不是 robot_control_suite 控制台本身：只看"端口能连上"会把别的程序（例如本机代理占着 8080）当成控制台；
# 控制台的状态接口返回含 units 的 JSON。环境里的 http_proxy 不能拦截本机请求，所以不走代理。
console_is_up() {
  python3 -c '
import json, sys, urllib.request
opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
try:
    with opener.open("http://127.0.0.1:%s/api/state" % sys.argv[1], timeout=2) as response:
        sys.exit(0 if "units" in json.load(response) else 1)
except Exception:
    sys.exit(1)
' "$1" 2>/dev/null
}

CONSOLE_DEFAULT_PORT="${FA_CONSOLE_PORT:-8765}"

# 本环境（宿主机或容器，PID 空间不同）里监听该端口的控制台进程号。
console_pids() {
  local port="$1" pid cmd
  for pid in $(pgrep -f "python3 -m robot_app.console" || true); do
    cmd="$(tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null)" || continue
    if [[ "${cmd}" == *"--port ${port} "* || "${cmd}" == *"--port ${port}" ]] \
       || { [[ "${cmd}" != *"--port "* && "${port}" == "${CONSOLE_DEFAULT_PORT}" ]]; }; then
      echo "${pid}"
    fi
  done
}

# 后台启动控制台并等它就绪：成功返回 0；端口被别的程序占用或启动失败返回 1（提示已经打印）。
console_launch() {
  local port="$1" host="$2" cfg="$3" log_dir="${WS_DIR}/logs/console" logfile pid attempt
  if console_is_up "${port}"; then return 0; fi
  if console_port_open "${port}"; then
    console_notify "端口 ${port} 被别的程序占用（查看：ss -ltnp | grep :${port}），控制台没有启动；换一个端口：--port ${CONSOLE_DEFAULT_PORT}"
    return 1
  fi
  mkdir -p "${log_dir}"
  logfile="${log_dir}/$(date +%Y%m%d_%H%M%S).log"
  (umask 077 && : > "${logfile}")
  ln -sfn "$(basename "${logfile}")" "${log_dir}/latest.log"
  setsid nohup bash "${WS_DIR}/scripts/start.sh" -c "${cfg}" console --port "${port}" --host "${host}" \
    > "${logfile}" 2>&1 < /dev/null &
  pid=$!
  for attempt in {1..60}; do
    console_is_up "${port}" && return 0
    kill -0 "${pid}" 2>/dev/null || break
    sleep 0.5
  done
  console_notify "控制台启动失败，日志：${log_dir}/latest.log"
  tail -n 5 "${logfile}" >&2
  return 1
}

# 停止控制台：先 TERM，5 秒内没退出再 KILL。控制台只提供网页，停止它不影响运控等模块。
console_stop() {
  local port="$1" pids pid attempt
  pids="$(console_pids "${port}")"
  if [[ -z "${pids}" ]]; then
    if console_port_open "${port}"; then
      echo "端口 ${port} 上有程序在监听，但不是本环境里的控制台（宿主机和容器的进程互相看不到：在启动它的那一边停止，或 ss -ltnp | grep :${port} 查看）" >&2
      return 1
    fi
    echo "控制台没有在运行（端口 ${port}）"
    return 0
  fi
  # shellcheck disable=SC2086
  kill ${pids} 2>/dev/null || true
  for attempt in {1..10}; do
    [[ -z "$(console_pids "${port}")" ]] && { echo "控制台已停止（端口 ${port}）"; return 0; }
    sleep 0.5
  done
  # shellcheck disable=SC2086
  kill -9 ${pids} 2>/dev/null || true
  echo "控制台已强制停止（端口 ${port}）"
}

console_status() {
  local port="$1" pids
  pids="$(console_pids "${port}" | tr '\n' ' ')"
  if console_is_up "${port}"; then
    echo "控制台运行中：http://127.0.0.1:${port}/${pids:+（进程 ${pids% }）}"
  elif console_port_open "${port}"; then
    echo "端口 ${port} 被别的程序占用，控制台没有运行" >&2
    return 1
  else
    echo "控制台没有运行（端口 ${port}）"
    return 1
  fi
}

cmd_open_console() {
  local port="${FA_CONSOLE_PORT:-8765}" host="127.0.0.1"  # 默认只监听本机；其他电脑访问要显式 --host 0.0.0.0
  while (( $# )); do
    case "$1" in
      --port) [[ -n "${2:-}" ]] || die "--port 需要端口号"; port="$2"; shift 2 ;;
      --host) [[ -n "${2:-}" ]] || die "--host 需要监听地址"; host="$2"; shift 2 ;;
      -h|--help) usage; return ;;
      *) die "未知控制台参数: $1" ;;
    esac
  done
  [[ "${port}" =~ ^[0-9]{1,5}$ ]] && (( 10#${port} > 0 && 10#${port} <= 65535 )) \
    || die "无效端口号: ${port}"
  local cfg log_dir="${WS_DIR}/logs/console"
  local page="${XDG_RUNTIME_DIR:-${HOME}}/.robot_control_suite_console_open.html"
  cfg="$(resolve_config "${CONFIG_ARG}")"

  # 先打开等待页，避免加载 ROS 时双击没有反馈。
  python3 - "${page}" "${port}" "${log_dir}/latest.log" <<'PY'
import json
import os
import sys
from pathlib import Path

page, port, logfile = sys.argv[1:]
url = json.dumps(f"http://127.0.0.1:{port}/")
message = json.dumps(f"控制台没有启动，日志见 {logfile}", ensure_ascii=True)
os.umask(0o077)
Path(page).write_text('''<!doctype html><meta charset="utf-8"><title>robot_control_suite 控制台</title>
<body style="font:16px sans-serif;padding:2em"><p id="m">正在启动控制台…</p>
<script>
const url = ''' + url + ''', base = url;
let tries = 0;
(async function wait() {
  try { await fetch(base, {mode: 'no-cors', cache: 'no-store'}); location.replace(url); }
  catch (e) {
    if (++tries > 120) document.getElementById('m').textContent = ''' + message + ''';
    else setTimeout(wait, 500);
  }
})();
</script>
''', encoding="utf-8")
PY
  xdg-open "${page}" >/dev/null 2>&1 \
    || console_notify "无法自动打开浏览器，请手动访问 http://127.0.0.1:${port}/"

  console_launch "${port}" "${host}" "${cfg}" || return 1
  console_notify "控制台已打开：http://127.0.0.1:${port}/（其他电脑访问需加 --host 0.0.0.0）"
}

cmd_console() {
  local sub="" open="" arg args=() port="${CONSOLE_DEFAULT_PORT}" host="127.0.0.1" cfg
  case "${1:-}" in start|stop|restart|status) sub="$1"; shift ;; esac
  for arg in "$@"; do
    if [[ "${arg}" == "--open" ]]; then open=1; else args+=("${arg}"); fi
  done
  if [[ -n "${sub}" ]]; then
    set -- ${args[@]+"${args[@]}"}
    while (( $# )); do
      case "$1" in
        --port) [[ -n "${2:-}" ]] || die "--port 需要端口号"; port="$2"; shift 2 ;;
        --host) [[ -n "${2:-}" ]] || die "--host 需要监听地址"; host="$2"; shift 2 ;;
        *) die "未知控制台参数: $1" ;;
      esac
    done
    [[ "${port}" =~ ^[0-9]{1,5}$ ]] && (( 10#${port} > 0 && 10#${port} <= 65535 )) || die "无效端口号: ${port}"
    cfg="$(resolve_config "${CONFIG_ARG}")"
    case "${sub}" in
      start)   console_launch "${port}" "${host}" "${cfg}" && echo "控制台运行中：http://127.0.0.1:${port}/" ;;
      stop)    console_stop "${port}" ;;
      status)  console_status "${port}" ;;
      restart) console_stop "${port}" && console_launch "${port}" "${host}" "${cfg}" \
                 && echo "控制台运行中：http://127.0.0.1:${port}/" ;;
    esac
  elif [[ -n "${open}" ]]; then
    cmd_open_console ${args[@]+"${args[@]}"}
  else
    source_ros --workspace
    exec python3 -m robot_app.console --config "$(resolve_config "${CONFIG_ARG}")" --ws-root "${WS_DIR}" ${args[@]+"${args[@]}"}
  fi
}

cmd_dyn_identify() {
  source_ros --workspace
  [[ -t 0 ]] || die "负载辨识向导是交互式的，请在终端里运行"
  local reuse="" sdk_root="${WS_DIR}/src/third_party/TJ_FX_ROBOT_CONTRL_SDK_${MARVIN_FIRMWARE:-343}"
  [[ "${1:-}" == "--reuse-noload" ]] && reuse=1
  if [[ -z "${reuse}" ]]; then  # a finished no-load capture can be reused to go straight to the loaded test
    read -r -p "复用上次空载数据（NoLoadData.csv）并跳过空载采集？[y/N]: " answer
    [[ "${answer}" =~ ^[Yy]$ ]] && reuse=1
  fi
  if [[ -n "${reuse}" ]]; then export TOOL_DYN_REUSE_NOLOAD=1; else unset TOOL_DYN_REUSE_NOLOAD; fi
  # The SDK loads libMarvinSDK.so from SDK_PYTHON; link it from contrlSDK100343 when it is missing.
  local control_sdk="contrlSDK100343"; [[ "${MARVIN_FIRMWARE:-343}" == 338 ]] && control_sdk="contrlSDK"
  if [[ ! -f "${sdk_root}/SDK_PYTHON/libMarvinSDK.so" && -f "${sdk_root}/${control_sdk}/libMarvinSDK.so" ]]; then
    ln -sf "${sdk_root}/${control_sdk}/libMarvinSDK.so" "${sdk_root}/SDK_PYTHON/libMarvinSDK.so"
  fi
  cd "${WS_DIR}"  # the wizard writes logs/dyn_identify/ relative to the working directory
  exec ros2 run marvin_hardware tool_dyn_identify_wizard
}

# 录制、回放、录包列表：逻辑在 robot_app.recording。
recording() {
  source_ros --workspace
  exec python3 -m robot_app.recording --config "$(resolve_config "${CONFIG_ARG}")" --ws-root "${WS_DIR}" \
    ${SET_ARGS[@]+"${SET_ARGS[@]}"} "$@"
}

# 总线测量：通过配置里的末端测它的总线，逻辑在 robot_app.bus_probe；选项原样交给 end_effector_hardware 的 bus_probe。
cmd_bus_probe() {
  source_ros --workspace
  exec python3 -m robot_app.bus_probe --config "$(resolve_config "${CONFIG_ARG}")" \
    ${SET_ARGS[@]+"${SET_ARGS[@]}"} "$@"
}

# 全局选项可以写在命令前后：-c 配置、--set 字段=值、--entry/--end-effector/--end-effector-bus、--rviz。
parse_options() {
  REST=()
  local key
  while (( $# )); do
    case "$1" in
      -c) [[ -n "${2:-}" ]] || die "-c 需要配置文件路径"; CONFIG_ARG="$2"; shift 2 ;;
      --set) [[ "${2:-}" == *=* ]] || die "--set 需要 字段=值，例如 --set robot.hardware=isaac"
             SET_ARGS+=(--set "$2"); shift 2 ;;
      --entry|--end-effector|--end-effector-bus)
        [[ -n "${2:-}" ]] || die "$1 需要取值"
        case "$1" in
          --entry) key="entry" ;;
          --end-effector) key="end_effector.type" ;;
          --end-effector-bus) key="end_effector.interface.bus" ;;
        esac
        SET_ARGS+=(--set "robot.${key}=$2"); shift 2 ;;
      --rviz) WITH_ARGS+=(--with rviz); shift ;;
      *) REST+=("$1"); shift ;;
    esac
  done
}
parse_options "$@"
set -- ${REST[@]+"${REST[@]}"}

case "${1:-}" in
  build) shift; cmd_build "$@" ;;
  check) shift; cmd_check "${1:-${CONFIG_ARG}}" ;;
  record|play|bags) recording "$@" ;;
  dyn-identify) shift; cmd_dyn_identify "$@" ;;
  bus_probe) shift; cmd_bus_probe "$@" ;;
  sim) SET_ARGS+=(--set robot.hardware=mock_components); cmd_start ;;
  isaac) SET_ARGS+=(--set robot.hardware=isaac); cmd_start ;;
  viz) units viz ;;
  up|down|restart|status|logs|diag) units "$@" ;;
  console) shift; cmd_console "$@" ;;
  -h|--help|help) usage ;;
  "") cmd_start ;;
  *.yaml|*.yml) CONFIG_ARG="$1"; cmd_start ;;
  *) die "未知命令: $1（./scripts/start.sh help 查看用法）" ;;
esac
