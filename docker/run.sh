#!/usr/bin/env bash
# 在宿主机运行：在持久容器 ocs2_control 里执行命令（真机运控、仿真、RViz、测试都用它）。
#   ./docker/run.sh                          进入容器 shell
#   ./docker/run.sh ./scripts/setup.sh       首次：依赖、编译、相机、手、XR 配对
#   ./docker/run.sh ./scripts/start.sh       启动运控、XR、相机（Ctrl+C 停止）
#   ./docker/run.sh xeyes                    验证 X11；RViz 等 ROS 程序先进 shell（./docker/run.sh），或用 start.sh --rviz
#   ./docker/run.sh docker/run-tests.sh      跑 colcon test（要先编译，且编译时打开 BUILD_TESTING）
#
# 仓库以和宿主机相同的绝对路径挂进容器，容器里改的代码就是宿主机上的文件；build/、install/ 里记录的绝对路径
# （CMake 缓存、install/setup.bash）在宿主机和容器里都有效。
# 容器是持久的：setup.sh 装的依赖、编译结果（仓库里的 build/、install/）都保留；停掉用 docker stop ocs2_control，
# 删掉重建用 OCS2_CONTROL_RECREATE=1 ./docker/run.sh（装过的依赖会丢，要重新 setup.sh deps）。
# 环境变量：IMAGE（默认 ocs2_control）、CONTAINER（默认 ocs2_control）、DISPLAY（默认 :1）。
set -Eeuo pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
REPO="$(cd -- "${HERE}/.." && pwd -P)"
IMAGE="${IMAGE:-ocs2_control}"
CONTAINER="${CONTAINER:-ocs2_control}"
DISPLAY="${DISPLAY:-:1}"
X11_DIR="/tmp/fiveages_w2-ocs2-x11"          # 固定目录挂进容器；每次运行刷新里面的 cookie（容器是持久的，不能每次换挂载）

(( $# )) || set -- bash

# ---- X11：把当前显示的 cookie 复制出来，host 字段置为 ffff（匹配任意主机名）-------------------------------------------
# Xorg 的 -auth 文件在 gdm 的运行目录里，~/.Xauthority 可能是过期的。
mkdir -p "${X11_DIR}"
GDM_AUTH="/run/user/$(id -u)/gdm/Xauthority"
SOURCE_AUTH="${XAUTHORITY:-}"
[[ -n "${SOURCE_AUTH}" ]] || { [[ -r "${GDM_AUTH}" ]] && SOURCE_AUTH="${GDM_AUTH}" || SOURCE_AUTH="${HOME}/.Xauthority"; }
command -v xauth >/dev/null || { echo "缺少 xauth：sudo apt install xauth" >&2; exit 1; }
: > "${X11_DIR}/xauth"
xauth -f "${SOURCE_AUTH}" nlist "${DISPLAY}" | sed -e 's/^..../ffff/' | xauth -f "${X11_DIR}/xauth" nmerge -
chmod 600 "${X11_DIR}/xauth"

# ---- 容器：不存在就创建（带齐真机需要的权限），停着就启动 ----------------------------------------------------------------
[[ -z "${OCS2_CONTROL_RECREATE:-}" ]] || docker rm -f "${CONTAINER}" >/dev/null 2>&1 || true

if ! docker container inspect "${CONTAINER}" >/dev/null 2>&1; then
  create_args=(
    --name "${CONTAINER}"
    --net=host --ipc=host
    --privileged                                  # 串口、USB 相机、SocketCAN 都要；USB 热插拔后设备号会变，精确映射会失效
    -v /dev:/dev
    --ulimit rtprio=99 --ulimit memlock=-1       # controller_manager 要 FIFO 实时调度
    --cap-add SYS_NICE
    -v "${REPO}:${REPO}" -w "${REPO}" -e WS="${REPO}"
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    -v "${X11_DIR}:/tmp/fiveages_w2-x11"
    -e QT_X11_NO_MITSHM=1
    # 沿用宿主机的通信环境（仓库里 config/ros_env.sh 写的仍然优先，entrypoint 会 source 它）
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
    -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"   # 有的基础镜像写死了 1，远程 RViz 就连不上
  )
  # 宿主机的代理（--net=host，127.0.0.1 的代理在容器里同样可达）：setup.sh 装依赖、下载 PC Service 要用
  for v in http_proxy https_proxy no_proxy HTTP_PROXY HTTPS_PROXY NO_PROXY ROS_AUTOMATIC_DISCOVERY_RANGE ZENOH_CONFIG_OVERRIDE; do
    [[ -z "${!v:-}" ]] || create_args+=(-e "${v}=${!v}")
  done
  docker run -d "${create_args[@]}" "${IMAGE}" sleep infinity >/dev/null
else
  [[ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER}")" == true ]] || docker start "${CONTAINER}" >/dev/null
fi

tty_flags=()
[[ -t 0 && -t 1 ]] && tty_flags=(-it)

exec docker exec "${tty_flags[@]}" \
  -e DISPLAY="${DISPLAY}" -e XAUTHORITY=/tmp/fiveages_w2-x11/xauth \
  "${CONTAINER}" bash -c '. ~/.fiveages_w2_env.sh; exec "$@"' ocs2_control "$@"
