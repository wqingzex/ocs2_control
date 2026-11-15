# ocs2_control 运行环境镜像：Ubuntu 24.04 + ROS 2 Jazzy，只准备系统环境和 fiveages 用户，不含源码，不编译。
# 仓库由 docker/run.sh 以和宿主机相同的绝对路径挂进去（环境变量 WS 就是这个路径），在容器里改代码、用仓库自己的 ./scripts/setup.sh 装依赖和编译
# （setup.sh 在容器里会自动跳过物理机的 udev 和 CAN 配置）。
#
#   git submodule update --init --recursive --depth 1       # 子模块要在宿主机上先拉好
#   docker build --network=host -f docker/Dockerfile -t ocs2_control .
#   ./docker/run.sh ./scripts/setup.sh                      # 首次：依赖、编译、相机、手、XR 配对
#   ./docker/run.sh ./scripts/start.sh                      # 之后：启动运控
#
# 基础镜像是 Docker Hub 上的公共镜像，任何机器都能构建；本机已有预装好的镜像时可以换掉：--build-arg BASE_IMAGE=<镜像>。
ARG BASE_IMAGE=ros:jazzy-ros-base
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG USER_NAME=fiveages
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    container=docker \
    QT_X11_NO_MITSHM=1

# setup.sh 用到的工具（git、rosdep、colcon、curl、sudo）、X11 显示库、总线调试工具（ip、candump）。
# 不删 /var/lib/apt/lists：运行时 setup.sh deps 缺什么会 sudo apt-get install，没有包列表就找不到包。
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git curl ca-certificates sudo \
        python3-colcon-common-extensions python3-rosdep python3-pytest python3-yaml \
        xdg-utils desktop-file-utils xauth x11-apps \
        libgl1 libgl1-mesa-dri libglx-mesa0 libxkbcommon-x11-0 \
        iproute2 can-utils

# 用户和宿主机的 uid:gid 一致（写回仓库的文件归属不变）。Ubuntu 24.04 的基础镜像里 uid/gid 1000 被 ubuntu 占着，先让出来。
# 加入 dialout（串口）、plugdev（USB 设备）；sudo 免密：setup.sh 装依赖要 apt，驱动启动 CAN 要 `sudo ip link set ...`。
RUN (userdel -r ubuntu 2>/dev/null || true) \
    && (groupdel ubuntu 2>/dev/null || true) \
    && groupadd -g "${USER_GID}" "${USER_NAME}" \
    && useradd -m -u "${USER_UID}" -g "${USER_GID}" -s /bin/bash -G dialout,plugdev "${USER_NAME}" \
    && printf '%s ALL=(root) NOPASSWD:ALL\nDefaults env_keep += "http_proxy https_proxy no_proxy HTTP_PROXY HTTPS_PROXY NO_PROXY"\n' "${USER_NAME}" > /etc/sudoers.d/fiveages_w2 \
    && chmod 0440 /etc/sudoers.d/fiveages_w2 && visudo -cf /etc/sudoers.d/fiveages_w2

USER ${USER_NAME}
ENTRYPOINT []

# 加载 ROS 和工作区；用 rmw_zenoh 且本机 7447 没有 Router 时在后台起一个（容器和宿主机共用网络，宿主机已有就直接用）。
# 放在单独文件里：.bashrc（交互 shell）和 docker/run.sh（任何命令）都会加载它，.bashrc 开头对非交互 shell 会直接返回。
# setup.sh / start.sh 自己也会加载 ROS 和 config/ros_env.sh。
RUN cat > ~/.fiveages_w2_env.sh <<'EOF'
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[ -f "${WS}/install/setup.bash" ] && source "${WS}/install/setup.bash"
if [ "${RMW_IMPLEMENTATION}" = rmw_zenoh_cpp ] && [ -z "${ZENOH_CONFIG_OVERRIDE}" ] \
   && ! (exec 3<>/dev/tcp/127.0.0.1/7447) 2>/dev/null; then
  setsid nohup ros2 run rmw_zenoh_cpp rmw_zenohd >/tmp/rmw_zenohd.log 2>&1 &
  sleep 2
fi
EOF
RUN echo '. ~/.fiveages_w2_env.sh' >> ~/.bashrc
CMD ["bash"]
