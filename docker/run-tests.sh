#!/usr/bin/env bash
# 容器内运行：对仓库自己的包跑 colcon test（不含 third_party），再汇总结果。
# 额外参数原样传给 colcon test，例如：./docker/run.sh docker/run-tests.sh --packages-select robot_utils
# 要先编译，且编译时打开 BUILD_TESTING（setup.sh 的发布编译是关的：colcon build ... -DBUILD_TESTING=ON）。
set -Eeuo pipefail

cd "${WS:?WS 未设置：用 ./docker/run.sh docker/run-tests.sh 运行}"
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source install/setup.bash
set -u

if (( $# == 0 )); then
  mapfile -t packages < <(colcon list --names-only --base-paths src/robot_control src/robot_description \
    src/robot_hardware src/robot_hmi src/robot_launch src/robot_utils src/robot_payload src/teleop)
  set -- --packages-select "${packages[@]}"
fi

test_status=0
# 工作区分层检查（包环、向上的依赖、用了没声明）和它自己的测试；只用标准库，先于 colcon test 报错
python3 scripts/check_layers.py || test_status=$?
python3 -m pytest -q -p no:cacheprovider scripts/test_check_layers.py || test_status=$?
colcon test --event-handlers console_direct+ "$@" || test_status=$?
colcon test-result --verbose || test_status=$?
exit "${test_status}"
