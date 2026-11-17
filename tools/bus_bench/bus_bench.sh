#!/usr/bin/env bash
# Run bus_bench.py with the ROS environment. Inside the container:  ./docker/run.sh tools/bus_bench/bus_bench.sh --help
# DDS is kept on this machine (UDP, localhost only, own domain) so the benchmark's traffic never reaches the robot's network
# and never meets another ROS system on the same network. Override with BB_DOMAIN / BB_RMW / FASTDDS_BUILTIN_TRANSPORTS.
set -Eeuo pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
WS="${WS:-$(cd -- "${HERE}/../.." && pwd -P)}"

set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source "${WS}/install/setup.bash"
set -u

export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export RMW_IMPLEMENTATION="${BB_RMW:-rmw_fastrtps_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID="${BB_DOMAIN:-77}"

cd "${WS}"
exec python3 "${HERE}/bus_bench.py" "$@"
