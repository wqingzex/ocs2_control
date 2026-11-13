#!/bin/bash
# Log volume and CPU/memory of the three simulation modes, in two windows each:
#   idle   the controllers are up and in HOLD
#   active HOME -> HOLD -> OCS2, i.e. the MPC is running (the state is checked before the window starts)
#
# usage: scripts/measure_log_baseline.sh [TAG]      (needs a sourced ROS workspace, e.g. inside the build container)
# output: /tmp/ll/<TAG>_<mode>.log (console log) and /tmp/ll/<TAG>_<mode>_{idle,active}.json
# Compare TAG runs before and after a logging change; see docs/日志.md.
TAG=${1:-base}
OUT=/tmp/ll
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "${OUT}"

fsm() { ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "{data: $1}" > /dev/null 2>&1; }

run_mode() {   # name package launch robot [extra launch arguments]
  local name=$1 pkg=$2 launch=$3 robot=$4; shift 4
  local log="${OUT}/${TAG}_${name}.log"
  : > "${log}"
  setsid ros2 launch "${pkg}" "${launch}" robot:="${robot}" hardware:=mock_components launch_mode:=control_only "$@" > "${log}" 2>&1 &
  local launch_pid=$! active=0 last=-1 stable=0 i
  for i in $(seq 1 100); do  # wait until the number of active controllers stops changing
    active=$(ros2 control list_controllers 2>/dev/null | grep -cw active)
    if [ "${active}" -ge 1 ] && [ "${active}" = "${last}" ]; then stable=$((stable + 1)); else stable=0; fi
    last=${active}; [ ${stable} -ge 4 ] && break; sleep 3
  done
  echo "[${name}] active controllers: ${active}"
  sleep 20
  python3 "${HERE}/measure_log_load.py" --log "${log}" --seconds 60 --label "${name} idle(HOLD)" --json "${OUT}/${TAG}_${name}_idle.json"
  fsm 1; sleep 9    # HOME
  fsm 2; sleep 2    # HOLD (the MPC state is entered from HOLD)
  fsm 3; sleep 4    # OCS2
  echo "[${name}] fsm_state before the active window: $(timeout 5 ros2 topic echo --once /fsm_state 2>/dev/null | tr -d '\n' | cut -c1-60)"
  python3 "${HERE}/measure_log_load.py" --log "${log}" --seconds 60 --label "${name} active(OCS2)" --json "${OUT}/${TAG}_${name}_active.json"
  kill -INT -"${launch_pid}" 2>/dev/null; sleep 8; kill -TERM -"${launch_pid}" 2>/dev/null; sleep 2
  for p in ros2_control_no robot_state_pub teleop_command_ spawner; do pkill -KILL -x "${p}"; done
  sleep 2
}

run_mode split_body robot_bringup split_body.launch.py fiveages_w2
run_mode whole_body robot_bringup whole_body.launch.py fiveages_w2
run_mode dual_arm   robot_bringup dual_arm.launch.py   m6_ccs variant:=desktop
echo "done: ${OUT}/${TAG}_*"
