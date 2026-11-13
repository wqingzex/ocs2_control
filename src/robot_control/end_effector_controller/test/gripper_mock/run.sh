#!/bin/bash
# One-joint gripper regression test on mock_components (no robot URDF needed). Run inside the build container:
#   bash run.sh <ctrl_params.yaml> [needs_release] [force]
# needs_release=1: the controller starts in HOLD (joint-space controller instances), so fsm_command 3 is sent first.
# Expected (gripper_joint limits 0..0.08, initial 0.0): open 0.08, 50% 0.04, close 0.0, HOLD ignores commands,
# release keeps the position, re-activate keeps the position, HOME returns to 0.0.
YAML=${1:?ctrl yaml}; RELEASE=${2:-0}; FORCE=${3:-0}   # FORCE=1: constant effort 1.0 > threshold, tests the force feedback
HERE=$(cd "$(dirname "$0")" && pwd); W=$(mktemp -d); NAME=gripper_controller
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash 2>/dev/null
cleanup() { pkill -f "[r]os2_control_node"; pkill -f "[r]obot_state_publisher"; pkill -f "[r]os2 run"; sleep 2; }
cleanup 2>/dev/null; trap cleanup EXIT
URDF_FILE=gripper.urdf; [ "$FORCE" = 1 ] && URDF_FILE=gripper_effort.urdf
URDF=$(cat "$HERE/$URDF_FILE"); cd "$W"
(ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$URDF" > rsp.log 2>&1 &)
(ros2 run controller_manager ros2_control_node --ros-args --params-file "$HERE/$YAML" -p robot_description:="$URDF" > cm.log 2>&1 &)
sleep 4
for c in $NAME joint_state_broadcaster; do
  ros2 run controller_manager spawner $c -c /controller_manager --controller-manager-timeout 30 >> spawn.log 2>&1
done
pos() { timeout 5 ros2 topic echo --once /joint_states 2>/dev/null | awk '/^position:/{getline; print $2; exit}'; }
pub() { timeout 6 ros2 topic pub -t 4 -r 5 "$@" >/dev/null 2>&1; sleep 1; }
fail=0
finish() {
  [ "$fail" = 1 ] && { echo "--- controller log (state switches, warnings):"; grep -E "Switched|HOME|WARN|ERROR|died|crash|terminate|Force|target|Closing|Opening" "$W/cm.log" | grep -v "RT sched\|pal_stat" | tail -25; }
  exit $fail
}
check() { # name expected actual
  if python3 -c "import sys; sys.exit(0 if abs(float('$2')-float('${3:-nan}'))<2e-3 else 1)" 2>/dev/null; then
    echo "PASS $1: $3"; else echo "FAIL $1: expected $2, got ${3:-<none>}"; fail=1; fi; }
pub1() { timeout 8 ros2 topic pub -t 1 -w 1 "$@" >/dev/null 2>&1; sleep 1.5; }   # exactly one message (repeats would re-arm the force feedback)
fsm() { pub /fsm_command std_msgs/msg/Int32 "{data: $1}"; }
cmd() { pub /$NAME/target_command std_msgs/msg/Int32 "{data: $1}"; }
pct() { pub /$NAME/target_percent std_msgs/msg/Float64 "{data: $1}"; }

if [ "$FORCE" = 1 ]; then
  [ "$RELEASE" = 1 ] && fsm 3
  cmd 1; check "open" 0.08 "$(pos)"
  pub1 /$NAME/target_command std_msgs/msg/Int32 "{data: 0}"; check "close stops half way (force ratio 0.5)" 0.04 "$(pos)"
  cmd 1; pub /gripper_joint/position_command std_msgs/msg/Float64 "{data: 0.0}"; check "direct position ignores force" 0.0 "$(pos)"
  finish
fi
check "activate (holds current position)" 0.0 "$(pos)"
[ "$RELEASE" = 1 ] && fsm 3
cmd 1; check "open" 0.08 "$(pos)"
pct 0.5; check "percent 0.5" 0.04 "$(pos)"
cmd 0; check "close" 0.0 "$(pos)"
cmd 1; fsm 2; check "HOLD keeps position" 0.08 "$(pos)"
cmd 0; check "command ignored while HOLD" 0.08 "$(pos)"
fsm 3; check "release keeps position" 0.08 "$(pos)"
cmd 0; check "close after release" 0.0 "$(pos)"
cmd 1; check "open before re-activate" 0.08 "$(pos)"
ros2 control set_controller_state $NAME inactive >/dev/null 2>&1; sleep 1
ros2 control set_controller_state $NAME active >/dev/null 2>&1; sleep 2
check "re-activate keeps position" 0.08 "$(pos)"
[ "$RELEASE" = 1 ] && fsm 3   # joint-space controller instances: HOME is only reachable from HOLD
[ "$RELEASE" = 1 ] && fsm 2
fsm 1; for i in 1 2 3 4 5 6 7 8; do p=$(pos); [ "$(python3 -c "print(abs(float('${p:-9}'))<2e-3)")" = True ] && break; sleep 1; done
check "HOME returns to initial" 0.0 "$p"
finish
