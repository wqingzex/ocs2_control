#!/usr/bin/env python3
"""头、腰的小幅微调（相对当前位置），只在 MOVEJ 下执行。
  ./scripts/nudge.py show                  当前关节位置和状态机状态
  ./scripts/nudge.py fsm hold|movej        发状态机命令（hold=2，movej=4）
  ./scripts/nudge.py head <1|2> <增量>     头关节 1 或 2，单位 rad
  ./scripts/nudge.py waist <增量>          转腰 body_joint4，单位 rad
增量绝对值不超过 0.1 rad。话题是全身模式（robot.entry=whole_body）的
/whole_body_controller/joint_target_position/head 和 .../body；到位门槛 0.003 rad。
需要先 source install/setup.bash（容器 shell 里已经是）。"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32

MAX_DELTA = 0.1
TOLERANCE = 0.003  # rad; the joints settle about 0.002 short of the target
MOVEJ = 4
HEAD = ["head_joint1", "head_joint2"]
BODY = ["body_joint1", "body_joint2", "body_joint3", "body_joint4"]
FSM = {"hold": 2, "movej": MOVEJ}


def die(message):
    print("错误:", message)
    sys.exit(1)


rclpy.init()
node = Node("w2_nudge")
joints, state = {}, {}
node.create_subscription(JointState, "/joint_states", lambda m: joints.update(zip(m.name, m.position)), 10)
node.create_subscription(Int32, "/fsm_state", lambda m: state.update(value=m.data),
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
fsm_pub = node.create_publisher(Int32, "/fsm_command", 10)
head_pub = node.create_publisher(Float64MultiArray, "/whole_body_controller/joint_target_position/head", 10)
body_pub = node.create_publisher(Float64MultiArray, "/whole_body_controller/joint_target_position/body", 10)


def spin(seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def show():
    names = HEAD + BODY
    print("fsm_state:", state.get("value", "未知（还没收到）"), "（2=HOLD 4=MOVEJ）")
    print({name: round(joints.get(name, float("nan")), 4) for name in names})


spin(1.5)  # 等发现和第一帧
args = sys.argv[1:]
if not args:
    die(__doc__)
if args[0] == "show":
    show()
elif args[0] == "fsm" and len(args) == 2 and args[1] in FSM:
    for _ in range(5):
        fsm_pub.publish(Int32(data=FSM[args[1]]))
        spin(0.1)
    spin(2.0)
    show()
elif args[0] in ("head", "waist"):
    try:
        joint, delta = (int(args[1]), float(args[2])) if args[0] == "head" else (4, float(args[1]))
    except (IndexError, ValueError):
        die(__doc__)
    if abs(delta) > MAX_DELTA:
        die(f"增量 {delta} 超过 {MAX_DELTA} rad")
    if state.get("value") != MOVEJ:
        die("状态机不在 MOVEJ（fsm_state 不是 4），先执行: ./scripts/nudge.py fsm movej")
    names = HEAD if args[0] == "head" else BODY
    if joint not in range(1, len(names) + 1) or any(n not in joints for n in names):
        die("关节号不对，或还没收到 /joint_states")
    target = [joints[n] for n in names]
    target[joint - 1] += delta
    print("目标:", dict(zip(names, (round(v, 4) for v in target))))
    (head_pub if args[0] == "head" else body_pub).publish(Float64MultiArray(data=target))
    for second in range(1, 11):
        spin(1.0)
        print(f"t+{second}s:", {n: round(joints[n], 4) for n in names})
        error = abs(joints[names[joint - 1]] - target[joint - 1])
        if error <= TOLERANCE:
            print(f"到位（静差 {error:.4f} rad）")
            break
    else:
        print(f"10 秒内没有到位，差 {error:.4f} rad（门槛 {TOLERANCE} rad）")
else:
    die(__doc__)
rclpy.shutdown()
