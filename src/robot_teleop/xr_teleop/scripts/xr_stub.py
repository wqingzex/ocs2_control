#!/usr/bin/env python3
"""Headset-free stand-in for xr_target_node (same node name, which XrInputHandler requires).

    ros2 run xr_teleop xr_stub.py [--script steps.json] [--silent]   # timeline format: docs/使用手册.md section 6.4"""
import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

HEAD = (0.0, 0.0, 1.5)
LEFT = (0.4, 0.2, 1.0)
RIGHT = (0.4, -0.2, 1.0)


def make_pose(position, yaw=0.0):
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = position
    msg.orientation.z = math.sin(yaw / 2.0)
    msg.orientation.w = math.cos(yaw / 2.0)
    return msg


class XrStub(Node):
    def __init__(self, rate_hz, steps, silent=False):
        super().__init__("xr_target_node")
        if silent:
            # Only make /xr_target_node exist (e.g. while `ros2 bag play` replays /xr/*).
            self.get_logger().info("xr_stub --silent: node present, publishing nothing")
            return
        depth = 50
        self.pub = {
            "head": self.create_publisher(Pose, "/teleop/head_pose", depth),
            "left": self.create_publisher(Pose, "/teleop/left_ee_pose", depth),
            "right": self.create_publisher(Pose, "/teleop/right_ee_pose", depth),
            "event": self.create_publisher(Int32, "/teleop/controller_state", depth),
            "axes": self.create_publisher(Twist, "/teleop/thumbstick_axes", depth),
            "trigger": self.create_publisher(Twist, "/teleop/trigger_values", depth),
            "grip": self.create_publisher(Twist, "/teleop/grip_values", depth),
            "status": self.create_publisher(Bool, "/teleop/connection_status", 1),
        }
        self.steps = sorted(steps, key=lambda s: s["at"])
        self.state = {"left_delta": [0.0, 0.0, 0.0], "right_delta": [0.0, 0.0, 0.0],
                      "left_grip": False, "right_grip": False, "head_yaw_deg": 0.0}
        self.start = time.monotonic()
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(f"xr_stub publishing /xr/* at {rate_hz:.0f} Hz ({len(self.steps)} scripted steps)")

    def tick(self):
        now = time.monotonic() - self.start
        event = 0
        while self.steps and self.steps[0]["at"] <= now:
            step = self.steps.pop(0)
            for key in ("left_delta", "right_delta", "left_grip", "right_grip", "head_yaw_deg"):
                if key in step:
                    self.state[key] = step[key]
            event = int(step.get("event", event))
            self.get_logger().info(f"⏱️ t={now:.2f}s step {step}")
        s = self.state
        self.pub["head"].publish(make_pose(HEAD, math.radians(s["head_yaw_deg"])))
        self.pub["left"].publish(make_pose([a + b for a, b in zip(LEFT, s["left_delta"])]))
        self.pub["right"].publish(make_pose([a + b for a, b in zip(RIGHT, s["right_delta"])]))
        self.pub["event"].publish(Int32(data=event))
        axes = Twist()
        axes.linear.z = 1.0 if s["left_grip"] else 0.0
        axes.angular.z = 1.0 if s["right_grip"] else 0.0
        self.pub["axes"].publish(axes)
        self.pub["trigger"].publish(Twist())
        grip = Twist()
        grip.linear.x = axes.linear.z
        grip.angular.x = axes.angular.z
        self.pub["grip"].publish(grip)
        self.pub["status"].publish(Bool(data=True))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--rate", type=float, default=90.0, help="publish rate in Hz (default 90)")
    parser.add_argument("--script", help="JSON timeline file")
    parser.add_argument("--silent", action="store_true",
                        help="publish nothing; only provide the /xr_target_node node (bag playback)")
    args, ros_args = parser.parse_known_args(argv)
    steps = []
    if args.script:
        with open(args.script, encoding="utf-8") as handle:
            steps = json.load(handle)
        if not isinstance(steps, list) or not all(isinstance(s, dict) and "at" in s for s in steps):
            parser.error("--script must be a JSON list of objects with an 'at' field")
    rclpy.init(args=ros_args)
    node = XrStub(args.rate, steps, silent=args.silent)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
