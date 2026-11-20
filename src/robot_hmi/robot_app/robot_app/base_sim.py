"""Simulated chassis for hardware = mock_components: the base the controllers drive in simulation.

It listens to base_velocity_command (Twist: vx, vy along the base axes, wz), treats the command like a real driver
would (speed limits, acceleration limits, a command that stops arriving stops the base), integrates the pose and
publishes base_odom and the world -> base TF (the controller does not need it: it plans in the frame of
the base; RViz needs it to show the robot moving). A real chassis driver replaces the integration with the vendor's calls.

    python3 -m robot_app.base_sim [--ros-args -p max_linear:=0.5 ...]"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class Limits:
    max_linear: float = 0.5            # m/s, per axis
    max_angular: float = 0.5           # rad/s
    max_linear_accel: float = 0.5      # m/s^2
    max_angular_accel: float = 1.0     # rad/s^2
    command_timeout_s: float = 0.3     # no command for this long: stop

    def __post_init__(self) -> None:
        for name, value in vars(self).items():
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"base limit '{name}' must be a positive number, got {value!r}")


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def approach(current: float, target: float, max_step: float) -> float:
    delta = target - current
    return target if abs(delta) <= max_step else current + math.copysign(max_step, delta)


class SimulatedBase:
    """Planar base: pose in the world frame, velocity along the base axes."""

    def __init__(self, limits: Limits):
        self.limits = limits
        self.x = self.y = self.yaw = 0.0
        self.vx = self.vy = self.wz = 0.0
        self._command: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._command_time: Optional[float] = None

    def command(self, vx: float, vy: float, wz: float, now: float) -> bool:
        """A velocity request. A request that is not finite stops the base; returns whether it was accepted."""
        finite = all(math.isfinite(v) for v in (vx, vy, wz))
        if not finite:
            vx = vy = wz = 0.0
        limits = self.limits
        self._command = (clamp(vx, limits.max_linear), clamp(vy, limits.max_linear), clamp(wz, limits.max_angular))
        self._command_time = now
        return finite

    def stale(self, now: float) -> bool:
        return self._command_time is None or now - self._command_time > self.limits.command_timeout_s

    def step(self, dt: float, now: float) -> None:
        target = (0.0, 0.0, 0.0) if self.stale(now) else self._command
        linear_step = self.limits.max_linear_accel * dt
        self.vx = approach(self.vx, target[0], linear_step)
        self.vy = approach(self.vy, target[1], linear_step)
        self.wz = approach(self.wz, target[2], self.limits.max_angular_accel * dt)
        cos_yaw, sin_yaw = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (self.vx * cos_yaw - self.vy * sin_yaw) * dt
        self.y += (self.vx * sin_yaw + self.vy * cos_yaw) * dt
        self.yaw += self.wz * dt


def run() -> int:
    import rclpy
    from geometry_msgs.msg import Twist, TransformStamped
    from nav_msgs.msg import Odometry
    from tf2_ros import TransformBroadcaster

    rclpy.init()
    node = rclpy.create_node("base_sim")
    defaults = Limits()
    limits = Limits(
        max_linear=node.declare_parameter("max_linear", defaults.max_linear).value,
        max_angular=node.declare_parameter("max_angular", defaults.max_angular).value,
        max_linear_accel=node.declare_parameter("max_linear_accel", defaults.max_linear_accel).value,
        max_angular_accel=node.declare_parameter("max_angular_accel", defaults.max_angular_accel).value,
        command_timeout_s=node.declare_parameter("command_timeout_s", defaults.command_timeout_s).value,
    )
    world_frame = node.declare_parameter("world_frame", "world").value
    base_frame = node.declare_parameter("base_frame", "base_footprint").value
    command_topic = node.declare_parameter("command_topic", "base_velocity_command").value
    rate_hz = node.declare_parameter("rate_hz", 50.0).value
    # Off: only base_odom is published (RViz then shows the robot where the controller's identity TF puts it).
    publish_tf = node.declare_parameter("publish_tf", True).value
    if not (math.isfinite(rate_hz) and rate_hz > 0.0):
        raise ValueError(f"rate_hz must be a positive number, got {rate_hz!r}")

    base = SimulatedBase(limits)
    clock = node.get_clock()

    def seconds() -> float:
        return clock.now().nanoseconds * 1e-9

    def on_command(msg: Twist) -> None:
        if not base.command(msg.linear.x, msg.linear.y, msg.angular.z, seconds()):
            node.get_logger().warn("base command is not finite: the base is stopped", throttle_duration_sec=1.0)

    node.create_subscription(Twist, command_topic, on_command, 10)
    odometry_publisher = node.create_publisher(Odometry, "base_odom", 10)
    tf_broadcaster = TransformBroadcaster(node)
    last = [seconds()]

    def tick() -> None:
        now = seconds()
        base.step(now - last[0], now)
        last[0] = now
        stamp = clock.now().to_msg()
        z, w = math.sin(base.yaw / 2.0), math.cos(base.yaw / 2.0)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = world_frame
        transform.child_frame_id = base_frame
        transform.transform.translation.x, transform.transform.translation.y = base.x, base.y
        transform.transform.rotation.z, transform.transform.rotation.w = z, w
        if publish_tf:
            tf_broadcaster.sendTransform(transform)
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = world_frame
        odometry.child_frame_id = base_frame
        odometry.pose.pose.position.x, odometry.pose.pose.position.y = base.x, base.y
        odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w = z, w
        odometry.twist.twist.linear.x, odometry.twist.twist.linear.y = base.vx, base.vy
        odometry.twist.twist.angular.z = base.wz
        odometry_publisher.publish(odometry)

    node.create_timer(1.0 / rate_hz, tick)
    node.get_logger().info(
        f"Simulated base: {command_topic} -> TF {world_frame}->{base_frame} and base_odom; {limits}")
    rclpy.spin(node)
    return 0


if __name__ == "__main__":
    sys.exit(run())
