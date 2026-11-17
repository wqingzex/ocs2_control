"""Finding the headset: a small ROS node that listens for its connection state and device id (both latched by the
bridge), and the wait for its tracking data.

rclpy is imported only when a probe is created, so the rest of the package works (and tests) without ROS."""
from __future__ import annotations

import threading
import time
from typing import Optional


class XrProbe:
    def __init__(self, node_name: str = "xr_teleop_probe"):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import Bool, String

        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(node_name)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.xr_connected: Optional[bool] = None
        self.xr_device_id = ""
        self.node.create_subscription(Bool, "/teleop/connection_status", self._on_connected, latched)
        self.node.create_subscription(String, "/teleop/device_id", self._on_device_id, latched)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        # spin_once in a loop so close() can stop the thread before the node is destroyed.
        while not self._stop.is_set() and self._rclpy.ok():
            self._executor.spin_once(timeout_sec=0.1)

    def _on_connected(self, msg) -> None:
        self.xr_connected = msg.data

    def _on_device_id(self, msg) -> None:
        self.xr_device_id = msg.data

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._executor.shutdown()
        self.node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.try_shutdown()


def wait_for_headset(probe, expected_id: str, timeout_s: Optional[float], io) -> Optional[str]:
    """Wait until the probe reports a connected headset and return its device id; None when the time is up.

    With an expected id only that headset counts. Asking whether to bind a new one is the session's job."""
    deadline = time.monotonic() + timeout_s if timeout_s is not None and timeout_s > 0 else None
    io.info("等待头显连接（不限时，Ctrl+C 取消）..." if deadline is None
            else f"等待头显连接（最多 {timeout_s:.0f} s，Ctrl+C 取消）...")
    while deadline is None or time.monotonic() < deadline:
        device_id = getattr(probe, "xr_device_id", "")
        if probe.xr_connected and device_id and (not expected_id or expected_id == device_id):
            return device_id
        time.sleep(0.5)
    return None
