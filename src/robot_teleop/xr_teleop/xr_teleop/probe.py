"""A small ROS node that listens for the headset: connection state and device id (both latched by the bridge).

rclpy is imported only when a probe is created, so the rest of the package works (and tests) without ROS."""
from __future__ import annotations

import threading
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
