"""ROS helpers for start.sh and the console: FSM state and commands, topic rates, camera frames.

rclpy is imported lazily so the rest of teleop_app works (and tests) without ROS."""
from __future__ import annotations

import threading
import time
from collections import deque
from typing import Dict, Iterable, Optional

FSM_NAMES = {1: "HOME", 2: "HOLD", 3: "OCS2", 4: "MOVEJ", 100: "REST"}
FSM_HOLD = 2

# Buttons of the RViz OCS2 FSM panel (control_rviz_panels/src/fsm_panel.cpp):
# action -> /fsm_command value, and the actions each /fsm_state offers.
FSM_COMMANDS = {"HOME": 1, "HOLD": 2, "OCS2": 3, "MOVEJ": 4, "SWITCH_POSE": 100}
FSM_ACTIONS = {1: ("HOLD", "SWITCH_POSE"), 2: ("HOME", "OCS2", "MOVEJ"), 3: ("HOLD",), 4: ("HOLD",)}
SWITCH_POSE_PULSE_S = 0.1  # SWITCH_POSE is 100 followed by 0 (panel: button press / release)


# Body modes of the RViz FSM panel's combo box: (state in /whole_body_controller/current_state.body_state,
# label, /mode_command string, WholeBodyCapability field that must be true for the mode to be offered).
BODY_MODES = (
    ("FREE", 0, "自由", "BODY_FREE", None),
    ("VERTICAL", 1, "竖直", "BODY_RELATIVE", "has_body_relative_constraint"),
    ("TRACKING", 2, "跟随", "BODY_TRACKING", "body_tracking_ee_enabled"),
    ("LOCKED", 3, "锁定", "BODY_LOCK", "has_waist_lock"),
    ("HEAD_COUPLED", 4, "锁头", "BODY_HEAD_COUPLED", "has_head_coupling"),
    ("CUSTOM_LOCKED", 5, "自定义", "BODY_CUSTOM_LOCK", "has_custom_joint_lock"),
)
BODY_COMMANDS = {name: command for name, _s, _l, command, _c in BODY_MODES}
WHOLE_BODY_CAPABILITY_TOPIC = "/whole_body_controller/whole_body_capabilities"
WHOLE_BODY_STATE_TOPIC = "/whole_body_controller/current_state"
MODE_COMMAND_TOPIC = "/mode_command"


def body_options(capability: Optional[Dict[str, bool]]) -> list:
    """The body modes the controller supports (same rules as the RViz panel); [] until it reported."""
    if capability is None:
        return []
    return [{"action": name, "label": label, "state": state}
            for name, state, label, _command, field in BODY_MODES if field is None or capability.get(field)]


def fsm_name(state: Optional[int]) -> str:
    if state is None:
        return "未知"
    return FSM_NAMES.get(state, str(state))


class RosProbe:
    """One background rclpy node: latest /fsm_state, /teleop/connection_status and message rates."""

    def __init__(self, node_name: str = "teleop_app_probe"):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import Bool, Int32, String

        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(node_name)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.fsm_state: Optional[int] = None
        self.xr_connected: Optional[bool] = None
        self.xr_device_id = ""
        self._stamps: Dict[str, deque] = {}
        self._subs = {}
        self._lock = threading.Lock()
        self.node.create_subscription(Int32, "/fsm_state", self._on_fsm, latched)
        self.node.create_subscription(Bool, "/teleop/connection_status", self._on_xr, latched)
        self.node.create_subscription(String, "/teleop/device_id", self._on_xr_device, latched)
        self._fsm_pub = self.node.create_publisher(Int32, "/fsm_command", 10)
        self.whole_body_capability: Optional[Dict[str, bool]] = None
        self.body_state: Optional[int] = None
        self._mode_pub = self.node.create_publisher(String, MODE_COMMAND_TOPIC, 10)
        self._String = String
        self._subscribe_whole_body(latched)
        self._Int32 = Int32
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        # spin_once in a loop so close() can stop the thread before the node is destroyed.
        while not self._stop.is_set() and self._rclpy.ok():
            self._executor.spin_once(timeout_sec=0.1)

    def _subscribe_whole_body(self, latched) -> None:
        """Body mode state from the WBC controller; without its message package the page just has no body card."""
        try:
            from robot_control_msgs.msg import WholeBodyCapability, WholeBodyCurrentState
        except ImportError:
            return
        self.node.create_subscription(WholeBodyCapability, WHOLE_BODY_CAPABILITY_TOPIC, self._on_capability, latched)
        self.node.create_subscription(WholeBodyCurrentState, WHOLE_BODY_STATE_TOPIC, self._on_whole_body_state, 10)

    def _on_capability(self, msg) -> None:
        self.whole_body_capability = {name: bool(getattr(msg, name)) for name in
                               {f for *_x, f in BODY_MODES if f}}

    def _on_whole_body_state(self, msg) -> None:
        self.body_state = int(msg.body_state)

    def send_body_mode(self, action: str) -> None:
        self._mode_pub.publish(self._String(data=BODY_COMMANDS[action]))

    def _on_fsm(self, msg) -> None:
        self.fsm_state = msg.data

    def _on_xr(self, msg) -> None:
        self.xr_connected = msg.data

    def _on_xr_device(self, msg) -> None:
        self.xr_device_id = msg.data

    # ---- rates ------------------------------------------------------------------------------------
    def watch(self, topics: Iterable[str]) -> None:
        """Start counting messages on topics that exist in the graph (others are retried next call)."""
        from rclpy.qos import qos_profile_sensor_data
        from rosidl_runtime_py.utilities import get_message
        types = dict(self.node.get_topic_names_and_types())
        for topic in topics:
            if topic in self._subs or topic not in types:
                continue
            msg_type = get_message(types[topic][0])
            self._stamps[topic] = deque(maxlen=400)
            self._subs[topic] = self.node.create_subscription(
                msg_type, topic, lambda _m, t=topic: self._tick(t), qos_profile_sensor_data, raw=True)

    def _tick(self, topic: str) -> None:
        with self._lock:
            self._stamps[topic].append(time.monotonic())

    # ---- camera frames ------------------------------------------------------------------------------
    def watch_image(self, topic: str) -> None:
        """Keep the latest sensor_msgs/CompressedImage of a topic (for MJPEG preview)."""
        if topic in getattr(self, "_images", {}):
            return
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage
        if not hasattr(self, "_images"):
            self._images = {}
        self._images[topic] = None
        self.node.create_subscription(CompressedImage, topic, lambda m, t=topic: self._on_image(t, m),
                                      qos_profile_sensor_data)

    def _on_image(self, topic: str, msg) -> None:
        with self._lock:
            self._images[topic] = (time.monotonic(), msg.format, bytes(msg.data))

    def latest_image(self, topic: str):
        """(monotonic time, format, bytes) of the newest frame, or None."""
        with self._lock:
            return getattr(self, "_images", {}).get(topic)

    def rate(self, topic: str, window_s: float = 2.0) -> Optional[float]:
        """Messages per second over the last window (None if the topic is not watched yet)."""
        if topic not in self._stamps:
            return None
        now = time.monotonic()
        with self._lock:
            recent = sum(1 for t in self._stamps[topic] if now - t <= window_s)
        return round(recent / window_s, 1)

    # ---- FSM ----------------------------------------------------------------------------------------
    def wait_fsm(self, timeout_s: float = 2.0) -> Optional[int]:
        deadline = time.monotonic() + timeout_s
        while self.fsm_state is None and time.monotonic() < deadline:
            time.sleep(0.05)
        return self.fsm_state

    def send_fsm(self, command: int) -> None:
        """Publish one /fsm_command (no state check: callers validate against FSM_ACTIONS)."""
        self._fsm_pub.publish(self._Int32(data=int(command)))

    def request_hold(self, timeout_s: float = 5.0) -> bool:
        """Send /fsm_command 2 until /fsm_state reports HOLD (or the timeout expires)."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.fsm_state == FSM_HOLD:
                return True
            self._fsm_pub.publish(self._Int32(data=FSM_HOLD))
            time.sleep(0.3)
        return self.fsm_state == FSM_HOLD

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._executor.shutdown()
        self.node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.try_shutdown()
