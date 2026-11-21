"""Return to the configured HOME and adjust it using measured joint positions."""
from decimal import Decimal
import math
import time
from threading import Event, Lock

from .ros_probe import FSM_COMMANDS, FSM_HOLD, RosProbe


ARRIVAL_TOLERANCE_RAD = 0.03
ARRIVAL_STABLE_S = 0.25
FEEDBACK_TIMEOUT_S = 0.5
RETURN_TIMEOUT_S = 30.0
ANGLE_STEP = Decimal("0.01")
JOG_TOLERANCE_RAD = 0.004
JOG_STABLE_S = 0.10
JOG_TIMEOUT_S = 5.0


def _active(probe, names):
    states = probe.controller_states()
    return states is not None and all(states.get(name) == "active" for name in names)


class JointFeedback:
    def __init__(self, probe, joints):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState

        self.joints = joints
        self._lock = Lock()
        self._sample = None
        probe.node.create_subscription(JointState, "/joint_states", self._receive, qos_profile_sensor_data)

    def _receive(self, message):
        positions = dict(zip(message.name, message.position))
        if len(message.name) == len(message.position) and self.joints <= positions.keys():
            measured = {joint: positions[joint] for joint in self.joints}
            if all(map(math.isfinite, measured.values())):
                with self._lock:
                    self._sample = (time.monotonic(), measured)

    def latest(self):
        with self._lock:
            row = self._sample
        return row if row is not None and time.monotonic() - row[0] < FEEDBACK_TIMEOUT_S else None


def _live_home(probe, controller, stop):
    from rcl_interfaces.srv import GetParameters

    client = probe.node.create_client(GetParameters, f"/{controller}/get_parameters")
    try:
        if not client.wait_for_service(timeout_sec=1.5) or stop.is_set():
            raise RuntimeError(f"无法核对 {controller} 的 HOME")
        request = client.call_async(GetParameters.Request(names=["joints", "home_1"]))
        deadline = time.monotonic() + 1.5
        while not request.done() and time.monotonic() < deadline and not stop.is_set():
            time.sleep(0.05)
        if not request.done() or request.exception() is not None:
            raise RuntimeError(f"未收到 {controller} 的 HOME 参数")
        joints, home = request.result().values
        return list(joints.string_array_value), list(home.double_array_value)
    finally:
        probe.node.destroy_client(client)


def return_home(settings, stop):
    """Use the normal HOME FSM transition. Do not move on stale state or a different loaded pose."""
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    required = (*settings, "joint_state_broadcaster")
    target = {joint: value for joints, values in settings.values() for joint, value in zip(joints, values)}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    probe = None
    commanded = arrived = False
    try:
        probe = RosProbe("setup_home_return", monitor_controllers=True)
        feedback = JointFeedback(probe, target.keys()).latest
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not stop.is_set():
            if _active(probe, required) and feedback() is not None and probe.fsm_state is not None:
                break
            time.sleep(0.05)
        if stop.is_set():
            return False
        if not _active(probe, required) or feedback() is None:
            raise RuntimeError("运控未就绪或关节反馈不可用，无法回 HOME")
        for name, (joints, values) in settings.items():
            live_joints, live_home = _live_home(probe, name, stop)
            if live_joints != joints or len(live_home) != len(values) or any(
                    not math.isclose(a, b, abs_tol=1e-9) for a, b in zip(live_home, values)):
                raise RuntimeError(f"{name} 已加载的 HOME 与当前配置不同，请核对后重启运控")

        if stop.is_set():
            return False
        if probe.fsm_state != FSM_HOLD or not _active(probe, required) or feedback() is None:
            raise RuntimeError("请确认控制器已激活、反馈正常且处于 HOLD，再回 HOME")
        probe.send_fsm(FSM_COMMANDS["HOME"])
        commanded = True

        entered = False
        stable_since = None
        deadline = time.monotonic() + RETURN_TIMEOUT_S
        while time.monotonic() < deadline and not stop.is_set():
            row = feedback()
            if row is None or not _active(probe, required):
                raise RuntimeError("回 HOME 时控制器停用或反馈中断")
            if probe.fsm_state == FSM_COMMANDS["HOME"]:
                entered = True
                stamp, measured = row
                if all(abs(measured[joint] - value) <= ARRIVAL_TOLERANCE_RAD for joint, value in target.items()):
                    if stable_since is None:
                        stable_since = stamp
                    if stamp - stable_since >= ARRIVAL_STABLE_S:
                        arrived = True
                        return True
                else:
                    stable_since = None
            elif entered or probe.fsm_state != FSM_HOLD:
                raise RuntimeError("控制器已离开 HOME，设置窗口停止等待")
            time.sleep(0.05)
        if stop.is_set():
            return False
        raise RuntimeError("等待 HOME 到位超时")
    finally:
        if probe is not None:
            try:
                if commanded and not arrived and _active(probe, required) and probe.fsm_state in (FSM_HOLD, FSM_COMMANDS["HOME"]):
                    # Send HOLD even if the HOME command has not yet appeared in /fsm_state.
                    probe.send_fsm(FSM_HOLD)
                    if not probe.request_hold(timeout_s=2.0):
                        raise RuntimeError("未确认控制器已回 HOLD，请查看运控状态")
            finally:
                probe.close()
        else:
            rclpy.try_shutdown()


class HomeJog:
    """One confirmed MOVEJ session and at most one uncompleted joint step."""

    def __init__(self, settings, limits):
        self.joints = {name: list(joints) for name, (joints, _) in settings.items()}
        self.limits = limits
        self.stop = Event()
        self._lock = Lock()
        self._positions = None
        self._ready = False
        self._step = None

    def snapshot(self):
        with self._lock:
            return self._positions, self._ready and not self.stop.is_set()

    def request_step(self, controller, joint, direction):
        with self._lock:
            if not self._ready or self.stop.is_set():
                return False
            self._step = (controller, joint, direction)
            self._ready = False
            return True

    def _update_positions(self, positions, idle):
        with self._lock:
            step, self._step = self._step, None
            self._positions = positions
            self._ready = idle and step is None
            return step

    def _wait_mode(self, probe, feedback, required, expected):
        deadline = time.monotonic() + 2.0
        while not self.stop.is_set():
            row = feedback.latest()
            if row is None or not _active(probe, required):
                raise RuntimeError("运控停用或关节反馈中断，实时调姿已停止")
            if probe.fsm_state == expected:
                return True
            if time.monotonic() >= deadline:
                raise RuntimeError(f"未确认运控进入 {'HOLD' if expected == FSM_HOLD else 'MOVEJ'}")
            self.stop.wait(0.025)
        return False

    def _step_target(self, controller, joint, direction, measured):
        if direction not in (-1, 1) or joint not in self.joints[controller]:
            raise ValueError("无效的实时调姿步进")
        start = measured[joint]
        lower, upper = self.limits[joint]
        goal = min(max(float(Decimal(str(start)) + ANGLE_STEP * direction), lower), upper)
        if abs(goal - start) > float(ANGLE_STEP) + 1e-9:
            raise ValueError(f"{joint} 的反馈超出模型限位")
        values = [goal if name == joint else measured[name] for name in self.joints[controller]]
        for name, value in zip(self.joints[controller], values):
            lower, upper = self.limits[name]
            if not lower <= value <= upper:
                raise ValueError(f"{name} 的反馈超出模型限位")
        return values, goal

    def _jog_loop(self, probe, feedback, required, send_target):
        motion = None
        settled = None
        while not self.stop.is_set():
            row = feedback.latest()
            if row is None or not _active(probe, required):
                raise RuntimeError("运控停用或关节反馈中断，实时调姿已停止")
            if probe.fsm_state != FSM_COMMANDS["MOVEJ"]:
                raise RuntimeError("运控已离开 MOVEJ，实时调姿已停止")
            stamp, measured = row
            if motion is not None:
                joint, goal, tolerance, sent = motion
                if stamp > sent and abs(measured[joint] - goal) <= tolerance:
                    if settled is None or abs(measured[joint] - settled[1]) > tolerance / 4:
                        settled = (stamp, measured[joint])
                    if stamp - settled[0] >= JOG_STABLE_S:
                        motion = None
                else:
                    settled = None
                if motion is not None and time.monotonic() - sent >= JOG_TIMEOUT_S:
                    raise RuntimeError(f"{joint} 点动未到位，实时调姿已停止")
            step = self._update_positions(measured, motion is None)
            if step is not None and not self.stop.is_set():
                controller, joint, direction = step
                values, goal = self._step_target(controller, joint, direction, measured)
                distance = abs(goal - measured[joint])
                if distance > 1e-9:
                    sent = time.monotonic()
                    send_target(controller, values)
                    motion = (joint, goal, min(JOG_TOLERANCE_RAD, distance / 2), sent)
                    settled = None
            self.stop.wait(0.025)

    def _run(self, probe, feedback, required, publishers, send_target):
        home, movej = FSM_COMMANDS["HOME"], FSM_COMMANDS["MOVEJ"]
        deadline = time.monotonic() + 5.0
        while not self.stop.is_set():
            pending = []
            if not _active(probe, required):
                pending.append("控制器激活")
            if feedback.latest() is None:
                pending.append("关节反馈")
            if probe.fsm_state is None:
                pending.append("FSM 状态")
            if not all(pub.get_subscription_count() for pub in publishers.values()):
                pending.append("MOVEJ 目标话题订阅")
            if not pending:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError(f"实时调姿未就绪：{'、'.join(pending)}")
            self.stop.wait(0.025)
        for controller, joints in self.joints.items():
            if self.stop.is_set():
                return None
            live_joints, _ = _live_home(probe, controller, self.stop)
            if live_joints != joints:
                raise RuntimeError(f"{controller} 已加载的关节顺序与配置不同")
        if self.stop.is_set():
            return None
        if not _active(probe, required) or feedback.latest() is None or probe.fsm_state not in (home, FSM_HOLD):
            raise RuntimeError("请确认运控已激活、关节反馈正常，并处于 HOME 或 HOLD")
        if probe.fsm_state == home:
            probe.send_fsm(FSM_HOLD)
        if not self._wait_mode(probe, feedback, required, FSM_HOLD) or self.stop.is_set():
            return None

        try:
            probe.send_fsm(movej)
            if not self._wait_mode(probe, feedback, required, movej):
                return None
            self._jog_loop(probe, feedback, required, send_target)
        finally:
            if probe.fsm_state in (movej, FSM_HOLD):
                probe.send_fsm(FSM_HOLD)
                if not probe.request_hold(timeout_s=2.0) or not _active(probe, required):
                    raise RuntimeError("未确认控制器已回 HOLD，请查看运控状态")
        if probe.fsm_state != FSM_HOLD:
            raise RuntimeError("未确认控制器已回 HOLD，请查看运控状态")

        # Pick up fresh measurements after the HOLD transition has stopped the jog.
        earliest = time.monotonic() + ARRIVAL_STABLE_S
        deadline = earliest + FEEDBACK_TIMEOUT_S
        while time.monotonic() < deadline:
            if probe.fsm_state != FSM_HOLD or not _active(probe, required):
                break
            row = feedback.latest()
            if row is not None and row[0] >= earliest:
                return row[1]
            time.sleep(0.025)
        raise RuntimeError("停止调姿后未收到 HOLD 下的新关节反馈")

    def run(self):
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import Float64MultiArray

        required = (*self.joints, "joint_state_broadcaster")
        joints = {joint for group in self.joints.values() for joint in group}
        if missing := joints - self.limits.keys():
            raise ValueError(f"模型缺少关节限位：{', '.join(sorted(missing))}")
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        probe = None
        try:
            probe = RosProbe("setup_home_jog", monitor_controllers=True)
            feedback = JointFeedback(probe, joints)
            publishers = {name: probe.node.create_publisher(Float64MultiArray, f"/{name}/joint_target_position", 10)
                          for name in self.joints}
            return self._run(probe, feedback, required, publishers,
                             lambda name, positions: publishers[name].publish(Float64MultiArray(data=positions)))
        finally:
            if probe is not None:
                probe.close()
            else:
                rclpy.try_shutdown()
