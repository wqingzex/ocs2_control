"""HOME edits use controller joint order and are saved only after confirmation."""
from concurrent.futures import ThreadPoolExecutor
from io import StringIO
from threading import Event
from types import SimpleNamespace

import pytest
import yaml

from robot_app.home_setup import terminal_home
from robot_config.setup_steps import Io


SETTINGS = {"arm_controller": (["left_joint1", "right_joint1"], [0.1, -0.1]),
            "head_joint_controller": (["head_joint1"], [0.0])}
LIMITS = {"left_joint1": (-1.0, 1.0), "right_joint1": (-1.0, 1.0), "head_joint1": (-0.5, 0.5)}
CONFIG = """# retain this comment
control:
  patch:
    arm_controller:
      ros__parameters:
        home_2: [0.2, -0.2]
"""


def test_terminal_home_saves_only_changed_poses(tmp_path, monkeypatch):
    path = tmp_path / "robot.local.yaml"
    path.write_text(CONFIG)
    io = Io(out=StringIO())
    answers = iter(["manual", "0.4, -0.3", "keep", "y"])
    monkeypatch.setattr(io, "ask", lambda *_: next(answers))
    assert terminal_home(path, CONFIG, SETTINGS, LIMITS, io)
    result = path.read_text()
    patch = yaml.safe_load(result)["control"]["patch"]
    assert "# retain this comment" in result
    assert patch == {"arm_controller": {"ros__parameters": {"home_1": [0.4, -0.3], "home_2": [0.2, -0.2]}}}


def test_home_window_validates_then_captures_by_joint_name(tmp_path, monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PyQt5.QtWidgets import QApplication, QDialog
    from robot_app import home_window

    app = QApplication.instance() or QApplication([])
    path = tmp_path / "robot.local.yaml"
    path.write_text(CONFIG)
    positions = {"right_joint1": -0.4321, "head_joint1": 0.2, "left_joint1": 0.3456}
    monkeypatch.setattr(home_window, "read_joint_positions", lambda names: {n: positions[n] for n in names})
    with ThreadPoolExecutor(max_workers=1) as executor:
        dialog = home_window.HomeWindow(path, CONFIG, SETTINGS, LIMITS, executor)
        dialog.fields["arm_controller"][0].setText("2.5")
        dialog.save_button.click()
        assert "left_joint1" in dialog.message.text() and path.read_text() == CONFIG
        dialog.capture_button.click()
        dialog.pending.result(timeout=2)
        dialog.poll_operation()
        assert path.read_text() == CONFIG
        assert [field.text() for field in dialog.fields["arm_controller"]] == ["0.35", "-0.43"]
        head = dialog.fields["head_joint_controller"][0]
        assert head.text() == "0.20"
        head.setText("0.2087")
        head.editingFinished.emit()
        assert head.text() == "0.21"
        dialog.save_button.click()
        assert dialog.result() == QDialog.Accepted
    patch = yaml.safe_load(path.read_text())["control"]["patch"]
    assert patch["arm_controller"]["ros__parameters"]["home_1"] == [0.3456, -0.4321]
    assert patch["head_joint_controller"]["ros__parameters"]["home_1"] == [0.2087]
    app.processEvents()


def test_home_window_keeps_configured_precision_at_joint_limits(tmp_path, monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PyQt5.QtWidgets import QApplication, QDialog, QTreeWidget
    from robot_app import home_window

    app = QApplication.instance() or QApplication([])
    path = tmp_path / "robot.local.yaml"
    path.write_text(CONFIG)
    settings = {"arm_controller": (["left_joint1"], [0.9996])}
    limits = {"left_joint1": (-0.97231, 0.9997)}
    with ThreadPoolExecutor(max_workers=1) as executor:
        dialog = home_window.HomeWindow(path, CONFIG, settings, limits, executor)
        field = dialog.fields["arm_controller"][0]
        table = dialog.findChild(QTreeWidget)
        row = table.topLevelItem(0).child(0)
        assert row.text(1) == "1.00" and table.itemWidget(row, 1) is None
        assert row.text(3) == "-0.97  至  1.00"
        field.editingFinished.emit()
        assert field.text() == "1.00" and field.value() == 0.9996
        field.clear()
        assert field.text() == ""
        dialog.save_button.click()
        assert dialog.result() == QDialog.Accepted
        assert path.read_text() == CONFIG
    app.processEvents()


def test_home_window_accepts_steps_and_direct_input_from_configured_values(tmp_path, monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PyQt5.QtWidgets import QApplication, QDialog, QToolButton, QTreeWidget
    from robot_app import home_window

    app = QApplication.instance() or QApplication([])
    path = tmp_path / "robot.local.yaml"
    path.write_text(CONFIG)
    settings = {"arm_controller": (["left_joint1", "right_joint1"], [0.1234, -0.1])}
    limits = {"left_joint1": (0.1, 0.135), "right_joint1": (-1.0, 1.0)}
    with ThreadPoolExecutor(max_workers=1) as executor:
        dialog = home_window.HomeWindow(path, CONFIG, settings, limits, executor)
        table = dialog.findChild(QTreeWidget)
        group = table.topLevelItem(0)
        left_minus, left_plus = table.itemWidget(group.child(0), 2).findChildren(QToolButton)
        left, right = dialog.fields["arm_controller"]

        assert left.text() == "0.12" and right.text() == "-0.10"
        left_minus.click()
        assert left.value() == 0.1134 and right.value() == -0.1
        left_plus.click()
        left_plus.click()
        assert left.text() == "0.13" and left.value() == 0.1334
        left_plus.click()
        assert left.value() == 0.135
        right.setText("-0.2576")
        right.editingFinished.emit()
        assert right.text() == "-0.26" and left.value() == 0.135
        assert group.child(0).text(1) == "0.12" and group.child(1).text(1) == "-0.10"
        assert path.read_text() == CONFIG

        dialog.save_button.click()
        assert dialog.result() == QDialog.Accepted
    patch = yaml.safe_load(path.read_text())["control"]["patch"]
    assert patch["arm_controller"]["ros__parameters"]["home_1"] == [0.135, -0.2576]
    app.processEvents()


def test_home_window_requires_confirmation_before_returning_home(tmp_path, monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PyQt5.QtWidgets import QApplication, QMessageBox
    from robot_app import home_window

    app = QApplication.instance() or QApplication([])
    path = tmp_path / "robot.local.yaml"
    path.write_text(CONFIG)
    called = []

    def return_home(settings, stop):
        called.append(settings)
        return not stop.wait(2)

    answers = iter([QMessageBox.No, QMessageBox.Yes])
    monkeypatch.setattr(QMessageBox, "question", lambda *_: next(answers))
    monkeypatch.setattr(home_window, "return_home", return_home)
    with ThreadPoolExecutor(max_workers=1) as executor:
        dialog = home_window.HomeWindow(path, CONFIG, SETTINGS, LIMITS, executor)
        dialog.return_button.click()
        assert dialog.pending is None and called == []
        dialog.return_button.click()
        assert not dialog.save_button.isEnabled()
        dialog.stop_button.click()
        assert dialog.pending.result(timeout=2) is False
        dialog.poll_operation()
        assert called == [SETTINGS] and dialog.save_button.isEnabled()
        assert "已停止" in dialog.message.text() and path.read_text() == CONFIG
    app.processEvents()


def test_home_window_live_adjustment_only_saves_final_measured_pose(tmp_path, monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PyQt5.QtWidgets import QApplication, QMessageBox, QToolButton, QTreeWidget
    from robot_app import home_window

    app = QApplication.instance() or QApplication([])
    path = tmp_path / "robot.local.yaml"
    path.write_text(CONFIG)
    steps = []
    measured = {"left_joint1": 0.1345, "right_joint1": -0.14, "head_joint1": 0.03}

    class FakeJog:
        def __init__(self, settings, limits):
            self.stop = Event()

        def run(self):
            assert self.stop.wait(2)
            return measured

        def snapshot(self):
            return measured, True

        def request_step(self, controller, joint, direction):
            steps.append((controller, joint, direction))
            return True

    answers = iter([QMessageBox.No, QMessageBox.Yes])
    monkeypatch.setattr(QMessageBox, "question", lambda *_: next(answers))
    monkeypatch.setattr(home_window, "HomeJog", FakeJog)
    with ThreadPoolExecutor(max_workers=1) as executor:
        dialog = home_window.HomeWindow(path, CONFIG, SETTINGS, LIMITS, executor)
        table = dialog.findChild(QTreeWidget)
        left = table.topLevelItem(0).child(0)
        _, plus = table.itemWidget(left, 2).findChildren(QToolButton)
        dialog.jog_button.click()
        assert dialog.pending is None
        dialog.jog_button.click()
        assert not dialog.save_button.isEnabled() and not plus.isEnabled()
        dialog.poll_operation()
        assert left.text(4) == "0.13" and plus.isEnabled()
        plus.click()
        assert steps == [("arm_controller", "left_joint1", 1)] and not plus.isEnabled()
        dialog.stop_button.click()
        dialog.pending.result(timeout=2)
        dialog.poll_operation()
        assert "已回 HOLD" in dialog.message.text()
        assert path.read_text() == CONFIG and dialog.save_button.isEnabled()
        dialog.save_button.click()
    patch = yaml.safe_load(path.read_text())["control"]["patch"]
    assert patch["arm_controller"]["ros__parameters"]["home_1"] == [0.1345, -0.14]
    app.processEvents()


def test_home_jog_aborts_on_feedback_loss_and_requests_hold(monkeypatch):
    from robot_app import home_motion

    jog = home_motion.HomeJog(SETTINGS, LIMITS)
    commands = []
    required = (*SETTINGS, "joint_state_broadcaster")
    active = dict.fromkeys(required, "active")
    probe = SimpleNamespace(fsm_state=home_motion.FSM_COMMANDS["HOME"], controller_states=lambda: active,
                            request_hold=lambda **_: True)

    def send_fsm(mode):
        commands.append(mode)
        probe.fsm_state = mode

    probe.send_fsm = send_fsm
    movej = home_motion.FSM_COMMANDS["MOVEJ"]
    frames = iter([(0, {})] * 4 + [None])
    feedback = SimpleNamespace(latest=lambda: next(frames, None))
    publishers = {name: SimpleNamespace(get_subscription_count=lambda: 1) for name in SETTINGS}
    monkeypatch.setattr(home_motion, "_live_home", lambda _, name, _stop: SETTINGS[name])

    def send_target(*_):
        pytest.fail("No step should be published without feedback")

    with pytest.raises(RuntimeError, match="关节反馈中断"):
        jog._run(probe, feedback, required, publishers, send_target)
    assert commands == [home_motion.FSM_HOLD, movej, home_motion.FSM_HOLD]
