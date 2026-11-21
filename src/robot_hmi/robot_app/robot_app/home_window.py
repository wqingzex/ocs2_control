"""Local HOME editor with an explicit return-to-HOME action."""
from concurrent.futures import ThreadPoolExecutor
from decimal import Decimal
from functools import partial
import math
import signal
from threading import Event

from PyQt5 import QtCore as C, QtWidgets as Q

from .home_motion import ANGLE_STEP, HomeJog, return_home
from .home_setup import home_updates, read_joint_positions, required_joints, save_home


def _angle_text(value):
    if type(value) in (int, float):
        return f"{value:.2f}"
    return "" if value is None else str(value)


class HomeAngleField(Q.QLineEdit):
    """Edit a HOME value with the configured angle as the fallback for an empty field."""

    def __init__(self, configured):
        super().__init__()
        self._configured = configured
        self.set_value(configured)
        self.editingFinished.connect(self._format_value)

    def set_value(self, value):
        self._value = value if type(value) in (int, float) else None
        self._display_text = _angle_text(value)
        self.setText(self._display_text)
        self.setModified(False)
        self.setCursorPosition(0)

    def value(self):
        text = self.text()
        if not text.strip() and self._configured is not None:
            return float(str(self._configured))
        if self._value is not None and text == self._display_text and not self.isModified():
            return self._value
        return float(text)

    def _format_value(self):
        if not self.text().strip():
            return
        try:
            value = self.value()
        except ValueError:
            return
        self.set_value(value)

    def step_by(self, amount, bounds):
        try:
            value = self.value() if self.text().strip() or self._configured is not None else 0.0
        except ValueError:
            self.setFocus()
            return
        if not math.isfinite(value):
            self.setFocus()
            return
        value = float(Decimal(str(value)) + amount)
        if bounds is not None:
            value = min(max(value, bounds[0]), bounds[1])
        self.set_value(value)


class HomeWindow(Q.QDialog):
    def __init__(self, path, original, settings, limits, executor):
        super().__init__()
        self.path, self.original = path, original
        self.settings, self.limits = settings, limits
        self.executor = executor
        self.pending = None
        self.motion_stop = None
        self.jog = None
        self.jog_ready = False
        self.closing = False
        self.setWindowTitle("HOME 位姿设置")
        self.resize(820, 650)
        layout = Q.QVBoxLayout(self)
        hint = Q.QLabel("可点 + / − 逐关节调整，也可直接输入弧度值。清空配置框会沿用当前 HOME。")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        self.message = Q.QLabel()
        self.message.setWordWrap(True)
        layout.addWidget(self.message)

        table = Q.QTreeWidget()
        table.setHeaderLabels(["关节", "当前 HOME（rad）", "配置值（rad）", "模型限位（rad）", "实测（rad）"])
        table.header().setSectionResizeMode(Q.QHeaderView.ResizeToContents)
        table.header().setSectionResizeMode(2, Q.QHeaderView.Fixed)
        table.header().resizeSection(2, 230)
        self.fields = {}
        self.joint_rows = {}
        self.step_buttons = []
        for name, (joints, home) in settings.items():
            group = Q.QTreeWidgetItem(table, [name])
            self.fields[name] = []
            if not isinstance(home, list):
                home = []
            for index, joint in enumerate(joints):
                bounds = limits.get(joint)
                interval = "  至  ".join(f"{edge:.2f}" for edge in bounds) if bounds else "缺少限位"
                configured = home[index] if index < len(home) else None
                row = Q.QTreeWidgetItem(group, [joint, _angle_text(configured) or "—", "", interval, "—"])
                self.joint_rows[joint] = row
                field = HomeAngleField(configured)
                editor = Q.QWidget()
                controls = Q.QHBoxLayout(editor)
                controls.setContentsMargins(0, 0, 0, 0)
                controls.setSpacing(4)
                controls.addWidget(field, 1)
                for symbol, direction, label in (("−", -1, "减少"), ("+", 1, "增加")):
                    button = Q.QToolButton()
                    button.setText(symbol)
                    button.setAccessibleName(f"{joint} {label} {ANGLE_STEP} rad")
                    button.setToolTip(f"{label} {ANGLE_STEP} rad")
                    button.clicked.connect(partial(self.step_joint, name, joint, field, bounds, direction))
                    controls.addWidget(button)
                    self.step_buttons.append(button)
                table.setItemWidget(row, 2, editor)
                self.fields[name].append(field)
        table.expandAll()
        layout.addWidget(table, 1)

        row = Q.QHBoxLayout()
        self.return_button = Q.QPushButton("回已配置 HOME")
        self.return_button.clicked.connect(self.start_return_home)
        self.return_button.setAutoDefault(False)
        row.addWidget(self.return_button)
        self.jog_button = Q.QPushButton("开启实时调姿")
        self.jog_button.clicked.connect(self.start_jog)
        self.jog_button.setAutoDefault(False)
        row.addWidget(self.jog_button)
        self.stop_button = Q.QPushButton("停止回 HOME")
        self.stop_button.clicked.connect(self.stop_motion)
        self.stop_button.setEnabled(False)
        self.stop_button.setAutoDefault(False)
        row.addWidget(self.stop_button)
        self.capture_button = Q.QPushButton("读取当前关节反馈")
        self.capture_button.clicked.connect(self.capture)
        self.capture_button.setAutoDefault(False)
        row.addWidget(self.capture_button)
        row.addStretch()
        cancel = Q.QPushButton("取消")
        cancel.clicked.connect(self.reject)
        cancel.setAutoDefault(False)
        row.addWidget(cancel)
        self.save_button = Q.QPushButton("确认并保存")
        self.save_button.clicked.connect(self.save)
        self.save_button.setAutoDefault(False)
        row.addWidget(self.save_button)
        layout.addLayout(row)

        self.timer = C.QTimer(self)
        self.timer.timeout.connect(self.poll_operation)
        self.timer.start(100)

    def show_status(self, message, error=False):
        self.message.setText(message)
        self.message.setStyleSheet("color:#b42318" if error else "")

    def capture(self):
        self.show_status("等待读取当前关节反馈…")
        self.pending = self.executor.submit(read_joint_positions, required_joints(self.settings))
        self.update_buttons()

    def step_joint(self, controller, joint, field, bounds, direction, _checked=False):
        if self.jog is not None:
            if self.jog.request_step(controller, joint, direction):
                self.jog_ready = False
                self.show_status(f"正在点动 {joint}，等待关节反馈…")
                self.update_buttons()
        elif self.pending is None:
            field.step_by(ANGLE_STEP * direction, bounds)

    def start_jog(self):
        answer = Q.QMessageBox.question(
            self, "开启实时调姿",
            "运控将进入 MOVEJ。每次点击 + 或 − 会从当前实测位置点动关节 0.01 rad。"
            "请确认运动范围内无人无物，并停止其他控制端操作。\n"
            "结束调姿后会回 HOLD，并把实测位置填入 HOME 编辑框。是否开启？",
            Q.QMessageBox.Yes | Q.QMessageBox.No, Q.QMessageBox.No,
        )
        if answer != Q.QMessageBox.Yes:
            return
        self.jog = HomeJog(self.settings, self.limits)
        self.motion_stop = self.jog.stop
        self.show_status("正在核对关节反馈并切到 MOVEJ…")
        self.pending = self.executor.submit(self.jog.run)
        self.update_buttons()

    def start_return_home(self):
        try:
            home_updates(self.settings, self.limits, {name: home for name, (_, home) in self.settings.items()})
        except ValueError as exc:
            self.show_status(str(exc), True)
            return
        answer = Q.QMessageBox.question(self, "回 HOME",
            "机器人将移动到运控已加载的 HOME。请确认运动范围内无人无物，是否执行？",
            Q.QMessageBox.Yes | Q.QMessageBox.No, Q.QMessageBox.No)
        if answer != Q.QMessageBox.Yes:
            return
        self.motion_stop = Event()
        self.show_status("正在核对运控状态并等待 HOME 到位…")
        self.pending = self.executor.submit(return_home, self.settings, self.motion_stop)
        self.update_buttons()

    def stop_motion(self):
        if self.motion_stop is not None:
            self.motion_stop.set()
            self.show_status("正在结束调姿并回 HOLD…" if self.jog is not None else "正在停止回 HOME…")
            self.update_buttons()

    def update_buttons(self):
        ready = self.pending is None
        for button in (self.return_button, self.jog_button, self.capture_button, self.save_button):
            button.setEnabled(ready)
        for button in self.step_buttons:
            button.setEnabled(ready or (self.jog_ready and self.jog is not None and not self.jog.stop.is_set()))
        for fields in self.fields.values():
            for field in fields:
                field.setReadOnly(self.jog is not None)
        self.stop_button.setText("结束调姿并回 HOLD" if self.jog is not None else "停止回 HOME")
        self.stop_button.setEnabled(self.motion_stop is not None and not self.motion_stop.is_set())

    def _show_feedback(self, positions):
        for joint, row in self.joint_rows.items():
            row.setText(4, f"{positions[joint]:.2f}" if positions is not None else "—")

    def _fill_values(self, positions):
        for name, (joints, _) in self.settings.items():
            for joint, field in zip(joints, self.fields[name]):
                field.set_value(positions[joint])

    def poll_operation(self):
        if self.pending is None:
            return
        if not self.pending.done():
            if self.jog is not None:
                positions, ready = self.jog.snapshot()
                self._show_feedback(positions)
                if ready != self.jog_ready:
                    self.jog_ready = ready
                    if ready:
                        self.show_status("MOVEJ 实时调姿已开启，按 + 或 − 逐关节点动。")
                    self.update_buttons()
            return
        try:
            result = self.pending.result()
            if self.jog is not None:
                self._show_feedback(result)
                if result is None:
                    self.show_status("已取消实时调姿。")
                else:
                    self._fill_values(result)
                    self.show_status("已回 HOLD，实测位置已填入 HOME，请核对后保存。")
            elif self.motion_stop is not None:
                self.show_status("HOME 已到位，可调整数值或读取反馈。" if result else "已停止回 HOME。")
            else:
                self._fill_values(result)
                self._show_feedback(result)
                self.show_status("已读取反馈，请核对后保存。")
        except (RuntimeError, ValueError, OSError) as exc:
            if self.jog is not None:
                self._show_feedback(None)
                self.closing = False
            self.show_status(str(exc), True)
        finally:
            self.pending = None
            self.motion_stop = None
            self.jog = None
            self.jog_ready = False
            self.update_buttons()
            if self.closing:
                self.reject()

    def reject(self):
        if self.motion_stop is not None:
            self.closing = True
            self.stop_motion()
            return
        super().reject()

    def save(self):
        candidates = {}
        try:
            for name, fields in self.fields.items():
                values = []
                for joint, field in zip(self.settings[name][0], fields):
                    try:
                        values.append(field.value())
                    except ValueError as exc:
                        field.setFocus()
                        raise ValueError(f"请输入 {joint} 的弧度值") from exc
                candidates[name] = values
            save_home(self.path, self.original, home_updates(self.settings, self.limits, candidates))
        except (RuntimeError, ValueError, OSError) as exc:
            self.show_status(str(exc), True)
            return
        self.accept()


def run_home_window(path, original, settings, limits, io):
    app = Q.QApplication.instance() or Q.QApplication(["HOME 位姿设置"])
    with ThreadPoolExecutor(max_workers=1) as executor:
        window = HomeWindow(path, original, settings, limits, executor)
        interrupted = []

        def cancel(*_):
            interrupted.append(True)
            window.reject()

        handler = signal.signal(signal.SIGINT, cancel)
        try:
            saved = window.exec_() == Q.QDialog.Accepted
        finally:
            signal.signal(signal.SIGINT, handler)
            window.timer.stop()
        if interrupted:
            raise KeyboardInterrupt
    if saved:
        io.info("HOME 已确认，下次启动运控时加载")
    return saved
