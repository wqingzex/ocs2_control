"""What every device setup shares: the session base class, the Qt window shell and the console I/O.

A device package (w2_camera, w2_hand) supplies a session (the protocol: state, update, save) and a panel (the
cards the window shows). The window knows neither device: it calls the panel for everything that differs.
Qt is imported only when a window is created, so config and protocol tests run without it."""
from __future__ import annotations

import argparse
import fcntl
import os
import sys
import threading
from pathlib import Path

import yaml

from .config_edit import confirmed


class Io:
    """Console I/O; tests replace ask()."""

    def __init__(self, assume_yes: bool = False, out=None):
        self.assume_yes = assume_yes
        self.out = out or sys.stdout

    def info(self, msg: str) -> None:
        print(f">>> {msg}", file=self.out)

    def warn(self, msg: str) -> None:
        print(f"[WARN] {msg}", file=self.out)

    def error(self, msg: str) -> None:
        print(f"[ERROR] {msg}", file=self.out)

    def ask(self, question: str, default: str) -> str:
        if self.assume_yes or not sys.stdin.isatty():
            return default
        answer = input(f"{question} [{default}]: ").strip()
        return answer or default


class SetupSession:
    """A draft of one device's configuration; only save() touches robot.local.yaml.

    A subclass sets kind ('cameras' | 'hand') and apply_on_open, and implements state(), update(), save() and
    create_panel(window)."""
    kind = ''
    apply_on_open = False  # the window applies the draft as soon as it opens (cameras start their previews)

    def __init__(self, config_path, ws_root):
        self.config_path, self.ws_root = config_path, ws_root
        self.original = Path(config_path).read_text(encoding='utf-8')
        self.raw = yaml.safe_load(self.original) or {}
        self.lock = threading.RLock()
        self.done = threading.Event()
        self.saved, self.cancelled, self.revision = False, False, 0

    def active(self):
        if self.done.is_set():
            raise ValueError('配置会话已结束')

    def cancel(self):
        with self.lock:
            if not self.saved:
                self.cancelled = True
                self.done.set()

    def base_state(self):
        return {'kind': self.kind, 'revision': self.revision, 'saved': self.saved,
                'cancelled': self.cancelled, 'previous_confirmed': confirmed(self.raw, self.kind)}

    def close(self):
        self.cancel()


def create_window(session):
    """The window of a session. Its device-specific part is session.create_panel(window), which provides:
    title, size, hint, swap_label, apply_label, build(grid, options), apply_payload(), swap(), confirmed(), refresh(busy)."""
    from concurrent.futures import ThreadPoolExecutor
    from PyQt5 import QtCore, QtWidgets as Q

    class Window(Q.QWidget):
        def __init__(self):
            super().__init__()
            self.session = session
            self.state = session.state()
            self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='w2_setup')
            self.future = None
            self.dirty = False
            self.closing = False
            self.saved = False
            self.cards = {}
            self.panel = session.create_panel(self)
            self.setWindowTitle(self.panel.title)
            self.resize(*self.panel.size)
            layout = Q.QVBoxLayout(self)
            heading = Q.QLabel(self.windowTitle())
            heading.setStyleSheet('font-size:22px;font-weight:600')
            layout.addWidget(heading)
            hint = Q.QLabel(self.panel.hint + '\n左右均以机器人自身方向为准。确认保存前，原配置保持不变。')
            hint.setWordWrap(True)
            layout.addWidget(hint)
            self.message = Q.QLabel('正在读取设备…')
            self.message.setWordWrap(True)
            layout.addWidget(self.message)
            self.options = Q.QWidget()
            self.option_layout = Q.QHBoxLayout(self.options)
            self.option_layout.setContentsMargins(0, 0, 0, 0)
            layout.addWidget(self.options)
            scroll = Q.QScrollArea()
            scroll.setWidgetResizable(True)
            content = Q.QWidget()
            grid = Q.QGridLayout(content)
            scroll.setWidget(content)
            layout.addWidget(scroll, 1)
            self.panel.build(grid, self.option_layout)
            buttons = Q.QHBoxLayout()
            self.cancel_button = Q.QPushButton('取消，保留原配置')
            self.cancel_button.clicked.connect(self.close)
            self.swap_button = Q.QPushButton(self.panel.swap_label)
            self.swap_button.clicked.connect(self.swap)
            self.apply_button = Q.QPushButton(self.panel.apply_label)
            self.apply_button.clicked.connect(self.apply)
            self.save_button = Q.QPushButton('确认并保存')
            self.save_button.setDefault(True)
            self.save_button.clicked.connect(self.save)
            for button in (self.cancel_button, self.swap_button, self.apply_button):
                buttons.addWidget(button)
            buttons.addStretch(1)
            buttons.addWidget(self.save_button)
            layout.addLayout(buttons)
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.tick)
            self.timer.start(250)
            self.refresh()

        def combo(self, options, selected):
            combo = Q.QComboBox()
            for value, title in options:
                combo.addItem(title, value)
            index = combo.findData(selected)
            combo.setCurrentIndex(max(0, index))
            combo.currentIndexChanged.connect(self.mark_dirty)
            return combo

        def show_message(self, text, error=False):
            self.message.setText(text)
            self.message.setStyleSheet('color:#b42318' if error else 'color:#176b39')

        def mark_dirty(self, *_):
            self.dirty = True
            for card in self.cards.values():
                card['check'].setChecked(False)
            self.show_message('配置已修改，请先应用，再核对实物并重新确认。')
            self.refresh()

        def submit(self, function, payload, success):
            if self.future or self.closing:
                return
            self.success = success
            self.future = self.executor.submit(function, payload)
            self.show_message('正在处理设备，请稍候…')
            self.refresh()

        def apply(self):
            payload = self.panel.apply_payload()
            if payload is not None:  # None: the operator declined something the panel asked
                self.submit(session.update, payload, 'applied')

        def swap(self):
            if self.panel.swap():
                self.apply()

        def save(self):
            self.submit(session.save, {'revision': self.state['revision'], 'confirmed': self.panel.confirmed()}, 'saved')

        def tick(self):
            if self.future:
                if not self.future.done():
                    return
                future, self.future = self.future, None
                try:
                    self.state = future.result()
                    if self.success == 'saved':
                        self.saved = True
                        self.show_message('配置已保存，返回终端继续初始化。')
                        self.close()
                        return
                    if self.success == 'applied':
                        self.dirty = False
                    self.show_message('请核对画面和实物，并逐项勾选确认。')
                except (ValueError, RuntimeError, OSError) as exc:
                    self.state = session.state()
                    self.show_message(str(exc), True)
                except Exception as exc:
                    self.state = session.state()
                    self.show_message('设备操作失败：' + str(exc), True)
                if self.closing:
                    self.close()
                    return
            elif self.closing:
                self.close()
                return
            else:
                self.state = session.state()
            self.refresh()

        def refresh(self):
            busy = self.future is not None or self.closing
            if hasattr(self, 'apply_button'):
                for button in (self.apply_button, self.swap_button):
                    button.setEnabled(not busy)
                self.save_button.setEnabled(not busy and not self.dirty)
            self.options.setEnabled(not busy)
            self.panel.refresh(busy)

        def closeEvent(self, event):
            if self.future:
                self.closing = True
                self.show_message('正在结束本次设备操作，随后释放设备并关闭窗口。')
                self.refresh()
                event.ignore()
                return
            self.timer.stop()
            if not self.saved:
                session.cancel()
            self.executor.shutdown(wait=True)
            event.accept()

    return Window()


def run_wizard(session, io):
    import signal
    try:
        from PyQt5 import QtCore, QtWidgets, sip
    except ImportError as exc:
        raise RuntimeError('缺少本地配置窗口依赖 python3-pyqt5，请先运行 ./scripts/setup.sh deps') from exc
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY') or os.environ.get('QT_QPA_PLATFORM') == 'offscreen'):
        raise RuntimeError('未找到桌面显示；请在桌面终端执行，或为容器映射 DISPLAY、X11 和授权文件')
    with open(session.config_path + '.wizard.lock', 'a', encoding='utf-8') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('此配置文件已有初始化窗口，请先完成或取消该窗口') from exc
        app = QtWidgets.QApplication.instance()
        if app is None:
            # QLabel previews use raster painting and do not need a GLX visual.
            os.environ.setdefault('QT_XCB_GL_INTEGRATION', 'none')
            QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_UseSoftwareOpenGL)
            app = QtWidgets.QApplication(['W2 初始化'])
        app.setQuitOnLastWindowClosed(False)
        window = create_window(session)
        loop = QtCore.QEventLoop()
        window.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        window.destroyed.connect(loop.quit)
        # Python must regularly regain control to handle Ctrl+C during Qt.exec_().
        interrupts = QtCore.QTimer()
        interrupts.timeout.connect(lambda: None)
        interrupts.start(100)
        interrupted = []
        previous_handler = signal.getsignal(signal.SIGINT)

        def cancel_signal(_number, _frame):
            interrupted.append(True)
            if not sip.isdeleted(window):
                window.close()

        signal.signal(signal.SIGINT, cancel_signal)
        io.info('已打开本地配置窗口；核对实物后确认保存，关闭窗口或 Ctrl+C 取消')
        io.out.flush()
        window.show()
        if session.apply_on_open:
            QtCore.QTimer.singleShot(0, window.apply)
        try:
            loop.exec_()
            if interrupted:
                raise KeyboardInterrupt
            return session.saved
        finally:
            signal.signal(signal.SIGINT, previous_handler)
            interrupts.stop()
            if not sip.isdeleted(window):
                window.close()


def main_for(step, argv=None, *, failure_hint='', reset=None):
    """The command line of one setup step: step(config_path, ws_root, io) -> bool; exit code 0 / 1 / 130 (Ctrl+C).
    reset(config_path, ws_root, io) is run instead for --reset (only the load identification has one)."""
    parser = argparse.ArgumentParser(prog='setup.sh')
    parser.add_argument('--config', required=True)
    parser.add_argument('--ws-root', required=True)
    parser.add_argument('--yes', action='store_true')
    if reset:
        parser.add_argument('--reset', action='store_true', help='back to the default load (empties both overrides)')
    args = parser.parse_args(argv)
    io = Io(assume_yes=args.yes)
    try:
        run = reset if reset and args.reset else step
        ok = run(args.config, args.ws_root, io)
    except KeyboardInterrupt:
        io.info('已取消初始化')
        return 130
    except (RuntimeError, ValueError, OSError) as exc:
        io.error(str(exc))
        ok = False
    if not ok and failure_hint:
        io.error(failure_hint)
    return 0 if ok else 1
