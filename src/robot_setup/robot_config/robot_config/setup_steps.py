"""The setup steps that belong to the configuration, and what every device setup shares.

Shared: the session base class, the Qt window shell and the console I/O (camera_setup, end_effector_setup and xr_setup build
on them). Steps: `config` (write robot.local.yaml from the template of an entry, or add the keys it lacks) and `dyn` (tool
load identification: run the Marvin wizard per arm, then write the result to the config).

The command line of these steps, which scripts/setup.sh runs:
  python3 -m robot_config.setup_steps config --config FILE [--entry E] [--yes] [--status]
  python3 -m robot_config.setup_steps dyn    --config FILE --ws-root DIR [--yes] [--reset]
  python3 -m robot_config.setup_steps status --config FILE STEP     (cameras | ee | dyn: does it still need the operator)
  python3 -m robot_config.setup_steps check  FILE                    (the configuration summary)"""
from __future__ import annotations

import argparse
import copy
import fcntl
import json
import math
import os
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable, List, Optional, Sequence, Tuple

import yaml

from . import config_file
from .config import FREEFORM, entries, template_document, template_text
from .config_file import DYN_LENGTH, DYN_SIDES, confirmed, dyn_status, dyn_values

# ---- shared by every device setup ---------------------------------------------------------------------------

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

    A subclass sets kind ('cameras' | 'ee') and apply_on_open, and implements state(), update(), save() and
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
            self.apply_again = False
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
            self.apply_button.setVisible(bool(self.panel.apply_label))  # a panel that applies by itself has no label
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

        def apply(self, *_):
            if self.future or self.closing:
                self.apply_again = True  # a change made while busy is applied once the current operation ends
                return
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
            if self.apply_again and not self.future and not self.closing:
                self.apply_again = False
                self.apply()

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


# ---- config: write robot.local.yaml from the template, or add the keys it lacks -------------------------------

DEFAULT_ENTRY = "whole_body"


def _paths(node: Any, prefix: str = "") -> List[str]:
    """Every key path of a config; control.patch and setup are free-form and end the path."""
    found: List[str] = []
    for key, value in node.items():
        path = f"{prefix}.{key}" if prefix else key
        if isinstance(value, dict) and path not in FREEFORM:
            found.extend(_paths(value, path))
        else:
            found.append(path)
    return found


def unknown_keys(local: Any, template: Any) -> List[str]:
    """Key paths of the file that the template does not have."""
    return sorted(set(_paths(local)) - set(_paths(template)))


def missing_keys(local: Any, template: Any) -> List[str]:
    """Key paths of the template that the file lacks."""
    present = set(_paths(local))
    return [path for path in _paths(template) if path not in present and not _has_parent(present, path)]


def _has_parent(present: set, path: str) -> bool:
    """A leaf of the template that sits below a leaf of the file (e.g. a list or a free-form mapping) is not missing."""
    parts = path.split(".")
    return any(".".join(parts[:n]) in present for n in range(1, len(parts)))


def add_missing(local: Any, template: Any, prefix: str = "") -> List[str]:
    """Add the template's keys the mapping lacks (values from the template); returns the paths added."""
    added: List[str] = []
    for key, value in template.items():
        path = f"{prefix}.{key}" if prefix else key
        if key not in local:
            local[key] = copy.deepcopy(value)
            added.append(path)
        elif isinstance(value, dict) and isinstance(local[key], dict) and path not in FREEFORM:
            added.extend(add_missing(local[key], value, path))
    return added


# Keys (dotted paths) the template no longer has: an old file keeps them until `setup.sh config` deletes them.
RETIRED_KEYS = ("navigation", "joystick", "robot.end_effector.interface.left.channel", "robot.end_effector.interface.right.channel")


def drop_retired(local: Any) -> List[str]:
    """Delete the retired keys from the mapping; returns the paths deleted."""
    dropped = []
    for path in RETIRED_KEYS:
        *parents, key = path.split(".")
        node = local
        for name in parents:
            node = node.get(name) if isinstance(node, dict) else None
        if isinstance(node, dict) and key in node:
            del node[key]
            dropped.append(path)
    return dropped


def _entry_of(raw: Any, requested: Optional[str], io: Io) -> str:
    entry = requested or ((raw.get("robot") or {}).get("entry") if raw else None)
    if not entry:
        entry = io.ask(f"控制入口 {' | '.join(entries())}", DEFAULT_ENTRY)
    if entry not in entries():
        raise ValueError(f"robot.entry 只能是 {list(entries())}，不是 {entry!r}")
    return entry


def sync_status(path: str) -> Tuple[bool, str]:
    """(needs work, reason) without writing anything."""
    if not os.path.isfile(path):
        return True, "还没有本机配置"
    _, raw = config_file.load_rt(path)
    entry = (raw.get("robot") or {}).get("entry")
    if entry not in entries():
        return True, f"robot.entry 缺失或不对（{entry!r}）"
    template = template_document(entry)
    missing, unknown = missing_keys(raw, template), unknown_keys(raw, template)
    if missing:
        return True, f"缺 {len(missing)} 个键（如 {missing[0]}）"
    if unknown:
        return True, f"有 {len(unknown)} 个多余的键（如 {unknown[0]}）"
    return False, f"配置完整（入口 {entry}）"


def sync_run(path: str, requested_entry: Optional[str], io: Io) -> bool:
    if not os.path.isfile(path):
        entry = _entry_of(None, requested_entry, io)
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(template_text(entry))
        io.info(f"已按入口 {entry} 从模板生成 {path}；[必改] 的项请核对，设备相关的项由 ./scripts/setup.sh 后面的步骤写入")
        return True
    yaml_rt, raw = config_file.load_rt(path)
    entry = _entry_of(raw, requested_entry, io)
    if (raw.get("robot") or {}).get("entry") not in (None, entry):
        io.error(f"配置里的 robot.entry 是 {raw['robot']['entry']!r}，不是 {entry!r}：改入口要手动改配置里的值，别的键按新入口的模板核对")
        return False
    template = template_document(entry)
    if "robot" not in raw:
        raw["robot"] = {}
    raw["robot"].setdefault("entry", entry)
    dropped = drop_retired(raw)
    added = add_missing(raw, template)
    unknown = unknown_keys(raw, template)
    if added or dropped:
        shutil.copy2(path, path + ".bak")
        config_file.save_rt(path, yaml_rt, raw)
        io.info(f"已改动配置（原文件备份为 {path}.bak）：补上 {len(added)} 个缺的键，删掉 {len(dropped)} 个已退役的段")
        for item in added:
            io.info(f"  + {item}")
        for item in dropped:
            io.info(f"  - {item}（已退役）")
    else:
        io.info("配置完整，没有要补的键")
    if unknown:
        io.error("下面这些键不在模板里，程序不会读它们，加载时会报错；删掉或改成模板里的写法：")
        for item in unknown:
            io.error(f"  - {item}")
        return False
    return True


# ---- dyn: tool load identification ----------------------------------------------------------------------------

WIZARD = ("ros2", "run", "dyn_setup", "tool_dyn_identify_wizard")
ARM_OF_SIDE = {"left": "A", "right": "B"}  # the wizard's own names for the arms
YES = ("y", "yes", "是")


def output_dir(ws_root: str) -> str:
    return os.path.join(ws_root, "logs", "dyn_identify")


def result_path(ws_root: str, arm: str) -> str:
    return os.path.join(output_dir(ws_root), f"result_{arm}.json")


MTIME_TOLERANCE_S = 0.01  # file timestamps can lag time.time() by up to ~1 ms (coarse kernel clock, overlayfs)


def read_result(path: str, newer_than: float) -> Optional[List[float]]:
    """The 10 identified values the wizard wrote after newer_than, or None."""
    try:
        if os.path.getmtime(path) < newer_than - MTIME_TOLERANCE_S:
            return None
        with open(path, encoding="utf-8") as handle:
            values = json.load(handle)["params"]
    except (OSError, ValueError, KeyError, TypeError):
        return None
    ok = isinstance(values, list) and len(values) == DYN_LENGTH and \
        all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values)
    return [float(v) for v in values] if ok else None


def remember(ws_root: str, side: str, values: Optional[Sequence[float]]) -> str:
    """Keep the value that is about to be replaced; returns the file it went to."""
    os.makedirs(output_dir(ws_root), exist_ok=True)
    path = os.path.join(output_dir(ws_root), f"previous_{side}_{time.strftime('%Y%m%d_%H%M%S')}.json")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump({"side": side, "params": list(values) if values else []}, handle)
    return path


def step_dyn(config_path: str, ws_root: str, io, runner: Callable[..., int] = subprocess.call) -> bool:
    """Identify the arms the operator picks (default: none, keep the current values)."""
    _, raw = config_file.load_rt(config_path)
    status = dyn_status(raw)
    io.info(f"负载参数 {status.state}：{status.reason}")
    if (raw.get("robot") or {}).get("hardware", "real") != "real":
        return True
    ok = True
    for side in DYN_SIDES:
        arm = ARM_OF_SIDE[side]
        label = "左" if side == "left" else "右"
        if io.ask(f"辨识{label}臂的工具负载？需要真机，约 10 分钟，手臂会运动 y/n", "n").lower() not in YES:
            continue
        started = time.time()
        code = runner(list(WIZARD), env=dict(os.environ, TOOL_DYN_ARM=arm), cwd=ws_root)
        values = read_result(result_path(ws_root, arm), started)
        if code != 0 or values is None:
            io.error(f"{label}臂辨识没有得到结果（退出码 {code}）；日志见 logs/dyn_identify/")
            ok = False
            continue
        _, raw = config_file.load_rt(config_path)
        old = dyn_values(raw, side)
        io.info(f"{label}臂辨识结果: {[round(v, 6) for v in values]}")
        io.info(f"当前值: {list(old) if old else '空（使用默认）'}")
        if io.ask("写入配置？y/n", "y").lower() not in YES:
            io.info("未写入，保持原值")
            continue
        io.info(f"原值已记录到 {remember(ws_root, side, old)}")
        config_file.set_float_list(config_path, "robot.end_effector.dyn_param", side, values)
        io.info(f"已写入 robot.end_effector.dyn_param.{side}")
    return ok


def reset_dyn(config_path: str, ws_root: str, io) -> bool:
    """Back to the defaults: both overrides are emptied (the old values are kept in logs/dyn_identify)."""
    _, raw = config_file.load_rt(config_path)
    for side in DYN_SIDES:
        old = dyn_values(raw, side)
        if old:
            io.info(f"{side} 原值已记录到 {remember(ws_root, side, old)}")
        config_file.set_float_list(config_path, "robot.end_effector.dyn_param", side, [])
    io.info("已清空 robot.end_effector.dyn_param.left / right，使用默认负载")
    return True


# ---- the command line ------------------------------------------------------------------------------------------

def config_command(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="setup.sh config")
    parser.add_argument("--config", required=True)
    parser.add_argument("--entry", choices=entries(), default=None)
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--status", action="store_true", help="only print whether the file needs work (exit 1 if so)")
    args = parser.parse_args(argv)
    if args.status:
        needed, reason = sync_status(args.config)
        print(f"{'需配置' if needed else '已配置'}：{reason}")
        return 1 if needed else 0
    io = Io(assume_yes=args.yes)
    try:
        ok = sync_run(args.config, args.entry, io)
    except KeyboardInterrupt:
        io.info("已取消")
        return 130
    except ValueError as exc:
        io.error(str(exc))
        ok = False
    return 0 if ok else 1


def dyn_command(argv=None) -> int:
    return main_for(step_dyn, argv, reset=reset_dyn)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    commands = ("config", "dyn", "status", "check")
    if not args or args[0] not in commands:
        print(f"usage: python3 -m robot_config.setup_steps {{{'|'.join(commands)}}} ...", file=sys.stderr)
        return 2
    command, rest = args[0], args[1:]
    if command == "config":
        return config_command(rest)
    if command == "dyn":
        return dyn_command(rest)
    return config_file.main([command, *rest])


if __name__ == "__main__":
    sys.exit(main())
