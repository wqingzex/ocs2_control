"""Camera setup (./scripts/setup.sh cam): the draft session, the driver previews and the window panel.

    python3 -m camera_setup.session --config C --ws-root W [--yes]"""
from __future__ import annotations

import copy
import os
import sys
import tempfile
import threading
import time
import uuid
from dataclasses import asdict
from pathlib import Path
from typing import Callable, List

import yaml

from robot_config import config_file
from robot_config.config import ROLES, ROTATIONS, section_defaults, validate_camera_assignments
from robot_config.config_file import confirmed
from robot_config.setup_steps import Io, SetupSession, main_for, run_wizard
from robot_runtime.processes import OwnedDrivers, require_idle

from launch_helpers.devices.camera import DRIVER_READY_LOG, DRIVER_READY_TIMEOUT_S

from . import discovery
from .discovery import camera_error, find_camera_processes, stop_camera_processes

TITLES = {'head': '头部', 'left_wrist': '左腕', 'right_wrist': '右腕'}
MAX_EXPOSURE, MAX_GAIN = 100000, 1000  # generous bounds of the input boxes; the driver answers if it refuses a value
MANUAL_EXPOSURE, MANUAL_GAIN = 50, 16  # what the boxes show while auto exposure is on (the wrist template values)
IMAGING_DEBOUNCE_MS = 300


class CameraPreviews:
    """Owned drivers have private names, config files and process records."""
    def __init__(self, ws_root, raw):
        from .discovery import ImageProbe
        self.ws_root, self.raw = ws_root, raw
        self.probe = ImageProbe('camera_setup_' + uuid.uuid4().hex[:8])
        self.directory = tempfile.TemporaryDirectory(prefix='camera_setup_')
        self.prefix = 'setup_' + uuid.uuid4().hex[:8] + '_'
        self.supervisor = OwnedDrivers(os.path.join(ws_root, 'logs', 'setup', self.prefix))
        self.entries, self.imaging_errors = {}, {}

    def configure(self, device, rotation=0):
        """Start the preview driver of one camera. The driver always publishes upright (rotation 0): the window
        rotates what it shows, so choosing an angle never restarts a driver (a restarted Gemini 305 can hang)."""
        rotation = 0
        old = self.entries.get(device.serial)
        if (old and old['rotation'] == rotation and self.supervisor.is_running(old['unit'])
                and (self.frame(device.serial) or not self.supervisor.status(old['unit']).last_error)):
            return
        if old:
            self.supervisor.stop(old['unit'])
        index = list(self.entries).index(device.serial) if old else len(self.entries)
        role = 'left_wrist' if '305' in device.model else 'head'
        cfg = copy.deepcopy(self.raw)
        defaults = section_defaults('cameras')
        cfg['cameras'] = {**defaults, **copy.deepcopy(self.raw.get('cameras') or {})}
        cfg['cameras']['enable'] = True
        for name in ROLES:
            # Keep the imaging fields (white balance, exposure, gain): the loader refuses an incomplete entry.
            entry = {**(defaults.get(name) or {}), **(cfg['cameras'].get(name) or {})}
            entry.update({'enable': name == role, 'serial': device.serial if name == role else '',
                          'usb_port': device.usb_port if name == role else '', 'rotation': rotation})
            cfg['cameras'][name] = entry
        path = Path(self.directory.name) / f'camera_{index}.yaml'
        path.write_text(yaml.safe_dump(cfg, sort_keys=False))
        name = f'camera_preview_{index}'
        prefix = self.prefix + str(index) + '_' + uuid.uuid4().hex[:6] + '_'
        topic = f'/{prefix}camera_{role}/color/image_raw/compressed'
        self.probe.watch_image(topic)
        self.supervisor.configure(name,
            ('ros2', 'launch', 'robot_startup', 'cameras.launch.py', f'config:={path}', f'name_prefix:={prefix}'))
        self.entries[device.serial] = {'unit': name, 'topic': topic, 'rotation': rotation,
                                      'stream': 'color' if role == 'head' else 'right_color',
                                      'since': time.monotonic(), 'generation': uuid.uuid4().hex}
        # Return once this driver has initialized the device: the drivers share one lock, so a camera started
        # while another is initializing would give up.
        self.supervisor.start(name, ready_log=DRIVER_READY_LOG, ready_timeout_s=DRIVER_READY_TIMEOUT_S)

    def frame(self, serial):
        entry = self.entries.get(serial)
        frame = self.probe.latest_image(entry['topic']) if entry else None
        if frame and frame[0] > entry['since'] and time.monotonic() - frame[0] < 2:
            return frame
        return None

    def send_imaging(self, device, exposure, gain):
        """Set the exposure of one preview driver; returns the errors (an empty list when every call was accepted).
        Auto exposure only switches auto on; a fixed exposure switches it off first, then sets exposure and gain."""
        entry = self.entries.get(device.serial)
        if not entry:
            return ['预览驱动还没有启动']
        base = f"/{entry['topic'].split('/')[1]}/set_{entry['stream']}_"
        calls = [('auto_exposure', 'bool', True)] if exposure == -1 else [
            ('auto_exposure', 'bool', False), ('exposure', 'int', exposure), ('gain', 'int', gain)]
        for name, kind, value in calls:
            ok, message = self.probe.set_value(base + name, kind, value)
            if not ok:
                return [f'{base + name}：{message}']
        return []

    def set_imaging(self, device, exposure, gain):
        """Apply the exposure in the background: the window must not wait for the driver."""
        def work():
            errors = self.send_imaging(device, exposure, gain)
            self.imaging_errors[device.serial] = '；'.join(errors)
        threading.Thread(target=work, daemon=True, name='camera_imaging').start()

    def fps(self, serial):
        entry = self.entries.get(serial)
        return self.probe.rate(entry['topic']) if entry else 0.0

    def status(self, serial):
        entry = self.entries.get(serial)
        if not entry:
            return {'fresh': False, 'running': False, 'error': '', 'generation': '', 'fps': 0.0}
        status = self.supervisor.status(entry['unit'])
        fresh = bool(self.frame(serial))
        error = '' if fresh else camera_error(status.last_error or '')
        if not status.running and not error:
            error = '预览进程已退出，请检查设备占用和连接'
        return {'fresh': fresh, 'running': status.running, 'error': error,
                'generation': entry['generation'], 'fps': self.fps(serial),
                'imaging_error': getattr(self, 'imaging_errors', {}).get(serial, '')}

    def close(self):
        if getattr(self, 'closed', False):
            return
        self.closed = True
        errors = []
        try:
            for name in self.supervisor.names():
                try:
                    self.supervisor.stop(name)
                except (RuntimeError, OSError) as exc:
                    errors.append(str(exc))
        finally:
            self.probe.close()
            self.directory.cleanup()
        if errors:
            raise RuntimeError('；'.join(errors))


class CameraSetup(SetupSession):
    kind = 'cameras'
    apply_on_open = True

    def __init__(self, config_path, ws_root, found, preview_factory=CameraPreviews):
        super().__init__(config_path, ws_root)
        self.devices = found
        self.by_serial = {device.serial: device for device in found}
        if not found:
            raise ValueError('没有发现相机；检查连接与容器 USB 映射')
        self.preview_factory, self.previews, self.errors = preview_factory, None, {}
        self.mapping = {d.serial: {'role': '', 'rotation': 0} for d in found}
        defaults, cameras = section_defaults('cameras'), (self.raw.get('cameras') or {})
        self.imaging_lock = threading.Lock()  # a window change must not wait for a slow update
        self.imaging = {}
        for device in found:  # what the preview driver starts with: the entry of the role its model is previewed as
            entry = {**defaults['left_wrist' if '305' in device.model else 'head'],
                     **(cameras.get('left_wrist' if '305' in device.model else 'head') or {})}
            self.imaging[device.serial] = {'exposure': entry['exposure'], 'gain': entry['gain']}
        suggestions = discovery.suggest_roles(found, config_file.camera_serials(config_path))
        for role, device in suggestions.items():
            if device:
                self.mapping[device.serial] = {'role': role,
                    'rotation': ((self.raw.get('cameras') or {}).get(role) or {}).get('rotation', 0)}

    def state(self):
        with self.lock:
            return {**self.base_state(), 'devices': [
                {**asdict(device), **self.mapping[device.serial], **self.imaging[device.serial], 'index': index,
                 **(self.previews.status(device.serial) if self.previews else {'fresh': False, 'generation': '', 'fps': 0.0}),
                 'error': self.errors.get(device.serial, '') or
                          (self.previews.status(device.serial)['error'] if self.previews else '')}
                for index, device in enumerate(self.devices)]}

    def own_camera_names(self):
        return {entry['topic'].split('/')[1] for entry in self.previews.entries.values()} \
            if self.previews is not None and hasattr(self.previews, 'entries') else set()

    def conflicts(self):
        return find_camera_processes(self.devices, self.own_camera_names())

    def update(self, request):
        with self.lock:
            self.active()
            mapping = request.get('mapping')
            if not isinstance(mapping, dict) or set(mapping) != set(self.mapping):
                raise ValueError('相机列表已变化，请重新进入配置')
            roles = []
            for serial, values in mapping.items():
                role, rotation = values.get('role'), values.get('rotation')
                if role not in (*ROLES, '') or type(rotation) is not int or rotation not in ROTATIONS:
                    raise ValueError('相机位置或旋转角度无效')
                if role:
                    if role in roles:
                        raise ValueError('一个位置只能分配一台相机')
                    if (role == 'head') == ('305' in self.by_serial[serial].model):
                        raise ValueError('头部使用 Gemini 330 系列，腕部使用 Gemini 305')
                    roles.append(role)
            conflicts = self.conflicts()
            if conflicts:
                approved = request.get('stop_old_processes')
                if not isinstance(approved, list):
                    raise ValueError('发现旧相机采集进程，请先确认停止后再重启配置')
                stop_camera_processes(approved, self.devices, self.own_camera_names())
            self.mapping = copy.deepcopy(mapping)
            self.revision += 1
            require_idle(self.ws_root, ['camera_' + role for role in ROLES])
            if self.previews is None:
                self.previews = self.preview_factory(self.ws_root, self.raw)
            for device in self.devices:
                self.errors.pop(device.serial, None)
                try:
                    self.previews.configure(device, self.mapping[device.serial]['rotation'])
                except (RuntimeError, OSError) as exc:
                    self.errors[device.serial] = str(exc)
            return self.state()

    def set_imaging(self, request):
        """Set the exposure and gain of one camera at once. What the operator confirmed (position, direction) does
        not change, so this neither starts a driver nor invalidates the confirmation."""
        serial, exposure, gain = request.get('serial'), request.get('exposure'), request.get('gain')
        if serial not in self.by_serial:
            raise ValueError('未知相机')
        def valid(value, floor):
            return type(value) is int and (value == -1 or value >= floor)
        if not (valid(exposure, 1) and valid(gain, 0) and (exposure == -1) == (gain == -1)):
            raise ValueError('曝光和增益必须同为 -1（自动曝光）或同为固定整数（曝光 ≥ 1，增益 ≥ 0）')
        self.active()
        with self.imaging_lock:
            self.imaging[serial] = {'exposure': exposure, 'gain': gain}
        if self.previews:
            self.previews.set_imaging(self.by_serial[serial], exposure, gain)

    def image(self, index):
        if not 0 <= index < len(self.devices):
            raise ValueError('未知相机')
        return self.previews.frame(self.devices[index].serial) if self.previews else None

    def save(self, request):
        with self.lock:
            self.active()
            if request.get('revision') != self.revision:
                raise ValueError('配置已变化，请重新核对画面')
            selected = {values['role']: serial for serial, values in self.mapping.items() if values['role']}
            if not selected:
                raise ValueError('请至少选择一台相机，或在终端取消启用相机')
            if set(request.get('confirmed', [])) != set(selected):
                raise ValueError('请逐一确认所选画面的位置和方向；左右以机器人自身为准')
            for role, serial in selected.items():
                if self.previews is None or self.errors.get(serial) or not self.previews.frame(serial):
                    raise ValueError(f'{role} 未收到最新画面，不能确认保存')
            assignments = {role: {'enable': False} for role in ROLES}
            for role, serial in selected.items():
                assignments[role] = {'enable': True, 'serial': serial,
                    'usb_port': self.by_serial[serial].usb_port, 'rotation': self.mapping[serial]['rotation'],
                    **self.imaging[serial]}
            validate_camera_assignments(assignments)
            config_file.save_setup(self.config_path, self.kind, {'cameras': {'enable': True, **assignments}}, self.original)
            self.saved = True
            self.done.set()
            return self.state()

    def close(self):
        super().close()
        if self.previews:
            self.previews.close()
    def create_panel(self, window):
        return CameraPanel(window)


class CameraPanel:
    """The cards of the camera window: one per camera with its preview, position and rotation."""
    title, size, swap_label, apply_label = 'W2 相机配置', (1160, 730), '交换左右腕', ''  # every change applies itself
    hint = '根据实时画面选择头部、左腕、右腕。依次遮挡左右腕镜头核对位置，并确认画面方向；曝光和增益改动立即生效。'

    def __init__(self, window):
        self.window = window

    def build(self, grid, options):
        from PyQt5 import QtCore, QtWidgets as Q
        window = self.window
        for device in window.state['devices']:
            box = Q.QGroupBox(device['model'])
            body = Q.QVBoxLayout(box)
            identity = Q.QLabel('序列号：' + device['serial'] + '\n连接：' + (device['connection'] or '未知')
                                + '  端口：' + (device['usb_port'] or '未知'))
            identity.setWordWrap(True)
            body.addWidget(identity)
            image = Q.QLabel('等待相机画面')
            image.setAlignment(QtCore.Qt.AlignCenter)
            image.setMinimumSize(280, 230)
            image.setStyleSheet('background:#111;color:#bbb;border-radius:5px')
            body.addWidget(image, 1)
            feedback = Q.QLabel('等待最新画面')
            feedback.setWordWrap(True)
            body.addWidget(feedback)
            form = Q.QFormLayout()
            role = window.combo([('', '不使用'), *TITLES.items()], device['role'])
            rotation = window.combo([(value, f'{value}°') for value in ROTATIONS], device['rotation'])
            form.addRow('位置', role)
            form.addRow('画面旋转', rotation)
            for combo in (role, rotation):  # the preview follows at once; no driver is restarted
                combo.currentIndexChanged.connect(window.apply)
            imaging = self.imaging_controls(box, form, device)
            body.addLayout(form)
            check = Q.QCheckBox('我已核对该画面的位置和方向')
            body.addWidget(check)
            window.cards[device['serial']] = {'box': box, 'image': image, 'feedback': feedback,
                                              'role': role, 'rotation': rotation, 'check': check, 'stamp': None,
                                              **imaging}
            grid.addWidget(box, 0, device['index'])

    def imaging_controls(self, parent, form, device):
        """Auto exposure, exposure and gain of one card. A change is sent to the driver 300 ms after the last edit."""
        from PyQt5 import QtCore, QtWidgets as Q
        auto, exposure, gain = Q.QCheckBox('自动曝光'), Q.QSpinBox(), Q.QSpinBox()
        exposure.setRange(1, MAX_EXPOSURE)
        gain.setRange(0, MAX_GAIN)
        auto.setChecked(device['exposure'] == -1)
        exposure.setValue(device['exposure'] if device['exposure'] != -1 else MANUAL_EXPOSURE)
        gain.setValue(device['gain'] if device['gain'] != -1 else MANUAL_GAIN)
        exposure.setEnabled(not auto.isChecked())
        gain.setEnabled(not auto.isChecked())
        form.addRow('曝光', auto)
        form.addRow('曝光值', exposure)
        form.addRow('增益', gain)
        timer = QtCore.QTimer(parent)
        timer.setSingleShot(True)
        timer.setInterval(IMAGING_DEBOUNCE_MS)
        serial = device['serial']
        timer.timeout.connect(lambda: self.send_imaging(serial))

        def changed(*_):
            exposure.setEnabled(not auto.isChecked())
            gain.setEnabled(not auto.isChecked())
            timer.start()
        for control, signal in ((auto, auto.toggled), (exposure, exposure.valueChanged), (gain, gain.valueChanged)):
            signal.connect(changed)
        return {'auto': auto, 'exposure': exposure, 'gain': gain, 'timer': timer}

    def send_imaging(self, serial):
        card = self.window.cards[serial]
        auto = card['auto'].isChecked()
        try:
            self.window.session.set_imaging({'serial': serial, 'exposure': -1 if auto else card['exposure'].value(),
                                             'gain': -1 if auto else card['gain'].value()})
        except (ValueError, RuntimeError, OSError) as exc:
            self.window.show_message(str(exc), True)

    def mapping(self):
        return {serial: {'role': card['role'].currentData(), 'rotation': card['rotation'].currentData()}
                for serial, card in self.window.cards.items()}

    def apply_payload(self):
        """The request for session.update, or None when the operator declines to stop the old capture processes."""
        from PyQt5 import QtWidgets as Q
        window = self.window
        payload = {'mapping': self.mapping()}
        conflicts = window.session.conflicts()
        if conflicts:
            details = '\n'.join(f"PID {item['pid']}  {item['camera']}  {item['serial']}" for item in conflicts)
            answer = Q.QMessageBox.question(window, '停止旧相机采集并重启配置',
                '检测到以下旧相机进程：\n' + details +
                '\n\n是否停止这些相机采集进程，并自动重启当前相机预览？只处理上述相机进程，不停止运控。',
                Q.QMessageBox.Yes | Q.QMessageBox.No, Q.QMessageBox.No)
            if answer != Q.QMessageBox.Yes:
                window.show_message('未停止旧相机进程。请释放设备后，再改一次位置或旋转即可重试。', True)
                return None
            payload['stop_old_processes'] = conflicts
        return payload

    def swap(self):
        for card in self.window.cards.values():
            value = card['role'].currentData()
            value = {'left_wrist': 'right_wrist', 'right_wrist': 'left_wrist'}.get(value, value)
            card['role'].blockSignals(True)  # both roles change before one apply: halfway they would be a duplicate
            card['role'].setCurrentIndex(card['role'].findData(value))
            card['role'].blockSignals(False)
            card['check'].setChecked(False)
        self.window.dirty = True
        return True

    def confirmed(self):
        return [card['role'].currentData() for card in self.window.cards.values() if card['check'].isChecked()]

    def refresh(self, busy):
        from PyQt5 import QtCore, QtGui
        window = self.window
        for device in window.state['devices']:
            card = window.cards[device['serial']]
            card['role'].setEnabled(not busy)
            card['rotation'].setEnabled(not busy)
            card['auto'].setEnabled(not busy)
            card['exposure'].setEnabled(not busy and not card['auto'].isChecked())
            card['gain'].setEnabled(not busy and not card['auto'].isChecked())
            card['feedback'].setText((device.get('error') or (
                f"画面正常，{device.get('fps', 0.0):.1f} fps" if device['fresh'] else '等待最新画面；检查设备连接或占用'))
                + (f"\n曝光设置失败：{device['imaging_error']}" if device.get('imaging_error') else ''))
            fresh = device['fresh'] and not device.get('error')
            card['check'].setEnabled(not busy and not window.dirty and bool(device['role']) and fresh)
            if not fresh:
                card['check'].setChecked(False)
                card['image'].clear()
                card['image'].setText('等待最新画面')
                card['stamp'] = None
            elif not busy:
                frame = window.session.image(device['index'])
                if frame:
                    image = QtGui.QImage.fromData(frame[2])
                    if image.isNull():
                        card['check'].setEnabled(False)
                        card['check'].setChecked(False)
                        card['feedback'].setText('画面解码失败，请检查驱动输出')
                    else:
                        angle = card['rotation'].currentData() or 0
                        if angle:  # clockwise, like the driver's rotation
                            image = image.transformed(QtGui.QTransform().rotate(angle))
                        card['image'].setPixmap(QtGui.QPixmap.fromImage(image).scaled(
                            card['image'].size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))


def step_cameras(config_path: str, ws_root: str, io: Io,
                 scanner: Callable[[], List[discovery.Device]] = discovery.scan, runner=None) -> bool:
    _, raw = config_file.load_rt(config_path)
    if (raw.get("cameras") or {}).get("enable") is False:
        answer = io.ask("相机当前关闭，是否现在配置并启用？y/n", "y")
        if answer.lower() not in ("y", "yes", "是"):
            io.info("按选择保持相机关闭")
            return True
    io.info("扫描 Orbbec 相机 ...")
    try:
        devs = scanner()
    except RuntimeError as exc:
        io.error(str(exc))
        return False
    if not devs:
        io.error("没有发现相机，初始化已停止。请检查 USB/GMSL 连接、设备权限及占用情况；容器需映射相机 USB 设备。")
        return False
    for i, d in enumerate(devs, 1):
        io.info(f"  [{i}] {d.model:<24} 序列号 {d.serial:<16} 连接 {d.connection or '-':<8} 端口 {d.usb_port or '-'}")
    for serial, node in discovery.inaccessible_usb_devices():
        io.warn(f'检测到相机 {serial}，但当前环境无法访问 {node}；检查权限或容器 USB 映射，热插拔后设备编号可能变化')
    if len(devs) < len(ROLES):
        io.warn(f"只发现 {len(devs)} 台相机（正常为 {len(ROLES)} 台）")
    if io.assume_yes:
        cams = raw.get('cameras') or {}
        saved = {value.get('serial') for role, value in cams.items()
                 if role in ROLES and isinstance(value, dict) and value.get('enable', True)}
        if confirmed(raw, 'cameras') and saved.issubset({d.serial for d in devs}):
            io.info('复用已通过画面确认的相机配置')
            return True
        io.error('首次相机配置或设备映射变化需要在窗口中确认；未保存配置，请去掉 --yes 重试')
        return False
    session = CameraSetup(config_path, ws_root, devs)
    try:
        ok = (runner or run_wizard)(session, io)
        if ok:
            io.info(f'相机画面与位置已确认，已写入 {config_path}')
        else:
            io.error('相机配置已取消，未保存本次修改')
        return ok
    finally:
        session.close()


def main(argv=None):
    return main_for(lambda *args: step_cameras(*args), argv,
                    failure_hint='相机配置失败，已退出；重试本步：./scripts/setup.sh cam；从本步继续到末尾：./scripts/setup.sh cam --from')


if __name__ == '__main__':
    sys.exit(main())
