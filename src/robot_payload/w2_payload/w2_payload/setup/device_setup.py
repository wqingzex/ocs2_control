"""Draft camera/hand setup sessions; only save() touches robot.local.yaml."""
from __future__ import annotations

import copy
from dataclasses import asdict
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import uuid

import yaml

from ..config import edit as config_edit
from ..devices import cameras as devices
from ..config import DEFAULTS, HANDS, HAND_DRIVES, ROTATIONS, normalize_hand_drive, validate
from ..devices.hand_probe import serial_devices
from .state import confirmed
from ..runtime import OwnedDrivers, require_idle
from ..devices.cameras import find_camera_processes, stop_camera_processes, camera_error

ROLES = devices.ROLES


class CameraPreviews:
    """Owned drivers have private names, config files and process records."""
    def __init__(self, ws_root, raw):
        from ..devices.cameras import ImageProbe
        self.ws_root, self.raw = ws_root, raw
        self.probe = ImageProbe('w2_camera_setup_' + uuid.uuid4().hex[:8])
        self.directory = tempfile.TemporaryDirectory(prefix='camera_setup_')
        self.prefix = 'setup_' + uuid.uuid4().hex[:8] + '_'
        self.supervisor = OwnedDrivers(os.path.join(ws_root, 'logs', 'setup', self.prefix))
        self.entries = {}

    def configure(self, device, rotation):
        old = self.entries.get(device.serial)
        if (old and old['rotation'] == rotation and self.supervisor.is_running(old['unit'])
                and (self.frame(device.serial) or not self.supervisor.status(old['unit']).last_error)):
            return
        if old:
            self.supervisor.stop(old['unit'])
        index = list(self.entries).index(device.serial) if old else len(self.entries)
        role = 'left_wrist' if '305' in device.model else 'head'
        cfg = copy.deepcopy(self.raw)
        cfg['cameras'] = {**copy.deepcopy(DEFAULTS['cameras']), **copy.deepcopy(self.raw.get('cameras') or {})}
        cfg['cameras']['enable'] = True
        for name in ROLES:
            cfg['cameras'][name] = {'enable': name == role, 'serial': device.serial if name == role else '',
                                    'usb_port': device.usb_port if name == role else '', 'rotation': rotation}
        path = Path(self.directory.name) / f'camera_{index}.yaml'
        path.write_text(yaml.safe_dump(cfg, sort_keys=False))
        name = f'camera_preview_{index}'
        prefix = self.prefix + str(index) + '_' + uuid.uuid4().hex[:6] + '_'
        topic = f'/{prefix}camera_{role}/color/image_raw/compressed'
        self.probe.watch_image(topic)
        self.supervisor.configure(name,
            ('ros2', 'launch', 'w2_payload', 'cameras.launch.py', f'config:={path}', f'name_prefix:={prefix}'))
        self.entries[device.serial] = {'unit': name, 'topic': topic, 'rotation': rotation,
                                      'since': time.monotonic(), 'generation': uuid.uuid4().hex}
        self.supervisor.start(name)

    def frame(self, serial):
        entry = self.entries.get(serial)
        frame = self.probe.latest_image(entry['topic']) if entry else None
        if frame and frame[0] > entry['since'] and time.monotonic() - frame[0] < 2:
            return frame
        return None

    def status(self, serial):
        entry = self.entries.get(serial)
        if not entry:
            return {'fresh': False, 'running': False, 'error': '', 'generation': ''}
        status = self.supervisor.status(entry['unit'])
        fresh = bool(self.frame(serial))
        error = '' if fresh else camera_error(status.last_error or '')
        if not status.running and not error:
            error = '预览进程已退出，请检查设备占用和连接'
        return {'fresh': fresh, 'running': status.running, 'error': error,
                'generation': entry['generation']}

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


class SetupSession:
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


class CameraSetup(SetupSession):
    kind = 'cameras'

    def __init__(self, config_path, ws_root, found, preview_factory=CameraPreviews):
        super().__init__(config_path, ws_root)
        self.devices = found
        self.by_serial = {device.serial: device for device in found}
        if not found:
            raise ValueError('没有发现相机；检查连接与容器 USB 映射')
        self.preview_factory, self.previews, self.errors = preview_factory, None, {}
        self.mapping = {d.serial: {'role': '', 'rotation': 0} for d in found}
        suggestions = devices.suggest_roles(found, config_edit.camera_serials(config_path))
        for role, device in suggestions.items():
            if device:
                self.mapping[device.serial] = {'role': role,
                    'rotation': ((self.raw.get('cameras') or {}).get(role) or {}).get('rotation', 0)}

    def state(self):
        with self.lock:
            return {**self.base_state(), 'devices': [
                {**asdict(device), **self.mapping[device.serial], 'index': index,
                 **(self.previews.status(device.serial) if self.previews else {'fresh': False, 'generation': ''}),
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
                    'usb_port': self.by_serial[serial].usb_port, 'rotation': self.mapping[serial]['rotation']}
            config_edit.validate_camera_assignments(assignments)
            config_edit.save_setup(self.config_path, self.kind, {'cameras': {'enable': True, **assignments}}, self.original)
            self.saved = True
            self.done.set()
            return self.state()

    def close(self):
        super().close()
        if self.previews:
            self.previews.close()


def hand_request(request):
    """Bound native SDK work and reclaim it even if the setup window exits."""
    try:
        result = subprocess.run([sys.executable, '-m', 'w2_payload.devices.hand_probe'],
            input=json.dumps(request), text=True, capture_output=True, timeout=8, check=False)
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError('手部操作超时，已终止本次设备连接；请检查设备状态后重试') from exc
    lines = [line.split('=', 1)[1] for line in result.stdout.splitlines() if line.startswith('W2_HAND_RESULT=')]
    if not lines:
        raise RuntimeError('手部操作未返回结果：' + (result.stderr.strip()[-300:] or '设备进程异常退出'))
    data = json.loads(lines[-1])
    if not data.get('ok'):
        raise RuntimeError(data.get('message', '手部操作失败'))
    return data


class HandSetup(SetupSession):
    kind = 'hand'

    def __init__(self, config_path, ws_root, ports=serial_devices, worker=hand_request):
        super().__init__(config_path, ws_root)
        self.ports, self.worker = ports(), worker
        robot, hardware = self.raw.get('robot') or {}, self.raw.get('hardware') or {}
        self.draft = {'hand': robot.get('hand', 'linkerhand_l6'),
                      'drive': normalize_hand_drive(robot.get('hand_drive', 'usb')),
                      'device_ip': hardware.get('device_ip', '192.168.1.190')}
        for index, side in enumerate(('left', 'right')):
            preferred = hardware.get('usb_' + side + '_port', '')
            matching = next((p['path'] for p in self.ports if os.path.realpath(p['path']) == os.path.realpath(preferred)), '')
            self.draft[side + '_port'] = matching or (self.ports[index]['path'] if index < len(self.ports) else '')
            self.draft[side + '_channel'] = hardware.get(side + '_ee_channel', 1)
        self.results = {}

    def state(self):
        with self.lock:
            return {**self.base_state(), 'ports': self.ports, 'draft': self.draft, 'results': self.results}

    def update(self, request):
        import ipaddress
        with self.lock:
            self.active()
            draft = request.get('draft')
            if not isinstance(draft, dict) or set(draft) != set(self.draft):
                raise ValueError('手部配置字段无效')
            if draft['hand'] not in HANDS or draft['drive'] not in HAND_DRIVES:
                raise ValueError('请选择正确的末端与接线方式')
            if draft['hand'] == 'xhand1' and draft['drive'] != 'usb':
                raise ValueError('XHand1 仅支持 USB 接线')
            if draft['hand'] != 'none':
                if draft['drive'] == 'usb':
                    available = {item['path'] for item in self.ports}
                    if any(draft[side + '_port'] not in available for side in ('left', 'right')):
                        raise ValueError('请选择可用串口；容器需映射串口设备')
                    if draft['hand'] == 'xhand1' and os.path.realpath(draft['left_port']) == os.path.realpath(draft['right_port']):
                        raise ValueError('两只 XHand1 不能使用同一串口')
                else:
                    ipaddress.IPv4Address(draft['device_ip'])
                    if any(type(draft[side + '_channel']) is not int or draft[side + '_channel'] not in (1, 2, 3)
                           for side in ('left', 'right')):
                        raise ValueError('485 通道必须是 1、2 或 3')
            self.draft = copy.deepcopy(draft)
            self.results = {}
            self.revision += 1
            return self.state()

    def action(self, request):
        with self.lock:
            self.active()
            if request.get('revision') != self.revision:
                raise ValueError('配置已变化，请先应用配置')
            side, operation = request.get('side'), request.get('operation')
            if side not in ('left', 'right') or operation not in ('probe', 'test') or self.draft['hand'] == 'none':
                raise ValueError('无效的手部操作')
            if operation == 'test' and request.get('allow_motion') is not True:
                raise ValueError('开合测试需要用户主动确认')
            if self.draft['drive'] == '485' and request.get('allow_controller_connection') is not True:
                raise ValueError('连接机械臂末端 485 需要先确认原运控已停止')
            require_idle(self.ws_root, ['control'])
            self.results.pop(side, None)
            try:
                self.results[side] = self.worker({**self.draft, 'side': side, 'operation': operation,
                    'port': self.draft[side + '_port'], 'channel': self.draft[side + '_channel'], 'ws_root': self.ws_root})
            except (RuntimeError, OSError) as exc:
                self.results[side] = {'ok': False, 'message': str(exc)}
                raise
            return self.state()

    def save(self, request):
        with self.lock:
            self.active()
            if request.get('revision') != self.revision:
                raise ValueError('配置已变化，请重新确认左右手')
            if self.draft['hand'] != 'none':
                if set(request.get('confirmed', [])) != {'left', 'right'}:
                    raise ValueError('请分别核对实物并确认左手、右手')
                if any(not (self.results.get(side, {}).get('ok') and self.results[side].get('tested')) for side in ('left', 'right')):
                    raise ValueError('左右手均需先完成开合测试后才能保存')
                # A cable can be removed after the user first inspects the hand.
                # Recheck at the explicit Save action; this never requests motion.
                for side in ('left', 'right'):
                    self.action({'revision': self.revision, 'side': side, 'operation': 'probe',
                                 'allow_controller_connection': self.draft['drive'] == '485'})
            hardware = ({'usb_' + side + '_port': self.draft[side + '_port'] for side in ('left', 'right')}
                        if self.draft['drive'] == 'usb' else {'device_ip': self.draft['device_ip'],
                            **{side + '_ee_channel': self.draft[side + '_channel'] for side in ('left', 'right')}})
            changes = {'robot': {'hand': self.draft['hand'], 'hand_drive': self.draft['drive']}}
            if self.draft['hand'] != 'none':
                changes['hardware'] = hardware
            # Validate the staged robot options before committing any fields.
            candidate = copy.deepcopy(self.raw)
            candidate.setdefault('robot', {}).update(changes['robot'])
            candidate.setdefault('cameras', {})['enable'] = False
            validate(candidate)
            config_edit.save_setup(self.config_path, self.kind, changes, self.original)
            self.saved = True
            self.done.set()
            return self.state()
