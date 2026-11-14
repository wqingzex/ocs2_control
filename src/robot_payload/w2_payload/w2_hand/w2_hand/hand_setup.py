"""Hand setup (./scripts/setup.sh hand): the draft session, the bounded device probe and the window panel.

    python3 -m w2_hand.hand_setup --config C --ws-root W [--yes]"""
from __future__ import annotations

import copy
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Callable, List

from w2_payload import config_edit
from w2_payload.config import HANDS, HAND_DRIVES, normalize_hand_drive, validate
from w2_payload.config_edit import confirmed
from w2_payload.device_setup import Io, SetupSession, main_for, run_wizard
from robot_runtime.processes import require_idle

from .hand import is_serial_number_path, serial_devices


def mapping_fields(raw):
    """The hand, its wiring and the ports or channels that wiring uses."""
    robot, hw = raw.get("robot") or {}, raw.get("hardware") or {}
    value = {"hand": robot.get("hand", "linkerhand_l6"),
             "drive": normalize_hand_drive(robot.get("hand_drive", "usb"))}
    if value["hand"] != "none":
        keys = {"usb_left_port": "", "usb_right_port": ""} \
            if value["drive"] == "usb" else {"device_ip": "192.168.1.190", "left_ee_channel": 1, "right_ee_channel": 1}
        value.update({key: hw.get(key, default) for key, default in keys.items()})
    return value


config_edit.register_fingerprint('hand', mapping_fields)


def hand_request(request):
    """Bound native SDK work and reclaim it even if the setup window exits."""
    try:
        result = subprocess.run([sys.executable, '-m', 'w2_hand.hand'],
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
            matching = next((p['path'] for p in self.ports if p['path'] == preferred), '')
            if matching:
                self.draft[side + '_port'] = matching
            elif is_serial_number_path(preferred):
                self.draft[side + '_port'] = ''  # the adapter named in the config is not plugged in: do not guess another
            else:
                # not configured by serial number yet (empty, or an old /dev/ttyUSBn): offer the next adapter to choose
                self.draft[side + '_port'] = self.ports[index]['path'] if index < len(self.ports) else ''
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
                    if any(not is_serial_number_path(draft[side + '_port']) for side in ('left', 'right')):
                        raise ValueError('灵巧手只能用 /dev/serial/by-id 下按适配器序列号识别的串口，不能用 /dev/ttyUSBn 等编号')
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

    def create_panel(self, window):
        return HandPanel(window)


class HandPanel:
    """The cards of the hand window: the options row and one card per hand."""
    title, size, swap_label, apply_label = 'W2 左右手配置', (860, 730), '交换左右手串口', '应用配置'
    hint = '分别点击“开合测试”：手会慢速半握再张开，据此核对是机器人的左手还是右手。'

    def __init__(self, window):
        self.window = window

    def build(self, grid, options):
        from PyQt5 import QtWidgets as Q
        window = self.window
        draft = window.state['draft']
        self.hand = window.combo([('none', '不使用手'), ('linkerhand_l6', 'LinkerHand L6'), ('xhand1', 'XHand1')], draft['hand'])
        self.drive = window.combo([('usb', 'usb（485 转 USB 接电脑）'), ('485', '485（接机械臂接口）')], draft['drive'])
        self.device_ip = Q.QLineEdit(str(draft['device_ip']))
        self.device_ip.textEdited.connect(window.mark_dirty)
        self.ip_label = Q.QLabel('485 控制器 IP')
        for widget in (Q.QLabel('末端'), self.hand, Q.QLabel('接线'), self.drive, self.ip_label, self.device_ip):
            options.addWidget(widget)
        ports = [(p['path'], p['path'].rsplit('/', 1)[-1]) for p in window.state['ports']]  # label = vendor_model_serial
        if not ports:
            ports = [('', '未发现带序列号的串口（/dev/serial/by-id）；容器需映射设备')]
        for index, side in enumerate(('left', 'right')):
            title = '左手' if side == 'left' else '右手'
            box = Q.QGroupBox(title)
            body = Q.QVBoxLayout(box)
            form = Q.QFormLayout()
            port = window.combo(ports, draft[side + '_port'])
            port_label = Q.QLabel('USB 串口')
            form.addRow(port_label, port)
            channel = window.combo([(value, str(value)) for value in (1, 2, 3)], draft[side + '_channel'])
            channel_label = Q.QLabel('485 通道')
            arm = Q.QLabel('485：机器人' + ('左臂（A）' if side == 'left' else '右臂（B）'))
            body.addWidget(arm)
            form.addRow(channel_label, channel)
            body.addLayout(form)
            actions = Q.QHBoxLayout()
            test = Q.QPushButton('开合测试')
            test.clicked.connect(lambda _=False, s=side: self.hand_action(s, 'test'))
            actions.addWidget(test)
            body.addLayout(actions)
            feedback = Q.QLabel('尚未测试')
            feedback.setWordWrap(True)
            body.addWidget(feedback)
            body.addStretch(1)
            check = Q.QCheckBox('我已核对该设备是机器人的' + title)
            body.addWidget(check)
            window.cards[side] = {'box': box, 'port': port, 'port_label': port_label, 'channel': channel,
                                  'channel_label': channel_label, 'arm': arm, 'test': test,
                                  'feedback': feedback, 'check': check}
            grid.addWidget(box, 0, index)

    def draft(self):
        draft = {'hand': self.hand.currentData(), 'drive': self.drive.currentData(),
                 'device_ip': self.device_ip.text().strip()}
        for side, card in self.window.cards.items():
            draft[side + '_port'] = card['port'].currentData()
            draft[side + '_channel'] = card['channel'].currentData()
        return draft

    def apply_payload(self):
        return {'draft': self.draft()}

    def swap(self):
        if self.drive.currentData() != 'usb':
            self.window.show_message('485 左右臂接口固定，请核对并调整实物接线。', True)
            return False
        left, right = self.window.cards['left']['port'], self.window.cards['right']['port']
        first, second = left.currentData(), right.currentData()
        left.setCurrentIndex(left.findData(second))
        right.setCurrentIndex(right.findData(first))
        return True

    def confirmed(self):
        return [side for side, card in self.window.cards.items() if card['check'].isChecked()]

    def hand_action(self, side, operation):
        from PyQt5 import QtWidgets as Q
        window = self.window
        if window.dirty:
            window.show_message('请先应用配置，再做开合测试。', True)
            return
        payload = {'side': side, 'operation': operation, 'revision': window.state['revision']}
        if window.state['draft']['drive'] == '485':
            answer = Q.QMessageBox.question(window, '连接末端 485',
                '将连接机械臂控制器的末端 485。请确认原运控已经停止；只操作手部。是否继续？',
                Q.QMessageBox.Yes | Q.QMessageBox.No, Q.QMessageBox.No)
            if answer != Q.QMessageBox.Yes:
                return
            payload['allow_controller_connection'] = True
        if operation == 'test':
            answer = Q.QMessageBox.question(window, '开合测试',
                '所选的手将慢速半握再张开并回到原位。请确认手周围无遮挡，并观察是哪只手。是否执行？',
                Q.QMessageBox.Yes | Q.QMessageBox.No, Q.QMessageBox.No)
            if answer != Q.QMessageBox.Yes:
                return
            payload['allow_motion'] = True
        window.cards[side]['check'].setChecked(False)
        window.submit(window.session.action, payload, 'hand')

    def refresh(self, busy):
        window = self.window
        drive, enabled = self.drive.currentData(), self.hand.currentData() != 'none'
        self.ip_label.setVisible(drive == '485' and enabled)
        self.device_ip.setVisible(drive == '485' and enabled)
        self.drive.setEnabled(not busy and enabled)
        for side, card in window.cards.items():
            card['box'].setEnabled(not busy and enabled)
            card['port'].setVisible(drive == 'usb')
            card['port_label'].setVisible(drive == 'usb')
            for name in ('channel', 'channel_label', 'arm'):
                card[name].setVisible(drive == '485')
            result = window.state['results'].get(side, {})
            card['feedback'].setText(result.get('message', '尚未测试'))
            card['test'].setEnabled(not busy and not window.dirty and enabled)
            card['check'].setEnabled(not busy and not window.dirty and enabled and bool(result.get('ok')))
            if not result.get('ok'):
                card['check'].setChecked(False)


def serial_ports() -> List[str]:
    return [entry['path'] for entry in serial_devices()]


def step_hand(config_path: str, ws_root: str, io: Io, ports: Callable[[], List[str]] = serial_ports,
              runner=None) -> bool:
    _, raw = config_edit.load_rt(config_path)
    if io.assume_yes:
        if confirmed(raw, 'hand'):
            robot, hw = raw.get('robot') or {}, raw.get('hardware') or {}
            if robot.get('hand') != 'none' and normalize_hand_drive(robot.get('hand_drive', 'usb')) == 'usb':
                configured = [hw.get('usb_' + side + '_port', '') for side in ('left', 'right')]
                if not all(is_serial_number_path(path) for path in configured):
                    io.error('手部串口没有按适配器序列号配置（/dev/serial/by-id/…），请去掉 --yes 重新选择左右手')
                    return False
                if any(path not in set(ports()) for path in configured):
                    io.error('已确认的手部串口当前不可用，请检查连接或容器设备映射')
                    return False
            io.info('复用已确认左右实物的手部配置')
            return True
        io.error('首次手部配置或设备映射变化需要交互确认左右手；未保存配置，请去掉 --yes 重试')
        return False
    entries = lambda: [{'path': path, 'device': os.path.realpath(path)} for path in ports()]
    session = HandSetup(config_path, ws_root or str(Path(config_path).resolve().parent.parent), ports=entries)
    try:
        ok = (runner or run_wizard)(session, io)
        if ok:
            io.info(f'左右手配置已确认，已写入 {config_path}')
        else:
            io.warn('手部配置已取消，未保存本次修改')
        return ok
    finally:
        session.close()


def main(argv=None):
    return main_for(lambda *args: step_hand(*args), argv)


if __name__ == '__main__':
    sys.exit(main())
