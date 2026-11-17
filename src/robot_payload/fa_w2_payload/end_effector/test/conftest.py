import copy
"""Fixtures of the hand setup tests."""
import time

import pytest

from end_effector import hand_setup

LEFT_BY_ID = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AAAA-if00-port0'


RIGHT_BY_ID = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BBBB-if00-port0'


PORTS = [{'path': LEFT_BY_ID, 'device': '/dev/ttyUSB0'},
         {'path': RIGHT_BY_ID, 'device': '/dev/ttyUSB2'}]


CONFIG = '''# preserve this comment
robot: {end_effector: {type: linkerhand_l6, interface: {bus: usb_rs485, left: {port: /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AAAA-if00-port0}, right: {port: /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BBBB-if00-port0}}}}
cameras:
  head: {serial: HEAD}
  left_wrist: {serial: LEFT}
  right_wrist: {serial: RIGHT}
xr: {device_id: PICO-BOUND}
control: {limits: {max_joint_speed: 17}}
'''


@pytest.fixture
def config(tmp_path):
    path = tmp_path / 'config/robot.local.yaml'
    path.parent.mkdir()
    path.write_text(CONFIG)
    return path


@pytest.fixture
def hand(config):
    calls = []

    def worker(request):
        calls.append(request)
        return {'ok': True, 'tested': request['operation'] == 'test', 'positions': [0] * 6, 'message': '收到设备反馈'}

    session = hand_setup.HandSetup(str(config), str(config.parent.parent), lambda: PORTS, worker)
    session.calls = calls
    yield session
    session.close()


def prepare_hand(hand):
    hand.update({'draft': copy.deepcopy(hand.draft)})
    for side in ('left', 'right'):
        hand.action({'revision': hand.revision, 'side': side, 'operation': 'test', 'allow_motion': True})


@pytest.fixture
def qt_app(monkeypatch):
    monkeypatch.setenv('QT_QPA_PLATFORM', 'offscreen')
    from PyQt5.QtWidgets import QApplication
    app = QApplication.instance() or QApplication(['w2-setup-test'])
    app.setQuitOnLastWindowClosed(False)
    return app


def wait_qt(app, window):
    deadline = time.monotonic() + 3
    while window.future and time.monotonic() < deadline:
        app.processEvents()
        window.tick()
        time.sleep(0.01)
    app.processEvents()
    assert window.future is None
