import copy
"""Fixtures of the hand setup tests."""
import time

import pytest
import yaml

from end_effector_setup import session as ee_setup
from robot_config.testing import full_config

LEFT_BY_ID = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AAAA-if00-port0'


RIGHT_BY_ID = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BBBB-if00-port0'


PORTS = [{'path': LEFT_BY_ID, 'device': '/dev/ttyUSB0'},
         {'path': RIGHT_BY_ID, 'device': '/dev/ttyUSB2'}]


CONFIG = "# preserve this comment\n" + yaml.safe_dump(full_config(
    robot={"end_effector": {"type": "linkerhand_l6", "interface": {
        "bus": "usb_rs485", "left": {"port": LEFT_BY_ID}, "right": {"port": RIGHT_BY_ID}}}},
    cameras={"head": {"serial": "HEAD"}, "left_wrist": {"serial": "LEFT"}, "right_wrist": {"serial": "RIGHT"}},
    xr={"device_id": "PICO-BOUND"}, control={"limits": {"max_joint_speed": 17}}), sort_keys=False)


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

    session = ee_setup.EeSetup(str(config), str(config.parent.parent), lambda: PORTS, worker)
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
