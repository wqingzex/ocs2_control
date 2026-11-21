import copy
"""Fixtures of the camera setup tests."""
import time

import pytest

from camera_setup import session as camera_setup
from camera_setup.discovery import Device

DEVICES = [Device('Orbbec Gemini 335Lg', 'HEAD', 'USB3.0', '2-2'),
           Device('Orbbec Gemini 305', 'LEFT', 'USB3.0', '2-4'),
           Device('Orbbec Gemini 305', 'RIGHT', 'USB3.0', '2-7')]


CONFIG = '''# preserve this comment
robot: {end_effector: {type: linkerhand_l6, interface: {bus: usb_rs485, left: {port: /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AAAA-if00-port0}, right: {port: /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BBBB-if00-port0}}}}
cameras:
  head: {serial: HEAD}
  left_wrist: {serial: LEFT}
  right_wrist: {serial: RIGHT}
xr: {device_id: PICO-BOUND}
control: {limits: {max_joint_speed: 17}}
'''


class FakePreviews:
    def __init__(self, *args):
        self.fresh = set()
        self.configured = []
        self.closed = False

    def configure(self, device, rotation, imaging=None, *, color_eye="auto"):
        self.configured.append((device.serial, rotation))
        self.fresh.discard(device.serial)

    def frame(self, serial):
        return (time.monotonic(), 'jpeg', b'frame') if serial in self.fresh else None

    def set_imaging(self, device, exposure, gain):
        self.imaging = getattr(self, 'imaging', []) + [(device.serial, exposure, gain)]

    def status(self, serial):
        return {'fresh': serial in self.fresh, 'running': True, 'error': '', 'generation': str(len(self.configured))}

    def close(self):
        self.closed = True


@pytest.fixture
def config(tmp_path):
    path = tmp_path / 'config/robot.local.yaml'
    path.parent.mkdir()
    path.write_text(CONFIG)
    return path


@pytest.fixture
def camera(config):
    session = camera_setup.CameraSetup(str(config), str(config.parent.parent), DEVICES, FakePreviews)
    yield session
    session.close()


def prepare_camera(camera):
    camera.update({'mapping': copy.deepcopy(camera.mapping)})
    camera.previews.fresh = set(camera.by_serial)


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


@pytest.fixture
def make_supervisor(tmp_path):
    created = []

    def factory(units):
        sup = Supervisor(UnitSet(units), str(tmp_path / "run"), str(tmp_path / "logs"))
        created.append(sup)
        return sup

    yield factory
    for sup in created:  # never leave processes behind
        for name in sup.units.names():
            try:
                sup.stop(name, timeout_s=1.0)
            except UnitError:
                pass


def wait_for(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False
