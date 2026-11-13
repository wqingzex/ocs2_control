"""Setup transactions and native windows, without opening real hardware."""
import copy
import io
import time

import pytest
import yaml

from w2_payload.config import edit as config_edit
from w2_payload.setup import device_setup as setup_devices, state as setup_state
from w2_payload.setup import steps as init_steps
from w2_payload.devices.cameras import Device

DEVICES = [Device('Orbbec Gemini 335Lg', 'HEAD', 'USB3.0', '2-2'),
           Device('Orbbec Gemini 305', 'LEFT', 'USB3.0', '2-4'),
           Device('Orbbec Gemini 305', 'RIGHT', 'USB3.0', '2-7')]
PORTS = [{'path': '/dev/ttyUSB0', 'device': '/dev/ttyUSB0', 'stable': False},
         {'path': '/dev/ttyUSB2', 'device': '/dev/ttyUSB2', 'stable': False}]
CONFIG = '''# preserve this comment
robot: {hand: linkerhand_l6, hand_drive: usb}
cameras:
  head: {serial: HEAD}
  left_wrist: {serial: LEFT}
  right_wrist: {serial: RIGHT}
xr: {device_id: PICO-BOUND}
hardware: {usb_left_port: /dev/ttyUSB0, usb_right_port: /dev/ttyUSB2, max_joint_speed: 17}
'''


class FakePreviews:
    def __init__(self, *args):
        self.fresh = set()
        self.configured = []
        self.closed = False

    def configure(self, device, rotation):
        self.configured.append((device.serial, rotation))
        self.fresh.discard(device.serial)

    def frame(self, serial):
        return (time.monotonic(), 'jpeg', b'frame') if serial in self.fresh else None

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
    session = setup_devices.CameraSetup(str(config), str(config.parent.parent), DEVICES, FakePreviews)
    yield session
    session.close()


@pytest.fixture
def hand(config):
    calls = []

    def worker(request):
        calls.append(request)
        return {'ok': True, 'tested': request['operation'] == 'test', 'positions': [0] * 6, 'message': '收到设备反馈'}

    session = setup_devices.HandSetup(str(config), str(config.parent.parent), lambda: PORTS, worker)
    session.calls = calls
    yield session
    session.close()


def prepare_camera(camera):
    camera.update({'mapping': copy.deepcopy(camera.mapping)})
    camera.previews.fresh = set(camera.by_serial)


def prepare_hand(hand):
    hand.update({'draft': copy.deepcopy(hand.draft)})
    for side in ('left', 'right'):
        hand.action({'revision': hand.revision, 'side': side, 'operation': 'test', 'allow_motion': True})


def test_camera_cancel_preserves_original_config(camera, config):
    prepare_camera(camera)
    assert config.read_text() == CONFIG
    camera.cancel()
    camera.close()
    assert config.read_text() == CONFIG and camera.previews.closed


def test_camera_visual_confirmation_saves_rotations_and_preserves_other_sections(camera, config):
    mapping = copy.deepcopy(camera.mapping)
    mapping['LEFT'] = {'role': 'right_wrist', 'rotation': 90}
    mapping['RIGHT'] = {'role': 'left_wrist', 'rotation': 270}
    camera.update({'mapping': mapping})
    camera.previews.fresh = set(camera.by_serial)
    camera.save({'revision': camera.revision, 'confirmed': ['head', 'left_wrist', 'right_wrist']})
    raw = yaml.safe_load(config.read_text())
    assert raw['cameras']['right_wrist']['serial'] == 'LEFT'
    assert raw['cameras']['right_wrist']['rotation'] == 90
    assert raw['cameras']['left_wrist']['rotation'] == 270
    assert raw['xr']['device_id'] == 'PICO-BOUND' and raw['hardware']['max_joint_speed'] == 17
    assert '# preserve this comment' in config.read_text() and setup_state.confirmed(raw, 'cameras')
    config_edit.set_cameras(str(config), {'left_wrist': {'rotation': 180}})
    assert not setup_state.confirmed(yaml.safe_load(config.read_text()), 'cameras')


def test_camera_requires_fresh_frame_and_every_enabled_role_confirmation(camera):
    prepare_camera(camera)
    with pytest.raises(ValueError, match='逐一确认'):
        camera.save({'revision': camera.revision, 'confirmed': ['head']})
    camera.previews.fresh.remove('LEFT')
    with pytest.raises(ValueError, match='最新画面'):
        camera.save({'revision': camera.revision, 'confirmed': list(setup_devices.ROLES)})


def test_camera_reassignment_invalidates_previous_revision(camera):
    prepare_camera(camera)
    previous = camera.revision
    mapping = copy.deepcopy(camera.mapping)
    mapping['LEFT']['rotation'] = 90
    camera.update({'mapping': mapping})
    with pytest.raises(ValueError, match='配置已变化'):
        camera.save({'revision': previous, 'confirmed': list(setup_devices.ROLES)})


def test_camera_rejects_duplicate_roles_and_incompatible_models(camera):
    mapping = copy.deepcopy(camera.mapping)
    mapping['RIGHT']['role'] = 'left_wrist'
    with pytest.raises(ValueError, match='只能分配一台'):
        camera.update({'mapping': mapping})
    mapping['HEAD']['role'], mapping['LEFT']['role'], mapping['RIGHT']['role'] = 'left_wrist', 'head', ''
    with pytest.raises(ValueError, match='头部使用'):
        camera.update({'mapping': mapping})


def test_camera_does_not_stop_other_modules(camera, monkeypatch):
    def busy(*args):
        raise ValueError('模块仍在运行，请先自行停止')
    monkeypatch.setattr(setup_devices, 'require_idle', busy)
    with pytest.raises(ValueError, match='自行停止'):
        camera.update({'mapping': copy.deepcopy(camera.mapping)})
    assert camera.previews is None


def test_camera_does_not_overwrite_concurrent_config_changes(camera, config):
    prepare_camera(camera)
    config_edit.set_fields(str(config), 'xr', {'device_id': 'NEW-ID'})
    changed = config.read_text()
    with pytest.raises(ValueError, match='其他程序修改'):
        camera.save({'revision': camera.revision, 'confirmed': list(setup_devices.ROLES)})
    assert config.read_text() == changed and not camera.saved


def test_hand_requires_feedback_and_manual_confirmation(hand, config):
    assert not hand.calls and config.read_text() == CONFIG
    with pytest.raises(ValueError, match='确认左手'):
        hand.save({'revision': 0, 'confirmed': []})
    with pytest.raises(ValueError, match='开合测试'):
        hand.save({'revision': 0, 'confirmed': ['left', 'right']})
    prepare_hand(hand)
    hand.save({'revision': hand.revision, 'confirmed': ['left', 'right']})
    assert [call['operation'] for call in hand.calls] == ['test'] * 2 + ['probe'] * 2  # the recheck at save never moves
    raw = yaml.safe_load(config.read_text())
    assert setup_state.confirmed(raw, 'hand')
    assert raw['hardware']['max_joint_speed'] == 17 and raw['xr']['device_id'] == 'PICO-BOUND'


def test_hand_swap_invalidates_feedback_and_confirmation(hand, config):
    prepare_hand(hand)
    previous = hand.revision
    draft = copy.deepcopy(hand.draft)
    draft['left_port'], draft['right_port'] = draft['right_port'], draft['left_port']
    hand.update({'draft': draft})
    assert not hand.results and config.read_text() == CONFIG
    with pytest.raises(ValueError, match='配置已变化'):
        hand.save({'revision': previous, 'confirmed': ['left', 'right']})
    prepare_hand(hand)
    hand.save({'revision': hand.revision, 'confirmed': ['left', 'right']})
    raw = yaml.safe_load(config.read_text())
    assert raw['hardware']['usb_left_port'] == '/dev/ttyUSB2'
    config_edit.set_usb_ports(str(config), '/dev/ttyUSB9', None)
    assert not setup_state.confirmed(yaml.safe_load(config.read_text()), 'hand')


def test_hand_motion_requires_explicit_action_and_allow_flag(hand):
    with pytest.raises(ValueError, match='主动确认'):
        hand.action({'revision': 0, 'side': 'left', 'operation': 'test'})
    assert hand.calls == []
    hand.action({'revision': 0, 'side': 'left', 'operation': 'test', 'allow_motion': True})
    assert hand.calls[0]['operation'] == 'test' and hand.results['left']['tested']


def test_hand_485_requires_controller_connection_confirmation(hand):
    hand.update({'draft': {**hand.draft, 'drive': '485'}})
    with pytest.raises(ValueError, match='原运控已停止'):
        hand.action({'revision': hand.revision, 'side': 'left', 'operation': 'probe'})
    assert hand.calls == []
    hand.action({'revision': hand.revision, 'side': 'left', 'operation': 'probe', 'allow_controller_connection': True})
    assert hand.calls[0]['drive'] == '485' and hand.calls[0]['channel'] == 1


def test_hand_does_not_open_hardware_when_control_is_running(hand, monkeypatch):
    def busy(*args):
        raise ValueError('模块仍在运行，请先自行停止')
    monkeypatch.setattr(setup_devices, 'require_idle', busy)
    with pytest.raises(ValueError, match='自行停止'):
        hand.action({'revision': 0, 'side': 'left', 'operation': 'probe'})
    assert hand.calls == []


def test_hand_error_invalidates_old_feedback(hand):
    prepare_hand(hand)
    hand.worker = lambda request: (_ for _ in ()).throw(RuntimeError('设备断开'))
    with pytest.raises(RuntimeError, match='设备断开'):
        hand.action({'revision': hand.revision, 'side': 'left', 'operation': 'probe'})
    assert hand.results['left']['ok'] is False


def test_hand_can_explicitly_disable_without_opening_devices(hand, config):
    hand.update({'draft': {**hand.draft, 'hand': 'none'}})
    hand.save({'revision': hand.revision, 'confirmed': []})
    assert not hand.calls and yaml.safe_load(config.read_text())['robot']['hand'] == 'none'


def test_xhand_rejects_arm485_and_duplicate_port(hand):
    with pytest.raises(ValueError, match='仅支持 USB'):
        hand.update({'draft': {**hand.draft, 'hand': 'xhand1', 'drive': '485'}})
    with pytest.raises(ValueError, match='同一串口'):
        hand.update({'draft': {**hand.draft, 'hand': 'xhand1', 'right_port': hand.draft['left_port']}})


def test_cancelled_session_cannot_commit(hand, config):
    prepare_hand(hand)
    hand.cancel()
    with pytest.raises(ValueError, match='会话已结束'):
        hand.save({'revision': hand.revision, 'confirmed': ['left', 'right']})
    assert config.read_text() == CONFIG


def test_unattended_setup_cannot_claim_first_visual_or_hand_confirmation(config):
    output = io.StringIO()
    io_object = init_steps.Io(assume_yes=True, out=output)
    assert not init_steps.step_cameras(str(config), str(config.parent.parent), io_object, scanner=lambda: DEVICES)
    assert not init_steps.step_hand(str(config), io_object)
    assert config.read_text() == CONFIG and '未保存配置' in output.getvalue()


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


def test_native_hand_window_changes_are_drafts_until_both_sides_confirm(hand, config, qt_app, monkeypatch):
    from w2_payload.setup.wizard import create_window
    from PyQt5.QtWidgets import QMessageBox
    monkeypatch.setattr(QMessageBox, 'question', lambda *args, **kwargs: QMessageBox.Yes)
    window = create_window(hand)
    window.show()
    window.apply()
    wait_qt(qt_app, window)
    assert 'probe' not in window.cards['left'] and window.cards['left']['test'].text() == '开合测试'
    for side in ('left', 'right'):
        window.hand_action(side, 'test')
        wait_qt(qt_app, window)
        assert window.cards[side]['check'].isEnabled()
        window.cards[side]['check'].setChecked(True)
    assert config.read_text() == CONFIG
    window.save()
    wait_qt(qt_app, window)
    assert hand.saved and setup_state.confirmed(yaml.safe_load(config.read_text()), 'hand')
    assert [call['operation'] for call in hand.calls] == ['test'] * 2 + ['probe'] * 2


def test_hand_disconnect_between_confirmation_and_save_does_not_write(hand, config):
    prepare_hand(hand)
    hand.worker = lambda request: (_ for _ in ()).throw(RuntimeError('设备断开'))
    with pytest.raises(RuntimeError, match='设备断开'):
        hand.save({'revision': hand.revision, 'confirmed': ['left', 'right']})
    assert config.read_text() == CONFIG and not hand.saved


def test_native_camera_window_requires_new_images_after_rotating(camera, config, qt_app):
    from PyQt5 import QtCore, QtGui
    from w2_payload.setup.wizard import create_window
    image = QtGui.QImage(80, 60, QtGui.QImage.Format_RGB32)
    image.fill(QtGui.QColor('green'))
    encoded = QtCore.QByteArray()
    buffer = QtCore.QBuffer(encoded)
    buffer.open(QtCore.QIODevice.WriteOnly)
    assert image.save(buffer, 'PNG')
    window = create_window(camera)
    window.show()
    window.apply()
    wait_qt(qt_app, window)
    camera.previews.frame = lambda serial: (time.monotonic(), 'png', bytes(encoded)) if serial in camera.previews.fresh else None
    camera.previews.fresh = set(camera.by_serial)
    window.tick()
    assert window.cards['LEFT']['check'].isEnabled()
    window.cards['LEFT']['check'].setChecked(True)
    rotation = window.cards['LEFT']['rotation']
    rotation.setCurrentIndex(rotation.findData(90))
    assert window.dirty and not window.cards['LEFT']['check'].isChecked()
    window.apply()
    wait_qt(qt_app, window)
    assert not window.cards['LEFT']['check'].isEnabled()
    camera.previews.fresh = set(camera.by_serial)
    window.tick()
    for card in window.cards.values():
        assert card['check'].isEnabled()
        card['check'].setChecked(True)
    window.save()
    wait_qt(qt_app, window)
    assert yaml.safe_load(config.read_text())['cameras']['left_wrist']['rotation'] == 90


def test_native_setup_event_loop_closes_without_changing_config(hand, config, qt_app, monkeypatch):
    from PyQt5 import QtCore
    from w2_payload.setup import wizard as setup_wizard
    create = setup_wizard.create_window
    def auto_close(session):
        window = create(session)
        QtCore.QTimer.singleShot(50, window.close)
        return window
    monkeypatch.setattr(setup_wizard, 'create_window', auto_close)
    assert not setup_wizard.run_wizard(hand, init_steps.Io(out=io.StringIO()))
    assert config.read_text() == CONFIG and hand.cancelled


def test_shared_setup_and_camera_launch_import_without_teleoperation_or_xr():
    import subprocess
    import sys
    result = subprocess.run([sys.executable, '-c', '''
import importlib.abc, sys
class BlockTeleop(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.startswith(('fa_teleop', 'PXREA')):
            raise ImportError('teleoperation is deliberately unavailable')
sys.meta_path.insert(0, BlockTeleop())
from w2_payload import config
from w2_payload.config import edit
from w2_payload.setup import steps, device_setup, wizard, check
from w2_payload.devices import camera_launch
assert not any(name.startswith('fa_teleop') for name in sys.modules)
'''], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_existing_process_detection_is_read_only_and_checks_process_identity(tmp_path):
    import json
    import os
    from w2_payload.runtime import process_stat, require_idle
    run = tmp_path / 'run'
    run.mkdir()
    start, _ = process_stat(os.getpid())
    state = run / 'control.json'
    state.write_text(json.dumps({'pid': os.getpid(), 'start_ticks': start}))
    with pytest.raises(ValueError, match='自行停止'):
        require_idle(str(tmp_path), ['control'])
    state.write_text(json.dumps({'pid': os.getpid(), 'start_ticks': start + 1}))
    require_idle(str(tmp_path), ['control'])  # stale record must not identify an unrelated process


def test_owned_preview_cleanup_does_not_stop_other_processes(tmp_path):
    import subprocess
    import sys
    from w2_payload.runtime import OwnedDrivers
    other = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
    owner = OwnedDrivers(tmp_path)
    owner.configure('preview', [sys.executable, '-c', 'import time; time.sleep(60)'])
    try:
        owner.start('preview')
        assert owner.is_running('preview')
        owner.stop('preview')
        assert not owner.is_running('preview') and other.poll() is None
    finally:
        owner.stop('preview')
        other.terminate()
        other.wait(timeout=3)


def test_owned_cleanup_stops_driver_that_created_a_separate_session(tmp_path):
    import sys
    from w2_payload.runtime import OwnedDrivers, process_stat
    pid_file = tmp_path / 'driver.pid'
    script = ('import subprocess,sys,time; '
              'p=subprocess.Popen([sys.executable,"-c","import time;time.sleep(60)"],start_new_session=True); '
              f'open({str(pid_file)!r},"w").write(str(p.pid)); '
              'time.sleep(60)')
    owner = OwnedDrivers(tmp_path / 'logs')
    owner.configure('preview', [sys.executable, '-c', script])
    try:
        owner.start('preview')
        deadline = time.monotonic() + 3
        while not pid_file.exists() and time.monotonic() < deadline:
            owner.is_running('preview')
            time.sleep(.02)
        child = int(pid_file.read_text())
        assert owner.is_running('preview') and process_stat(child) is not None
        owner.stop('preview')
        assert process_stat(child) is None
    finally:
        owner.stop('preview')


def test_image_probe_uses_its_own_context_and_only_subscribes_to_selected_image():
    import uuid
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    from w2_payload.devices.cameras import ImageProbe
    previous = rclpy.ok()
    probe = ImageProbe('setup_image_test_' + uuid.uuid4().hex[:8])
    topic = '/setup_test_' + uuid.uuid4().hex + '/image'
    try:
        probe.watch_image(topic)
        publisher = probe.node.create_publisher(CompressedImage, topic, qos_profile_sensor_data)
        deadline = time.monotonic() + 3
        while probe.latest_image(topic) is None and time.monotonic() < deadline:
            publisher.publish(CompressedImage(format='jpeg', data=b'image-test'))
            time.sleep(.05)
        assert probe.latest_image(topic)[2] == b'image-test'
        assert set(probe.subscriptions) == {topic}
    finally:
        probe.close()
    assert not probe.context.ok() and rclpy.ok() == previous


def test_camera_window_declining_old_process_cleanup_does_not_start_or_stop(camera, qt_app, monkeypatch):
    from PyQt5.QtWidgets import QMessageBox
    from w2_payload.setup.wizard import create_window
    old = [{'pid': 123, 'start_ticks': 5, 'camera': 'camera_left_wrist', 'serial': 'LEFT'}]
    monkeypatch.setattr(camera, 'conflicts', lambda: old)
    monkeypatch.setattr(QMessageBox, 'question', lambda *args: QMessageBox.No)
    window = create_window(camera)
    window.apply()
    assert window.future is None and camera.previews is None
    window.close()


def test_camera_window_confirming_cleanup_restarts_preview_without_saving(camera, config, qt_app, monkeypatch):
    from PyQt5.QtWidgets import QMessageBox
    from w2_payload.setup.wizard import create_window
    old = [{'pid': 123, 'start_ticks': 5, 'camera': 'camera_left_wrist', 'serial': 'LEFT'}]
    monkeypatch.setattr(camera, 'conflicts', lambda: old)
    calls = []
    monkeypatch.setattr(setup_devices, 'stop_camera_processes', lambda approved, *args: calls.append(approved))
    monkeypatch.setattr(QMessageBox, 'question', lambda *args: QMessageBox.Yes)
    window = create_window(camera)
    window.apply()
    wait_qt(qt_app, window)
    assert calls == [old] and camera.previews is not None
    assert config.read_text() == CONFIG
    window.close()


def test_native_hand_test_dialog_no_never_moves_a_finger(hand, qt_app, monkeypatch):
    from PyQt5.QtWidgets import QMessageBox
    from w2_payload.setup.wizard import create_window
    window = create_window(hand)
    monkeypatch.setattr(QMessageBox, 'question', lambda *args: QMessageBox.No)
    window.hand_action('left', 'test')
    assert hand.calls == [] and window.future is None
    window.close()


def test_native_window_close_discards_draft(hand, config, qt_app):
    from w2_payload.setup.wizard import create_window
    window = create_window(hand)
    window.hand.setCurrentIndex(window.hand.findData('none'))
    window.apply()
    wait_qt(qt_app, window)
    window.close()
    assert hand.cancelled and config.read_text() == CONFIG
