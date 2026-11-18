"""Camera setup transactions and the native camera window, without opening real hardware."""
import copy
import io
import time

import pytest
import yaml

from camera_setup import session as camera_setup
from robot_config import config_file
from robot_config.setup_steps import Io

from conftest import CONFIG, DEVICES, prepare_camera, wait_qt


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
    assert raw['xr']['device_id'] == 'PICO-BOUND' and raw['control']['limits']['max_joint_speed'] == 17
    assert '# preserve this comment' in config.read_text() and config_file.confirmed(raw, 'cameras')
    config_file.set_cameras(str(config), {'left_wrist': {'rotation': 180}})
    assert not config_file.confirmed(yaml.safe_load(config.read_text()), 'cameras')


def test_camera_requires_fresh_frame_and_every_enabled_role_confirmation(camera):
    prepare_camera(camera)
    with pytest.raises(ValueError, match='逐一确认'):
        camera.save({'revision': camera.revision, 'confirmed': ['head']})
    camera.previews.fresh.remove('LEFT')
    with pytest.raises(ValueError, match='最新画面'):
        camera.save({'revision': camera.revision, 'confirmed': list(camera_setup.ROLES)})


def test_camera_reassignment_invalidates_previous_revision(camera):
    prepare_camera(camera)
    previous = camera.revision
    mapping = copy.deepcopy(camera.mapping)
    mapping['LEFT']['rotation'] = 90
    camera.update({'mapping': mapping})
    with pytest.raises(ValueError, match='配置已变化'):
        camera.save({'revision': previous, 'confirmed': list(camera_setup.ROLES)})


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
    monkeypatch.setattr(camera_setup, 'require_idle', busy)
    with pytest.raises(ValueError, match='自行停止'):
        camera.update({'mapping': copy.deepcopy(camera.mapping)})
    assert camera.previews is None


def test_camera_does_not_overwrite_concurrent_config_changes(camera, config):
    prepare_camera(camera)
    config_file.set_fields(str(config), 'xr', {'device_id': 'NEW-ID'})
    changed = config.read_text()
    with pytest.raises(ValueError, match='其他程序修改'):
        camera.save({'revision': camera.revision, 'confirmed': list(camera_setup.ROLES)})
    assert config.read_text() == changed and not camera.saved


def test_native_camera_window_requires_new_images_after_rotating(camera, config, qt_app):
    from PyQt5 import QtCore, QtGui
    from robot_config.setup_steps import create_window
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


def test_choosing_a_rotation_turns_the_picture_without_pressing_apply(camera, config, qt_app):
    from PyQt5 import QtCore, QtGui
    from robot_config.setup_steps import create_window
    image = QtGui.QImage(80, 40, QtGui.QImage.Format_RGB32)  # wide: a quarter turn makes it tall
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
    card = window.cards['LEFT']
    window.tick()
    flat = card['image'].pixmap()
    assert flat.width() > flat.height()
    rotation = card['rotation']
    rotation.setCurrentIndex(rotation.findData(90))
    wait_qt(qt_app, window)  # applies by itself
    assert not window.dirty and camera.mapping['LEFT']['rotation'] == 90
    camera.previews.fresh = set(camera.by_serial)  # the fake preview forgets its frames on every update
    window.tick()
    turned = card['image'].pixmap()
    assert turned.height() > turned.width()
    window.close()


def test_camera_window_declining_old_process_cleanup_does_not_start_or_stop(camera, qt_app, monkeypatch):
    from PyQt5.QtWidgets import QMessageBox
    from robot_config.setup_steps import create_window
    old = [{'pid': 123, 'start_ticks': 5, 'camera': 'camera_left_wrist', 'serial': 'LEFT'}]
    monkeypatch.setattr(camera, 'conflicts', lambda: old)
    monkeypatch.setattr(QMessageBox, 'question', lambda *args: QMessageBox.No)
    window = create_window(camera)
    window.apply()
    assert window.future is None and camera.previews is None
    window.close()


def test_camera_window_confirming_cleanup_restarts_preview_without_saving(camera, config, qt_app, monkeypatch):
    from PyQt5.QtWidgets import QMessageBox
    from robot_config.setup_steps import create_window
    old = [{'pid': 123, 'start_ticks': 5, 'camera': 'camera_left_wrist', 'serial': 'LEFT'}]
    monkeypatch.setattr(camera, 'conflicts', lambda: old)
    calls = []
    monkeypatch.setattr(camera_setup, 'stop_camera_processes', lambda approved, *args: calls.append(approved))
    monkeypatch.setattr(QMessageBox, 'question', lambda *args: QMessageBox.Yes)
    window = create_window(camera)
    window.apply()
    wait_qt(qt_app, window)
    assert calls == [old] and camera.previews is not None
    assert config.read_text() == CONFIG
    window.close()


def test_image_probe_uses_its_own_context_and_only_subscribes_to_selected_image():
    import uuid
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    from camera_setup.discovery import ImageProbe
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


def test_unattended_setup_cannot_claim_first_visual_confirmation(config):
    output = io.StringIO()
    assert not camera_setup.step_cameras(str(config), str(config.parent.parent), Io(assume_yes=True, out=output),
                                         scanner=lambda: DEVICES)
    assert config.read_text() == CONFIG and '未保存配置' in output.getvalue()


def test_camera_previews_can_be_created(tmp_path):
    # The other tests skip the constructor (CameraPreviews.__new__) or replace it with a factory, so a stale import
    # in it went unnoticed until the setup window tried to show its first preview.
    previews = camera_setup.CameraPreviews(str(tmp_path), {})
    try:
        assert previews.probe is not None and previews.entries == {}
    finally:
        previews.probe.close()
        previews.directory.cleanup()


def test_the_preview_config_keeps_every_camera_field_the_loader_requires(tmp_path):
    # robot_config.load() refuses a config whose camera entries lack the imaging fields; the config the setup window
    # writes for a preview driver replaced each entry with four fields, so the driver never started.
    import types
    commands = []
    supervisor = types.SimpleNamespace(configure=lambda name, command: commands.append(command),
                                       start=lambda *args, **kwargs: None, stop=lambda unit: None,
                                       is_running=lambda unit: False)
    pool = camera_setup.CameraPreviews.__new__(camera_setup.CameraPreviews)
    pool.ws_root, pool.raw = str(tmp_path), {'cameras': {'head': {'serial': 'OLD'}}}
    pool.probe = types.SimpleNamespace(watch_image=lambda topic: None)
    pool.directory = types.SimpleNamespace(name=str(tmp_path))
    pool.prefix, pool.supervisor, pool.entries = 'setup_test_', supervisor, {}
    pool.configure(DEVICES[1], 90)
    path = next(arg for arg in commands[0] if str(arg).startswith('config:=')).split('=', 1)[1]
    cameras = yaml.safe_load(open(path))['cameras']
    required = {'enable', 'serial', 'usb_port', 'rotation', 'white_balance', 'ae_max_exposure', 'exposure', 'gain'}
    for role in ('head', 'left_wrist', 'right_wrist'):
        assert required <= set(cameras[role]), (role, sorted(required - set(cameras[role])))
    # The preview driver always runs upright; the window turns the picture, so a rotation never restarts a driver.
    assert cameras['left_wrist']['enable'] and cameras['left_wrist']['rotation'] == 0
    assert not cameras['head']['enable'] and cameras['head']['serial'] == ''
    supervisor.is_running = lambda unit: True
    supervisor.status = lambda unit: types.SimpleNamespace(last_error='')
    pool.probe.latest_image = lambda topic: None
    pool.configure(DEVICES[1], 270)
    assert len(commands) == 1, 'choosing another angle must not start the driver again'


def test_every_relative_import_of_the_package_resolves():
    import ast
    import importlib.util
    import pathlib
    package = pathlib.Path(camera_setup.__file__).parent
    missing = []
    for path in package.glob('*.py'):
        for node in ast.walk(ast.parse(path.read_text(encoding='utf-8'))):
            if isinstance(node, ast.ImportFrom) and node.level == 1 and node.module:
                if importlib.util.find_spec('camera_setup.' + node.module) is None:
                    missing.append(f'{path.name}:{node.lineno} from .{node.module}')
    assert not missing, missing


def test_exposure_and_gain_start_from_the_template_of_each_camera(camera):
    imaging = {d['serial']: (d['exposure'], d['gain']) for d in camera.state()['devices']}
    assert imaging == {'HEAD': (-1, -1), 'LEFT': (50, 16), 'RIGHT': (50, 16)}


def test_set_imaging_reaches_the_driver_and_keeps_the_confirmation(camera):
    prepare_camera(camera)
    revision = camera.revision
    camera.set_imaging({'serial': 'LEFT', 'exposure': 80, 'gain': 20})
    assert camera.revision == revision, 'exposure is not part of what the operator confirmed'
    assert camera.previews.imaging == [('LEFT', 80, 20)]
    assert {d['serial']: (d['exposure'], d['gain']) for d in camera.state()['devices']}['LEFT'] == (80, 20)
    camera.update({'mapping': copy.deepcopy(camera.mapping)})  # another apply does not forget it
    assert {d['serial']: (d['exposure'], d['gain']) for d in camera.state()['devices']}['LEFT'] == (80, 20)


@pytest.mark.parametrize('exposure, gain', [(-1, 16), (50, -1), (0, 16), (50, -2), (True, 16), (1.5, 16), ('50', 16)])
def test_set_imaging_rejects_values_the_config_would_refuse(camera, exposure, gain):
    with pytest.raises(ValueError, match='曝光'):
        camera.set_imaging({'serial': 'LEFT', 'exposure': exposure, 'gain': gain})
    assert not getattr(camera.previews, 'imaging', None)


def test_set_imaging_rejects_an_unknown_camera(camera):
    with pytest.raises(ValueError, match='未知相机'):
        camera.set_imaging({'serial': 'NOPE', 'exposure': 50, 'gain': 16})


def test_saving_writes_exposure_and_gain_of_every_selected_camera(camera, config):
    prepare_camera(camera)
    camera.set_imaging({'serial': 'LEFT', 'exposure': 80, 'gain': 20})
    camera.set_imaging({'serial': 'HEAD', 'exposure': -1, 'gain': -1})
    camera.save({'revision': camera.revision, 'confirmed': list(camera_setup.ROLES)})
    cameras = yaml.safe_load(config.read_text())['cameras']
    assert (cameras['left_wrist']['exposure'], cameras['left_wrist']['gain']) == (80, 20)
    assert (cameras['head']['exposure'], cameras['head']['gain']) == (-1, -1)
    assert (cameras['right_wrist']['exposure'], cameras['right_wrist']['gain']) == (50, 16)


class FakeProbe:
    def __init__(self, refuse=()):
        self.calls, self.refuse = [], set(refuse)

    def set_value(self, service, kind, value, timeout=2.0):
        self.calls.append((service, kind, value))
        return (False, 'refused') if service.rsplit('/', 1)[1] in self.refuse else (True, '')


def previews_with(probe):
    import types
    pool = camera_setup.CameraPreviews.__new__(camera_setup.CameraPreviews)
    pool.probe = probe
    pool.entries = {'LEFT': {'topic': '/setup_ab_camera_left_wrist/color/image_raw/compressed', 'stream': 'right_color'},
                    'HEAD': {'topic': '/setup_ab_camera_head/color/image_raw/compressed', 'stream': 'color'}}
    return pool


def test_manual_exposure_switches_auto_off_and_then_sets_exposure_and_gain():
    probe = FakeProbe()
    assert previews_with(probe).send_imaging(DEVICES[1], 80, 20) == []
    assert probe.calls == [('/setup_ab_camera_left_wrist/set_right_color_auto_exposure', 'bool', False),
                           ('/setup_ab_camera_left_wrist/set_right_color_exposure', 'int', 80),
                           ('/setup_ab_camera_left_wrist/set_right_color_gain', 'int', 20)]


def test_auto_exposure_only_switches_auto_on_and_the_head_uses_the_color_stream():
    probe = FakeProbe()
    assert previews_with(probe).send_imaging(DEVICES[0], -1, -1) == []
    assert probe.calls == [('/setup_ab_camera_head/set_color_auto_exposure', 'bool', True)]


def test_a_refused_driver_call_is_reported_and_stops_the_sequence():
    probe = FakeProbe(refuse={'set_right_color_exposure'})
    errors = previews_with(probe).send_imaging(DEVICES[1], 80, 20)
    assert len(errors) == 1 and 'set_right_color_exposure' in errors[0]
    assert len(probe.calls) == 2


def test_window_exposure_controls_send_manual_values_and_switch_auto_back_on(camera, qt_app):
    from robot_config.setup_steps import create_window
    window = create_window(camera)
    window.show()
    window.apply()
    wait_qt(qt_app, window)
    card = window.cards['LEFT']
    assert not card['auto'].isChecked() and card['exposure'].value() == 50 and card['gain'].value() == 16
    assert window.cards['HEAD']['auto'].isChecked() and not window.cards['HEAD']['exposure'].isEnabled()
    card['exposure'].setValue(90)
    assert card['timer'].isActive(), 'a change is sent after a short pause, not on every key press'
    card['timer'].stop()
    window.panel.send_imaging('LEFT')
    assert camera.previews.imaging[-1] == ('LEFT', 90, 16)
    card['auto'].setChecked(True)
    window.panel.send_imaging('LEFT')
    assert camera.previews.imaging[-1] == ('LEFT', -1, -1) and not card['exposure'].isEnabled()
    card['timer'].stop()
    assert not window.dirty and camera.revision == window.state['revision'], 'exposure never invalidates the confirmation'
    window.close()


def test_window_shows_a_driver_error_of_the_exposure_call(camera, qt_app):
    from robot_config.setup_steps import create_window
    window = create_window(camera)
    window.show()
    window.apply()
    wait_qt(qt_app, window)
    camera.previews.status = lambda serial: {'fresh': True, 'running': True, 'error': '', 'generation': '1',
                                             'fps': 30.0, 'imaging_error': '相机驱动没有响应'}
    window.tick()
    assert '曝光设置失败：相机驱动没有响应' in window.cards['LEFT']['feedback'].text()
    window.close()
