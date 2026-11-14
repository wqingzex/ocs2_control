"""Camera setup transactions and the native camera window, without opening real hardware."""
import copy
import io
import time

import pytest
import yaml

from w2_camera import camera_setup
from w2_payload import config_edit
from w2_payload.device_setup import Io

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
    assert raw['xr']['device_id'] == 'PICO-BOUND' and raw['hardware']['max_joint_speed'] == 17
    assert '# preserve this comment' in config.read_text() and config_edit.confirmed(raw, 'cameras')
    config_edit.set_cameras(str(config), {'left_wrist': {'rotation': 180}})
    assert not config_edit.confirmed(yaml.safe_load(config.read_text()), 'cameras')


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
    config_edit.set_fields(str(config), 'xr', {'device_id': 'NEW-ID'})
    changed = config.read_text()
    with pytest.raises(ValueError, match='其他程序修改'):
        camera.save({'revision': camera.revision, 'confirmed': list(camera_setup.ROLES)})
    assert config.read_text() == changed and not camera.saved


def test_native_camera_window_requires_new_images_after_rotating(camera, config, qt_app):
    from PyQt5 import QtCore, QtGui
    from w2_payload.device_setup import create_window
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


def test_camera_window_declining_old_process_cleanup_does_not_start_or_stop(camera, qt_app, monkeypatch):
    from PyQt5.QtWidgets import QMessageBox
    from w2_payload.device_setup import create_window
    old = [{'pid': 123, 'start_ticks': 5, 'camera': 'camera_left_wrist', 'serial': 'LEFT'}]
    monkeypatch.setattr(camera, 'conflicts', lambda: old)
    monkeypatch.setattr(QMessageBox, 'question', lambda *args: QMessageBox.No)
    window = create_window(camera)
    window.apply()
    assert window.future is None and camera.previews is None
    window.close()


def test_camera_window_confirming_cleanup_restarts_preview_without_saving(camera, config, qt_app, monkeypatch):
    from PyQt5.QtWidgets import QMessageBox
    from w2_payload.device_setup import create_window
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
    from w2_camera.camera import ImageProbe
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
