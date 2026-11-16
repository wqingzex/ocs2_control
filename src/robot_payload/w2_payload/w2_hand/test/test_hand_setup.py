"""Hand setup transactions and the native hand window, without opening real hardware."""
import copy
import io

import pytest
import yaml

from w2_hand import hand_setup
from w2_payload.device_setup import Io
from w2_payload import config_edit

from conftest import CONFIG, LEFT_BY_ID, PORTS, RIGHT_BY_ID, prepare_hand, wait_qt


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
    assert config_edit.confirmed(raw, 'hand')
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
    assert raw['hardware']['usb_left_port'] == RIGHT_BY_ID
    config_edit.set_usb_ports(str(config), '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_CCCC-if00-port0', None)
    assert not config_edit.confirmed(yaml.safe_load(config.read_text()), 'hand')


def test_serial_devices_lists_only_by_id_serial_number_links(tmp_path, monkeypatch):
    """A hand is identified by the adapter's serial number (/dev/serial/by-id). ttyUSBn is numbered in enumeration
    order and changes when a port is replugged; by-path depends on the physical socket: neither is offered."""
    from w2_hand import hand as hand_probe
    by_id = tmp_path / 'by-id'
    by_id.mkdir()
    (tmp_path / 'ttyUSB2').write_text('')
    (by_id / 'usb-FTDI_AAAA-if00-port0').symlink_to(tmp_path / 'ttyUSB2')
    seen = []

    def fake_glob(pattern):
        seen.append(pattern)
        return [str(p) for p in sorted(by_id.iterdir())] if pattern == '/dev/serial/by-id/*' else [str(tmp_path / 'ttyUSB2')]

    monkeypatch.setattr(hand_probe.glob, 'glob', fake_glob)
    assert [item['path'] for item in hand_probe.serial_devices()] == [str(by_id / 'usb-FTDI_AAAA-if00-port0')]
    assert seen == ['/dev/serial/by-id/*']


@pytest.mark.parametrize('port', ['/dev/ttyUSB0', '/dev/ttyACM0', '/dev/serial/by-path/pci-0000:00:14.0-usb-0:5.2:1.0-port0'])
def test_hand_update_rejects_ports_that_are_not_serial_number_links(hand, port):
    draft = copy.deepcopy(hand.draft)
    draft['left_port'] = port
    with pytest.raises(ValueError, match='序列号'):
        hand.update({'draft': draft})
    assert hand.draft['left_port'] == LEFT_BY_ID


def test_hand_draft_for_an_old_numbered_config_is_a_choice_not_a_save(tmp_path):
    """An old /dev/ttyUSB0 in the config is only a starting point: the draft offers an adapter by serial number and
    nothing is written (or confirmed) until the operator confirms left and right."""
    path = tmp_path / 'cfg' / 'robot.yaml'
    path.parent.mkdir()
    path.write_text(CONFIG.replace(LEFT_BY_ID, '/dev/ttyUSB0'), encoding='utf-8')
    session = hand_setup.HandSetup(str(path), str(tmp_path), lambda: PORTS, lambda request: {})
    try:
        assert session.draft['left_port'] == LEFT_BY_ID and session.draft['right_port'] == RIGHT_BY_ID
        assert path.read_text(encoding='utf-8') == CONFIG.replace(LEFT_BY_ID, '/dev/ttyUSB0')
    finally:
        session.close()


def test_hand_draft_keeps_empty_when_the_configured_adapter_is_unplugged(tmp_path):
    """A config that names an adapter by serial number which is not on the line does not get another one guessed."""
    path = tmp_path / 'cfg' / 'robot.yaml'
    path.parent.mkdir()
    path.write_text(CONFIG, encoding='utf-8')
    session = hand_setup.HandSetup(str(path), str(tmp_path), lambda: PORTS[1:], lambda request: {})
    try:
        assert session.draft['left_port'] == ''
        assert session.draft['right_port'] == RIGHT_BY_ID
    finally:
        session.close()


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
    monkeypatch.setattr(hand_setup, 'require_idle', busy)
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


def test_usb_can_is_not_confirmed_in_the_setup_tool(hand):
    with pytest.raises(ValueError, match='USB-CAN'):
        hand.update({'draft': {**hand.draft, 'drive': 'usb_can'}})
    assert not hand.calls


def test_wuji_is_not_confirmed_in_the_setup_tool(hand):
    with pytest.raises(ValueError, match='Wuji'):
        hand.update({'draft': {**hand.draft, 'hand': 'wuji_hand2', 'drive': 'ethernet'}})
    assert not hand.calls


def test_cancelled_session_cannot_commit(hand, config):
    prepare_hand(hand)
    hand.cancel()
    with pytest.raises(ValueError, match='会话已结束'):
        hand.save({'revision': hand.revision, 'confirmed': ['left', 'right']})
    assert config.read_text() == CONFIG


def test_unattended_setup_cannot_claim_first_hand_confirmation(config):
    output = io.StringIO()
    assert not hand_setup.step_hand(str(config), str(config.parent.parent), Io(assume_yes=True, out=output))
    assert config.read_text() == CONFIG and '未保存配置' in output.getvalue()


def test_native_hand_window_changes_are_drafts_until_both_sides_confirm(hand, config, qt_app, monkeypatch):
    from w2_payload.device_setup import create_window
    from PyQt5.QtWidgets import QMessageBox
    monkeypatch.setattr(QMessageBox, 'question', lambda *args, **kwargs: QMessageBox.Yes)
    window = create_window(hand)
    window.show()
    window.apply()
    wait_qt(qt_app, window)
    assert 'probe' not in window.cards['left'] and window.cards['left']['test'].text() == '开合测试'
    for side in ('left', 'right'):
        window.panel.hand_action(side, 'test')
        wait_qt(qt_app, window)
        assert window.cards[side]['check'].isEnabled()
        window.cards[side]['check'].setChecked(True)
    assert config.read_text() == CONFIG
    window.save()
    wait_qt(qt_app, window)
    assert hand.saved and config_edit.confirmed(yaml.safe_load(config.read_text()), 'hand')
    assert [call['operation'] for call in hand.calls] == ['test'] * 2 + ['probe'] * 2


def test_hand_disconnect_between_confirmation_and_save_does_not_write(hand, config):
    prepare_hand(hand)
    hand.worker = lambda request: (_ for _ in ()).throw(RuntimeError('设备断开'))
    with pytest.raises(RuntimeError, match='设备断开'):
        hand.save({'revision': hand.revision, 'confirmed': ['left', 'right']})
    assert config.read_text() == CONFIG and not hand.saved


def test_native_hand_test_dialog_no_never_moves_a_finger(hand, qt_app, monkeypatch):
    from PyQt5.QtWidgets import QMessageBox
    from w2_payload.device_setup import create_window
    window = create_window(hand)
    monkeypatch.setattr(QMessageBox, 'question', lambda *args: QMessageBox.No)
    window.panel.hand_action('left', 'test')
    assert hand.calls == [] and window.future is None
    window.close()


def test_native_window_close_discards_draft(hand, config, qt_app):
    from w2_payload.device_setup import create_window
    window = create_window(hand)
    window.panel.hand.setCurrentIndex(window.panel.hand.findData('none'))
    window.apply()
    wait_qt(qt_app, window)
    window.close()
    assert hand.cancelled and config.read_text() == CONFIG


def test_native_setup_event_loop_closes_without_changing_config(hand, config, qt_app, monkeypatch):
    from PyQt5 import QtCore
    from w2_payload import device_setup as setup_wizard
    create = setup_wizard.create_window
    def auto_close(session):
        window = create(session)
        QtCore.QTimer.singleShot(50, window.close)
        return window
    monkeypatch.setattr(setup_wizard, 'create_window', auto_close)
    assert not setup_wizard.run_wizard(hand, Io(out=io.StringIO()))
    assert config.read_text() == CONFIG and hand.cancelled
