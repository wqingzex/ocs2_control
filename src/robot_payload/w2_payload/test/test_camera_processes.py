"""Old-camera cleanup is limited to identified devices and approved process identities."""
import signal
from types import SimpleNamespace

import pytest

from w2_payload.devices import cameras as processes
from w2_payload.devices.cameras import Device
from w2_payload.setup.device_setup import CameraPreviews


def process(root, pid, args, ticks=500, state='S'):
    directory = root / str(pid)
    directory.mkdir(exist_ok=True)
    fields = [state] + ['0'] * 19
    fields[19] = str(ticks)
    (directory / 'stat').write_text(f'{pid} (camera) ' + ' '.join(fields))
    (directory / 'cmdline').write_bytes(b'\0'.join(value.encode() for value in args))


@pytest.fixture
def cameras(tmp_path, monkeypatch):
    monkeypatch.setattr(processes, 'PROC', tmp_path)
    process(tmp_path, 101, ['ros2', 'launch', 'orbbec_camera', 'gemini305.launch.py',
                            'camera_name:=camera_left_wrist', 'serial_number:=SERIAL'])
    process(tmp_path, 102, ['/opt/ros/lib/component_container', '--ros-args',
                            '__node:=camera_container', '__ns:=/camera_left_wrist'])
    process(tmp_path, 103, ['ros2', 'launch', 'robot_bringup', 'split_body.launch.py',
                            'camera_name:=camera_left_wrist', 'serial_number:=SERIAL'])
    process(tmp_path, 104, ['ros2', 'launch', 'orbbec_camera', 'gemini305.launch.py',
                            'camera_name:=camera_other', 'serial_number:=OTHER'])
    return tmp_path, [Device('Gemini 305', 'SERIAL')]


def test_finds_only_camera_launch_and_its_dedicated_driver(cameras):
    _, devices = cameras
    assert [item['pid'] for item in processes.find_camera_processes(devices)] == [101, 102]
    assert processes.find_camera_processes(devices, {'camera_left_wrist'}) == []


def test_stop_only_signals_approved_camera_processes(cameras, monkeypatch):
    root, devices = cameras
    approved = processes.find_camera_processes(devices)
    signals = []
    def kill(pid, number):
        signals.append((pid, number))
        path = root / str(pid) / 'stat'
        path.write_text(path.read_text().replace(' S ', ' Z '))
    monkeypatch.setattr(processes.os, 'kill', kill)
    processes.stop_camera_processes(approved, devices)
    assert signals == [(101, signal.SIGINT), (102, signal.SIGINT)]
    assert processes.read_process(root / '103') is not None


def test_new_or_reused_pid_requires_new_confirmation(cameras, monkeypatch):
    root, devices = cameras
    approved = processes.find_camera_processes(devices)
    path = root / '101' / 'stat'
    path.write_text(path.read_text().replace('500', '501'))
    monkeypatch.setattr(processes.os, 'kill', lambda *args: pytest.fail('unapproved process must not be signalled'))
    with pytest.raises(ValueError, match='列表已变化'):
        processes.stop_camera_processes(approved, devices)


def test_orphan_driver_is_found_by_its_open_usb_device(cameras, monkeypatch):
    root, devices = cameras
    (root / '101' / 'stat').unlink()  # the launch parent is already gone
    monkeypatch.setattr(processes, 'usb_device_nodes', lambda _: {12: 'SERIAL'})
    monkeypatch.setattr(processes, 'open_usb_serials', lambda pid, nodes: {'SERIAL'} if pid == 102 else set())
    found = processes.find_camera_processes(devices)
    assert [item['pid'] for item in found] == [102] and found[0]['serial'] == 'SERIAL'
    assert processes.find_camera_processes(devices, {'camera_left_wrist'}) == []


def test_busy_error_is_readable_and_old_error_does_not_mask_recovered_frames():
    error = 'Failed to get device info uvc_open failed: [Path: 2-7-8.0, Return Code: -6]'
    assert '其他采集程序占用' in processes.camera_error(error)
    pool = CameraPreviews.__new__(CameraPreviews)
    pool.entries = {'SERIAL': {'unit': 'preview', 'generation': 'new'}}
    pool.supervisor = SimpleNamespace(status=lambda _: SimpleNamespace(running=True, last_error=error))
    pool.frame = lambda _: None
    assert '其他采集程序占用' in pool.status('SERIAL')['error']
    pool.frame = lambda _: (1, 'jpeg', b'frame')
    assert pool.status('SERIAL')['fresh'] and pool.status('SERIAL')['error'] == ''
