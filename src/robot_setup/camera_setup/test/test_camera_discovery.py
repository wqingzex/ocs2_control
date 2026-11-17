"""Old-camera cleanup is limited to identified devices and approved process identities."""
import signal
from types import SimpleNamespace

import pytest

from camera_setup import discovery as processes
from camera_setup.discovery import Device
from camera_setup.session import CameraPreviews


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
    process(tmp_path, 103, ['ros2', 'launch', 'robot_startup', 'split_body.launch.py',
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
    pool.entries = {'SERIAL': {'unit': 'preview', 'generation': 'new', 'topic': '/t'}}
    pool.supervisor = SimpleNamespace(status=lambda _: SimpleNamespace(running=True, last_error=error))
    pool.probe = SimpleNamespace(rate=lambda _: 0.0)
    pool.frame = lambda _: None
    assert '其他采集程序占用' in pool.status('SERIAL')['error']
    pool.frame = lambda _: (1, 'jpeg', b'frame')
    assert pool.status('SERIAL')['fresh'] and pool.status('SERIAL')['error'] == ''


def test_image_probe_rate_counts_frames_of_the_last_window_only(monkeypatch):
    from camera_setup.discovery import ImageProbe, RATE_WINDOW_S
    probe = ImageProbe.__new__(ImageProbe)
    probe.lock, probe.images, probe.arrivals = __import__('threading').Lock(), {}, {}
    message = SimpleNamespace(format='jpeg', data=b'x')
    clock = iter([0.0] * 20 + [RATE_WINDOW_S + 5.0] * 5)
    monkeypatch.setattr(processes, 'time', SimpleNamespace(monotonic=lambda: next(clock)))
    for _ in range(20):
        probe.receive('/t', message)
    assert probe.rate('/t') == 0.0  # the 20 frames are older than the window by now
    assert probe.rate('/never') == 0.0


def test_image_probe_rate_is_frames_per_second(monkeypatch):
    from camera_setup.discovery import ImageProbe, RATE_WINDOW_S
    probe = ImageProbe.__new__(ImageProbe)
    probe.lock, probe.images, probe.arrivals = __import__('threading').Lock(), {}, {}
    message = SimpleNamespace(format='jpeg', data=b'x')
    now = [100.0]
    monkeypatch.setattr(processes, 'time', SimpleNamespace(monotonic=lambda: now[0]))
    for index in range(int(30 * RATE_WINDOW_S)):
        now[0] = 100.0 + index / 30
        probe.receive('/t', message)
    now[0] = 100.0 + RATE_WINDOW_S - 0.01
    assert probe.rate('/t') == pytest.approx(30.0, abs=1.0)


def test_preview_status_reports_the_frame_rate():
    pool = CameraPreviews.__new__(CameraPreviews)
    pool.entries = {'SERIAL': {'unit': 'preview', 'generation': 'g', 'topic': '/t'}}
    pool.supervisor = SimpleNamespace(status=lambda _: SimpleNamespace(running=True, last_error=''))
    pool.probe = SimpleNamespace(rate=lambda topic: 29.5 if topic == '/t' else 0.0)
    pool.frame = lambda _: (1, 'jpeg', b'frame')
    assert pool.status('SERIAL')['fps'] == 29.5
    assert pool.status('OTHER')['fps'] == 0.0
