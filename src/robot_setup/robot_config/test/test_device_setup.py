"""The setup framework and process tracking shared by the device packages."""
import time

import pytest

def test_shared_setup_and_camera_launch_import_without_teleoperation_or_xr():
    import subprocess
    import sys
    result = subprocess.run([sys.executable, '-c', '''
import importlib.abc, sys
class BlockTeleop(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.startswith(('robot_app', 'PXREA')):
            raise ImportError('teleoperation is deliberately unavailable')
sys.meta_path.insert(0, BlockTeleop())
from robot_config import config
from robot_config import config_edit, config_status, device_setup, load_identify
assert not any(name.startswith('robot_app') for name in sys.modules)
'''], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_existing_process_detection_is_read_only_and_checks_process_identity(tmp_path):
    import json
    import os
    from robot_runtime.processes import process_stat, require_idle
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
    from robot_runtime.processes import OwnedDrivers
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
    from robot_runtime.processes import OwnedDrivers, process_stat
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
