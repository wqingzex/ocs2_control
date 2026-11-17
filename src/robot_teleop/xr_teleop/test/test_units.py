"""The PC Service unit: its output is kept in its own log, and a service that was running already makes none."""
from robot_runtime.units import Unit
from xr_teleop.units import PcServiceDriver


def fake_pc_service(monkeypatch, tmp_path, already_running=False):
    """A runService.sh that prints, and a process table that shows the service once the script has run."""
    from xr_teleop import pc_service
    marker = tmp_path / "service_started"
    script = tmp_path / "runService.sh"
    script.write_text(f"#!/bin/bash\necho 'service says hello'\necho 'service complains' >&2\ntouch {marker}\n")
    monkeypatch.setattr(pc_service, "running_pids", lambda: {4242} if already_running or marker.exists() else set())
    return script


def test_the_pc_service_output_is_kept_in_its_own_log(make_supervisor, tmp_path, monkeypatch):
    import time
    script = fake_pc_service(monkeypatch, tmp_path)
    sup = make_supervisor({"pc_service": Unit("pc_service", "PC Service", (), driver=PcServiceDriver(str(script)))})
    sup.start("pc_service")
    latest = tmp_path / "logs" / "pc_service" / "latest.log"
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline and "complains" not in (latest.read_text() if latest.exists() else ""):
        time.sleep(0.05)
    text = latest.read_text()
    assert "service says hello" in text and "service complains" in text      # stdout and stderr, as they are
    assert latest.is_symlink() and sup.log_path("pc_service") is not None
    assert sup.status("pc_service").log is not None and sup.log_tail("pc_service")[0] == "service says hello"


def test_a_pc_service_that_was_running_already_makes_no_log(make_supervisor, tmp_path, monkeypatch):
    script = fake_pc_service(monkeypatch, tmp_path, already_running=True)
    sup = make_supervisor({"pc_service": Unit("pc_service", "PC Service", (), driver=PcServiceDriver(str(script)))})
    sup.start("pc_service")
    assert not (tmp_path / "logs" / "pc_service" / "latest.log").exists()


def test_a_zombie_service_is_not_running(monkeypatch):
    """A PC Service that exited stays a zombie when PID 1 does not reap: it must not stop start() from launching one."""
    import os
    import time
    from types import SimpleNamespace
    from xr_teleop import pc_service
    child = os.fork()
    if child == 0:
        os._exit(0)
    try:
        for _ in range(50):  # wait until the child has exited and is a zombie
            if pc_service._is_zombie(child):
                break
            time.sleep(0.02)
        assert pc_service._is_zombie(child) and not pc_service._is_zombie(os.getpid())
        monkeypatch.setattr(pc_service.subprocess, "run", lambda *a, **k: SimpleNamespace(stdout=f"{child}\n{os.getpid()}\n"))
        assert pc_service.running_pids() == {os.getpid()}
        assert pc_service._is_zombie(99999999)  # gone counts as not running
    finally:
        os.waitpid(child, 0)


def test_the_bridge_does_not_inherit_a_proxy():
    """gRPC sends even the connection to the PC Service on 127.0.0.1 through http_proxy."""
    from xr_teleop.config import DEFAULTS
    from xr_teleop.units import xr_units
    units = xr_units({"xr": dict(DEFAULTS["xr"])})
    assert {"http_proxy", "https_proxy", "HTTP_PROXY", "HTTPS_PROXY"} <= set(units["xr"].unset_env)


def test_the_bridge_gets_no_key_scheme_parameter():
    """One key table: the bridge has no control_scheme parameter any more."""
    from xr_teleop.config import DEFAULTS
    from xr_teleop.units import xr_units
    cmd = xr_units({"xr": dict(DEFAULTS["xr"])})["xr"].cmd
    assert not any("control_scheme" in arg for arg in cmd)
    assert "control_scheme" not in DEFAULTS["xr"]
