"""start.sh bus-probe: which bus_probe command lines come out of the config, and when it refuses (no ROS needed)."""
import pytest
import yaml

from robot_app import bus_probe, config
from robot_config.testing import full_config

LEFT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_LEFT-if00-port0"
RIGHT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_RIGHT-if00-port0"


THREE_CAMERAS = {"head": {"serial": "A"}, "left_wrist": {"serial": "B"}, "right_wrist": {"serial": "C"}}


def sections(device="xhand1", bus="usb_rs485", left=LEFT, right=RIGHT):
    return {"cameras": THREE_CAMERAS, "robot": {"end_effector": {
        "type": device, "interface": {"bus": bus, "left": {"port": left}, "right": {"port": right}}}}}


def make_cfg(device="xhand1", **kwargs):
    return config.validate(full_config(**sections(device, **kwargs)))


def test_both_sides_give_one_command_each_with_the_port_from_the_config():
    commands = bus_probe.probe_commands(make_cfg("xhand1"))
    assert commands == [
        ["ros2", "run", "end_effector_hardware", "bus_probe", "--device", "xhand1", "--side", "left", "--port", LEFT],
        ["ros2", "run", "end_effector_hardware", "bus_probe", "--device", "xhand1", "--side", "right", "--port", RIGHT]]


def test_one_side_and_extra_options_are_passed_through():
    commands = bus_probe.probe_commands(make_cfg("linkerhand_l6"), "right", ["--duration", "5", "--yes"])
    assert commands == [["ros2", "run", "end_effector_hardware", "bus_probe", "--device", "linkerhand_l6",
                         "--side", "right", "--port", RIGHT, "--duration", "5", "--yes"]]


@pytest.mark.parametrize("device,bus", [("none", "usb_rs485"), ("wuji_hand2", "ethernet"), ("jodell_rg75", "arm_rs485")])
def test_an_end_effector_without_a_usb_bus_is_refused(device, bus):
    with pytest.raises(bus_probe.ProbeError, match=device):
        bus_probe.probe_commands(make_cfg(device, bus=bus))


def test_a_bus_that_only_exists_in_the_arm_driver_is_refused():
    with pytest.raises(bus_probe.ProbeError, match="arm_rs485"):
        bus_probe.probe_commands(make_cfg("linkerhand_l6", bus="arm_rs485"))


def test_a_side_without_a_port_is_refused_and_says_what_to_run():
    with pytest.raises(bus_probe.ProbeError, match=r"right\.port.*setup\.sh ee"):
        bus_probe.probe_commands(make_cfg("xhand1", right=""))
    assert len(bus_probe.probe_commands(make_cfg("xhand1", right=""), "left")) == 1


def write_config(tmp_path):
    path = tmp_path / "robot.local.yaml"
    path.write_text(yaml.safe_dump(full_config(**sections())))
    return str(path)


def test_main_refuses_while_the_controller_runs(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(bus_probe, "controller_running", lambda: True)
    monkeypatch.setattr(bus_probe.subprocess, "run", lambda *a, **k: pytest.fail("must not start the probe"))
    assert bus_probe.main(["--config", write_config(tmp_path)]) == 2
    assert "start.sh down" in capsys.readouterr().err


def test_main_runs_each_side_and_returns_the_worst_exit_code(tmp_path, monkeypatch):
    monkeypatch.setattr(bus_probe, "controller_running", lambda: False)
    started = []

    class Done:
        def __init__(self, code):
            self.returncode = code

    def fake_run(command, check):
        started.append(command)
        return Done(0 if "left" in command else 1)

    monkeypatch.setattr(bus_probe.subprocess, "run", fake_run)
    assert bus_probe.main(["--config", write_config(tmp_path), "--duration", "3"]) == 1
    assert [c[c.index("--side") + 1] for c in started] == ["left", "right"]
    assert all(c[-2:] == ["--duration", "3"] for c in started)


def test_main_reports_a_bad_config(tmp_path, capsys):
    path = tmp_path / "robot.local.yaml"
    path.write_text("robot: {entry: whole_body}\n")
    assert bus_probe.main(["--config", str(path)]) == 2
    assert "配置错误" in capsys.readouterr().err
