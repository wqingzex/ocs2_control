"""Load identification step and the readiness rules of the setup steps (no hardware, no ROS)."""
import json
import os
import time

import pytest

from fa_w2_payload import config_edit
from fa_w2_payload import load_identify as dyn
from fa_w2_payload.config_status import CONFIGURED, DEFAULT, NEEDED, camera_status, dyn_status, ee_status
from fa_w2_payload.config_edit import receipt

TEN = [0.5, -1.0, 2.0, 3.0, 0.001, 0.0, 0.0, 0.002, 0.0, 0.003]
CONFIG = """\
# my robot
robot:
  hardware: real
  end_effector:
    type: linkerhand_l6
    dyn_param:
      left: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]   # hand written
      right: []
"""


def loads(**sides):
    """A raw config with the load override of the given sides."""
    return {"robot": {"end_effector": {"dyn_param": sides}}}


class ScriptedIo:
    """Answers questions in order; a missing answer takes the default (like --yes)."""

    def __init__(self, *answers):
        self.answers, self.lines = list(answers), []

    def ask(self, question, default):
        self.lines.append(question)
        return self.answers.pop(0) if self.answers else default

    def info(self, message):
        self.lines.append(message)

    def warn(self, message):
        self.lines.append(message)

    error = warn


def write_config(tmp_path, text=CONFIG):
    path = tmp_path / "robot.yaml"
    path.write_text(text, encoding="utf-8")
    return str(path)


def wizard(ws_root, results):
    """A stand-in for the Marvin wizard: writes result_<arm>.json like the real one and returns its exit code."""
    calls = []

    def run(cmd, env, cwd):
        calls.append((list(cmd), env["TOOL_DYN_ARM"], cwd))
        arm = env["TOOL_DYN_ARM"]
        outcome = results.get(arm)
        if outcome is not None:
            os.makedirs(dyn.output_dir(ws_root), exist_ok=True)
            with open(dyn.result_path(ws_root, arm), "w", encoding="utf-8") as handle:
                json.dump({"arm": arm, "params": outcome}, handle)
        return 0 if outcome is not None else 1

    run.calls = calls
    return run


@pytest.mark.parametrize("raw, state", [
    ({}, DEFAULT),
    ({"robot": {"end_effector": {"type": "none"}}}, DEFAULT),
    ({"robot": {"end_effector": {"type": "xhand1"}}}, DEFAULT),
    ({"robot": {"hardware": "isaac"}}, DEFAULT),
    (loads(left=TEN, right=TEN), CONFIGURED),
    (loads(left=TEN), NEEDED),
    (loads(left=TEN[:9], right=TEN), NEEDED),
    (loads(left=["a"] * 10, right=TEN), NEEDED),
])
def test_load_readiness(raw, state):
    assert dyn_status(raw).state == state


def test_cameras_and_hand_are_ready_only_with_a_matching_receipt():
    raw = {"cameras": {"head": {"serial": "A"}}}
    assert camera_status(raw).needed and ee_status(raw).needed
    raw["setup"] = {"cameras": receipt(raw, "cameras"), "ee": receipt(raw, "ee")}
    assert camera_status(raw).state == CONFIGURED and ee_status(raw).state == CONFIGURED
    raw["cameras"]["head"]["serial"] = "OTHER"  # the mapping changed after it was confirmed
    assert camera_status(raw).needed
    assert camera_status({"cameras": {"enable": False}}).state == DEFAULT
    def hand(type, bus="usb_rs485"):
        return {"robot": {"end_effector": {"type": type, "interface": {"bus": bus}}}}

    assert ee_status(hand("none")).state == DEFAULT
    assert ee_status(hand("linkerhand_l6", "usb_can")).state == DEFAULT
    assert ee_status(hand("wuji_hand2", "ethernet")).state == DEFAULT


def test_nothing_is_identified_by_default(tmp_path):
    path, io = write_config(tmp_path), ScriptedIo()
    run = wizard(str(tmp_path), {"A": TEN, "B": TEN})
    assert dyn.step_dyn(path, str(tmp_path), io, runner=run) and run.calls == []
    assert open(path, encoding="utf-8").read() == CONFIG


def test_identifying_one_arm_writes_flow_style_values_and_keeps_the_old_ones(tmp_path):
    path = write_config(tmp_path)
    io = ScriptedIo("y", "y", "n")  # identify left, write it, skip right
    run = wizard(str(tmp_path), {"A": TEN})
    assert dyn.step_dyn(path, str(tmp_path), io, runner=run)
    cmd, arm, cwd = run.calls[0]
    assert cmd[-1] == "tool_dyn_identify_wizard" and arm == "A" and cwd == str(tmp_path)
    text = open(path, encoding="utf-8").read()
    assert "# my robot" in text and "left: [0.5, -1.0, 2.0, 3.0, 0.001, 0.0, 0.0, 0.002, 0.0, 0.003]" in text
    assert "right: []" in text
    previous = [f for f in os.listdir(dyn.output_dir(str(tmp_path))) if f.startswith("previous_left_")]
    assert json.load(open(os.path.join(dyn.output_dir(str(tmp_path)), previous[0])))["params"][:3] == [1.0, 2.0, 3.0]


def test_a_result_that_is_not_written_leaves_the_config_alone(tmp_path):
    path = write_config(tmp_path)
    io = ScriptedIo("y", "n", "n")
    assert dyn.step_dyn(path, str(tmp_path), io, runner=wizard(str(tmp_path), {"A": TEN}))
    assert open(path, encoding="utf-8").read() == CONFIG


def test_a_failed_wizard_is_reported_and_writes_nothing(tmp_path):
    path = write_config(tmp_path)
    io = ScriptedIo("n", "y")  # right arm only
    assert not dyn.step_dyn(path, str(tmp_path), io, runner=wizard(str(tmp_path), {}))
    assert any("没有得到结果" in line for line in io.lines)
    assert open(path, encoding="utf-8").read() == CONFIG


def test_stale_and_malformed_results_are_ignored(tmp_path):
    result = dyn.result_path(str(tmp_path), "A")
    os.makedirs(os.path.dirname(result))
    for params in (TEN[:9], TEN + [1.0], ["x"] * 10, [float("nan")] * 10, None):
        with open(result, "w", encoding="utf-8") as handle:
            json.dump({"params": params}, handle)
        assert dyn.read_result(result, 0) is None
    with open(result, "w", encoding="utf-8") as handle:
        json.dump({"params": TEN}, handle)
    assert dyn.read_result(result, 0) == TEN
    assert dyn.read_result(result, time.time() + 60) is None  # older than the run that just finished
    assert dyn.read_result(str(tmp_path / "missing.json"), 0) is None


def test_result_written_during_the_clock_granularity_of_the_start_time_is_accepted(tmp_path):
    """File timestamps can lag time.time() by up to a millisecond (overlayfs): that must not drop a fresh result."""
    result = dyn.result_path(str(tmp_path), "A")
    os.makedirs(os.path.dirname(result))
    with open(result, "w", encoding="utf-8") as handle:
        json.dump({"params": TEN}, handle)
    started = os.path.getmtime(result) + 0.001
    assert dyn.read_result(result, started) == TEN


def test_simulation_skips_the_step_without_asking(tmp_path):
    path = write_config(tmp_path, "robot: {hardware: mock_components}\n")
    io = ScriptedIo("y", "y")
    run = wizard(str(tmp_path), {"A": TEN})
    assert dyn.step_dyn(path, str(tmp_path), io, runner=run) and run.calls == [] and len(io.answers) == 2


def test_reset_empties_both_overrides_and_keeps_the_old_values(tmp_path):
    path = write_config(tmp_path)
    assert dyn.reset_dyn(path, str(tmp_path), ScriptedIo())
    text = open(path, encoding="utf-8").read()
    assert "left: []" in text and "right: []" in text and "# my robot" in text
    assert [f for f in os.listdir(dyn.output_dir(str(tmp_path))) if f.startswith("previous_left_")]
    assert dyn_status(config_edit.load_rt(path)[1]).state == DEFAULT


def test_set_float_list_keeps_comments_and_creates_missing_sections(tmp_path):
    path = write_config(tmp_path, "# top\nrobot: {hardware: real}\n")
    config_edit.set_float_list(path, "robot.end_effector.dyn_param", "left", [1, 2])
    text = open(path, encoding="utf-8").read()
    assert text.startswith("# top") and "left: [1.0, 2.0]" in text
