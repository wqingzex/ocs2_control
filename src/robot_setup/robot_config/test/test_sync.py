"""setup.sh config: a new file from the template, and the keys an old file lacks, with values and comments kept."""
import io

import pytest
import yaml

from robot_config import sync
from robot_config.config import validate
from robot_config.device_setup import Io
from robot_config.template import template_for


class Quiet(Io):
    def __init__(self, answers=()):
        super().__init__(out=io.StringIO())
        self.answers = list(answers)

    def ask(self, question, default):
        return self.answers.pop(0) if self.answers else default


def read(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_a_new_file_is_the_template_of_the_entry(tmp_path):
    path = tmp_path / "config" / "robot.local.yaml"
    assert sync.run(str(path), "dual_arm", Quiet())
    assert read(path) == template_for("dual_arm")
    assert "[必改]" in path.read_text(encoding="utf-8")


def test_a_new_file_asks_for_the_entry_when_none_is_given(tmp_path):
    path = tmp_path / "robot.local.yaml"
    assert sync.run(str(path), None, Quiet(["split_body"]))
    assert read(path)["robot"]["entry"] == "split_body"
    other = tmp_path / "other.yaml"
    assert sync.run(str(other), None, Quiet())  # --yes / enter: the default
    assert read(other)["robot"]["entry"] == "whole_body"


def test_missing_keys_are_added_and_nothing_else_changes(tmp_path):
    path = tmp_path / "robot.local.yaml"
    sync.run(str(path), "whole_body", Quiet())
    text = path.read_text(encoding="utf-8").replace("  frame_timeout_ms: 300", "  frame_timeout_ms: 450")
    text = "\n".join(line for line in text.split("\n") if "stats_period_s" not in line and "joystick" not in line
                     and 'device: ""' not in line)  # drop two keys and a whole section
    path.write_text(text + "\n# my own note\n", encoding="utf-8")
    assert sync.status(str(path))[0]

    assert sync.run(str(path), None, Quiet())

    after = read(path)
    assert after["robot"]["body"]["interface"]["stats_period_s"] == 0 and after["joystick"] == {"device": ""}
    assert after["xr"]["frame_timeout_ms"] == 450  # the value in the file wins
    assert "# my own note" in path.read_text(encoding="utf-8")
    assert (tmp_path / "robot.local.yaml.bak").exists()
    assert not sync.status(str(path))[0]


def test_a_complete_file_is_left_alone(tmp_path):
    path = tmp_path / "robot.local.yaml"
    sync.run(str(path), "whole_body", Quiet())
    before = path.read_bytes()
    assert sync.run(str(path), None, Quiet())
    assert path.read_bytes() == before and not (tmp_path / "robot.local.yaml.bak").exists()


def test_a_key_that_is_not_in_the_template_is_reported_not_deleted(tmp_path):
    path = tmp_path / "robot.local.yaml"
    sync.run(str(path), "whole_body", Quiet())
    path.write_text(path.read_text(encoding="utf-8") + "platform:\n  chassis: linkhou_s2\n", encoding="utf-8")
    out = Quiet()
    assert not sync.run(str(path), None, out)
    assert "platform.chassis" in out.out.getvalue() and "platform:" in path.read_text(encoding="utf-8")
    assert sync.status(str(path))[0] and "多余" in sync.status(str(path))[1]


def test_free_form_contents_are_never_missing_or_unknown(tmp_path):
    path = tmp_path / "robot.local.yaml"
    sync.run(str(path), "whole_body", Quiet())
    doc = read(path)
    doc["control"]["patch"] = {"body_joint_controller": {"ros__parameters": {"home_1": [0.0]}}}
    doc["setup"] = {"cameras": {"confirmed": True}}
    path.write_text(yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")
    assert not sync.status(str(path))[0]


def test_the_entry_of_the_file_cannot_be_changed_by_the_option(tmp_path):
    path = tmp_path / "robot.local.yaml"
    sync.run(str(path), "whole_body", Quiet())
    out = Quiet()
    assert not sync.run(str(path), "dual_arm", out)
    assert "robot.entry" in out.out.getvalue()


def test_status_of_a_missing_file_or_a_file_without_an_entry(tmp_path):
    assert sync.status(str(tmp_path / "none.yaml")) == (True, "还没有本机配置")
    path = tmp_path / "robot.local.yaml"
    path.write_text("xr: {enable: true}\n")
    assert sync.status(str(path))[0]


def test_a_synced_old_file_validates(tmp_path):
    """A file from before most keys existed becomes valid after the sync."""
    path = tmp_path / "robot.local.yaml"
    path.write_text(yaml.safe_dump({"robot": {"entry": "whole_body", "hardware": "mock_components"},
                                    "cameras": {"enable": False}}), encoding="utf-8")
    assert sync.run(str(path), None, Quiet())
    assert validate(read(path))["robot"]["hardware"] == "mock_components"


def test_the_command_line(tmp_path, capsys):
    path = tmp_path / "robot.local.yaml"
    assert sync.main(["--config", str(path), "--entry", "split_body", "--yes"]) == 0
    assert sync.main(["--config", str(path), "--status"]) == 0
    assert "已配置" in capsys.readouterr().out
    path.write_text("robot: {entry: whole_body}\n")
    assert sync.main(["--config", str(path), "--status"]) == 1
