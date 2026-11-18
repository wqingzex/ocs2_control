"""parse_task_info: what the .info says decides dual_arm_mode, and a bad file stops the launch."""
import pytest

from launch_helpers.control.task_info import parse_task_info


def write(tmp_path, text):
    path = tmp_path / "task.info"
    path.write_text(text)
    return str(path)


def test_dual_arm_mode_comes_from_the_flag_or_the_second_end_effector(tmp_path):
    assert parse_task_info(write(tmp_path, "dualArmMode true\n"))[0] is True
    assert parse_task_info(write(tmp_path, 'eeFrame1 "right_ee"\n'))[0] is True
    assert parse_task_info(write(tmp_path, 'eeFrame "left_ee"\n'))[0] is False


def test_commented_out_settings_do_not_count(tmp_path):
    text = '; dualArmMode true\n  ; eeFrame1 "right_ee"\nbaseFrame "base_link" ; was "world"\n'
    dual, base_frame, _ = parse_task_info(write(tmp_path, text))
    assert dual is False and base_frame == "base_link"


def test_wheel_based_model_puts_the_targets_in_the_world_frame(tmp_path):
    dual, base_frame, marker = parse_task_info(write(tmp_path, 'baseFrame "base_link"\nmanipulatorModelType 1\n'))
    assert (base_frame, marker) == ("world", "world")


def test_a_missing_file_raises_instead_of_assuming_single_arm(tmp_path):
    with pytest.raises(FileNotFoundError):
        parse_task_info(str(tmp_path / "nope.info"))
