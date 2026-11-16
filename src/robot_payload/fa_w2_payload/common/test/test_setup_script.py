"""Check setup.sh's complete sequence without installing packages or accessing hardware."""
import os
import subprocess
from pathlib import Path

import pytest


def run_init(tmp_path, *, container, failed_step="", args=(), config_text="{}\n", stub_marvin=True, stdin=None, fake_ws=False):
    script = Path(__file__).resolve().parents[5] / "scripts" / "setup.sh"
    config = tmp_path / "config.yaml"
    config.write_text(config_text)
    record = tmp_path / "steps.txt"
    record.write_text("")
    harness = '''source "$1"
shift
CONFIG="$W2_TEST_CONFIG"
[[ -z "$W2_TEST_WS" ]] || WS_DIR="$W2_TEST_WS"
is_container() { [[ "$W2_TEST_CONTAINER" == 1 ]]; }
record_step() { printf '%s\\n' "$1" >> "$W2_TEST_STEPS"; }
'''
    for function, step in (
        ("step_deps", "deps"), ("host_udev", "udev"), ("host_sudoers", "can"),
        ("host_pc_service", "sdk"), ("step_marvin", "marvin"), ("step_build", "build"), ("step_cameras", "cameras"),
        ("step_hand", "hand"), ("step_dyn", "dyn"), ("step_xr", "xr"), ("step_summary", "summary"),
    ):
        if function == "step_marvin" and not stub_marvin:
            continue
        harness += f'{function}() {{ record_step {step}; [[ "$W2_TEST_FAIL" != {step} ]]; }}\n'
    harness += 'main "$@"\n'
    env = dict(os.environ, W2_TEST_CONFIG=str(config), W2_TEST_STEPS=str(record), W2_TEST_WS=str(tmp_path) if fake_ws else "",
               W2_TEST_CONTAINER="1" if container else "0", W2_TEST_FAIL=failed_step)
    result = subprocess.run(["bash", "-c", harness, "--", str(script), *args], env=env, input=stdin,
                            text=True, capture_output=True, timeout=10)
    return result, record.read_text().splitlines()


@pytest.mark.parametrize("container", [False, True])
def test_default_init_keeps_camera_hand_and_xr_steps(tmp_path, container):
    result, steps = run_init(tmp_path, container=container)
    assert result.returncode == 0, result.stdout + result.stderr
    host_setup = [] if container else ["udev", "can"]
    assert steps == ["deps", *host_setup, "sdk", "marvin", "build", "cameras", "hand", "xr", "summary"]
    assert "9/9" in result.stdout and "dyn: 默认值" in result.stdout


@pytest.mark.parametrize("failed_step", ["hand", "xr", "summary"])
def test_unfinished_configuration_does_not_report_init_success(tmp_path, failed_step):
    result, steps = run_init(tmp_path, container=True, failed_step=failed_step)
    assert result.returncode == 1
    assert steps == ["deps", "sdk", "marvin", "build", "cameras", "hand", "xr", "summary"]
    assert "初始化尚未完成" in result.stderr
    assert "下一步：./scripts/start.sh" not in result.stdout


@pytest.mark.parametrize("container", [False, True])
def test_camera_failure_exits_before_hand_xr_or_summary(tmp_path, container):
    result, steps = run_init(tmp_path, container=container, failed_step="cameras")
    assert result.returncode == 1
    host_setup = [] if container else ["udev", "can"]
    assert steps == ["deps", *host_setup, "sdk", "marvin", "build", "cameras"]
    assert "初始化结果" not in result.stdout
    assert "下一步：./scripts/start.sh" not in result.stdout


@pytest.mark.parametrize("command,expected", [
    ("cam", "cameras"), ("hand", "hand"), ("xr", "xr"),
    ("build", "build"), ("marvin", "marvin"),
    ("deps", "deps"), ("host", "sdk"), ("check", "summary"),
    ("cameras", "cameras"), ("summary", "summary"),
])
def test_single_step_never_runs_other_stages(tmp_path, command, expected):
    result, steps = run_init(tmp_path, container=True, args=(command,))
    assert result.returncode == 0, result.stdout + result.stderr
    assert steps == [expected]
    assert "1/1" in result.stdout
    assert "所选步骤已完成" in result.stdout


def test_camera_retry_does_not_repeat_dependency_install_or_build(tmp_path):
    failure, _ = run_init(tmp_path, container=True, failed_step="cameras")
    assert failure.returncode == 1
    retry, steps = run_init(tmp_path, container=True, args=("cam",))
    assert retry.returncode == 0
    assert steps == ["cameras"]


@pytest.mark.parametrize("container", [False, True])
@pytest.mark.parametrize("command,index", [
    ("deps", 0), ("host", 1), ("marvin", 2), ("build", 3), ("cam", 4),
    ("hand", 5), ("xr", 7), ("check", 8),
])
def test_from_runs_only_the_selected_step_and_remaining_stages(tmp_path, container, command, index):
    result, steps = run_init(tmp_path, container=container, args=(command, "--from"))
    assert result.returncode == 0, result.stdout + result.stderr
    host = ["sdk"] if container else ["udev", "can", "sdk"]
    stages = [["deps"], host, ["marvin"], ["build"], ["cameras"], ["hand"], [], ["xr"], ["summary"]]  # dyn: default usable
    assert steps == [step for stage in stages[index:] for step in stage]
    assert f"{index + 1}/9" in result.stdout
    assert "9/9" in result.stdout
    assert "下一步：./scripts/start.sh" in result.stdout


@pytest.mark.parametrize("args", [("--from",), ("cam", "xr", "--from")])
def test_from_rejects_missing_or_multiple_start_steps_without_running_anything(tmp_path, args):
    result, steps = run_init(tmp_path, container=True, args=args)
    assert result.returncode == 1
    assert steps == []
    assert "--from" in result.stderr


def test_from_does_not_continue_after_camera_failure(tmp_path):
    result, steps = run_init(tmp_path, container=True, failed_step="cameras", args=("cam", "--from"))
    assert result.returncode == 1
    assert steps == ["cameras"]


def test_from_works_with_existing_flags(tmp_path):
    result, steps = run_init(tmp_path, container=True, args=("build", "--from", "--yes", "--minimal"))
    assert result.returncode == 0
    assert steps == ["build", "cameras", "hand", "xr", "summary"]


def test_cancelled_xr_step_stops_init_without_running_final_check(tmp_path):
    script = Path(__file__).resolve().parents[5] / "scripts" / "setup.sh"
    config = tmp_path / "config.yaml"
    config.write_text("{}\n")
    harness = '''source "$1"
CONFIG="$W2_TEST_CONFIG"
source_ros() { :; }
python3() { return 130; }
step_summary() { printf 'UNEXPECTED_CHECK\\n'; }
main xr --from
'''
    result = subprocess.run(["bash", "-c", harness, "--", str(script)],
                            env=dict(os.environ, W2_TEST_CONFIG=str(config)),
                            text=True, capture_output=True, timeout=5)
    assert result.returncode == 130
    assert "UNEXPECTED_CHECK" not in result.stdout


# ---- steps that are configured, or have a usable default, are not asked again --------------------
def configured_config(**overrides):
    """A config whose camera and hand mappings carry valid confirmation receipts and a paired headset."""
    import yaml
    from fa_w2_payload.config_edit import receipt
    raw = {"cameras": {"head": {"serial": "A"}, "left_wrist": {"serial": "B"}, "right_wrist": {"serial": "C"}},
           "xr": {"device_id": "PICO-1"}, "hardware": {"marvin_firmware": 338}}
    raw.update(overrides)
    raw["setup"] = {"cameras": receipt(raw, "cameras"), "hand": receipt(raw, "hand")}
    return yaml.safe_dump(raw)


def test_configured_steps_are_skipped_in_the_full_flow(tmp_path):
    result, steps = run_init(tmp_path, container=True, config_text=configured_config())
    assert result.returncode == 0, result.stdout + result.stderr
    assert steps == ["deps", "sdk", "build", "summary"]
    for line in ("已选择 338", "相机映射已确认", "左右手实物已确认", "已绑定头显 PICO-1", "使用 linkerhand_l6 的内置辨识值"):
        assert line in result.stdout
    assert result.stdout.count("已跳过") == 5


def test_force_configures_everything_again(tmp_path):
    result, steps = run_init(tmp_path, container=True, config_text=configured_config(), args=("--force",))
    assert result.returncode == 0, result.stdout + result.stderr
    assert steps == ["deps", "sdk", "marvin", "build", "cameras", "hand", "dyn", "xr", "summary"]
    assert "已跳过" not in result.stdout


@pytest.mark.parametrize("step, expected", [("cam", "cameras"), ("hand", "hand"), ("dyn", "dyn"), ("xr", "xr")])
def test_a_single_step_always_runs_even_when_it_is_configured(tmp_path, step, expected):
    result, steps = run_init(tmp_path, container=True, config_text=configured_config(), args=(step,))
    assert result.returncode == 0, result.stdout + result.stderr
    assert steps == [expected]


def test_a_half_filled_load_is_not_a_usable_default(tmp_path):
    text = configured_config(defaults={"left_dyn_param": [1.0] * 10})
    result, steps = run_init(tmp_path, container=True, config_text=text)
    assert steps == ["deps", "sdk", "build", "dyn", "summary"]  # the reason (only the left arm is filled) is config_status's, tested there


def test_simulation_and_no_hand_need_no_load_identification(tmp_path):
    for robot, reason in (({"hardware": "mock_components"}, "不是真机"), ({"end_effector": "none"}, "没有末端")):
        text = configured_config(robot=robot)
        result, steps = run_init(tmp_path, container=True, config_text=text)
        assert "dyn" not in steps and reason in result.stdout


def test_reset_is_only_for_the_dyn_step(tmp_path):
    result, steps = run_init(tmp_path, container=True, args=("cam", "--reset"))
    assert result.returncode != 0 and steps == [] and "--reset 只用于 dyn" in result.stderr
    result, steps = run_init(tmp_path, container=True, args=("dyn", "--reset"))
    assert result.returncode == 0 and steps == ["dyn"]


def test_the_old_build_workspace_command_is_gone(tmp_path):
    result, steps = run_init(tmp_path, container=True, args=("build-workspace",))
    assert result.returncode != 0 and steps == [] and "未知参数" in result.stderr


# ---- the Marvin (Tianji) SDK version: hardware.marvin_firmware, 338 or 343 --------------------------
def built_with(tmp_path, firmware):
    cache = tmp_path / "build" / "marvin_hardware" / "CMakeCache.txt"
    cache.parent.mkdir(parents=True)
    cache.write_text(f"CMAKE_BUILD_TYPE:STRING=Release\nMARVIN_FIRMWARE:STRING={firmware}\n")


def call(tmp_path, function, config_text="{}\n"):
    """Run one setup.sh function against a temporary config and workspace root."""
    script = Path(__file__).resolve().parents[5] / "scripts" / "setup.sh"
    config = tmp_path / "config.yaml"
    config.write_text(config_text)
    harness = 'source "$1"; CONFIG="$W2_TEST_CONFIG"; WS_DIR="$W2_TEST_WS"; ' + function
    return subprocess.run(["bash", "-c", harness, "--", str(script)], text=True, capture_output=True, timeout=10,
                          env=dict(os.environ, W2_TEST_CONFIG=str(config), W2_TEST_WS=str(tmp_path)))


def test_marvin_is_asked_and_the_choice_is_written_to_the_config(tmp_path):
    result, _ = run_init(tmp_path, container=True, args=("marvin",), stub_marvin=False, stdin="338\n",
                         config_text="# 本机配置\nhardware:\n  body_can_interface: can0   # 腰、头\n")
    assert result.returncode == 0, result.stdout + result.stderr
    text = (tmp_path / "config.yaml").read_text()
    assert "marvin_firmware: 338" in text and "# 腰、头" in text   # comments are kept
    assert "急停" in result.stdout + result.stderr                  # the prompt says what a wrong choice does


def test_marvin_rejects_an_answer_that_is_not_338_or_343(tmp_path):
    result, _ = run_init(tmp_path, container=True, args=("marvin",), stub_marvin=False, stdin="340\n")
    assert result.returncode != 0 and "338 或 343" in result.stderr
    assert "marvin_firmware" not in (tmp_path / "config.yaml").read_text()


@pytest.mark.parametrize("built, expected", [(None, "343"), ("338", "338")])
def test_marvin_yes_keeps_the_version_already_built_or_takes_343(tmp_path, built, expected):
    if built:
        built_with(tmp_path, built)
    result, _ = run_init(tmp_path, container=True, args=("marvin", "--yes"), stub_marvin=False, fake_ws=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert f"marvin_firmware: {expected}" in (tmp_path / "config.yaml").read_text()


def test_build_passes_the_chosen_version_to_cmake(tmp_path):
    result = call(tmp_path, "marvin_cmake_arg", "hardware: {marvin_firmware: 338}\n")
    assert result.returncode == 0 and result.stdout == "-DMARVIN_FIRMWARE=338" and result.stderr == ""


def test_build_without_a_choice_keeps_the_built_version_and_warns(tmp_path):
    built_with(tmp_path, "338")
    result = call(tmp_path, "marvin_cmake_arg")
    assert result.returncode == 0 and result.stdout == "-DMARVIN_FIRMWARE=338"
    assert "./scripts/setup.sh marvin" in result.stderr


def test_an_invalid_version_in_the_config_is_an_error(tmp_path):
    result = call(tmp_path, "marvin_cmake_arg", "hardware: {marvin_firmware: 340}\n")
    assert result.returncode != 0 and "hardware.marvin_firmware" in result.stderr and "338 或 343" in result.stderr
