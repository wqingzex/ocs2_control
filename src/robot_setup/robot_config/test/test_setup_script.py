"""Check setup.sh's complete sequence without installing packages or accessing hardware."""
import os
import subprocess
from pathlib import Path

import pytest


def run_init(tmp_path, *, container, failed_step="", args=(), config_text="{}\n", stub_marvin=True, stdin=None, fake_ws=False):
    script = Path(__file__).resolve().parents[4] / "scripts" / "setup.sh"
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
        ("step_deps", "deps"), ("step_config", "config"), ("host_udev", "udev"), ("host_sudoers", "can"),
        ("host_pc_service", "sdk"), ("step_marvin", "marvin"), ("step_build", "build"), ("step_cameras", "cameras"),
        ("step_home", "home"), ("step_ee", "ee"), ("step_dyn", "dyn"), ("step_xr", "xr"), ("step_summary", "summary"),
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
    result, steps = run_init(tmp_path, container=container, args=("--yes",))
    assert result.returncode == 0, result.stdout + result.stderr
    host_setup = [] if container else ["udev", "can"]
    assert steps == ["config", "deps", *host_setup, "sdk", "marvin", "build", "cameras", "ee", "xr", "home", "summary"]
    assert "11/11" in result.stdout and "dyn: 默认值" in result.stdout


@pytest.mark.parametrize("failed_step", ["ee", "xr", "summary"])
def test_unfinished_configuration_does_not_report_init_success(tmp_path, failed_step):
    result, steps = run_init(tmp_path, container=True, failed_step=failed_step, args=("--yes",))
    assert result.returncode == 1
    assert steps == ["config", "deps", "sdk", "marvin", "build", "cameras", "ee", "xr", "home", "summary"]
    assert "初始化尚未完成" in result.stderr
    assert "下一步：./scripts/start.sh" not in result.stdout


@pytest.mark.parametrize("container", [False, True])
def test_camera_failure_exits_before_hand_xr_or_summary(tmp_path, container):
    result, steps = run_init(tmp_path, container=container, failed_step="cameras", args=("--yes",))
    assert result.returncode == 1
    host_setup = [] if container else ["udev", "can"]
    assert steps == ["config", "deps", *host_setup, "sdk", "marvin", "build", "cameras"]
    assert "初始化结果" not in result.stdout
    assert "下一步：./scripts/start.sh" not in result.stdout


@pytest.mark.parametrize("command,expected", [
    ("cam", "cameras"), ("ee", "ee"), ("xr", "xr"),
    ("build", "build"), ("marvin", "marvin"), ("home", "home"),
    ("deps", "deps"), ("config", "config"), ("host", "sdk"), ("check", "summary"),
    ("cameras", "cameras"), ("summary", "summary"),
])
def test_single_step_never_runs_other_stages(tmp_path, command, expected):
    result, steps = run_init(tmp_path, container=True, args=(command,))
    assert result.returncode == 0, result.stdout + result.stderr
    assert steps == [expected]
    assert "1/1" in result.stdout
    assert "所选步骤已完成" in result.stdout


def test_camera_retry_does_not_repeat_dependency_install_or_build(tmp_path):
    failure, _ = run_init(tmp_path, container=True, failed_step="cameras", args=("--yes",))
    assert failure.returncode == 1
    retry, steps = run_init(tmp_path, container=True, args=("cam",))
    assert retry.returncode == 0
    assert steps == ["cameras"]


@pytest.mark.parametrize("container", [False, True])
@pytest.mark.parametrize("command,index", [
    ("config", 0), ("deps", 1), ("host", 2), ("marvin", 3), ("build", 4), ("cam", 5),
    ("ee", 6), ("xr", 8), ("home", 9), ("check", 10),
])
def test_from_runs_only_the_selected_step_and_remaining_stages(tmp_path, container, command, index):
    result, steps = run_init(tmp_path, container=container, args=(command, "--from", "--yes"))
    assert result.returncode == 0, result.stdout + result.stderr
    host = ["sdk"] if container else ["udev", "can", "sdk"]
    stages = [["config"], ["deps"], host, ["marvin"], ["build"], ["cameras"], ["ee"], [], ["xr"], ["home"], ["summary"]]
    assert steps == [step for stage in stages[index:] for step in stage]
    assert f"{index + 1}/11" in result.stdout
    assert "11/11" in result.stdout
    assert "下一步：./scripts/start.sh" in result.stdout


@pytest.mark.parametrize("args", [("--from",), ("cam", "xr", "--from")])
def test_from_rejects_missing_or_multiple_start_steps_without_running_anything(tmp_path, args):
    result, steps = run_init(tmp_path, container=True, args=args)
    assert result.returncode == 1
    assert steps == []
    assert "--from" in result.stderr


def test_from_does_not_continue_after_camera_failure(tmp_path):
    result, steps = run_init(tmp_path, container=True, failed_step="cameras", args=("cam", "--from", "--yes"))
    assert result.returncode == 1
    assert steps == ["cameras"]


def test_from_works_with_existing_flags(tmp_path):
    result, steps = run_init(tmp_path, container=True, args=("build", "--from", "--yes"))
    assert result.returncode == 0
    assert steps == ["build", "cameras", "ee", "xr", "home", "summary"]


def test_cancelled_xr_step_stops_init_without_running_final_check(tmp_path):
    script = Path(__file__).resolve().parents[4] / "scripts" / "setup.sh"
    config = tmp_path / "config.yaml"
    config.write_text("{}\n")
    harness = '''source "$1"
CONFIG="$W2_TEST_CONFIG"
source_ros() { :; }
load_build_selection() { BUILD_TELEOP=xr; }
stop_old_xr_setup() { :; }
python3() { return 130; }
step_summary() { printf 'UNEXPECTED_CHECK\\n'; }
main xr --from --yes
'''
    result = subprocess.run(["bash", "-c", harness, "--", str(script)],
                            env=dict(os.environ, W2_TEST_CONFIG=str(config)),
                            text=True, capture_output=True, timeout=5)
    assert result.returncode == 130
    assert "UNEXPECTED_CHECK" not in result.stdout


# ---- the interactive wizard asks before every step ------------------------------------------------
def _merge(target, values):
    for key, value in values.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _merge(target[key], value)
        else:
            target[key] = value


def configured_config(**overrides):
    """A complete config whose camera and hand mappings carry valid confirmation receipts and a paired headset."""
    import yaml
    from robot_config.config_file import receipt
    from robot_config.testing import full_config
    raw = full_config(cameras={"head": {"serial": "A"}, "left_wrist": {"serial": "B"}, "right_wrist": {"serial": "C"}},
                      xr={"device_id": "PICO-1"}, robot={"arm": {"interface": {"firmware": 338}}})
    for key, value in overrides.items():  # a given section is merged into the section, not replacing it
        _merge(raw[key], value)
    raw["setup"] = {"cameras": receipt(raw, "cameras"), "ee": receipt(raw, "ee")}
    return yaml.safe_dump(raw)


@pytest.mark.parametrize("args,stdin", [((), "\n" * 11), (("--yes",), None)])
def test_configured_steps_default_to_skip_in_the_full_flow(tmp_path, args, stdin):
    result, steps = run_init(tmp_path, container=True, config_text=configured_config(), args=args, stdin=stdin)
    assert result.returncode == 0, result.stdout + result.stderr
    assert steps == ["deps", "sdk", "build", "home", "summary"]
    for line in ("配置完整", "已选择 338", "相机映射已确认", "左右手实物已确认", "已绑定头显 PICO-1", "使用 linkerhand_l6 的内置辨识值"):
        assert line in result.stdout
    assert result.stdout.count("已跳过") == 6
    assert result.stdout.count("是否执行本步") == (0 if args else 11)


def test_full_wizard_can_run_a_step_with_defaults_and_skip_other_steps(tmp_path):
    answers = ["n"] * 7 + ["y", "n", "y", "n"]
    result, steps = run_init(tmp_path, container=True, config_text=configured_config(), stdin="\n".join(answers) + "\n")
    assert result.returncode == 0, result.stdout + result.stderr
    assert steps == ["dyn", "home"]
    assert result.stdout.count("是否执行本步") == 11
    dyn_output = result.stdout.split("8/11 负载辨识", 1)[1].split("9/11", 1)[0]
    assert dyn_output.index("使用 linkerhand_l6 的内置辨识值") < dyn_output.index("是否执行本步")
    assert "下一步：./scripts/start.sh" not in result.stdout


def test_from_prompt_retries_invalid_answer_and_can_skip_starting_step(tmp_path):
    result, steps = run_init(tmp_path, container=True, config_text=configured_config(),
                             args=("cam", "--from"), stdin="?\nn\ny\nn\nn\nn\ny\n")
    assert result.returncode == 0, result.stdout + result.stderr
    assert steps == ["ee", "summary"]
    assert "请输入 y 或 n" in result.stdout and result.stdout.count("是否执行本步") == 7


def test_closed_input_does_not_execute_an_unconfirmed_wizard_step(tmp_path):
    result, steps = run_init(tmp_path, container=True, stdin="")
    assert result.returncode == 1 and steps == []
    assert "未收到本步选择" in result.stderr


def test_force_configures_everything_again(tmp_path):
    result, steps = run_init(tmp_path, container=True, config_text=configured_config(), args=("--force", "--yes"))
    assert result.returncode == 0, result.stdout + result.stderr
    assert steps == ["config", "deps", "sdk", "marvin", "build", "cameras", "ee", "dyn", "xr", "home", "summary"]
    assert "已跳过" not in result.stdout


@pytest.mark.parametrize("step, expected", [("cam", "cameras"), ("ee", "ee"), ("dyn", "dyn"), ("xr", "xr"), ("config", "config")])
def test_a_single_step_always_runs_even_when_it_is_configured(tmp_path, step, expected):
    result, steps = run_init(tmp_path, container=True, config_text=configured_config(), args=(step,))
    assert result.returncode == 0, result.stdout + result.stderr
    assert steps == [expected]


def test_a_half_filled_load_is_not_a_usable_default(tmp_path):
    text = configured_config(robot={"end_effector": {"dyn_param": {"left": [1.0] * 10}}})
    result, steps = run_init(tmp_path, container=True, config_text=text, args=("--yes",))
    assert steps == ["deps", "sdk", "build", "dyn", "home", "summary"]


def test_simulation_and_no_hand_need_no_load_identification(tmp_path):
    for robot, reason in (({"hardware": "mock_components"}, "不是真机"), ({"end_effector": {"type": "none"}}, "没有末端")):
        text = configured_config(robot=robot)
        result, steps = run_init(tmp_path, container=True, config_text=text, args=("--yes",))
        assert "dyn" not in steps and reason in result.stdout


def test_reset_is_only_for_the_dyn_step(tmp_path):
    result, steps = run_init(tmp_path, container=True, args=("cam", "--reset"))
    assert result.returncode != 0 and steps == [] and "--reset 只用于 dyn" in result.stderr
    result, steps = run_init(tmp_path, container=True, args=("dyn", "--reset"))
    assert result.returncode == 0 and steps == ["dyn"]


@pytest.mark.parametrize("selection,removed,exit_code", [
    ("1\n", {"build"}, 0),
    ("2 3\n", {"install", "config.yaml"}, 0),
    ("1 2 3\n", {"build", "install", "config.yaml"}, 0),
    ("\n", set(), 0),
    ("1 9\n", set(), 1),
])
def test_clean_changes_only_the_selected_paths_and_backs_up_config(tmp_path, selection, removed, exit_code):
    for name in ("build", "install", "log"):
        (tmp_path / name).mkdir()
    config = "# 保留原配置\nrobot: {entry: whole_body}\n"
    result, steps = run_init(tmp_path, container=True, args=("clean",), config_text=config,
                             fake_ws=True, stdin=selection)
    assert result.returncode == exit_code, result.stdout + result.stderr
    assert steps == []
    for name in ("build", "install", "log", "config.yaml"):
        assert (tmp_path / name).exists() == (name not in removed)
    backups = list(tmp_path.glob("config.yaml.before-clean.*"))
    if "config.yaml" in removed:
        assert len(backups) == 1 and backups[0].read_text() == config
    else:
        assert backups == []


def test_the_old_build_workspace_command_is_gone(tmp_path):
    result, steps = run_init(tmp_path, container=True, args=("build-workspace",))
    assert result.returncode != 0 and steps == [] and "未知参数" in result.stderr


# ---- the Marvin (Tianji) SDK version: robot.arm.interface.firmware, 338 or 343 --------------------------
def built_with(tmp_path, firmware):
    cache = tmp_path / "build" / "marvin_hardware" / "CMakeCache.txt"
    cache.parent.mkdir(parents=True)
    cache.write_text(f"CMAKE_BUILD_TYPE:STRING=Release\nMARVIN_FIRMWARE:STRING={firmware}\n")


def call(tmp_path, function, config_text="{}\n"):
    """Run one setup.sh function against a temporary config and workspace root."""
    script = Path(__file__).resolve().parents[4] / "scripts" / "setup.sh"
    config = tmp_path / "config.yaml"
    config.write_text(config_text)
    harness = 'source "$1"; CONFIG="$W2_TEST_CONFIG"; WS_DIR="$W2_TEST_WS"; ' + function
    return subprocess.run(["bash", "-c", harness, "--", str(script)], text=True, capture_output=True, timeout=10,
                          env=dict(os.environ, W2_TEST_CONFIG=str(config), W2_TEST_WS=str(tmp_path)))


@pytest.mark.parametrize("dexcap", [False, True])
def test_deps_uses_the_selected_devices(tmp_path, dexcap):
    import yaml
    config = yaml.safe_dump({
        "robot": {"entry": "dual_arm", "arm": {"interface": {"firmware": 338}}},
        "teleop": {"type": "dexcap" if dexcap else "xr"},
        "cameras": {"enable": not dexcap},
    })
    result = call(tmp_path, r"""
        git() { printf '%s\n' "$@"; }
        colcon_ignore_submodule_dirs() { :; }
        source_ros() { :; }
        as_root() { "$@"; }
        deps_rosdep_update() { :; }
        colcon() {
            printf '%s\n' "$@" >&2
            echo src/required_package
        }
        rosdep() { printf '%s\n' "$@"; }
        main deps
    """, config)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "src/third_party/ocs2_ros2" in result.stdout
    assert "src/third_party/TJ_FX_ROBOT_CONTRL_SDK_338" in result.stdout
    assert "TJ_FX_ROBOT_CONTRL_SDK_343" not in result.stdout
    assert ("src/third_party/dexcap-sdk-cpp" in result.stdout) == dexcap
    assert ("src/third_party/OrbbecSDK_ROS2" in result.stdout) != dexcap
    assert "xCoreSDK-CPP" not in result.stdout
    assert "--packages-up-to" in result.stderr and "m6_ccs_description" in result.stderr
    assert "juxie_hardware" not in result.stderr and "fa_w2_description" not in result.stderr
    assert ("dexcap_teleop" in result.stderr) == dexcap
    assert ("xr_setup" in result.stderr) != dexcap
    assert "--from-paths\nsrc/required_package\n" in result.stdout


def test_marvin_is_asked_and_the_choice_is_written_to_the_config(tmp_path):
    result, _ = run_init(tmp_path, container=True, args=("marvin",), stub_marvin=False, stdin="338\n",
                         config_text="# 本机配置\nrobot:\n  body:\n    interface: {can: can0}   # 腰、头\n")
    assert result.returncode == 0, result.stdout + result.stderr
    text = (tmp_path / "config.yaml").read_text()
    assert "firmware: 338" in text and "# 腰、头" in text   # comments are kept
    assert "急停" in result.stdout + result.stderr                  # the prompt says what a wrong choice does


def test_marvin_rejects_an_answer_that_is_not_338_or_343(tmp_path):
    result, _ = run_init(tmp_path, container=True, args=("marvin",), stub_marvin=False, stdin="340\n")
    assert result.returncode != 0 and "338 或 343" in result.stderr
    assert "firmware" not in (tmp_path / "config.yaml").read_text()


@pytest.mark.parametrize("built, expected", [(None, "343"), ("338", "338")])
def test_marvin_yes_keeps_the_version_already_built_or_takes_343(tmp_path, built, expected):
    if built:
        built_with(tmp_path, built)
    result, _ = run_init(tmp_path, container=True, args=("marvin", "--yes"), stub_marvin=False, fake_ws=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert f"firmware: {expected}" in (tmp_path / "config.yaml").read_text()


def test_build_passes_the_chosen_version_to_cmake(tmp_path):
    result = call(tmp_path, "marvin_cmake_arg", "robot: {arm: {interface: {firmware: 338}}}\n")
    assert result.returncode == 0 and result.stdout == "-DMARVIN_FIRMWARE=338" and result.stderr == ""


def test_build_without_a_choice_keeps_the_built_version_and_warns(tmp_path):
    built_with(tmp_path, "338")
    result = call(tmp_path, "marvin_cmake_arg")
    assert result.returncode == 0 and result.stdout == "-DMARVIN_FIRMWARE=338"
    assert "./scripts/setup.sh marvin" in result.stderr


def test_build_filters_its_own_overlay_and_keeps_incremental_outputs(tmp_path):
    built_with(tmp_path, "338")
    (tmp_path / "install").mkdir()
    (tmp_path / "log").mkdir()
    result = call(tmp_path, r"""
        export COLCON_PREFIX_PATH="$WS_DIR/install:$WS_DIR/install/old:/other_ws/install"
        export AMENT_PREFIX_PATH="$WS_DIR/install/old"
        colcon_ignore_submodule_dirs() { :; }
        source_ros() { :; }
        colcon() {
            printf '%s\n' "$@"
            printf 'colcon_prefix=%s\n' "$COLCON_PREFIX_PATH"
            printf 'ament_prefix=%s\n' "${AMENT_PREFIX_PATH-unset}"
        }
        main build
    """)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "-DMARVIN_FIRMWARE=338" in result.stdout
    assert "-DXR_TELEOP_BUILD_NATIVE=1" in result.stdout
    assert "fa_w2_description" in result.stdout
    assert "rokae_hardware" not in result.stdout and "dexcap_teleop" not in result.stdout
    assert "colcon_prefix=/other_ws/install\n" in result.stdout
    assert "ament_prefix=unset\n" in result.stdout
    assert all((tmp_path / name).is_dir() for name in ("build", "install", "log"))
    assert (tmp_path / "config.yaml").is_file()


@pytest.mark.parametrize("option", ["--clean", "--minimal"])
def test_build_flags_removed_in_favor_of_clean_and_config(tmp_path, option):
    result, steps = run_init(tmp_path, container=True, args=("build", option))
    assert result.returncode == 1 and steps == []
    assert f"未知参数: {option}" in result.stderr


def test_an_invalid_version_in_the_config_is_an_error(tmp_path):
    result = call(tmp_path, "marvin_cmake_arg", "robot: {arm: {interface: {firmware: 340}}}\n")
    assert result.returncode != 0 and "robot.arm.interface.firmware" in result.stderr and "338 或 343" in result.stderr


# ---- the config step is real here: it writes and completes robot.local.yaml --------------------------
def _run_config_step(tmp_path, args, config_text=None):
    script = Path(__file__).resolve().parents[4] / "scripts" / "setup.sh"
    config = tmp_path / "config" / "robot.local.yaml"
    if config_text is not None:
        config.parent.mkdir(parents=True, exist_ok=True)
        config.write_text(config_text)
    harness = 'source "$1"\nshift\nCONFIG="$W2_TEST_CONFIG"\nmain "$@"\n'
    result = subprocess.run(["bash", "-c", harness, "--", str(script), *args], text=True, capture_output=True, timeout=30,
                            env=dict(os.environ, W2_TEST_CONFIG=str(config)))
    return result, config


def test_the_config_step_creates_the_file_from_the_template_of_the_entry(tmp_path):
    import yaml
    from robot_config.config import template_for
    result, config = _run_config_step(tmp_path, ("config", "--entry", "dual_arm", "--yes"))
    assert result.returncode == 0, result.stdout + result.stderr
    assert yaml.safe_load(config.read_text()) == template_for("dual_arm")


def test_the_config_step_adds_the_keys_an_old_file_lacks(tmp_path):
    import yaml
    result, config = _run_config_step(tmp_path, ("config", "--yes"), "robot: {entry: whole_body, hardware: mock_components}\n")
    assert result.returncode == 0, result.stdout + result.stderr
    cfg = yaml.safe_load(config.read_text())
    assert cfg["robot"]["hardware"] == "mock_components" and cfg["teleop"]["xr"]["network_mode"] == "ethernet"
    assert (config.parent / "robot.local.yaml.bak").exists()


def test_a_step_that_needs_the_file_says_how_to_create_it(tmp_path):
    result, _ = _run_config_step(tmp_path, ("cam",))
    assert result.returncode != 0 and "setup.sh config" in result.stderr


def fake_process(argv0):
    """A process whose command line is argv0, so pgrep -f sees it like the real one."""
    import subprocess as sp
    return sp.Popen(["bash", "-c", 'exec -a "$0" sleep 60', argv0])


def test_every_old_camera_process_is_stopped_before_the_camera_step_starts(tmp_path):
    script = Path(__file__).resolve().parents[4] / "scripts" / "setup.sh"
    leftovers = [
        fake_process("python3 -m camera_setup.session --config x --ws-root y"),
        fake_process("/opt/ros/jazzy/lib/rclcpp_components/component_container --ros-args -r __ns:=/setup_0123abcd_0_ab12cd_camera_head"),
        fake_process("ros2 launch robot_startup cameras.launch.py config:=c.yaml name_prefix:=setup_0123abcd_1_ab12cd_"),
    ]
    running = [
        fake_process("/opt/ros/jazzy/lib/rclcpp_components/component_container --ros-args -r __ns:=/camera_head"),
        fake_process("ros2 launch robot_startup cameras.launch.py config:=robot.local.yaml name_prefix:="),
    ]
    try:
        import time
        time.sleep(0.3)
        result = subprocess.run(["bash", "-c", 'source "$1"; stop_old_camera_setup', "--", str(script)],
                                text=True, capture_output=True, timeout=30)
        assert result.returncode == 0, result.stdout + result.stderr
        assert "清理上次相机配置残留的进程" in result.stdout
        assert all(process.wait(timeout=5) is not None for process in leftovers + running), "the cameras of a running robot go too"
    finally:
        for process in leftovers + running:
            process.kill()
            process.wait()


def test_nothing_is_stopped_or_printed_when_there_are_no_leftovers(tmp_path):
    script = Path(__file__).resolve().parents[4] / "scripts" / "setup.sh"
    result = subprocess.run(["bash", "-c", 'source "$1"; stop_old_camera_setup', "--", str(script)],
                            text=True, capture_output=True, timeout=30)
    assert result.returncode == 0 and "清理" not in result.stdout


def test_the_camera_step_cleans_up_first(tmp_path):
    script = (Path(__file__).resolve().parents[4] / "scripts" / "setup.sh").read_text()
    body = script[script.index("step_cameras() {"):].split("\n", 1)[0]
    assert body.index("stop_old_camera_setup") < body.index("run_python_step cameras")


def test_old_xr_setup_processes_are_stopped_before_the_xr_step_starts():
    script = Path(__file__).resolve().parents[4] / "scripts" / "setup.sh"
    leftovers = [
        fake_process("python3 -m xr_setup.session --config x --ws-root y xr"),
        fake_process("/ws/install/xr_teleop/lib/xr_teleop/xr_bridge_node --ros-args"),
        fake_process("RoboticsServiceProcess"),
    ]
    unrelated = fake_process("python3 -m camera_setup.session --config x --ws-root y")
    try:
        import time
        time.sleep(0.3)
        result = subprocess.run(["bash", "-c", 'source "$1"; stop_old_xr_setup', "--", str(script)],
                                text=True, capture_output=True, timeout=30)
        assert result.returncode == 0, result.stdout + result.stderr
        assert "清理上次XR配置残留的进程" in result.stdout
        assert all(process.wait(timeout=5) is not None for process in leftovers)
        assert unrelated.poll() is None
    finally:
        for process in leftovers + [unrelated]:
            process.kill()
            process.wait()


def test_the_xr_step_cleans_up_first():
    script = (Path(__file__).resolve().parents[4] / "scripts" / "setup.sh").read_text()
    body = script.split("step_xr() {", 1)[1].split("\n}", 1)[0]
    assert body.index("stop_old_xr_setup") < body.index("run_python_step xr")
