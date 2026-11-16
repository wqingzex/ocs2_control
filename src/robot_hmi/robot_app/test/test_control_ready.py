"""The launch log decides whether `up control` really came up (lines copied from real runs)."""
from robot_app.control_ready import FAILED, PENDING, READY, judge, wait_ready

STARTED = """[INFO] [spawner-4]: process started with pid [11]
[INFO] [spawner-5]: process started with pid [12]
[INFO] [spawner-7]: process started with pid [13]
"""
FINISHED = """[INFO] [spawner-4]: process has finished cleanly [pid 11]
[INFO] [spawner-5]: process has finished cleanly [pid 12]
[INFO] [spawner-7]: process has finished cleanly [pid 13]
"""
TWO_MANAGERS = """[spawner-4] [WARN] [2026-11-14 18:08:52.840] [spawner_joint_state_broadcaster]: Controller already loaded, skipping load_controller
[ros2_control_node-3] [ERROR] [2026-11-14 18:08:52.841] [controller_manager]: Could not configure controller with name 'joint_state_broadcaster' because no controller with this name exists
[spawner-4] [ERROR] [2026-11-14 18:08:52.841] [spawner_joint_state_broadcaster]: Failed to configure controller
[ERROR] [spawner-4]: process has died [pid 11, exit code 1, cmd '/opt/ros/jazzy/lib/controller_manager/spawner joint_state_broadcaster'].
"""
BAD_INFO_FILE = """[ros2_control_node-3] Exception thrown during init stage with message: [loadHumanoidModeSchedule] initialModeSchedule.modeSequence[0] must be a structured entry with bodyMode and headMode
[spawner-5] [FATAL] [2026-11-14 17:58:29.140] [spawner_whole_body_controller]: Failed loading controller whole_body_controller
[ERROR] [spawner-5]: process has died [pid 12, exit code 1, cmd 'spawner whole_body_controller'].
"""


def test_all_spawners_finished_means_ready():
    assert judge(STARTED + FINISHED, alive=True).state == READY


def test_still_waiting_while_a_spawner_has_not_finished():
    partial = STARTED + "[INFO] [spawner-4]: process has finished cleanly [pid 11]\n"
    assert judge(partial, alive=True).state == PENDING
    assert judge("", alive=True).state == PENDING  # nothing logged yet


def test_a_dead_spawner_fails_and_the_reason_lines_are_reported():
    verdict = judge(STARTED + TWO_MANAGERS, alive=True)
    assert verdict.state == FAILED
    assert any("Failed to configure controller" in line for line in verdict.reasons)
    assert any("no controller with this name exists" in line for line in verdict.reasons)


def test_the_controller_exception_is_reported():
    verdict = judge(STARTED + BAD_INFO_FILE, alive=True)
    assert verdict.state == FAILED
    assert any("modeSequence[0] must be a structured entry" in line for line in verdict.reasons)


def test_failure_wins_over_other_spawners_finishing():
    assert judge(STARTED + FINISHED + TWO_MANAGERS, alive=True).state == FAILED


def test_the_launch_exiting_is_a_failure_even_without_a_spawner_line():
    verdict = judge(STARTED, alive=False)
    assert verdict.state == FAILED and verdict.reasons[0] == "运控进程已退出"


def test_wait_ready_returns_when_the_log_turns_ready(tmp_path):
    log = tmp_path / "control.log"
    log.write_text(STARTED)
    clock = {"t": 0.0}

    def sleep(seconds):
        clock["t"] += seconds
        if clock["t"] >= 2.0:
            log.write_text(STARTED + FINISHED)

    verdict = wait_ready(str(log), lambda: True, timeout_s=30, sleep=sleep, now=lambda: clock["t"])
    assert verdict.state == READY and clock["t"] < 30


def test_wait_ready_gives_up_pending_at_the_timeout(tmp_path):
    log = tmp_path / "control.log"
    log.write_text(STARTED)
    clock = {"t": 0.0}
    verdict = wait_ready(str(log), lambda: True, timeout_s=5,
                         sleep=lambda s: clock.update(t=clock["t"] + s), now=lambda: clock["t"])
    assert verdict.state == PENDING


def test_wait_ready_survives_a_missing_log(tmp_path):
    clock = {"t": 0.0}
    verdict = wait_ready(str(tmp_path / "none.log"), lambda: False, timeout_s=5,
                         sleep=lambda s: clock.update(t=clock["t"] + s), now=lambda: clock["t"])
    assert verdict.state == FAILED
