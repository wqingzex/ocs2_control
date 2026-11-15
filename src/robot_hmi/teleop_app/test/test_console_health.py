"""The console's live status: what the page shows for the controller, headset and cameras."""
from teleop_app.console import health

STARTED = "[INFO] [spawner-4]: process started with pid [11]\n[INFO] [spawner-5]: process started with pid [12]\n"
FINISHED = ("[INFO] [spawner-4]: process has finished cleanly [pid 11]\n"
            "[INFO] [spawner-5]: process has finished cleanly [pid 12]\n")
HAND_DEAD = ("[ros2_control_node-3] [ERROR] [x] [linkerhand_6_left_system]: Failed to read initial position; refusing to activate the hand\n"
             "[ros2_control_node-3] Exception thrown during init stage with message: hand did not answer\n"
             "[ERROR] [ros2_control_node-3]: process has died [pid 9, exit code -6, cmd 'ros2_control_node'].\n")
SELECTED = "[INFO] [xr_target_node]: 头显选择：PA9410MGL7110258G\n"
CONNECTED = "[INFO] [xr_target_node]: SDK 事件: PC Service 已连接 \n"


def unit(name, running=True, log=None, uptime=5.0, **extra):
    return {"name": name, "title": extra.pop("title", name), "running": running, "log": log, "uptime_s": uptime, **extra}


def reader(texts):
    return lambda path: texts.get(path, "")


def by_key(rows):
    return {row["key"]: row for row in rows}


def test_a_controller_launch_without_its_node_is_reported_as_failed_not_running():
    rows = by_key(health.summary([unit("control", log="c.log")], None, None, reader({"c.log": STARTED + HAND_DEAD})))
    assert rows["control"]["level"] == health.BAD and rows["control"]["text"] == "异常"
    assert "hand did not answer" in rows["control"]["detail"] or "process has died" in rows["control"]["detail"]


def test_a_working_controller_is_ready_when_the_fsm_is_reported():
    rows = by_key(health.summary([unit("control", log="c.log")], 2, True,
                                 reader({"c.log": STARTED + FINISHED})))
    assert (rows["control"]["level"], rows["control"]["text"]) == (health.OK, "就绪")


def test_a_starting_controller_waits_then_turns_into_an_error():
    young = by_key(health.summary([unit("control", log="c.log", uptime=20)], None, None, reader({"c.log": STARTED})))
    assert young["control"]["text"] == "启动中" and young["control"]["level"] == health.WARN
    old = by_key(health.summary([unit("control", log="c.log", uptime=health.CONTROL_STARTING_S + 1)], None, None,
                                reader({"c.log": STARTED})))
    assert old["control"]["level"] == health.BAD and old["control"]["text"] == "没有响应"


def test_active_controllers_without_a_state_message_are_a_warning():
    rows = by_key(health.summary([unit("control", log="c.log", uptime=30)], None, None,
                                 reader({"c.log": STARTED + FINISHED})))
    assert rows["control"]["level"] == health.WARN and "/fsm_state" in rows["control"]["detail"]


def test_stopped_parts_are_off():
    rows = by_key(health.summary([unit("control", running=False), unit("xr", running=False),
                                  unit("pc_service", running=False)], None, None, reader({})))
    assert {rows[k]["level"] for k in ("control", "xr", "pc_service")} == {health.OFF}


def test_the_headset_status_says_what_is_missing():
    connected = by_key(health.summary([unit("xr", log="x.log")], 2, True, reader({})))
    assert (connected["xr"]["level"], connected["xr"]["text"]) == (health.OK, "头显已连接")
    waiting = by_key(health.summary([unit("xr", log="x.log")], 2, False, reader({"x.log": SELECTED + CONNECTED})))
    assert waiting["xr"]["level"] == health.WARN and "PA9410MGL7110258G" in waiting["xr"]["detail"]
    no_service = by_key(health.summary([unit("xr", log="x.log")], 2, False, reader({"x.log": SELECTED})))
    assert "还没有连上 PC Service" in no_service["xr"]["detail"]


def test_cameras_show_their_rate_or_that_no_frames_arrive():
    rows = by_key(health.summary([unit("camera_head", title="头部相机", is_camera=True, rates={"/t": 30.0}),
                                  unit("camera_left_wrist", title="左腕相机", is_camera=True, rates={"/t": 0.0}),
                                  unit("camera_right_wrist", running=False, title="右腕相机", is_camera=True, rates={})],
                                 None, None, reader({})))
    assert (rows["camera_head"]["label"], rows["camera_head"]["text"], rows["camera_head"]["level"]) == ("头部", "30 Hz", health.OK)
    assert rows["camera_left_wrist"]["level"] == health.WARN and rows["camera_left_wrist"]["text"] == "无画面"
    assert rows["camera_right_wrist"]["level"] == health.OFF


def test_the_log_reader_keeps_head_and_tail_of_a_big_log_and_rereads_only_on_change(tmp_path):
    log = tmp_path / "big.log"
    middle = "x" * (health.LOG_HEAD_BYTES + health.LOG_TAIL_BYTES + 1000)
    log.write_text("FIRST-LINE\n" + middle + "\nLAST-LINE\n")
    read = health.LogReader()
    text = read(str(log))
    assert "FIRST-LINE" in text and "LAST-LINE" in text and len(text) < len(middle)
    assert read(str(log)) is text  # unchanged file: served from the cache
    log.write_text("NEW\n")
    assert read(str(log)).strip() == "NEW"
    assert read(None) == "" and read(str(tmp_path / "missing.log")) == ""


def test_every_part_with_a_unit_names_it_so_the_page_can_start_or_restart_it():
    rows = by_key(health.summary(
        [unit("control", log="c.log"), unit("xr", log="x.log"), unit("pc_service"),
         unit("camera_head", title="头部相机", is_camera=True, rates={"/t": 30.0})],
        2, True, reader({"c.log": STARTED + FINISHED})))
    assert (rows["control"]["unit"], rows["xr"]["unit"], rows["pc_service"]["unit"], rows["camera_head"]["unit"]) == \
        ("control", "xr", "pc_service", "camera_head")
    assert "fsm" not in rows  # the control mode section shows the state machine


def test_a_part_whose_unit_is_not_configured_offers_no_button():
    rows = by_key(health.summary([], None, None, reader({})))
    assert rows["control"]["unit"] is None and rows["xr"]["unit"] is None and rows["pc_service"]["unit"] is None
