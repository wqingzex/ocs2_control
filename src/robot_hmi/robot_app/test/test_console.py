"""Web console API tests: FastAPI TestClient, fake ROS probe, shell commands as units."""
import os
import json
import time

import pytest
import yaml

fastapi = pytest.importorskip("fastapi")
from fastapi.testclient import TestClient  # noqa: E402

from robot_config import config_edit  # noqa: E402
from robot_config.testing import full_config  # noqa: E402
from robot_app.console import server  # noqa: E402
from camera_setup.discovery import parse_list_devices  # noqa: E402
from robot_runtime.units import Unit, UnitSet  # noqa: E402

SLEEPER = ("sh", "-c", "echo hello-from-unit; trap 'exit 0' INT; while true; do sleep 0.05; done")
CONFIG = "# 控制台测试配置\n" + yaml.safe_dump(full_config(
    robot={"hardware": "mock_components"}, xr={"start_pc_service": False},
    cameras={"head": {"serial": "CP0000000001", "rotation": 90}, "left_wrist": {"serial": "CP0000000002"},
             "right_wrist": {"serial": "CP0000000003"}}), sort_keys=False).replace("  head:\n", "  head:  # Gemini 335Lg\n", 1)
SCAN = """\
[INFO] [1] [list_device_node]: name: Orbbec Gemini 335Lg
[INFO] [1] [list_device_node]: serial: CP0000000001
[INFO] [1] [list_device_node]: connection: USB3.2
[INFO] [1] [list_device_node]: usb port: 2-1
[INFO] [1] [list_device_node]: name: Orbbec Gemini 305
[INFO] [1] [list_device_node]: serial: CP0000000009
[INFO] [1] [list_device_node]: connection: USB2.0
[INFO] [1] [list_device_node]: usb port: 2-2
"""


class FakeProbe:
    def __init__(self):
        self.fsm_state = 2
        self.xr_connected = True
        self.hold_ok = True
        self.hold_requests = 0
        self.frames = {}
        self.sent = []
        self.whole_body_capability = None
        self.body_state = None
        self.body_sent = []

    def send_body_mode(self, action):
        self.body_sent.append(action)

    def send_fsm(self, command):
        self.sent.append(command)

    def watch(self, topics):
        pass

    def rate(self, topic, window_s=2.0):
        return 30.0

    def watch_image(self, topic):
        self.frames.setdefault(topic, None)

    def latest_image(self, topic):
        return self.frames.get(topic)

    def request_hold(self, timeout_s):
        self.hold_requests += 1
        if self.hold_ok:
            self.fsm_state = 2
        return self.hold_ok


@pytest.fixture
def setup(tmp_path, monkeypatch):
    (tmp_path / "config").mkdir()
    cfg = tmp_path / "config" / "teleop.yaml"
    cfg.write_text(CONFIG, encoding="utf-8")
    fake_units = UnitSet({
        "control": Unit("control", "运控", SLEEPER, topics=("/fsm_state",)),
        "rviz": Unit("rviz", "RViz", SLEEPER, depends=("control",), on_demand=True),
        "joystick": Unit("joystick", "手柄遥操", SLEEPER, depends=("control",), on_demand=True),
        "xr": Unit("xr", "XR", SLEEPER, topics=("/teleop/connection_status",)),
        "camera_head": Unit("camera_head", "头部相机", SLEEPER, topics=("/camera_head/color/image_raw/compressed",),
                             group="camera"),
    })
    monkeypatch.setattr(server, "build_units", lambda cfg, path, ws_root: fake_units)
    monkeypatch.setattr(server, "PREVIEW_IDLE_S", 0.5)
    probe = FakeProbe()
    console = server.Console(str(cfg), str(tmp_path), probe_factory=lambda: probe,
                             scanner=lambda: parse_list_devices(SCAN),
                             joysticks=lambda: [{"id": 0, "name": "Xbox"}, {"id": 2, "name": "Logitech"}])
    static = tmp_path / "static"
    static.mkdir()
    (static / "index.html").write_text("<html>console</html>", encoding="utf-8")
    (static / "app.js").write_text("// js", encoding="utf-8")
    client = TestClient(server.create_app(console, str(static)))
    yield client, console, probe, cfg
    for name in console.supervisor.units.names():
        console.supervisor.stop(name, timeout_s=1.0)


def test_the_console_needs_no_login(setup):
    client, *_ = setup
    assert client.get("/api/state").status_code == 200  # no token, no header: the only limit is the listen address
    assert client.get("/").status_code == 200


def test_page_files_are_revalidated_by_the_browser(setup):
    client, *_ = setup
    for path in ("/", "/static/app.js"):
        response = client.get(path)
        assert response.status_code == 200
        assert response.headers["cache-control"] == "no-cache"  # an older cached copy must not be shown


def test_state_reports_units_and_probe_data(setup):
    client, *_ = setup
    state = client.get("/api/state").json()
    assert [u["name"] for u in state["units"]] == ["control", "rviz", "joystick", "xr", "camera_head"]
    assert state["fsm_name"] == "HOLD" and state["xr_connected"] is True
    assert state["units"][4]["is_camera"] and state["units"][0]["rates"] == {"/fsm_state": 30.0}


def test_start_stop_and_log(setup):
    client, *_ = setup
    assert client.post("/api/units/xr/start").status_code == 200
    assert next(u for u in client.get("/api/state").json()["units"] if u["name"] == "xr")["running"]
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline and \
            "hello-from-unit" not in "".join(client.get("/api/units/xr/log").json()["lines"]):
        time.sleep(0.05)
    assert "hello-from-unit" in "".join(client.get("/api/units/xr/log").json()["lines"])
    assert client.post("/api/units/xr/stop", json={"confirm": False}).status_code == 200
    assert client.post("/api/units/nope/start").status_code == 404


def test_state_carries_the_live_summary_and_no_activity_when_idle(setup):
    client, *_ = setup
    state = client.get("/api/state").json()
    assert state["activity"] is None
    rows = {row["key"]: row for row in state["summary"]}
    assert rows["control"]["level"] == "off" and rows["xr"]["level"] == "off"
    assert client.post("/api/units/xr/start").status_code == 200
    assert {row["key"]: row for row in client.get("/api/state").json()["summary"]}["xr"]["level"] == "ok"  # probe: connected


def test_a_start_publishes_the_step_it_is_working_on(setup):
    _, console, *_ = setup
    seen = []
    steps = [("启动运控", lambda: seen.append(dict(console.activity))), ("启动XR", lambda: seen.append(dict(console.activity)))]
    console._sequence("启动", steps)
    assert [(a["step"], a["index"], a["total"]) for a in seen] == [("启动运控", 1, 2), ("启动XR", 2, 2)]
    assert console.activity is None  # cleared when done


def test_the_activity_is_cleared_when_a_step_fails(setup):
    _, console, *_ = setup

    def boom():
        raise RuntimeError("no")

    with pytest.raises(RuntimeError):
        console._sequence("启动", [("启动运控", boom)])
    assert console.activity is None


def test_restart_all_stops_what_runs_and_starts_everything_again(setup):
    client, console, probe, _ = setup
    assert client.post("/api/units/xr/start").status_code == 200
    before = next(u for u in client.get("/api/state").json()["units"] if u["name"] == "xr")["pid"]
    assert client.post("/api/all/restart", json={"confirm": False}).status_code == 200
    units = {u["name"]: u for u in client.get("/api/state").json()["units"]}
    assert units["xr"]["running"] and units["xr"]["pid"] != before
    assert units["control"]["running"]            # started by "all" even though it was stopped before
    assert not units["rviz"]["running"]            # on-demand units stay out of "all"


def test_restarting_the_controller_brings_back_rviz_and_the_gamepad_it_took_down(setup):
    client, *_ = setup
    assert client.post("/api/units/control/start").status_code == 200
    assert client.post("/api/units/rviz/start").status_code == 200
    assert client.post("/api/units/joystick/start").status_code == 200
    pids = {u["name"]: u["pid"] for u in client.get("/api/state").json()["units"]}
    assert client.post("/api/units/control/restart", json={"confirm": False}).status_code == 200
    after = {u["name"]: u for u in client.get("/api/state").json()["units"]}
    assert all(after[n]["running"] and after[n]["pid"] != pids[n] for n in ("control", "rviz", "joystick"))


def test_restarting_a_unit_does_not_start_dependents_that_were_not_running(setup):
    client, *_ = setup
    assert client.post("/api/units/control/start").status_code == 200
    assert client.post("/api/units/control/restart", json={"confirm": False}).status_code == 200
    after = {u["name"]: u for u in client.get("/api/state").json()["units"]}
    assert after["control"]["running"] and not after["rviz"]["running"] and not after["joystick"]["running"]


def test_restart_all_outside_hold_asks_first(setup):
    client, console, probe, _ = setup
    assert client.post("/api/units/control/start").status_code == 200
    probe.fsm_state = 3
    response = client.post("/api/all/restart", json={"confirm": False})
    assert response.status_code == 409 and response.json()["detail"]["need_confirm"]
    assert probe.hold_requests == 0
    assert client.post("/api/all/restart", json={"confirm": True}).status_code == 200
    assert probe.hold_requests == 1


def test_stopping_control_outside_hold_needs_confirmation(setup):
    client, console, probe, _ = setup
    client.post("/api/units/control/start")
    probe.fsm_state = 3  # OCS2
    res = client.post("/api/units/control/stop", json={"confirm": False})
    assert res.status_code == 409 and res.json()["detail"]["need_confirm"] and "OCS2" in res.json()["detail"]["message"]
    assert console.supervisor.is_running("control") and probe.hold_requests == 0

    res = client.post("/api/units/control/stop", json={"confirm": True})
    assert res.status_code == 200 and probe.hold_requests == 1
    assert not console.supervisor.is_running("control")


def test_control_keeps_running_when_hold_fails(setup):
    client, console, probe, _ = setup
    client.post("/api/units/control/start")
    probe.fsm_state, probe.hold_ok = 3, False
    res = client.post("/api/all/stop", json={"confirm": True})
    assert res.status_code == 409 and res.json()["detail"]["hold_failed"]
    assert console.supervisor.is_running("control")


def test_scan_suggests_roles_and_reports_rotation(setup):
    client, console, _, _ = setup
    client.post("/api/units/camera_head/start")
    result = client.post("/api/cameras/scan").json()
    assert result["stopped"] == ["camera_head"] and not console.supervisor.is_running("camera_head")
    assert result["suggestion"]["head"] == "CP0000000001"
    assert result["suggestion"]["left_wrist"] == "CP0000000009"
    assert result["rotation"]["head"] == 90


def test_a_failed_scan_brings_back_the_cameras_it_stopped(setup):
    # Arrange
    client, console, _, _ = setup
    client.post("/api/units/camera_head/start")

    def broken_scan():
        raise RuntimeError("list_devices_node 不可用")
    console.scanner = broken_scan

    # Act
    response = client.post("/api/cameras/scan")

    # Assert
    assert response.status_code == 500
    assert "list_devices_node 不可用" in response.json()["detail"]["message"]
    assert console.supervisor.is_running("camera_head")


def test_assign_writes_config_keeping_comments(setup):
    client, _, _, cfg = setup
    body = {"head": {"enable": True, "serial": "CP0000000001", "rotation": 180},
            "left_wrist": {"enable": True, "serial": "CP0000000009", "usb_port": "2-2"},
            "right_wrist": {"enable": False}}
    assert client.post("/api/cameras/assign", json=body).status_code == 200
    assert config_edit.camera_serials(str(cfg))["left_wrist"] == "CP0000000009"
    text = cfg.read_text(encoding="utf-8")
    assert "# 控制台测试配置" in text and "# Gemini 335Lg" in text and "rotation: 180" in text

    dup = {"head": {"serial": "X"}, "left_wrist": {"serial": "X"}}
    assert client.post("/api/cameras/assign", json=dup).status_code == 400


@pytest.mark.parametrize("entry", [
    {"serial": "CP1", "rotation": 45},
    {"serial": "bad serial"},
    {"serial": "CP1", "enable": "yes"},
    {"serial": "CP1", "usb_port": "2 1"},
])
def test_invalid_camera_assignment_is_rejected_and_not_written(setup, entry):
    client, _, _, cfg = setup
    before = cfg.read_text(encoding="utf-8")
    res = client.post("/api/cameras/assign", json={"head": entry})
    assert res.status_code == 400
    assert cfg.read_text(encoding="utf-8") == before


def test_preview_streams_mjpeg_parts(setup):
    client, _, probe, _ = setup
    probe.frames["/camera_head/color/image_raw/compressed"] = (
        time.monotonic(), "rgb8; jpeg compressed bgr8", b"\xff\xd8JPEG\xff\xd9")
    with client.stream("GET", "/api/cameras/camera_head/stream") as res:
        assert res.headers["content-type"].startswith("multipart/x-mixed-replace")
        body = b"".join(res.iter_bytes())
    assert b"--frame" in body and b"Content-Type: image/jpeg" in body and b"\xff\xd8JPEG\xff\xd9" in body
    assert client.get("/api/cameras/xr/stream").status_code == 400


def test_static_files_cannot_escape_the_directory(setup):
    client, *_ = setup
    assert client.get("/static/app.js").status_code == 200
    assert client.get("/static/..%2Fconfig%2Fteleop.yaml").status_code == 404


# --- control mode (FSM) and RViz ------------------------------------------------------------------

def test_fsm_actions_follow_the_current_state(setup):
    client, _, probe, _ = setup
    assert client.get("/api/state").json()["fsm_actions"] == []  # controllers not running
    client.post("/api/units/control/start")
    assert client.get("/api/state").json()["fsm_actions"] == ["HOME", "OCS2", "MOVEJ"]
    probe.fsm_state = 3
    assert client.get("/api/state").json()["fsm_actions"] == ["HOLD"]
    probe.fsm_state = 1
    assert client.get("/api/state").json()["fsm_actions"] == ["HOLD", "SWITCH_POSE"]


def test_body_modes_follow_the_controller_capabilities(setup):
    client, _, probe, _ = setup
    assert client.get("/api/state").json()["body"] == {"options": [], "current": None}  # controllers not running
    client.post("/api/units/control/start")
    assert client.get("/api/state").json()["body"]["options"] == []  # nothing reported yet
    probe.whole_body_capability = {"has_body_relative_constraint": True, "body_tracking_ee_enabled": False, "has_waist_lock": True,
                            "has_head_coupling": False, "has_custom_joint_lock": False}
    probe.body_state = 1
    body = client.get("/api/state").json()["body"]
    assert [o["label"] for o in body["options"]] == ["自由", "竖直", "锁定"] and body["current"] == 1


def test_body_mode_command_is_published_only_when_offered(setup):
    client, _, probe, _ = setup
    assert client.post("/api/body/VERTICAL").status_code == 409  # controllers not running
    client.post("/api/units/control/start")
    probe.whole_body_capability = {"has_body_relative_constraint": True}
    assert client.post("/api/body/VERTICAL").status_code == 200 and probe.body_sent == ["VERTICAL"]
    assert client.post("/api/body/TRACKING").status_code == 409  # not supported by this controller
    assert client.post("/api/body/NOPE").status_code == 404
    assert probe.body_sent == ["VERTICAL"]


def test_allowed_fsm_command_is_published(setup):
    client, _, probe, _ = setup
    client.post("/api/units/control/start")
    res = client.post("/api/fsm/OCS2")
    assert res.status_code == 200 and probe.sent == [3]


def test_switch_pose_is_a_pulse(setup):
    client, _, probe, _ = setup
    client.post("/api/units/control/start")
    probe.fsm_state = 1  # HOME
    assert client.post("/api/fsm/SWITCH_POSE").status_code == 200
    assert probe.sent == [100, 0]


def test_disallowed_or_unknown_fsm_commands_are_rejected(setup):
    client, _, probe, _ = setup
    assert client.post("/api/fsm/OCS2").status_code == 400  # controllers not running
    client.post("/api/units/control/start")
    probe.fsm_state = 3
    res = client.post("/api/fsm/MOVEJ")
    assert res.status_code == 409 and "OCS2" in res.json()["detail"]["message"]
    assert client.post("/api/fsm/FLY").status_code == 404
    assert probe.sent == []


def test_starting_rviz_never_starts_the_controllers(setup):
    client, console, _, _ = setup
    res = client.post("/api/units/rviz/start")
    assert res.status_code == 400 and "需要先启动运控" in res.json()["detail"]["message"]
    assert not console.supervisor.is_running("control")


def test_rviz_is_not_started_by_start_all_and_closes_without_touching_control(setup):
    client, console, probe, _ = setup
    client.post("/api/all/start")
    sup = console.supervisor
    assert sup.is_running("control") and not sup.is_running("rviz")
    probe.fsm_state = 3  # teleop / inference running in OCS2
    assert client.post("/api/units/rviz/start").status_code == 200 and sup.is_running("rviz")
    assert client.post("/api/units/rviz/stop", json={"confirm": False}).status_code == 200
    assert not sup.is_running("rviz") and sup.is_running("control")
    assert probe.hold_requests == 0 and probe.sent == []


# --- joystick teleop -------------------------------------------------------------------------

def test_joystick_devices_are_listed_with_the_saved_choice(setup):
    client, *_ = setup
    data = client.get("/api/joysticks").json()
    assert data["devices"] == [{"id": 0, "name": "Xbox"}, {"id": 2, "name": "Logitech"}]
    assert data["selected"] == ""


def test_choosing_a_joystick_is_saved_and_valid_only(setup):
    client, console, _, cfg = setup
    assert client.post("/api/joysticks/select", json={"device": "/dev/input/js2"}).status_code == 200
    assert "js2" in cfg.read_text(encoding="utf-8")
    assert client.get("/api/joysticks").json()["selected"] == "/dev/input/js2"
    before = cfg.read_text(encoding="utf-8")
    assert client.post("/api/joysticks/select", json={"device": "/dev/input/event1"}).status_code == 400
    assert cfg.read_text(encoding="utf-8") == before
    assert client.post("/api/joysticks/select", json={"device": ""}).status_code == 200  # back to auto


def test_joystick_needs_the_controllers_and_never_starts_them(setup):
    client, console, *_ = setup
    res = client.post("/api/units/joystick/start")
    assert res.status_code == 400 and "需要先启动运控" in res.json()["detail"]["message"]
    client.post("/api/units/control/start")
    assert client.post("/api/units/joystick/start").status_code == 200
    assert console.supervisor.is_running("joystick")


def test_console_page_is_served_from_the_workspace_web_folder():
    from robot_app.console.__main__ import static_dir
    assert static_dir("/ws") == "/ws/src/robot_hmi/robot_app/web"


def test_state_lists_the_addresses_other_computers_can_use(setup):
    client, console, *_ = setup
    assert client.get("/api/state").json()["access_urls"] == []  # none set: nothing to show
    console.access_urls = ["http://192.168.1.50:8080/"]
    assert client.get("/api/state").json()["access_urls"] == ["http://192.168.1.50:8080/"]


def test_access_urls_list_one_address_per_interface():
    from robot_app.console.__main__ import access_urls
    urls = access_urls(["192.168.1.50 (eth0)", "10.0.0.7 (wlan0)"], 8080)
    assert urls == ["http://192.168.1.50:8080/", "http://10.0.0.7:8080/"]
    assert access_urls([], 8080) == []


def test_body_options_follow_the_panel_rules():
    from robot_app.ros_probe import body_options
    assert body_options(None) == []
    assert [o["action"] for o in body_options({})] == ["FREE"]
    everything = {f: True for f in ("has_body_relative_constraint", "body_tracking_ee_enabled", "has_waist_lock",
                                    "has_head_coupling", "has_custom_joint_lock")}
    assert [o["state"] for o in body_options(everything)] == [0, 1, 2, 3, 4, 5]


def test_events_follow_unit_and_mode_actions(setup, tmp_path):
    client, _, probe, _ = setup
    client.post("/api/units/control/start")
    client.post("/api/fsm/OCS2")
    probe.whole_body_capability = {"has_body_relative_constraint": True}
    client.post("/api/body/VERTICAL")
    client.post("/api/units/control/stop")
    lines = [json.loads(line) for line in (tmp_path / "logs" / "events.log").read_text().splitlines()]
    assert [e["kind"] for e in lines] == ["session", "start", "fsm_command", "body_mode", "stop"]
    assert lines[2]["action"] == "OCS2" and lines[2]["from"] and lines[3]["action"] == "VERTICAL"


def test_the_diagnostic_bundle_is_built_and_downloaded(setup, monkeypatch):
    from robot_app import diag
    monkeypatch.setattr(diag, "ros_info", lambda: "NODES")
    client, *_ = setup
    name = client.post("/api/diag").json()["name"]
    res = client.get(f"/api/diag/{name}")
    assert res.status_code == 200 and res.content[:2] == b"\x1f\x8b"  # gzip
    assert client.get("/api/diag/../../etc/passwd").status_code == 404
    assert client.get("/api/diag/diag_20200101_000000.tar.gz").status_code == 404  # not built


def test_console_listens_on_this_computer_only_by_default():
    from robot_app.console.__main__ import build_parser
    parser = build_parser()
    args = parser.parse_args(["--config", "c.yaml", "--ws-root", "/w"])
    assert args.host == "127.0.0.1"  # it can start and stop the robot: other computers are opt-in
    assert parser.parse_args(["--config", "c", "--ws-root", "/w", "--host", "0.0.0.0"]).host == "0.0.0.0"
