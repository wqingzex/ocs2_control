"""Recording and playback: topic sets, bag names and paths, listing, deleting, commands (no ROS needed)."""
import pytest
import yaml

from fa_teleop_runtime import config, recording
from fa_teleop_runtime.recording import RecordingError

THREE_CAMERAS = {"head": {"serial": "A"}, "left_wrist": {"serial": "B"}, "right_wrist": {"serial": "C"}}
CAMERA_TOPICS = ["/camera_head/color/image_raw/compressed", "/camera_left_wrist/color/image_raw/compressed",
                 "/camera_right_wrist/color/image_raw/compressed"]


def make_cfg(**sections):
    sections.setdefault("cameras", THREE_CAMERAS)
    return config.validate(sections)


def write_bag(root, name, topics=("/joint_states",), messages=10, seconds=2.5, started=1_700_000_000):
    folder = root / name
    folder.mkdir(parents=True)
    (folder / "data_0.mcap").write_bytes(b"x" * 2048)
    meta = {"rosbag2_bagfile_information": {
        "duration": {"nanoseconds": int(seconds * 1e9)}, "message_count": messages,
        "starting_time": {"nanoseconds_since_epoch": int(started * 1e9)},
        "topics_with_message_count": [{"topic_metadata": {"name": t}, "message_count": 1} for t in topics]}}
    (folder / "metadata.yaml").write_text(yaml.safe_dump(meta))
    return folder


def test_joint_set_is_joints_only_and_the_default():
    assert recording.topics_for(make_cfg()) == ["/joint_states", "/fsm_state"]


def test_collect_set_adds_the_enabled_cameras():
    topics = recording.topics_for(make_cfg(), "collect")
    assert topics[:2] == ["/joint_states", "/fsm_state"] and sorted(topics[2:]) == sorted(CAMERA_TOPICS)
    cfg = make_cfg(cameras={**THREE_CAMERAS, "left_wrist": {"enable": False, "serial": "B"}})
    assert sorted(recording.topics_for(cfg, "collect")[2:]) == sorted([CAMERA_TOPICS[0], CAMERA_TOPICS[2]])


def test_collect_needs_a_camera():
    with pytest.raises(RecordingError, match="数采需要相机"):
        recording.topics_for(make_cfg(cameras={"enable": False}), "collect")


def test_xr_set_keeps_the_former_topics():
    topics = recording.topics_for(make_cfg(), "xr")
    assert len(topics) == 13 and "/teleop/left_ee_pose" in topics and "/left_target" in topics


def test_custom_topic_list_is_validated_and_extra_topics_are_appended_once():
    cfg = make_cfg(record={"extra_topics": ["/tf", "/joint_states"]})
    assert recording.topics_for(cfg, "/a/b, /c") == ["/a/b", "/c", "/tf", "/joint_states"]
    assert recording.topics_for(cfg) == ["/joint_states", "/fsm_state", "/tf"]
    for bad in ("", "a/b", "/a b", "/a;rm", "/a/"):
        with pytest.raises(RecordingError, match="话题集"):
            recording.topics_for(make_cfg(), bad)


@pytest.mark.parametrize("value", ["tf", ["tf"], ["/a b"], [1], "/tf"])
def test_record_section_is_validated(value):
    with pytest.raises(config.ConfigError, match="record.extra_topics"):
        make_cfg(record={"extra_topics": value})


@pytest.mark.parametrize("name", ["", ".hidden", "-x", "a b", "a/b", "../x", "x" * 65, "名字"])
def test_bag_names_are_restricted(name):
    with pytest.raises(RecordingError, match="录包名"):
        recording.check_name(name)


def test_auto_names_carry_the_set_name():
    assert recording.new_bag_name("collect", 0).startswith("collect_")
    assert recording.new_bag_name("/a,/b", 0).startswith("custom_")


def test_bag_root_defaults_to_bags_in_the_workspace_and_resolves_relative_paths(tmp_path):
    assert recording.bag_root(make_cfg(), str(tmp_path)) == str(tmp_path / "bags")
    assert recording.bag_root(make_cfg(record={"dir": "data/x"}), str(tmp_path)) == str(tmp_path / "data/x")
    assert recording.bag_root(make_cfg(record={"dir": "/mnt/disk"}), str(tmp_path)) == "/mnt/disk"


def test_record_command_names_the_bag_and_creates_the_root(tmp_path):
    cmd, path = recording.record_command(make_cfg(), str(tmp_path), "joint", "ep1")
    assert path == str(tmp_path / "bags" / "ep1") and (tmp_path / "bags").is_dir()
    assert cmd == ("ros2", "bag", "record", "-o", path, "--topics", "/joint_states", "/fsm_state")


def test_record_refuses_existing_bags_and_a_nearly_full_disk(tmp_path, monkeypatch):
    write_bag(tmp_path / "bags", "ep1")
    with pytest.raises(RecordingError, match="已存在"):
        recording.record_command(make_cfg(), str(tmp_path), "joint", "ep1")
    monkeypatch.setattr(recording, "free_bytes", lambda path: 1024)
    with pytest.raises(RecordingError, match="磁盘剩余空间不足"):
        recording.record_command(make_cfg(), str(tmp_path), "joint", "ep2")


def test_list_bags_is_newest_first_and_reads_the_metadata(tmp_path):
    write_bag(tmp_path, "old", started=1_700_000_000)
    write_bag(tmp_path, "new", topics=("/joint_states", "/fsm_state"), messages=42, seconds=61, started=1_700_001_000)
    (tmp_path / "live").mkdir()
    (tmp_path / "live" / "data_0.mcap").write_bytes(b"x")
    (tmp_path / "notes.txt").write_text("not a bag")
    bags = recording.list_bags(str(tmp_path))
    assert sorted(b["name"] for b in bags) == ["live", "new", "old"]
    finished = [b["name"] for b in bags if not b["recording"]]
    assert finished == ["new", "old"]
    new = next(b for b in bags if b["name"] == "new")
    assert (new["messages"], new["duration_s"], new["topics"]) == (42, 61.0, ["/joint_states", "/fsm_state"])
    live = next(b for b in bags if b["name"] == "live")
    assert live["recording"] and live["duration_s"] is None
    assert recording.list_bags(str(tmp_path / "missing")) == []


def test_delete_removes_only_a_direct_child_bag(tmp_path):
    root = tmp_path / "bags"
    write_bag(root, "ep1")
    outside = tmp_path / "keep"
    outside.mkdir()
    (root / "link").symlink_to(outside)
    for name in ("../keep", "link", "missing", "a/b"):
        with pytest.raises(RecordingError):
            recording.delete_bag(str(root), name)
    assert outside.is_dir()
    recording.delete_bag(str(root), "ep1")
    assert not (root / "ep1").exists()


def test_resolve_bag_accepts_a_name_or_a_directory(tmp_path):
    root = tmp_path / "bags"
    write_bag(root, "ep1")
    other = write_bag(tmp_path, "elsewhere")
    assert recording.resolve_bag(str(root), "ep1") == str(root / "ep1")
    assert recording.resolve_bag(str(root), str(other)) == str(other)
    with pytest.raises(RecordingError, match="不存在"):
        recording.resolve_bag(str(root), "nope")


def test_play_command_options():
    assert recording.play_command("/b") == ("ros2", "bag", "play", "/b")
    assert recording.play_command("/b", rate=0.5, loop=True, paused=True, start=3.0) == (
        "ros2", "bag", "play", "/b", "--rate", "0.5", "--loop", "--start-paused", "--start-offset", "3.0")
    assert recording.play_command("/b", xr=True)[-8:] == ("--topics", *recording.XR_REPLAY_TOPICS)
    for kwargs in ({"rate": 0}, {"rate": -1}, {"start": -1}):
        with pytest.raises(RecordingError):
            recording.play_command("/b", **kwargs)


def test_formatting_helpers():
    assert recording.format_size(512) == "512 B" and recording.format_size(1536) == "1.5 KB"
    assert recording.format_size(5 * 1024 ** 3) == "5.0 GB"
    assert recording.format_duration(None) == "-" and recording.format_duration(125) == "02:05"


def test_bag_table_lists_every_bag(tmp_path, capsys):
    write_bag(tmp_path, "ep1", messages=7)
    recording.print_bags(recording.list_bags(str(tmp_path)))
    out = capsys.readouterr().out
    assert "ep1" in out and "00:02" in out and " KB" in out and "/joint_states" in out
