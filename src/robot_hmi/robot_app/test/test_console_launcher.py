"""Exercise the native double-click entry without starting ROS or robot hardware."""
import json
import os
import shlex
import shutil
import signal
import socket
import subprocess
import urllib.request
from pathlib import Path

import pytest
import yaml

from robot_config.testing import full_config


@pytest.fixture
def launcher():
    default = Path(__file__).resolve().parents[4] / "robot_control_suite控制台"
    path = Path(os.environ.get("CONSOLE_LAUNCHER", default))
    assert path.is_file(), "Build robot_app to generate robot_control_suite控制台 first"
    return path


def workspace(tmp_path, launcher, name):
    root = tmp_path / name
    root.mkdir()
    shutil.copy2(launcher, root / "robot_control_suite控制台")
    # Bash reads this non-executable script; no shell expansion should touch its path.
    (root / "scripts").mkdir()
    (root / "scripts" / "start.sh").write_text('''python3 - "$@" <<'PY'
import json, os, sys
print(json.dumps({"cwd": os.getcwd(), "args": sys.argv[1:]}))
PY
''', encoding="utf-8")
    return root


@pytest.mark.parametrize("name", ["workspace", "机器人 空格 '$`\"%; 目录"])
def test_opens_only_console_from_any_working_directory(tmp_path, launcher, name):
    root = workspace(tmp_path, launcher, name)
    result = subprocess.run([str(root / "robot_control_suite控制台")], cwd=tmp_path,
                            capture_output=True, text=True, check=True, timeout=5)
    assert json.loads(result.stdout) == {"cwd": str(root), "args": ["console", "--open"]}


def test_workspace_can_move_without_rebuilding(tmp_path, launcher):
    root = workspace(tmp_path, launcher, "before")
    moved = root.rename(tmp_path / "移动后的 仓库")
    result = subprocess.run([str(moved / "robot_control_suite控制台")], cwd=tmp_path,
                            capture_output=True, text=True, check=True, timeout=5)
    assert json.loads(result.stdout)["cwd"] == str(moved)


def test_missing_script_reports_failure_in_notification(tmp_path, launcher):
    shutil.copy2(launcher, tmp_path / "robot_control_suite控制台")
    commands = tmp_path / "bin"
    commands.mkdir()
    notify = commands / "notify-send"
    notify.write_text('#!/bin/bash\nprintf "%s\\n" "$@" > "$TEST_NOTIFICATION"\n')
    notify.chmod(0o755)
    message = tmp_path / "notification.txt"
    env = dict(os.environ, PATH=str(commands) + os.pathsep + os.environ["PATH"],
               TEST_NOTIFICATION=str(message))
    result = subprocess.run([str(tmp_path / "robot_control_suite控制台")], env=env,
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 1
    assert "start.sh" in result.stderr
    assert "请将 robot_control_suite控制台 保留在仓库根目录" in message.read_text()


def console_workspace(tmp_path, launcher):
    """A throw-away workspace with the real start.sh and a console that serves a one-line page; returns
    (root, env, port, browser_log, config)."""
    repo = Path(__file__).resolve().parents[4]
    root = tmp_path / "workspace"
    for name in ("config", "install", "src/robot_hmi/robot_app/web", "bin", "runtime"):
        (root / name).mkdir(parents=True, exist_ok=True)
    (root / "scripts").mkdir()
    shutil.copy2(repo / "scripts" / "start.sh", root / "scripts" / "start.sh")
    shutil.copy2(launcher, root / "robot_control_suite控制台")
    (root / "config/robot.local.yaml").write_text(
        yaml.safe_dump(full_config(robot={"hardware": "mock_components", "end_effector": {"type": "none"}},
                                   xr={"enable": False}, cameras={"enable": False}), sort_keys=False))
    package = shlex.quote(str(Path(__file__).resolve().parents[1]))
    (root / "install/setup.bash").write_text(f'export PYTHONPATH={package}:"${{PYTHONPATH:-}}"\n')
    (root / "src/robot_hmi/robot_app/web/index.html").write_text("<html>robot_control_suite console</html>")
    browser_log = root / "browser.txt"
    for name, script in (
        ("xdg-open", '#!/bin/bash\nprintf "%s\\n" "$1" >> "$TEST_BROWSER"\n'),
        ("notify-send", "#!/bin/bash\nexit 0\n"),
    ):
        path = root / "bin" / name
        path.write_text(script)
        path.chmod(0o755)
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    env = dict(os.environ, PATH=str(root / "bin") + os.pathsep + os.environ["PATH"],
               XDG_RUNTIME_DIR=str(root / "runtime"), TEST_BROWSER=str(browser_log),
               FA_CONSOLE_PORT=str(port), ROS_DOMAIN_ID="231", ROS_LOCALHOST_ONLY="1",
               RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
    config = str(root / "config/robot.local.yaml").encode()
    return root, env, port, browser_log, config


def test_executable_opens_console_and_reuses_process_without_starting_modules(tmp_path, launcher):
    root, env, port, browser_log, config = console_workspace(tmp_path, launcher)

    def server_pid():
        for proc in Path("/proc").glob("[0-9]*/cmdline"):
            try:
                args = proc.read_bytes().split(b"\0")
                if b"robot_app.console" in args and config in args:
                    return int(proc.parent.name)
            except OSError:
                pass
        return None

    try:
        subprocess.run([str(root / "robot_control_suite控制台")], env=env, cwd=tmp_path,
                       check=True, capture_output=True, text=True, timeout=20)
        pid = server_pid()
        assert pid is not None
        subprocess.run([str(root / "robot_control_suite控制台")], env=env, cwd=tmp_path,
                       check=True, capture_output=True, text=True, timeout=10)
        assert server_pid() == pid
        assert len(browser_log.read_text().splitlines()) == 2
        url = f"http://127.0.0.1:{port}"
        direct = urllib.request.build_opener(urllib.request.ProxyHandler({}))  # a http_proxy in the environment must not catch loopback
        with direct.open(url, timeout=5) as response:
            assert b"robot_control_suite console" in response.read()
        with direct.open(url + "/api/state", timeout=5) as response:
            state = json.load(response)
        assert state["units"]
        assert not any(unit["running"] for unit in state["units"])
    finally:
        pid = server_pid()
        if pid is not None:
            os.killpg(pid, signal.SIGTERM)


@pytest.mark.parametrize("custom_config", [False, True])
def test_default_start_runs_foreground_cli(tmp_path, custom_config):
    repo = Path(__file__).resolve().parents[4]
    root = tmp_path / "workspace"
    for name in ("config", "install", "fake/robot_app"):
        (root / name).mkdir(parents=True, exist_ok=True)
    (root / "scripts").mkdir()
    shutil.copy2(repo / "scripts" / "start.sh", root / "scripts" / "start.sh")
    config = root / ("custom.yaml" if custom_config else "config/robot.local.yaml")
    config.write_text("{}\n")
    package = root / "fake/robot_app"
    (package / "__init__.py").write_text("")
    (package / "check.py").write_text(
        "import sys\nfrom pathlib import Path\nassert Path(sys.argv[1]).is_file()\n")
    # Capture dispatch instead of starting any robot processes.
    (package / "units_cli.py").write_text("import json, sys\nprint(json.dumps(sys.argv[1:]))\n")
    prefix = shlex.quote(str(root / "fake"))
    (root / "install/setup.bash").write_text(f'export PYTHONPATH={prefix}:"${{PYTHONPATH:-}}"\n')
    args = ["bash", str(root / "scripts" / "start.sh")]
    if custom_config:
        args += ["-c", "custom.yaml"]
    result = subprocess.run(args, cwd=root, capture_output=True, text=True, check=True, timeout=5)
    assert json.loads(result.stdout) == [
        "--config", str(config), "--ws-root", str(root), "up", "--foreground"]


def test_console_subcommands_start_status_restart_stop(tmp_path, launcher):
    root, env, port, _, config = console_workspace(tmp_path, launcher)
    script = str(root / "scripts" / "start.sh")

    def run(*args, check=True):
        return subprocess.run([script, *args], env=env, cwd=tmp_path, capture_output=True, text=True,
                              timeout=40, check=check)

    def server_pids():
        found = []
        for proc in Path("/proc").glob("[0-9]*/cmdline"):
            try:
                args = proc.read_bytes().split(b"\0")
                if b"robot_app.console" in args and config in args:
                    found.append(int(proc.parent.name))
            except OSError:
                pass
        return found

    try:
        assert run("console", "status", check=False).returncode == 1  # nothing running yet
        assert "没有在运行" in run("console", "stop").stdout           # stopping nothing is not an error
        assert f"http://127.0.0.1:{port}/" in run("console", "start").stdout
        first = server_pids()
        assert len(first) == 1
        assert "运行中" in run("console", "status").stdout and str(first[0]) in run("console", "status").stdout
        assert run("console", "start").returncode == 0 and server_pids() == first  # already running: reused
        restarted = run("console", "restart")
        assert "运行中" in restarted.stdout
        second = server_pids()
        assert len(second) == 1 and second != first
        assert "已停止" in run("console", "stop").stdout
        assert server_pids() == []
        assert run("console", "status", check=False).returncode == 1
    finally:
        for pid in server_pids():
            os.killpg(pid, signal.SIGTERM)


def test_console_start_refuses_a_port_held_by_another_program(tmp_path, launcher):
    root, env, _, _, _ = console_workspace(tmp_path, launcher)
    with socket.socket() as holder:
        holder.bind(("127.0.0.1", 0))
        holder.listen()
        busy = holder.getsockname()[1]
        result = subprocess.run([str(root / "scripts" / "start.sh"), "console", "start", "--port", str(busy)],
                                env=env, cwd=tmp_path, capture_output=True, text=True, timeout=20)
    assert result.returncode == 1
    assert "被别的程序占用" in result.stderr
