"""The one command line of the setup steps: python3 -m robot_config.setup_steps <command>."""
import subprocess
import sys

import pytest
import yaml

from robot_config import setup_steps
from robot_config.testing import full_config


@pytest.fixture
def config(tmp_path):
    path = tmp_path / "robot.local.yaml"
    path.write_text(yaml.safe_dump(full_config(cameras={"enable": False})), encoding="utf-8")
    return str(path)


def test_an_unknown_or_missing_command_is_a_usage_error(capsys):
    assert setup_steps.main([]) == 2 and setup_steps.main(["build"]) == 2
    assert "usage" in capsys.readouterr().err


def test_config_creates_and_reports_the_file(tmp_path, capsys):
    path = str(tmp_path / "new.yaml")
    assert setup_steps.main(["config", "--config", path, "--entry", "dual_arm", "--yes"]) == 0
    assert setup_steps.main(["config", "--config", path, "--status"]) == 0
    assert "已配置" in capsys.readouterr().out


def test_status_prints_the_state_of_a_step(config, capsys):
    code = setup_steps.main(["status", "--config", config, "cameras"])
    assert code in (0, 1) and capsys.readouterr().out.strip()


def test_check_prints_the_summary(config, capsys):
    assert setup_steps.main(["check", config]) == 0
    assert "配置" in capsys.readouterr().out


def test_dyn_goes_to_the_load_identification(monkeypatch):
    from robot_config import load_identify
    seen = []
    monkeypatch.setattr(load_identify, "main", lambda args: seen.append(args) or 0)
    assert setup_steps.main(["dyn", "--config", "c.yaml", "--ws-root", "/ws", "--reset"]) == 0
    assert seen == [["--config", "c.yaml", "--ws-root", "/ws", "--reset"]]


def test_it_runs_as_a_module(config):
    result = subprocess.run([sys.executable, "-m", "robot_config.setup_steps", "config", "--config", config, "--status"],
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0 and "已配置" in result.stdout
