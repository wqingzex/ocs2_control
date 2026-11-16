"""Stopping the controllers asks first and goes to HOLD (units_cli.make_hold_hook)."""
import pytest

from robot_runtime.supervisor import UnitError


class FakeProbe:
    def __init__(self, state, hold_ok=True):
        self.state, self.hold_ok, self.hold_requests, self.closed = state, hold_ok, 0, False

    def wait_fsm(self, _timeout):
        return self.state

    def request_hold(self, _timeout):
        self.hold_requests += 1
        if self.hold_ok:
            self.state = 2
        return self.hold_ok

    def close(self):
        self.closed = True


def run_hook(state, answers, assume_yes=False, hold_ok=True):
    from robot_app.units_cli import make_hold_hook
    probe, asked = FakeProbe(state, hold_ok), []

    def ask(question):
        asked.append(question)
        return answers.pop(0)

    hook = make_hold_hook(assume_yes, ask, probe_factory=lambda: probe)
    try:
        hook("control")
        error = None
    except UnitError as exc:
        error = str(exc)
    return probe, asked, error


def test_stopping_control_in_hold_needs_no_confirmation():
    probe, asked, error = run_hook(state=2, answers=[])
    assert asked == [] and error is None and probe.hold_requests == 0 and probe.closed


def test_stopping_control_in_ocs2_asks_then_requests_hold():
    probe, asked, error = run_hook(state=3, answers=[True])
    assert "OCS2" in asked[0] and error is None and probe.hold_requests == 1


def test_declining_the_confirmation_aborts_the_stop():
    probe, _, error = run_hook(state=3, answers=[False])
    assert "已取消" in error and probe.hold_requests == 0


def test_hold_timeout_asks_again():
    _, asked, error = run_hook(state=3, answers=[True, False], hold_ok=False)
    assert len(asked) == 2 and "未回到 HOLD" in error


def test_yes_flag_skips_questions():
    probe, asked, error = run_hook(state=3, answers=[], assume_yes=True)
    assert asked == [] and error is None and probe.hold_requests == 1


def test_hook_ignores_other_units():
    from robot_app.units_cli import make_hold_hook
    make_hold_hook(False, lambda q: False, probe_factory=lambda: pytest.fail("probe must not be created"))("xr")
