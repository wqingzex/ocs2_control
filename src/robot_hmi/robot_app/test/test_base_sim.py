"""The simulated chassis: limits, acceleration, command timeout and the pose it integrates (no ROS needed)."""
import math

import pytest

from robot_app.base_sim import Limits, SimulatedBase


def run(base, seconds, dt=0.01, start=0.0, keep_commanding=None):
    """Step the base; keep_commanding = (vx, vy, wz) is re-sent every step like a controller does."""
    now = start
    for _ in range(round(seconds / dt)):
        if keep_commanding is not None:
            base.command(*keep_commanding, now)
        now += dt
        base.step(dt, now)
    return now


def test_the_base_speeds_up_no_faster_than_the_acceleration_limit():
    base = SimulatedBase(Limits(max_linear_accel=0.5))
    run(base, 0.2, keep_commanding=(0.4, 0.0, 0.0))
    assert base.vx == pytest.approx(0.1, abs=1e-9)  # 0.5 m/s^2 * 0.2 s


def test_the_speed_is_limited_and_the_pose_follows_the_velocity():
    base = SimulatedBase(Limits(max_linear=0.3, max_linear_accel=10.0))
    run(base, 2.0, keep_commanding=(5.0, 0.0, 0.0))
    assert base.vx == pytest.approx(0.3)
    assert base.x == pytest.approx(0.6, abs=0.01) and base.y == pytest.approx(0.0, abs=1e-9)


def test_a_base_turned_a_quarter_drives_along_the_world_y_axis():
    base = SimulatedBase(Limits(max_linear_accel=10.0))
    base.yaw = math.pi / 2
    run(base, 1.0, keep_commanding=(0.2, 0.0, 0.0))
    assert base.x == pytest.approx(0.0, abs=1e-9) and base.y == pytest.approx(0.2, abs=0.01)


def test_sideways_speed_is_along_the_base_y_axis():
    base = SimulatedBase(Limits(max_linear_accel=10.0))
    run(base, 1.0, keep_commanding=(0.0, 0.2, 0.0))
    assert base.y == pytest.approx(0.2, abs=0.01) and base.x == pytest.approx(0.0, abs=1e-9)


def test_turning_changes_the_yaw():
    base = SimulatedBase(Limits(max_angular=0.5, max_angular_accel=10.0))
    run(base, 2.0, keep_commanding=(0.0, 0.0, 0.2))
    assert base.yaw == pytest.approx(0.4, abs=0.01)


def test_a_command_that_stops_arriving_stops_the_base():
    base = SimulatedBase(Limits(command_timeout_s=0.3, max_linear_accel=1.0))
    now = run(base, 1.0, keep_commanding=(0.3, 0.0, 0.0))
    assert base.vx > 0.2
    run(base, 1.0, start=now)  # nothing is sent any more
    assert base.vx == 0.0


def test_the_base_does_not_move_before_the_first_command():
    base = SimulatedBase(Limits())
    run(base, 1.0)
    assert (base.x, base.y, base.yaw, base.vx) == (0.0, 0.0, 0.0, 0.0)


def test_a_command_that_is_not_finite_stops_the_base_and_says_so():
    base = SimulatedBase(Limits(max_linear_accel=10.0))
    now = run(base, 0.5, keep_commanding=(0.2, 0.0, 0.0))
    assert base.command(float("nan"), 0.0, 0.0, now) is False
    run(base, 0.5, start=now, keep_commanding=(float("nan"), 0.0, 0.0))
    assert base.vx == 0.0


def test_limits_must_be_positive_numbers():
    for bad in ({"max_linear": 0.0}, {"max_angular": -1.0}, {"command_timeout_s": float("nan")}):
        with pytest.raises(ValueError, match=next(iter(bad))):
            Limits(**bad)
