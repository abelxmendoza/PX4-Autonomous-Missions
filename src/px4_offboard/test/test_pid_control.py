from __future__ import annotations

import math

import pytest

from px4_offboard.pid_control import Pid, PidGains, VelocityPidController


def test_proportional_output_is_kp_times_error_and_clamped():
    pid = Pid(PidGains(kp=2.0), output_limit=3.0)
    assert pid.update(1.0, 0.1) == pytest.approx(2.0)
    assert pid.update(10.0, 0.1) == pytest.approx(3.0)
    assert pid.update(-10.0, 0.1) == pytest.approx(-3.0)


def test_integral_removes_a_steady_state_offset():
    # A plant that needs a constant 0.5 m/s of "push" to hold position
    # (e.g. wind): P alone leaves an offset, I drives it to zero.
    def settle(ki: float) -> float:
        pid = Pid(PidGains(kp=1.0, ki=ki, integral_limit=2.0), output_limit=3.0)
        pos, dt = 0.0, 0.05
        for _ in range(2000):
            cmd = pid.update(1.0 - pos, dt)
            pos += (cmd - 0.5) * dt  # plant: velocity = cmd - disturbance
        return 1.0 - pos

    assert abs(settle(ki=0.0)) > 0.4
    assert abs(settle(ki=0.5)) < 0.02


def test_anti_windup_stops_integral_growth_while_saturated():
    pid = Pid(PidGains(kp=1.0, ki=1.0, integral_limit=100.0), output_limit=1.0)
    for _ in range(500):
        pid.update(10.0, 0.1)  # error huge, output pinned at the limit
    assert pid.integral == pytest.approx(0.0, abs=1e-9)
    # ...so on target the controller doesn't overshoot on stored-up windup:
    assert pid.update(0.0, 0.1) == pytest.approx(0.0, abs=1e-9)


def test_integral_limit_bounds_the_integral_term():
    pid = Pid(PidGains(kp=0.0, ki=10.0, integral_limit=0.3), output_limit=5.0)
    for _ in range(100):
        pid.update(1.0, 0.1)
    assert pid.integral == pytest.approx(0.3)


def test_derivative_acts_on_measurement_not_error():
    # Target jumps (error steps 0 -> 5) while the vehicle is stationary:
    # derivative-on-error would spike; derivative-on-measurement must not.
    pid = Pid(PidGains(kp=0.0, kd=2.0), output_limit=10.0)
    assert pid.update(0.0, 0.1, measured_rate=0.0) == 0.0
    assert pid.update(5.0, 0.1, measured_rate=0.0) == 0.0
    # Moving toward the target damps the command.
    assert pid.update(5.0, 0.1, measured_rate=1.5) == pytest.approx(-3.0)


def test_nonfinite_error_and_bad_dt_are_safe_zero():
    pid = Pid(PidGains(kp=1.0), output_limit=1.0)
    assert pid.update(float("nan"), 0.1) == 0.0
    assert pid.update(1.0, 0.0) == 0.0
    assert pid.update(1.0, -0.1) == 0.0


def test_output_limit_must_be_positive():
    with pytest.raises(ValueError):
        Pid(PidGains(kp=1.0), output_limit=0.0)


def test_velocity_controller_points_toward_target_in_ned():
    ctl = VelocityPidController(PidGains(kp=1.0), PidGains(kp=1.0))
    v = ctl.command([10.0, -4.0, -5.0], [0.0, 0.0, -3.0], [0.0, 0.0, 0.0], 0.1)
    assert v[0] > 0.0  # north of us
    assert v[1] < 0.0  # west of us
    assert v[2] < 0.0  # target is higher (more negative down) -> climb


def test_horizontal_speed_is_capped_as_a_vector_not_per_axis():
    ctl = VelocityPidController(
        PidGains(kp=5.0), PidGains(kp=1.0), max_speed_xy=3.0
    )
    v = ctl.command([100.0, 100.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.1)
    assert math.hypot(v[0], v[1]) == pytest.approx(3.0)
    assert v[0] == pytest.approx(v[1])  # direction preserved


def test_closed_loop_reaches_the_target_without_overshooting():
    # Simple velocity-tracking plant (first-order lag) + the controller.
    ctl = VelocityPidController(
        PidGains(kp=0.8, kd=0.3), PidGains(kp=0.8, kd=0.3), max_speed_xy=3.0
    )
    pos, vel, dt = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.1
    target = [12.0, -6.0, 0.0]
    peak_n = 0.0
    for _ in range(600):
        cmd = ctl.command(target, pos, vel, dt)
        for i in range(3):
            vel[i] += (cmd[i] - vel[i]) * (dt / 0.4)
            pos[i] += vel[i] * dt
        peak_n = max(peak_n, pos[0])
    assert math.dist(pos, target) < 0.15
    assert peak_n < 12.0 * 1.05  # <5% overshoot


def test_reset_clears_integrators():
    ctl = VelocityPidController(
        PidGains(kp=0.1, ki=1.0), PidGains(kp=0.1, ki=1.0)
    )
    for _ in range(50):
        ctl.command([1.0, 1.0, 1.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.1)
    ctl.reset()
    v = ctl.command([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.1)
    assert v == [0.0, 0.0, 0.0]
