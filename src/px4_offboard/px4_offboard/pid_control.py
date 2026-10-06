"""Outer-loop PID: position error -> velocity command.

PX4 keeps running its own inner position/velocity/attitude/rate cascade; this
is only the guidance layer above it. Instead of streaming a position setpoint
and letting PX4's P-controller chase it, ``offboard_mission`` can stream a
*velocity* setpoint computed here from (target - estimated position). That is
what makes the position source swappable (PX4's own estimate or the stereo+IMU
fusion estimate): the loop closes on whichever estimate it is handed.

Pure Python, no ROS. NED frame throughout (Down positive).
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class PidGains:
    kp: float
    ki: float = 0.0
    kd: float = 0.0
    # |integral term| never exceeds this (in output units) -- bounds windup
    # independently of the saturation-based anti-windup below.
    integral_limit: float = 1.0


class Pid:
    """Single-axis PID with derivative on measurement and anti-windup.

    Derivative acts on the measured rate (``-measured_rate``), not on the
    error: with a fixed target, d(error)/dt == -velocity, and using the
    measurement avoids a derivative kick when the target jumps to the next
    waypoint.
    """

    def __init__(self, gains: PidGains, output_limit: float):
        if output_limit <= 0.0:
            raise ValueError("output_limit must be positive")
        self.gains = gains
        self.output_limit = output_limit
        self.integral = 0.0

    def reset(self) -> None:
        self.integral = 0.0

    def update(self, error: float, dt_s: float, measured_rate: float = 0.0) -> float:
        if dt_s <= 0.0 or not math.isfinite(error):
            return 0.0
        g = self.gains
        p = g.kp * error
        d = -g.kd * measured_rate
        unsaturated = p + self.integral + d
        saturated = max(-self.output_limit, min(self.output_limit, unsaturated))
        # Conditional integration: don't wind up while the output is pinned
        # at its limit and the error is still pushing the same direction.
        pushing_further = (unsaturated > saturated and error > 0.0) or (
            unsaturated < saturated and error < 0.0
        )
        if not pushing_further:
            self.integral += g.ki * error * dt_s
            self.integral = max(-g.integral_limit, min(g.integral_limit, self.integral))
        out = p + self.integral + d
        return max(-self.output_limit, min(self.output_limit, out))


class VelocityPidController:
    """3-axis NED position->velocity PID with a horizontal speed cap."""

    def __init__(
        self,
        xy_gains: PidGains,
        z_gains: PidGains,
        max_speed_xy: float = 3.0,
        max_speed_z: float = 1.5,
    ):
        self.max_speed_xy = max_speed_xy
        self.max_speed_z = max_speed_z
        self._pid_n = Pid(xy_gains, max_speed_xy)
        self._pid_e = Pid(xy_gains, max_speed_xy)
        self._pid_d = Pid(z_gains, max_speed_z)

    def reset(self) -> None:
        for pid in (self._pid_n, self._pid_e, self._pid_d):
            pid.reset()

    def command(
        self,
        target_ned: list[float],
        position_ned: list[float],
        velocity_ned: list[float],
        dt_s: float,
    ) -> list[float]:
        vn = self._pid_n.update(
            target_ned[0] - position_ned[0], dt_s, velocity_ned[0]
        )
        ve = self._pid_e.update(
            target_ned[1] - position_ned[1], dt_s, velocity_ned[1]
        )
        vd = self._pid_d.update(
            target_ned[2] - position_ned[2], dt_s, velocity_ned[2]
        )
        # Per-axis limits allow sqrt(2) * max on a diagonal; cap the vector so
        # "max horizontal speed" means what it says.
        speed = math.hypot(vn, ve)
        if speed > self.max_speed_xy:
            scale = self.max_speed_xy / speed
            vn, ve = vn * scale, ve * scale
        return [vn, ve, vd]
