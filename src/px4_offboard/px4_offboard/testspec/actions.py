"""Mission actions: each one is a bounded loop of velocity setpoints sent through
``VehicleInterface`` until a completion condition holds or its timeout (on the
adapter clock) expires. ``hold`` is bounded by its own duration.

A timeout is a test outcome (the target did not do what was asked in time), not
an infrastructure error. Cancellation and the run's wall-clock budget are
raised from ``ctx.tick()`` and handled by the runner.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Protocol

from px4_offboard.vehicle.interface import CommandRejected

from .schema import Step
from .telemetry import Sample

COMPLETED = "COMPLETED"
TIMEOUT = "TIMEOUT"
REJECTED = "REJECTED"
NOT_RUN = "NOT_RUN"

KP_PER_S = 1.0  # position error -> velocity command
SETTLE_MPS = 0.2  # a waypoint counts as reached only once the vehicle has slowed down
LAND_SETTLE_MPS = 0.1


class StepContext(Protocol):
    vehicle: object
    rate_hz: float

    def now(self) -> float: ...

    def tick(self, command: tuple[float, float, float]) -> Sample:
        """Send ``command``, advance one control period, record and return the new sample."""


@dataclass
class StepResult:
    index: int
    id: str
    action: str
    params: dict
    status: str = NOT_RUN
    detail: str = ""
    t_start_s: float | None = None
    t_end_s: float | None = None
    first_sample: int | None = None
    last_sample: int | None = None

    def as_dict(self) -> dict:
        return {
            "index": self.index,
            "id": self.id,
            "action": self.action,
            "params": self.params,
            "status": self.status,
            "detail": self.detail,
            "t_start_s": None if self.t_start_s is None else round(self.t_start_s, 6),
            "t_end_s": None if self.t_end_s is None else round(self.t_end_s, 6),
            "telemetry_samples": None if self.first_sample is None else [self.first_sample, self.last_sample],
        }


def _toward(s: Sample, target: tuple[float, float, float], h_speed: float, v_speed: float) -> tuple[float, float, float]:
    vn = KP_PER_S * (target[0] - s.north_m)
    ve = KP_PER_S * (target[1] - s.east_m)
    h = math.hypot(vn, ve)
    if h > h_speed:
        vn, ve = vn * h_speed / h, ve * h_speed / h
    vd = max(-v_speed, min(v_speed, KP_PER_S * (target[2] - s.down_m)))
    return (vn, ve, vd)


def _speed(s: Sample) -> float:
    return math.sqrt(s.vn_mps ** 2 + s.ve_mps ** 2 + s.vd_mps ** 2)


def _distance(s: Sample, target: tuple[float, float, float]) -> float:
    return math.dist((s.north_m, s.east_m, s.down_m), target)


def _loop(ctx: StepContext, last: Sample, timeout_s: float, command: Callable[[Sample], tuple], done: Callable[[Sample], bool]):
    t0 = ctx.now()
    s = last
    while ctx.now() - t0 < timeout_s - 1e-9:
        s = ctx.tick(command(s))
        if done(s):
            return COMPLETED, s
    return TIMEOUT, s


def _arm(ctx, step, last):
    ctx.vehicle.arm()
    return _loop(ctx, last, step.params["timeout_s"], lambda s: (0.0, 0.0, 0.0), lambda s: s.armed)


def _disarm(ctx, step, last):
    ctx.vehicle.disarm()
    return _loop(ctx, last, step.params["timeout_s"], lambda s: (0.0, 0.0, 0.0), lambda s: not s.armed)


def _goto_like(ctx, step, last, target, h_speed, v_speed):
    tol = step.params["tolerance_m"]
    return _loop(
        ctx, last, step.params["timeout_s"],
        lambda s: _toward(s, target, h_speed, v_speed),
        lambda s: _distance(s, target) <= tol and _speed(s) <= SETTLE_MPS,
    )


def _takeoff(ctx, step, last):
    p = step.params
    return _goto_like(ctx, step, last, (last.north_m, last.east_m, -p["altitude_m"]), p["speed_mps"], p["speed_mps"])


def _goto(ctx, step, last):
    p = step.params
    return _goto_like(ctx, step, last, (p["north_m"], p["east_m"], -p["altitude_m"]), p["speed_mps"], p["vertical_speed_mps"])


def _hold(ctx, step, last):
    target = (last.north_m, last.east_m, last.down_m)
    s = last
    deadline = ctx.now() + step.params["duration_s"]
    while ctx.now() < deadline - 1e-9:
        s = ctx.tick(_toward(s, target, 1.0, 1.0))
    return COMPLETED, s


def _land(ctx, step, last):
    p = step.params
    target = (last.north_m, last.east_m, 0.0)
    return _loop(
        ctx, last, p["timeout_s"],
        lambda s: _toward(s, target, p["speed_mps"], p["speed_mps"]),
        lambda s: s.altitude_m <= p["tolerance_m"] and abs(s.vd_mps) <= LAND_SETTLE_MPS,
    )


EXECUTORS = {
    "arm": _arm,
    "takeoff": _takeoff,
    "goto": _goto,
    "hold": _hold,
    "land": _land,
    "disarm": _disarm,
}


def execute(ctx: StepContext, step: Step, last: Sample, result: StepResult) -> Sample:
    """Run one step, filling ``result``. Returns the last sample seen."""
    result.t_start_s = ctx.now()
    result.first_sample = last.index + 1
    try:
        status, s = EXECUTORS[step.action](ctx, step, last)
    except CommandRejected as exc:
        status, s = REJECTED, last
        result.detail = f"vehicle rejected the command: {exc}"
    result.status = status
    result.t_end_s = ctx.now()
    result.last_sample = s.index if s.index >= result.first_sample else None
    if result.last_sample is None:
        result.first_sample = None
    if status == TIMEOUT:
        result.detail = f"not complete after {step.params['timeout_s']} s (adapter clock)"
    elif status == COMPLETED and not result.detail:
        result.detail = "completion condition met"
    return s
