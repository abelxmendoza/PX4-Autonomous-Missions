"""Deterministic fault schedule for the integrated mission.

This is not the visual-odometry fault rig in ``faults/``. That rig injects sensor
and link faults into an estimator. This schedule injects mission-level faults
on a simulated clock: time, mission state, a state transition, a supervisor
evaluation count, or an explicit trigger name.

A weather or airspace fault does not bypass those authorizers. Wrappers below
make ``latest`` fail while the fault is active, so the existing gate still
produces the denial. There is no physical dispenser feedback; ``DISPENSER_FAULT``
only clears the simulated acknowledgement flag.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

from .airspace import AirspaceFetch, Reason as AirspaceReason
from .weather import Reason as WeatherReason, WeatherFetch

POSITION_FAULT = "POSITION_FAULT"
ATTITUDE_FAULT = "ATTITUDE_FAULT"
ALTITUDE_FAULT = "ALTITUDE_FAULT"
WEATHER_FAULT = "WEATHER_FAULT"
AIRSPACE_FAULT = "AIRSPACE_FAULT"
GEOFENCE_FAULT = "GEOFENCE_FAULT"
VISION_FAULT = "VISION_FAULT"
OFFBOARD_FAULT = "OFFBOARD_FAULT"
DISPENSER_FAULT = "DISPENSER_FAULT"

CLASSES = (
    POSITION_FAULT,
    ATTITUDE_FAULT,
    ALTITUDE_FAULT,
    WEATHER_FAULT,
    AIRSPACE_FAULT,
    GEOFENCE_FAULT,
    VISION_FAULT,
    OFFBOARD_FAULT,
    DISPENSER_FAULT,
)


@dataclass(frozen=True)
class FaultSpec:
    fault_id: str
    fault_type: str
    at_time_s: float | None = None
    at_state: str | None = None
    at_transition_to: str | None = None
    at_evaluation: int | None = None
    trigger: str | None = None
    duration_s: float = 1.0

    def problems(self) -> str:
        if not self.fault_id:
            return "fault id is missing"
        if self.fault_type not in CLASSES:
            return f"unknown fault type {self.fault_type}"
        if self.duration_s < 0.0:
            return "fault duration must be non-negative"
        selected = [
            self.at_time_s is not None,
            self.at_state is not None,
            self.at_transition_to is not None,
            self.at_evaluation is not None,
            self.trigger is not None,
        ]
        if not any(selected):
            return "fault needs a time, state, transition, evaluation count, or trigger"
        return ""


@dataclass
class MissionFaultPlan:
    specs: tuple[FaultSpec, ...]
    triggers: set[str] = field(default_factory=set)
    active: dict[str, FaultSpec] = field(default_factory=dict)
    injections: list[dict] = field(default_factory=list)
    responses: list[dict] = field(default_factory=list)
    evaluations: int = 0
    _seen_ids: set[str] = field(default_factory=set)
    _opened: dict[str, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        ids = [spec.fault_id for spec in self.specs]
        if len(ids) != len(set(ids)):
            raise ValueError("duplicate fault id in the schedule")
        for spec in self.specs:
            problem = spec.problems()
            if problem:
                raise ValueError(problem)

    def prepare(self, orchestrator, vehicle) -> None:
        self.evaluations += 1
        now = vehicle.time_s
        state = orchestrator.state.value
        destinations = {item["destination"] for item in orchestrator.transitions}
        current: dict[str, FaultSpec] = {}
        for spec in self.specs:
            if not self._matches(spec, now, state, destinations):
                continue
            current[spec.fault_type] = spec
            if spec.fault_id not in self._opened:
                self._opened[spec.fault_id] = now
                self.injections.append({
                    "fault_id": spec.fault_id,
                    "fault_type": spec.fault_type,
                    "time_s": now,
                    "mission_state": state,
                    "target": spec.fault_type.split("_")[0].lower(),
                    "intended": _intended(spec.fault_type),
                    "simulated": True,
                })
        self.active = current

    def blocking(self, fault_type: str) -> bool:
        return fault_type in self.active

    def mutate(self, vehicle):
        if not self.active:
            return vehicle
        changes = {}
        if POSITION_FAULT in self.active or GEOFENCE_FAULT in self.active:
            changes["position_ned"] = None
        if ATTITUDE_FAULT in self.active:
            changes["attitude_wxyz"] = None
        if ALTITUDE_FAULT in self.active:
            changes["altitude"] = replace(
                vehicle.altitude, local_z_m=None, z_valid=False, baro_alt_m=None, dist_bottom_valid=False
            )
        if VISION_FAULT in self.active:
            changes["tag"] = None
        if OFFBOARD_FAULT in self.active:
            changes["offboard"] = False
        if not changes:
            return vehicle
        return replace(vehicle, **changes)

    def observe(self, orchestrator, output) -> None:
        now = orchestrator.transitions[-1]["time_s"] if orchestrator.transitions else 0.0
        for spec in self.active.values():
            if any(item["fault_id"] == spec.fault_id for item in self.responses):
                continue
            observed = _observed(spec.fault_type, orchestrator, output)
            if observed is None:
                continue
            opened = self._opened.get(spec.fault_id, now)
            self.responses.append({
                "fault_id": spec.fault_id,
                "fault_type": spec.fault_type,
                "time_s": now,
                "expected": _intended(spec.fault_type),
                "observed": observed,
                "latency_s": now - opened,
                "mission_state": orchestrator.state.value,
                "simulated": True,
            })

    def _matches(self, spec: FaultSpec, now: float, state: str, destinations: set[str]) -> bool:
        if spec.at_time_s is not None:
            return spec.at_time_s <= now < spec.at_time_s + max(spec.duration_s, 1e-9)
        if spec.at_state is not None:
            return state == spec.at_state
        if spec.at_transition_to is not None:
            return spec.at_transition_to in destinations
        if spec.at_evaluation is not None:
            return self.evaluations >= spec.at_evaluation
        if spec.trigger is not None:
            return spec.trigger in self.triggers
        return False


class WeatherFaultSwitch:
    """Existing weather provider, plus a scheduled failure. ``latest`` does not fetch."""

    def __init__(self, inner, plan: MissionFaultPlan) -> None:
        self.inner = inner
        self.plan = plan

    def latest(self, now_s: float) -> WeatherFetch:
        if self.plan.blocking(WEATHER_FAULT):
            return WeatherFetch(None, WeatherReason.PROVIDER_FAILURE, "injected weather fault")
        return self.inner.latest(now_s)


class AirspaceFaultSwitch:
    def __init__(self, inner, plan: MissionFaultPlan) -> None:
        self.inner = inner
        self.plan = plan

    def latest(self, now_s: float) -> AirspaceFetch:
        if self.plan.blocking(AIRSPACE_FAULT):
            return AirspaceFetch(None, AirspaceReason.PROVIDER_FAILURE, "injected airspace fault")
        return self.inner.latest(now_s)


def _intended(fault_type: str) -> str:
    return {
        POSITION_FAULT: "stop claiming the vehicle is inside the geofence and do not hold a position",
        ATTITUDE_FAULT: "withhold vision guidance while attitude is invalid",
        ALTITUDE_FAULT: "close the release with an altitude denial",
        WEATHER_FAULT: "close the release and keep samples already recorded",
        AIRSPACE_FAULT: "abort the release and do not resume it",
        GEOFENCE_FAULT: "treat position as unavailable",
        VISION_FAULT: "hold on marker loss without a descent velocity",
        OFFBOARD_FAULT: "record the loss and keep streaming a setpoint",
        DISPENSER_FAULT: "record a simulated closure failure and do not return",
    }[fault_type]


def _observed(fault_type: str, orchestrator, output) -> str | None:
    codes = {item["code"] for item in orchestrator.faults}
    if fault_type in (POSITION_FAULT, GEOFENCE_FAULT) and "position_unavailable" in codes:
        return "position_unavailable"
    if fault_type == ATTITUDE_FAULT and "attitude_unavailable" in codes:
        return "attitude_unavailable"
    if fault_type == ALTITUDE_FAULT and orchestrator.mission.reason:
        return orchestrator.mission.reason
    if fault_type == WEATHER_FAULT and orchestrator.mission.reason:
        return orchestrator.mission.reason
    if fault_type == AIRSPACE_FAULT and orchestrator.mission.reason:
        return orchestrator.mission.reason
    if fault_type == VISION_FAULT and any(
        code in ("marker_lost", "marker_lost_during_descent", "marker_stale", "marker_never_acquired") for code in codes
    ):
        return "marker_lost"
    if fault_type == OFFBOARD_FAULT and "offboard_lost" in codes:
        return "offboard_lost"
    if fault_type == DISPENSER_FAULT and "dispenser_closure_failed" in codes:
        return "dispenser_closure_failed"
    if output is not None and fault_type == OFFBOARD_FAULT and output.heartbeat:
        return "setpoint_continues"
    return None
