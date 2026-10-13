"""One sortie: seeding release, return to a recovery stage, then precision landing.

The seeding mission and the precision-landing controller are unchanged. This
module is the only place that decides which of them may publish a trajectory,
and it emits at most one setpoint per step.

PX4 drops OFFBOARD if ``OffboardControlMode`` stops arriving. The default
gap (``offboard_gap_s`` = 0.5) matches a 2 Hz minimum: the process that calls
``step`` has to keep calling it. A larger simulated gap is recorded; the
orchestrator does not sleep to create one.

Release authorization is the existing ``SeedingSupervisor``: mission phase,
release volume, altitude frame, estimator health, sensor freshness, the
weather authorizer, and the airspace authorizer. ``require_weather`` defaults
true. Setting it false is an invalid configuration, so a release cannot be
authorized by skipping weather. A clear airspace chart is recorded the same
way and is still not an approval to fly.
A simulated provider is allowed only through that authorizer, and the evidence
keeps the provider's provenance. A denial is not turned into an approval by a
later state.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Sequence

from .altitude_reference import Px4AltitudeSnapshot
from .precision_land import (
    CameraTag,
    LandCommand,
    LandState,
    PrecisionLand,
    VehicleSnapshot,
    offboard_setpoint,
)
from .seeding_mission import SeedingCommand, SeedingPhase
from .seeding_supervisor import SeedingSupervisor

_TERMINAL = frozenset()  # filled after the enum exists


class MissionState(Enum):
    IDLE = "idle"
    PREFLIGHT = "preflight"
    TAKEOFF = "takeoff"
    TRANSIT_TO_RELEASE = "transit_to_release"
    HOLD_FOR_RELEASE = "hold_for_release"
    DISPENSING = "dispensing"
    RETURN_TO_RECOVERY = "return_to_recovery"
    PRECISION_LANDING = "precision_landing"
    LANDED = "landed"
    ABORTED = "aborted"
    FAILED = "failed"


_TERMINAL = frozenset({MissionState.LANDED, MissionState.ABORTED, MissionState.FAILED})

# Explicit edges. Anything else is rejected and does not change state.
ALLOWED: frozenset[tuple[MissionState, MissionState]] = frozenset({
    (MissionState.IDLE, MissionState.PREFLIGHT),
    (MissionState.IDLE, MissionState.FAILED),
    (MissionState.PREFLIGHT, MissionState.TAKEOFF),
    (MissionState.PREFLIGHT, MissionState.FAILED),
    (MissionState.TAKEOFF, MissionState.TRANSIT_TO_RELEASE),
    (MissionState.TAKEOFF, MissionState.HOLD_FOR_RELEASE),
    (MissionState.TAKEOFF, MissionState.DISPENSING),
    (MissionState.TAKEOFF, MissionState.RETURN_TO_RECOVERY),
    (MissionState.TRANSIT_TO_RELEASE, MissionState.DISPENSING),
    (MissionState.TRANSIT_TO_RELEASE, MissionState.RETURN_TO_RECOVERY),
    (MissionState.TAKEOFF, MissionState.FAILED),
    (MissionState.TRANSIT_TO_RELEASE, MissionState.HOLD_FOR_RELEASE),
    (MissionState.TRANSIT_TO_RELEASE, MissionState.FAILED),
    (MissionState.HOLD_FOR_RELEASE, MissionState.TRANSIT_TO_RELEASE),
    (MissionState.HOLD_FOR_RELEASE, MissionState.DISPENSING),
    (MissionState.HOLD_FOR_RELEASE, MissionState.RETURN_TO_RECOVERY),
    (MissionState.HOLD_FOR_RELEASE, MissionState.FAILED),
    (MissionState.DISPENSING, MissionState.RETURN_TO_RECOVERY),
    (MissionState.DISPENSING, MissionState.FAILED),
    (MissionState.RETURN_TO_RECOVERY, MissionState.PRECISION_LANDING),
    (MissionState.RETURN_TO_RECOVERY, MissionState.FAILED),
    (MissionState.PRECISION_LANDING, MissionState.LANDED),
    (MissionState.PRECISION_LANDING, MissionState.ABORTED),
    (MissionState.PRECISION_LANDING, MissionState.FAILED),
})


class Owner(Enum):
    NONE = "none"
    SEEDING = "seeding"
    PRECISION_LAND = "precision_land"
    HOLD = "hold"
    PX4_LAND = "px4_land"


@dataclass(frozen=True)
class StagingConditions:
    """Precision landing starts only when all of these are true. Not a timer."""

    max_horizontal_m: float = 2.0
    min_altitude_m: float = 4.0
    max_altitude_m: float = 8.0
    max_horizontal_speed_mps: float = 0.8
    staging_altitude_m: float = 6.0

    def problems(self) -> str:
        if self.min_altitude_m <= 0.0 or self.staging_altitude_m <= 0.0:
            return "staging altitude must be positive"
        if self.min_altitude_m > self.max_altitude_m:
            return "staging min_altitude_m exceeds max_altitude_m"
        if not (self.min_altitude_m <= self.staging_altitude_m <= self.max_altitude_m):
            return "staging_altitude_m is outside the staging altitude band"
        if self.max_horizontal_m <= 0.0 or self.max_horizontal_speed_mps <= 0.0:
            return "staging distance and speed limits must be positive"
        return ""


@dataclass(frozen=True)
class OrchestratorConfig:
    """Weather is required. ``require_weather=False`` is rejected at start."""

    require_weather: bool = True
    offboard_gap_s: float = 0.5
    landing_timeout_s: float = 90.0
    takeoff_altitude_m: float = 2.0
    dispenser_ack: bool = True
    staging: StagingConditions = field(default_factory=StagingConditions)

    def problems(self) -> str:
        if not self.require_weather:
            return "weather authorization cannot be disabled"
        if self.offboard_gap_s <= 0.0 or self.landing_timeout_s <= 0.0:
            return "timeouts must be positive"
        if self.takeoff_altitude_m <= 0.0:
            return "takeoff_altitude_m must be positive"
        return self.staging.problems()


@dataclass(frozen=True)
class VehicleInput:
    """PX4 state for one orchestrator tick. Times are simulated seconds."""

    time_s: float
    position_ned: tuple[float, float, float] | None
    velocity_ned: tuple[float, float, float] | None
    attitude_wxyz: tuple[float, float, float, float] | None
    landed: bool
    armed: bool
    offboard: bool
    altitude: Px4AltitudeSnapshot
    tag: CameraTag | None = None
    dt_s: float = 0.0


def _gate_summary(records: list[dict]) -> dict:
    last = records[-1] if records else None
    return {
        "implemented": True,
        "consulted": bool(records),
        "authorized": None if last is None else last.get("authorized"),
        "reason_code": None if last is None else last.get("reason_code"),
        "provenance": None if last is None else last.get("provenance"),
        "source": None if last is None else last.get("source"),
        "operational_approval": False,
        "decision_count": len(records),
    }


@dataclass(frozen=True)
class OrchestratorOutput:
    state: MissionState
    owner: Owner
    heartbeat: bool
    dispense: bool
    request_arm: bool
    request_offboard: bool
    request_nav_land: bool
    position_ned: tuple[float, float, float] | None = None
    velocity_ned: tuple[float, float, float] | None = None
    use_position: bool = False


def _finite_xyz(value: Sequence[float] | None) -> tuple[float, float, float] | None:
    if value is None or len(value) != 3:
        return None
    out = (float(value[0]), float(value[1]), float(value[2]))
    if not all(math.isfinite(v) for v in out):
        return None
    return out


def _nan_velocity() -> tuple[float, float, float]:
    nan = float("nan")
    return (nan, nan, nan)


class MissionOrchestrator:
    def __init__(
        self,
        supervisor: SeedingSupervisor,
        lander: PrecisionLand | None = None,
        config: OrchestratorConfig | None = None,
    ) -> None:
        self.supervisor = supervisor
        self.lander = lander if lander is not None else PrecisionLand()
        self.config = config if config is not None else OrchestratorConfig()
        self._config_error = self.config.problems()
        self.state = MissionState.IDLE
        self.owner = Owner.NONE
        self.transitions: list[dict] = []
        self.ownership: list[dict] = []
        self.faults: list[dict] = []
        self.return_progress: list[dict] = []
        self._started = False
        self._last_step_t: float | None = None
        self._offboard_wanted = False
        self._precision_t0: float | None = None
        self._dispenser_was_open = False
        self._dispenser_closed = True
        self._handed_off = False
        self._release_aborted = False
        self._rejected_external = 0
        self._last_position: tuple[float, float, float] | None = None
        self._offboard_lost = False
        self._attitude_fault = False
        self._hold_position: tuple[float, float, float] | None = None
        self._progress_t: float | None = None
        self._descent_tag_ages: list[float] = []

    @property
    def mission(self):
        return self.supervisor.mission

    def start(self, time_s: float) -> None:
        """Begin the sortie. A second call does not restart it."""
        if self._started:
            code = "duplicate_enable" if self.state not in _TERMINAL else "invalid_transition"
            self._fault(
                time_s,
                code,
                recovery="ignore_enable",
                result="rejected",
                position=self._last_position,
            )
            return
        self._started = True
        if self._config_error:
            self._goto(MissionState.FAILED, "invalid_configuration", time_s, detail=self._config_error)
            return
        self._goto(MissionState.PREFLIGHT, "start", time_s)

    def submit(self, claimant: str, time_s: float) -> bool:
        """Refuse a trajectory publisher that is not the active owner."""
        if claimant == self.owner.value and self.owner is not Owner.NONE:
            return True
        self._rejected_external += 1
        self._fault(
            time_s,
            "ownership_conflict",
            recovery="reject_setpoint",
            result="rejected",
            position=self._last_position,
            extra={"claimant": claimant},
        )
        return False

    def step(self, vehicle: VehicleInput) -> OrchestratorOutput:
        position = _finite_xyz(vehicle.position_ned)
        if position is not None:
            self._last_position = position
        self._note_gap(vehicle.time_s)
        self._last_step_t = vehicle.time_s

        if self.state in _TERMINAL:
            return self._terminal_output(vehicle, position)

        if not self._started or self.state is MissionState.IDLE:
            return self._output(vehicle, Owner.NONE, None, False, False, False)

        if self.state is not MissionState.PREFLIGHT and not vehicle.armed and not vehicle.landed:
            self._fault(vehicle.time_s, "unexpected_disarm", recovery="stop_setpoints", result="failed", position=position)
            self._goto(MissionState.FAILED, "unexpected_disarm", vehicle.time_s)
            self._set_owner(Owner.NONE, vehicle.time_s, "disarmed")
            return self._output(vehicle, Owner.NONE, None, False, False, False)

        if self.state is not MissionState.PREFLIGHT and position is None:
            self._fault(
                vehicle.time_s,
                "position_unavailable",
                recovery="nav_land",
                result="failed",
                position=None,
            )
            self._goto(MissionState.FAILED, "position_unavailable", vehicle.time_s)
            self._set_owner(Owner.PX4_LAND, vehicle.time_s, "no position estimate")
            return self._output(vehicle, Owner.PX4_LAND, None, False, False, True)

        if (
            self._offboard_wanted
            and not vehicle.offboard
            and position is not None
            and self.state is not MissionState.PREFLIGHT
            and not self._offboard_lost
        ):
            self._offboard_lost = True
            self._fault(
                vehicle.time_s,
                "offboard_lost",
                recovery="reassert_offboard",
                result="continuing",
                position=position,
            )
        elif vehicle.offboard:
            self._offboard_lost = False

        if self.state is MissionState.PREFLIGHT:
            return self._preflight(vehicle, position)

        if not self._handed_off:
            command = self._seeding_step(vehicle, position)
            self._follow_seeding(vehicle, position, command)
            if self.state in _TERMINAL:
                return self._terminal_output(vehicle, position)
            if self.state is MissionState.PRECISION_LANDING:
                return self._precision(vehicle, position)
            return self._seeding_output(vehicle, position, command)

        return self._precision(vehicle, position)

    def evidence(self) -> dict:
        mission = self.mission.report()
        weather_records = list(self.supervisor.weather_decisions)
        last_weather = weather_records[-1] if weather_records else None
        return {
            "schema": 1,
            "kind": "integrated_seeding_sortie",
            "simulated": True,
            "mission_result": self._result_name(),
            "mission_state": self.state.value,
            "controller_owner": self.owner.value,
            "airspace_evidence": _gate_summary(self.supervisor.airspace_decisions),
            "weather_evidence": {
                "implemented": True,
                "consulted": bool(weather_records),
                "required": self.config.require_weather,
                "authorized": None if last_weather is None else last_weather.get("authorized"),
                "reason_code": None if last_weather is None else last_weather.get("reason_code"),
                "provenance": None if last_weather is None else last_weather.get("provenance"),
                "source": None if last_weather is None else last_weather.get("source"),
                "clock": None if last_weather is None else last_weather.get("clock"),
                "simulated": (last_weather or {}).get("provenance") == "simulated",
                "operational_approval": False,
                "decision_count": len(weather_records),
                "units": "see weather.WeatherDecision",
            },
            "mission_state_history": list(self.transitions),
            "controller_ownership_history": list(self.ownership),
            "release_authorization_history": [item.as_dict() for item in self.supervisor.decisions],
            "release_samples": [
                {
                    "time_s": sample.time_s,
                    "frame": "local_ned",
                    "north_m": sample.north,
                    "east_m": sample.east,
                    "down_m": sample.down,
                    "altitude_m": sample.altitude_m,
                    "amount_kg": sample.amount_kg,
                    "source": "seeding_mission",
                    "simulated": True,
                    "estimated": True,
                }
                for sample in self.mission.samples
            ],
            "altitude_evidence": self.supervisor.evidence(),
            "return_progress": list(self.return_progress),
            "fault_events": list(self.faults),
            "dispenser_closed": self._dispenser_closed,
            "release_aborted": self._release_aborted,
            "rejected_external_setpoints": self._rejected_external,
            "precision_landing_evidence": {
                "state": self.lander.state.value,
                "failed": self.lander.failed,
                "active": self.lander.active,
                "timeout_s": self.lander.params.target_timeout_s,
                "max_fix_jump_m": self.lander.params.max_fix_jump_m,
                "source": "precision_land",
                "descent_tag_age_s": {
                    "min": None if not self._descent_tag_ages else min(self._descent_tag_ages),
                    "max": None if not self._descent_tag_ages else max(self._descent_tag_ages),
                    "samples": len(self._descent_tag_ages),
                    "units": "s",
                    "source": "precision_land_command.tag_age_s",
                    "note": "Age of the fix used on each descend tick. The 5 s timeout is unchanged.",
                },
            },
            "mission": mission,
            "config": {
                "require_weather": self.config.require_weather,
                "offboard_gap_s": self.config.offboard_gap_s,
                "landing_timeout_s": self.config.landing_timeout_s,
                "staging_altitude_m": self.config.staging.staging_altitude_m,
                "dispenser_ack": self.config.dispenser_ack,
            },
        }

    def _result_name(self) -> str:
        if self.state is MissionState.LANDED:
            return "completed"
        if self.state is MissionState.ABORTED:
            return "aborted_recovered"
        if self.state is MissionState.FAILED:
            return "failed"
        return "in_progress"

    def _preflight(self, vehicle: VehicleInput, position) -> OrchestratorOutput:
        if position is None or vehicle.attitude_wxyz is None:
            return self._output(vehicle, Owner.NONE, None, False, False, False)
        if not vehicle.armed:
            self._set_owner(Owner.HOLD, vehicle.time_s, "preflight hold")
            return self._output(vehicle, Owner.HOLD, position, True, True, False, heartbeat=True)
        self._goto(MissionState.TAKEOFF, "estimates valid and armed", vehicle.time_s)
        self._offboard_wanted = True
        command = self._seeding_step(vehicle, position)
        self._follow_seeding(vehicle, position, command)
        if self.state is MissionState.PRECISION_LANDING:
            return self._precision(vehicle, position)
        return self._seeding_output(vehicle, position, command)

    def _seeding_step(self, vehicle: VehicleInput, position) -> SeedingCommand:
        assert position is not None
        # landed=False: the seeding law must not finish itself on the ground.
        # Precision landing owns the touchdown after the staging handoff.
        return self.supervisor.step(
            position,
            vehicle.time_s,
            vehicle.altitude,
            landed=False,
            dt_s=max(0.0, vehicle.dt_s),
        )

    def _follow_seeding(self, vehicle: VehicleInput, position, command: SeedingCommand) -> None:
        assert position is not None
        if command.dispense:
            self._dispenser_was_open = True
            self._dispenser_closed = False
        phase = self.mission.phase
        if phase is SeedingPhase.CLIMB:
            low = -position[2] < self.config.takeoff_altitude_m
            dest = MissionState.TAKEOFF if low else MissionState.TRANSIT_TO_RELEASE
            if self.state is MissionState.HOLD_FOR_RELEASE and not low:
                reason = "left station"
            else:
                reason = "climbing" if low else "above takeoff altitude"
            self._goto(dest, reason, vehicle.time_s)
        elif phase is SeedingPhase.HOLD:
            self._goto(MissionState.HOLD_FOR_RELEASE, "on station", vehicle.time_s)
        elif phase is SeedingPhase.DISPENSE:
            self._goto(MissionState.DISPENSING, "hold complete", vehicle.time_s)
        elif phase in (SeedingPhase.RETURN, SeedingPhase.ABORT, SeedingPhase.COMPLETE):
            self._begin_return(vehicle, position, command)

        if self.mission.reason and phase is SeedingPhase.RETURN:
            self._release_aborted = True

    def _begin_return(self, vehicle: VehicleInput, position, command: SeedingCommand) -> None:
        if command.dispense:
            return
        if self.mission.reason == "mission timeout":
            self._fault(vehicle.time_s, "return_timeout", recovery="position_hold", result="failed", position=position)
            self._goto(MissionState.FAILED, "return_timeout", vehicle.time_s)
            self._set_owner(Owner.HOLD, vehicle.time_s, "return timed out")
            self._hold_position = position
            return
        if self._dispenser_was_open and not self.config.dispenser_ack:
            self._dispenser_closed = False
            self._fault(
                vehicle.time_s,
                "dispenser_closure_failed",
                recovery="position_hold",
                result="failed",
                position=position,
            )
            self._goto(MissionState.FAILED, "dispenser_closure_failed", vehicle.time_s)
            self._set_owner(Owner.HOLD, vehicle.time_s, "dispenser did not close")
            self._hold_position = position
            return
        self._dispenser_closed = True
        if self.mission.reason:
            self._release_aborted = True
        self._goto(
            MissionState.RETURN_TO_RECOVERY,
            self.mission.reason or "release complete",
            vehicle.time_s,
            sample_count=len(self.mission.samples),
            dispense=False,
        )
        self._record_progress(vehicle.time_s, position, force=True)
        if self._staging_met(position, vehicle):
            self._handoff(vehicle, position)

    def _handoff(self, vehicle: VehicleInput, position) -> None:
        self._handed_off = True
        self._precision_t0 = vehicle.time_s
        self.lander.activate(position)
        self._goto(MissionState.PRECISION_LANDING, "staging conditions met", vehicle.time_s)
        self._set_owner(Owner.PRECISION_LAND, vehicle.time_s, "handoff")

    def _seeding_output(self, vehicle, position, command: SeedingCommand) -> OrchestratorOutput:
        if self.state in _TERMINAL:
            return self._terminal_output(vehicle, position)
        if self.state is MissionState.RETURN_TO_RECOVERY:
            self._record_progress(vehicle.time_s, position, force=False)
            if self.mission.reason == "mission timeout":
                self._fault(vehicle.time_s, "return_timeout", recovery="position_hold", result="failed", position=position)
                self._goto(MissionState.FAILED, "return_timeout", vehicle.time_s)
                self._set_owner(Owner.HOLD, vehicle.time_s, "return timed out")
                return self._output(vehicle, Owner.HOLD, position, False, False, False, heartbeat=True)
            if self._staging_met(position, vehicle):
                self._handoff(vehicle, position)
                return self._precision(vehicle, position)
            home = self.mission.profile.home_ned
            staged = (home[0], home[1], -self.config.staging.staging_altitude_m)
            self._set_owner(Owner.SEEDING, vehicle.time_s, "return")
            return self._output(vehicle, Owner.SEEDING, staged, False, False, False, heartbeat=True)
        target = command.position_ned
        self._set_owner(Owner.SEEDING, vehicle.time_s, self.state.value)
        return self._output(
            vehicle,
            Owner.SEEDING,
            target,
            command.dispense,
            False,
            False,
            heartbeat=True,
        )

    def _precision(self, vehicle: VehicleInput, position) -> OrchestratorOutput:
        assert position is not None
        if self._precision_t0 is not None and vehicle.time_s - self._precision_t0 > self.config.landing_timeout_s:
            self._fault(vehicle.time_s, "landing_timeout", recovery="position_hold", result="failed", position=position)
            self._goto(MissionState.FAILED, "landing_timeout", vehicle.time_s)
            self._set_owner(Owner.HOLD, vehicle.time_s, "landing timeout")
            return self._output(vehicle, Owner.HOLD, position, False, False, False, heartbeat=True)

        attitude = vehicle.attitude_wxyz
        if attitude is None or len(attitude) != 4 or not all(math.isfinite(float(v)) for v in attitude):
            if not self._attitude_fault:
                self._fault(
                    vehicle.time_s,
                    "attitude_unavailable",
                    recovery="position_hold_without_vision",
                    result="holding",
                    position=position,
                )
                self._attitude_fault = True
            self._hold_position = position
            self._set_owner(Owner.HOLD, vehicle.time_s, "no attitude")
            return self._output(vehicle, Owner.HOLD, position, False, False, False, heartbeat=True)
        self._attitude_fault = False

        velocity = _finite_xyz(vehicle.velocity_ned) or (0.0, 0.0, 0.0)
        snap = VehicleSnapshot(
            position_ned=position,
            velocity_ned=velocity,
            attitude_wxyz=tuple(float(v) for v in attitude),  # type: ignore[arg-type]
            landed=vehicle.landed,
            time_s=vehicle.time_s,
        )
        if vehicle.tag is not None and not self.lander.failed:
            self.lander.observe_tag(vehicle.tag, snap)
        command = self.lander.step(snap)
        if command.state is LandState.DESCEND and command.tag_age_s is not None:
            self._descent_tag_ages.append(float(command.tag_age_s))
        if self.lander.failed:
            self._fault(vehicle.time_s, "marker_lost", recovery="position_hold", result="failed", position=position)
            self._goto(MissionState.FAILED, "marker_lost", vehicle.time_s)
            self._hold_position = command.position_ned or position
            self._set_owner(Owner.PRECISION_LAND, vehicle.time_s, "marker lost")
            return self._from_land(vehicle, command)
        if command.state is LandState.FINISHED and vehicle.landed:
            dest = MissionState.ABORTED if self._release_aborted else MissionState.LANDED
            reason = "recovered after release abort" if self._release_aborted else "precision landing complete"
            self._goto(dest, reason, vehicle.time_s)
            self._set_owner(Owner.NONE, vehicle.time_s, "landed")
            return self._output(vehicle, Owner.NONE, None, False, False, False, heartbeat=False)
        self._set_owner(Owner.PRECISION_LAND, vehicle.time_s, command.state.value)
        return self._from_land(vehicle, command)

    def _from_land(self, vehicle: VehicleInput, command: LandCommand) -> OrchestratorOutput:
        fields = offboard_setpoint(command)
        if fields is None:
            return self._output(vehicle, self.owner, None, False, False, False, heartbeat=False)
        if fields["use_position"]:
            return self._output(
                vehicle,
                self.owner,
                fields["position_ned"],
                False,
                False,
                False,
                heartbeat=True,
                velocity=_nan_velocity(),
            )
        return self._output(
            vehicle,
            self.owner,
            None,
            False,
            False,
            False,
            heartbeat=True,
            velocity=fields["velocity_ned"],
            use_position=False,
        )

    def _staging_met(self, position: tuple[float, float, float], vehicle: VehicleInput) -> bool:
        if vehicle.attitude_wxyz is None:
            return False
        home = self.mission.profile.home_ned
        horizontal = math.hypot(position[0] - home[0], position[1] - home[1])
        altitude = -position[2]
        gate = self.config.staging
        if horizontal > gate.max_horizontal_m:
            return False
        if not (gate.min_altitude_m <= altitude <= gate.max_altitude_m):
            return False
        velocity = _finite_xyz(vehicle.velocity_ned)
        if velocity is None:
            return False
        if math.hypot(velocity[0], velocity[1]) > gate.max_horizontal_speed_mps:
            return False
        return True

    def _terminal_output(self, vehicle, position) -> OrchestratorOutput:
        hold = self._hold_position or position
        if self.owner is Owner.PX4_LAND:
            return self._output(vehicle, Owner.PX4_LAND, None, False, False, True, heartbeat=False)
        if self.owner is Owner.HOLD and hold is not None:
            return self._output(vehicle, Owner.HOLD, hold, False, False, False, heartbeat=True)
        if self.owner is Owner.PRECISION_LAND and hold is not None and self.lander.failed:
            return self._output(
                vehicle,
                Owner.PRECISION_LAND,
                hold,
                False,
                False,
                False,
                heartbeat=True,
                velocity=_nan_velocity(),
            )
        return self._output(vehicle, self.owner, None, False, False, False, heartbeat=False)

    def _output(
        self,
        vehicle: VehicleInput,
        owner: Owner,
        position: tuple[float, float, float] | None,
        dispense: bool,
        request_arm: bool,
        request_nav_land: bool,
        *,
        heartbeat: bool = False,
        velocity: tuple[float, float, float] | None = None,
        use_position: bool = True,
    ) -> OrchestratorOutput:
        if heartbeat and not request_nav_land:
            self._offboard_wanted = True
        return OrchestratorOutput(
            state=self.state,
            owner=owner,
            heartbeat=heartbeat and not request_nav_land,
            dispense=dispense,
            request_arm=request_arm,
            request_offboard=heartbeat and not request_nav_land,
            request_nav_land=request_nav_land,
            position_ned=position if use_position else None,
            velocity_ned=velocity if velocity is not None else (_nan_velocity() if use_position and position is not None else None),
            use_position=use_position and position is not None,
        )

    def _record_progress(self, time_s: float, position, force: bool) -> None:
        if not force and self._progress_t is not None and time_s - self._progress_t < 1.0:
            return
        self._progress_t = time_s
        home = self.mission.profile.home_ned
        self.return_progress.append({
            "time_s": time_s,
            "frame": "local_ned",
            "north_m": position[0],
            "east_m": position[1],
            "down_m": position[2],
            "horizontal_to_home_m": math.hypot(position[0] - home[0], position[1] - home[1]),
            "source": "px4_local_position",
            "simulated": True,
            "estimated": True,
            "units": "m",
        })

    def _note_gap(self, time_s: float) -> None:
        if self._last_step_t is None or not self._offboard_wanted:
            return
        gap = time_s - self._last_step_t
        if gap > self.config.offboard_gap_s:
            self._fault(
                time_s,
                "offboard_stream_gap",
                recovery="publish_this_step",
                result="continuing",
                position=self._last_position,
                extra={"gap_s": gap, "limit_s": self.config.offboard_gap_s},
            )

    def _goto(self, dest: MissionState, reason: str, time_s: float, **context) -> None:
        if dest is self.state:
            return
        if (self.state, dest) not in ALLOWED:
            self._fault(
                time_s,
                "invalid_transition",
                recovery="keep_state",
                result="rejected",
                position=self._last_position,
                extra={"source": self.state.value, "destination": dest.value},
            )
            return
        self.transitions.append({
            "time_s": time_s,
            "source": self.state.value,
            "destination": dest.value,
            "reason": reason,
            "context": context,
            "simulated": True,
        })
        self.state = dest

    def _set_owner(self, owner: Owner, time_s: float, reason: str) -> None:
        if owner is self.owner:
            return
        self.owner = owner
        self.ownership.append({
            "time_s": time_s,
            "owner": owner.value,
            "reason": reason,
            "simulated": True,
        })

    def _fault(
        self,
        time_s: float,
        code: str,
        *,
        recovery: str,
        result: str,
        position,
        extra: dict | None = None,
    ) -> None:
        self.faults.append({
            "code": code,
            "time_s": time_s,
            "mission_state": self.state.value,
            "owner": self.owner.value,
            "recovery": recovery,
            "result": result,
            "position_ned": None if position is None else [position[0], position[1], position[2]],
            "frame": "local_ned",
            "units": "m",
            "simulated": True,
            "estimated": True,
            "source": "px4_local_position",
            **(extra or {}),
        })
