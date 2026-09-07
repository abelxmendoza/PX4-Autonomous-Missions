"""Pure localization-state logic for GPS-denied mission scenarios.

ROS and PX4 sensor injection live in ``offboard_mission``; this module keeps
zone classification, source selection, transitions, and policy testable.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto


class LocalizationSource(Enum):
    GPS = "GPS"
    VIO = "VIO"
    GPS_DENIED_INJECTED = "GPS_DENIED_INJECTED"
    DEAD_RECKONING = "DEAD_RECKONING"
    UNKNOWN = "UNKNOWN"


class LocalizationPhase(Enum):
    GPS_OK = auto()
    IN_DENIED_ZONE = auto()
    GPS_INVALID = auto()
    NON_GPS_ACTIVE = auto()
    LOC_FAILSAFE = auto()


class LocalizationEvent(Enum):
    GPS_OK = "GPS_OK"
    ENTER_DENIED_ZONE = "ENTER_DENIED_ZONE"
    GPS_INVALID = "GPS_INVALID"
    NON_GPS_ACTIVE = "NON_GPS_ACTIVE"
    GPS_RESTORED = "GPS_RESTORED"
    LOC_FAILSAFE = "LOC_FAILSAFE"


# hold | land → failsafe when injected deny without healthy non-GPS
# continue → telemetry only
FAILSAFE_ACTIONS = frozenset({"hold", "land"})


@dataclass(frozen=True)
class GpsDeniedZone:
    """Axis-aligned GPS-denied prism in NED metres."""

    north_min: float
    north_max: float
    east_min: float
    east_max: float
    down_min: float = -12.0
    down_max: float = 0.5

    def contains(self, north_m: float, east_m: float, down_m: float) -> bool:
        return (
            self.north_min <= north_m <= self.north_max
            and self.east_min <= east_m <= self.east_max
            and self.down_min <= down_m <= self.down_max
        )


# Mid-course prism crossed by DEFAULT_COURSE (N≈15→32 along E≈0).
# The 16 m length intentionally exceeds PX4's GNSS fusion timeout so the
# simulated sensor failure becomes observable before the vehicle exits.
DEFAULT_GPS_DENIED_ZONE = GpsDeniedZone(
    north_min=15.5,
    north_max=31.5,
    east_min=-8.0,
    east_max=8.0,
    down_min=-12.0,
    down_max=0.5,
)


@dataclass(frozen=True)
class LocalizationInputs:
    north_m: float
    east_m: float
    down_m: float
    gps_xy_valid: bool
    dead_reckoning: bool = False
    eph_m: float | None = None
    inject_deny: bool = False
    non_gps_healthy: bool = False
    action: str = "hold"
    enabled: bool = True


@dataclass(frozen=True)
class LocalizationSnapshot:
    phase: LocalizationPhase
    source: LocalizationSource
    event: LocalizationEvent | None
    in_zone: bool
    gps_injected_deny: bool
    gps_xy_valid: bool
    raw_gps_xy_valid: bool
    dead_reckoning: bool
    eph_m: float | None
    failsafe: bool
    failsafe_reason: str | None


def classify_source(
    *,
    gps_injected_deny: bool,
    dead_reckoning: bool,
    raw_gps_xy_valid: bool,
    non_gps_healthy: bool,
) -> LocalizationSource:
    if non_gps_healthy and (gps_injected_deny or not raw_gps_xy_valid):
        return LocalizationSource.VIO
    if gps_injected_deny:
        return LocalizationSource.GPS_DENIED_INJECTED
    if non_gps_healthy:
        return LocalizationSource.DEAD_RECKONING
    if dead_reckoning:
        return LocalizationSource.DEAD_RECKONING
    if raw_gps_xy_valid:
        return LocalizationSource.GPS
    return LocalizationSource.UNKNOWN


def policy_requires_failsafe(
    action: str,
    gps_injected_deny: bool,
    non_gps_healthy: bool,
) -> bool:
    """Injected deny without healthy non-GPS requires a hold/land failsafe."""
    if not gps_injected_deny or non_gps_healthy:
        return False
    return action.lower() in FAILSAFE_ACTIONS


class LocalizationStateMachine:
    """Tracks localization phase and emits transition events."""

    def __init__(self, zone: GpsDeniedZone | None = None) -> None:
        self.zone = zone or DEFAULT_GPS_DENIED_ZONE
        self.phase = LocalizationPhase.GPS_OK
        self._last_event: LocalizationEvent | None = LocalizationEvent.GPS_OK

    def reset(self) -> None:
        self.phase = LocalizationPhase.GPS_OK
        self._last_event = LocalizationEvent.GPS_OK

    def update(self, inputs: LocalizationInputs) -> LocalizationSnapshot:
        in_zone = (
            self.zone.contains(inputs.north_m, inputs.east_m, inputs.down_m)
            if inputs.enabled
            else False
        )
        gps_injected_deny = bool(
            inputs.enabled and inputs.inject_deny and in_zone
        )
        effective_valid = bool(inputs.gps_xy_valid) and not gps_injected_deny
        source = classify_source(
            gps_injected_deny=gps_injected_deny,
            dead_reckoning=inputs.dead_reckoning,
            raw_gps_xy_valid=bool(inputs.gps_xy_valid),
            non_gps_healthy=inputs.non_gps_healthy,
        )

        event: LocalizationEvent | None = None
        failsafe = False
        failsafe_reason: str | None = None

        if self.phase == LocalizationPhase.LOC_FAILSAFE:
            # Sticky until reset — keep reporting failsafe.
            failsafe = True
            failsafe_reason = "localization failsafe (GPS denied, no non-GPS fix)"
            return LocalizationSnapshot(
                phase=self.phase,
                source=source,
                event=None,
                in_zone=in_zone,
                gps_injected_deny=gps_injected_deny,
                gps_xy_valid=effective_valid,
                raw_gps_xy_valid=bool(inputs.gps_xy_valid),
                dead_reckoning=bool(inputs.dead_reckoning),
                eph_m=inputs.eph_m,
                failsafe=failsafe,
                failsafe_reason=failsafe_reason,
            )

        # Desired phase from inputs (before policy failsafe).
        if not inputs.enabled or (not in_zone and effective_valid):
            desired = LocalizationPhase.GPS_OK
        elif in_zone and effective_valid and not gps_injected_deny:
            desired = LocalizationPhase.IN_DENIED_ZONE
        elif inputs.non_gps_healthy and (gps_injected_deny or not effective_valid):
            desired = LocalizationPhase.NON_GPS_ACTIVE
        elif gps_injected_deny or not effective_valid:
            desired = LocalizationPhase.GPS_INVALID
        elif in_zone:
            desired = LocalizationPhase.IN_DENIED_ZONE
        else:
            desired = LocalizationPhase.GPS_OK

        if policy_requires_failsafe(
            inputs.action, gps_injected_deny, inputs.non_gps_healthy
        ):
            desired = LocalizationPhase.LOC_FAILSAFE
            failsafe = True
            failsafe_reason = (
                f"localization failsafe (action={inputs.action.lower()}; "
                "injected GPS deny without healthy non-GPS)"
            )

        if desired != self.phase:
            event = self._transition_event(self.phase, desired, in_zone)
            self.phase = desired
            self._last_event = event
        else:
            event = None

        return LocalizationSnapshot(
            phase=self.phase,
            source=source,
            event=event,
            in_zone=in_zone,
            gps_injected_deny=gps_injected_deny,
            gps_xy_valid=effective_valid,
            raw_gps_xy_valid=bool(inputs.gps_xy_valid),
            dead_reckoning=bool(inputs.dead_reckoning),
            eph_m=inputs.eph_m,
            failsafe=failsafe,
            failsafe_reason=failsafe_reason,
        )

    @staticmethod
    def _transition_event(
        old: LocalizationPhase,
        new: LocalizationPhase,
        in_zone: bool,
    ) -> LocalizationEvent:
        if new == LocalizationPhase.LOC_FAILSAFE:
            return LocalizationEvent.LOC_FAILSAFE
        if new == LocalizationPhase.NON_GPS_ACTIVE:
            return LocalizationEvent.NON_GPS_ACTIVE
        if new == LocalizationPhase.GPS_INVALID:
            return LocalizationEvent.GPS_INVALID
        if new == LocalizationPhase.IN_DENIED_ZONE:
            return LocalizationEvent.ENTER_DENIED_ZONE
        if new == LocalizationPhase.GPS_OK:
            if old != LocalizationPhase.GPS_OK:
                return LocalizationEvent.GPS_RESTORED
            return LocalizationEvent.GPS_OK
        if in_zone:
            return LocalizationEvent.ENTER_DENIED_ZONE
        return LocalizationEvent.GPS_OK
