"""Cloud-seeding mission: climb, hold, dispense, return.

Positions are north, east, down. The release volume's altitudes are local
height above the origin. Barometric altitude, altitude above mean sea level,
and height above the ground are separate frames. ``release_violations``
rejects a release sample outside the volume. A caller can deny the release,
which closes the dispenser without recording a sample.

``sim_profile`` is the low-altitude stand-in a kinematic check can fly.
``profile_at_15000_ft`` is the same law with the release at 15,000 ft.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Sequence


# 15,000 ft * 0.3048 m/ft.
RELEASE_ALTITUDE_15000_FT_M = 4572.0


class SeedingPhase(Enum):
    CLIMB = "climb"
    HOLD = "hold"
    DISPENSE = "dispense"
    RETURN = "return"
    COMPLETE = "complete"
    ABORT = "abort"


class CommandMode(Enum):
    POSITION = "position"
    LAND = "land"


@dataclass(frozen=True)
class ReleaseVolume:
    """Axis-aligned release box. Altitudes are metres above the local origin."""

    north_min: float
    north_max: float
    east_min: float
    east_max: float
    altitude_floor_m: float
    altitude_ceiling_m: float

    def __post_init__(self) -> None:
        if self.north_min > self.north_max:
            raise ValueError("north_min must not exceed north_max")
        if self.east_min > self.east_max:
            raise ValueError("east_min must not exceed east_max")
        if self.altitude_floor_m <= 0.0:
            raise ValueError("altitude_floor_m must be positive")
        if self.altitude_floor_m > self.altitude_ceiling_m:
            raise ValueError("altitude_floor_m must not exceed altitude_ceiling_m")

    def contains(self, position_ned: Sequence[float]) -> bool:
        north, east, down = position_ned
        altitude = -down
        return (
            self.north_min <= north <= self.north_max
            and self.east_min <= east <= self.east_max
            and self.altitude_floor_m <= altitude <= self.altitude_ceiling_m
        )


@dataclass(frozen=True)
class SeedingProfile:
    """One seeding sortie. ``rate_kg_s`` is a stand-in release rate."""

    home_ned: tuple[float, float, float]
    release_ned: tuple[float, float, float]
    volume: ReleaseVolume
    hold_s: float
    dispense_s: float
    rate_kg_s: float
    arrival_m: float
    timeout_s: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "home_ned", _xyz(self.home_ned, "home_ned"))
        object.__setattr__(self, "release_ned", _xyz(self.release_ned, "release_ned"))
        if self.hold_s < 0.0:
            raise ValueError("hold_s must be non-negative")
        if self.dispense_s <= 0.0:
            raise ValueError("dispense_s must be positive")
        if self.rate_kg_s <= 0.0:
            raise ValueError("rate_kg_s must be positive")
        if self.arrival_m <= 0.0:
            raise ValueError("arrival_m must be positive")
        if self.timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        if not self.volume.contains(self.release_ned):
            raise ValueError("release point is outside the release volume")


@dataclass(frozen=True)
class ReleaseSample:
    time_s: float
    north: float
    east: float
    down: float
    altitude_m: float
    amount_kg: float


@dataclass(frozen=True)
class SeedingCommand:
    phase: SeedingPhase
    mode: CommandMode
    position_ned: tuple[float, float, float]
    dispense: bool
    reason: str = ""


def sim_profile() -> SeedingProfile:
    """Low-altitude stand-in. The release point is 15 m up, inside a 12–18 m band.

    Straight-line distance from the pad is 35 m, so a kinematic run at a few
    metres per second finishes in well under a minute.
    """
    volume = ReleaseVolume(
        north_min=20.0,
        north_max=40.0,
        east_min=5.0,
        east_max=15.0,
        altitude_floor_m=12.0,
        altitude_ceiling_m=18.0,
    )
    return SeedingProfile(
        home_ned=(0.0, 0.0, 0.0),
        release_ned=(30.0, 10.0, -15.0),
        volume=volume,
        hold_s=2.0,
        dispense_s=4.0,
        rate_kg_s=0.02,
        arrival_m=0.5,
        timeout_s=60.0,
    )


def profile_at_15000_ft() -> SeedingProfile:
    """Same mission with the release at 15,000 ft (4572 m).

    The box is 1 km across and 300 m thick, centered on that altitude.
    """
    altitude = RELEASE_ALTITUDE_15000_FT_M
    volume = ReleaseVolume(
        north_min=-500.0,
        north_max=500.0,
        east_min=-500.0,
        east_max=500.0,
        altitude_floor_m=altitude - 150.0,
        altitude_ceiling_m=altitude + 150.0,
    )
    return SeedingProfile(
        home_ned=(0.0, 0.0, 0.0),
        release_ned=(0.0, 0.0, -altitude),
        volume=volume,
        hold_s=30.0,
        dispense_s=120.0,
        rate_kg_s=0.05,
        arrival_m=5.0,
        timeout_s=3600.0,
    )


def release_violations(samples: Sequence[ReleaseSample], volume: ReleaseVolume) -> list[str]:
    """Reasons the release record is illegal. An empty list is a legal release."""
    problems: list[str] = []
    previous = -math.inf
    for sample in samples:
        where = (
            f"t={sample.time_s:.3f}s ({sample.north:.1f} n, {sample.east:.1f} e, "
            f"{sample.altitude_m:.1f} m)"
        )
        if sample.time_s < previous:
            problems.append(f"release time went backwards at {where}")
        previous = sample.time_s
        if sample.amount_kg < 0.0:
            problems.append(f"negative release amount at {where}")
        if abs(sample.altitude_m - (-sample.down)) > 1e-6:
            problems.append(f"release altitude does not match down at {where}")
        if sample.altitude_m < volume.altitude_floor_m:
            problems.append(
                f"release at {where} is below the {volume.altitude_floor_m:.1f} m floor"
            )
            continue
        if sample.altitude_m > volume.altitude_ceiling_m:
            problems.append(
                f"release at {where} is above the {volume.altitude_ceiling_m:.1f} m ceiling"
            )
            continue
        if not (
            volume.north_min <= sample.north <= volume.north_max
            and volume.east_min <= sample.east <= volume.east_max
        ):
            problems.append(f"release at {where} is outside the release area")
    return problems


class SeedingMission:
    """Climb to the release point, hold, dispense, then return to the pad.

    Leaving the volume after the dispenser has opened closes it on that tick
    and sends the vehicle home. The recorded samples stay inside the volume.
    A full release ends COMPLETE. A closed-early release ends ABORT, after landing.
    """

    def __init__(self, profile: SeedingProfile) -> None:
        self.profile = profile
        self._phase = SeedingPhase.CLIMB
        self._reason = ""
        self._t0: float | None = None
        self._last_t: float | None = None
        self._hold_started: float | None = None
        self._open_s = 0.0
        self._amount = 0.0
        self._samples: list[ReleaseSample] = []

    @property
    def phase(self) -> SeedingPhase:
        return self._phase

    @property
    def reason(self) -> str:
        return self._reason

    @property
    def samples(self) -> tuple[ReleaseSample, ...]:
        return tuple(self._samples)

    def report(self) -> dict:
        return {
            "phase": self._phase.value,
            "reason": self._reason,
            "release_s": self._open_s,
            "amount_kg": self._amount,
            "sample_count": len(self._samples),
            "violations": release_violations(self._samples, self.profile.volume),
        }

    def step(
        self,
        position_ned: Sequence[float],
        time_s: float,
        *,
        landed: bool = False,
        dt_s: float = 0.0,
        release_authorized: bool = True,
        release_denial: str = "",
    ) -> SeedingCommand:
        position = _xyz(position_ned, "position_ned")
        if self._last_t is not None and time_s < self._last_t:
            raise ValueError(f"time went backwards: {time_s} < {self._last_t}")
        if dt_s < 0.0:
            raise ValueError("dt_s must be non-negative")
        if self._t0 is None:
            self._t0 = time_s
        self._last_t = time_s

        if self._phase in (SeedingPhase.COMPLETE, SeedingPhase.ABORT):
            return self._command(self._phase, CommandMode.LAND, self.profile.home_ned, dispense=False)

        if self._reason:
            self._phase = SeedingPhase.RETURN
            return self._returning(position, landed)

        if time_s - self._t0 >= self.profile.timeout_s:
            self._reason = "mission timeout"
            self._phase = SeedingPhase.RETURN
            return self._returning(position, landed)

        if self._phase is SeedingPhase.CLIMB:
            if self._on_station(position):
                self._phase = SeedingPhase.HOLD
                self._hold_started = time_s
            else:
                return self._command(SeedingPhase.CLIMB, CommandMode.POSITION, self.profile.release_ned)

        if self._phase is SeedingPhase.HOLD:
            if not self._on_station(position):
                self._phase = SeedingPhase.CLIMB
                self._hold_started = None
                return self._command(SeedingPhase.CLIMB, CommandMode.POSITION, self.profile.release_ned)
            assert self._hold_started is not None
            if time_s - self._hold_started < self.profile.hold_s:
                return self._command(SeedingPhase.HOLD, CommandMode.POSITION, self.profile.release_ned)
            self._phase = SeedingPhase.DISPENSE

        if self._phase is SeedingPhase.DISPENSE:
            return self._dispense(
                position,
                time_s,
                dt_s,
                landed,
                release_authorized=release_authorized,
                release_denial=release_denial,
            )

        return self._returning(position, landed)

    def _on_station(self, position: tuple[float, float, float]) -> bool:
        return _near(position, self.profile.release_ned, self.profile.arrival_m) and self.profile.volume.contains(
            position
        )

    def _dispense(
        self,
        position: tuple[float, float, float],
        time_s: float,
        dt_s: float,
        landed: bool,
        *,
        release_authorized: bool,
        release_denial: str,
    ) -> SeedingCommand:
        if not release_authorized:
            self._reason = release_denial or "release not authorized"
            self._phase = SeedingPhase.RETURN
            return self._returning(position, landed)
        if not self.profile.volume.contains(position):
            self._reason = "left release volume"
            self._phase = SeedingPhase.RETURN
            return self._returning(position, landed)
        remaining = self.profile.dispense_s - self._open_s
        interval = min(dt_s, remaining)
        if interval > 0.0:
            amount = self.profile.rate_kg_s * interval
            self._open_s += interval
            self._amount += amount
            north, east, down = position
            self._samples.append(
                ReleaseSample(
                    time_s=time_s,
                    north=north,
                    east=east,
                    down=down,
                    altitude_m=-down,
                    amount_kg=amount,
                )
            )
        command = self._command(
            SeedingPhase.DISPENSE,
            CommandMode.POSITION,
            self.profile.release_ned,
            dispense=interval > 0.0,
        )
        if self._open_s >= self.profile.dispense_s - 1e-9:
            self._phase = SeedingPhase.RETURN
        return command

    def _returning(self, position: tuple[float, float, float], landed: bool) -> SeedingCommand:
        home = self.profile.home_ned
        if not _near(position, home, self.profile.arrival_m):
            return self._command(SeedingPhase.RETURN, CommandMode.POSITION, home)
        if not landed:
            return self._command(SeedingPhase.RETURN, CommandMode.LAND, home)
        self._phase = SeedingPhase.ABORT if self._reason else SeedingPhase.COMPLETE
        return self._command(self._phase, CommandMode.LAND, home)

    def _command(
        self,
        phase: SeedingPhase,
        mode: CommandMode,
        position_ned: tuple[float, float, float],
        *,
        dispense: bool = False,
    ) -> SeedingCommand:
        return SeedingCommand(phase, mode, position_ned, dispense, self._reason)


def _xyz(value: Sequence[float], name: str) -> tuple[float, float, float]:
    if len(value) != 3:
        raise ValueError(f"{name} must be north, east, down")
    return (float(value[0]), float(value[1]), float(value[2]))


def _near(position: Sequence[float], target: Sequence[float], radius_m: float) -> bool:
    return math.dist(position, target) <= radius_m
