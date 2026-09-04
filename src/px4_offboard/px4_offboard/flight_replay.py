"""Offline flight-log replay utilities (pure Python, no ROS).

Loads CSV telemetry written by ``offboard_mission`` into a structured
``FlightTrace`` for inspection and V&V checks.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Sequence


AIRBORNE_STATES = frozenset({"TAKEOFF", "HOVER", "MOVE"})
TERMINAL_STATES = frozenset({"LANDING", "FAILSAFE"})


@dataclass(frozen=True)
class FlightSample:
    """One control-loop telemetry row."""

    t_s: float
    state: str
    north: float
    east: float
    down: float
    tgt_n: float
    tgt_e: float
    tgt_d: float
    obstacle: str
    wp_index: int
    geocage: bool
    geofence: bool
    inside: bool
    caged: bool
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    vn: float = 0.0
    ve: float = 0.0
    vd: float = 0.0
    executive_mode: str | None = None
    battery_frac: float | None = None
    link_quality: float | None = None
    propellant_s: float | None = None
    obstacle_source: str | None = None
    sensor_fresh: bool = False
    lidar_front_m: float | None = None
    lidar_left_m: float | None = None
    lidar_right_m: float | None = None
    mapped_clearance_m: float | None = None
    nominal_n: float | None = None
    nominal_e: float | None = None
    nominal_d: float | None = None

    @property
    def altitude_m(self) -> float:
        return -self.down

    @property
    def position(self) -> tuple[float, float, float]:
        return (self.north, self.east, self.down)

    @property
    def setpoint(self) -> tuple[float, float, float]:
        return (self.tgt_n, self.tgt_e, self.tgt_d)


@dataclass
class FlightTrace:
    samples: list[FlightSample] = field(default_factory=list)
    source: str = ""
    columns: tuple[str, ...] = ()

    def __len__(self) -> int:
        return len(self.samples)

    @property
    def duration_s(self) -> float:
        if len(self.samples) < 2:
            return 0.0
        return self.samples[-1].t_s - self.samples[0].t_s

    def states(self) -> list[str]:
        return [s.state for s in self.samples]

    def unique_state_sequence(self) -> list[str]:
        """Collapse consecutive duplicates: MOVE,MOVE,LANDING → MOVE,LANDING."""
        seq: list[str] = []
        for sample in self.samples:
            if not seq or seq[-1] != sample.state:
                seq.append(sample.state)
        return seq

    def summary(self) -> dict:
        states = self.unique_state_sequence()
        avoid_rows = sum(1 for s in self.samples if s.obstacle)
        breach_rows = sum(1 for s in self.samples if not s.inside and s.geofence)
        clearances = [
            s.mapped_clearance_m
            for s in self.samples
            if s.mapped_clearance_m is not None
        ]
        return {
            "source": self.source,
            "samples": len(self.samples),
            "duration_s": round(self.duration_s, 2),
            "state_sequence": states,
            "max_altitude_m": round(
                max((s.altitude_m for s in self.samples), default=0.0), 2
            ),
            "final_wp_index": self.samples[-1].wp_index if self.samples else -1,
            "avoidance_samples": avoid_rows,
            "geofence_breach_samples": breach_rows,
            "has_executive": any(s.executive_mode for s in self.samples),
            "sensor_evidence_samples": sum(1 for s in self.samples if s.sensor_fresh),
            "minimum_mapped_clearance_m": (
                round(min(clearances), 3) if clearances else None
            ),
        }


def _parse_clock(value: str, day_offset_s: float) -> float:
    """Parse HH:MM:SS.mmm wall time into seconds (+ day_offset if wrap)."""
    parts = value.strip().split(":")
    if len(parts) != 3:
        raise ValueError(f"bad time field: {value!r}")
    hours = int(parts[0])
    minutes = int(parts[1])
    seconds = float(parts[2])
    return day_offset_s + hours * 3600.0 + minutes * 60.0 + seconds


def _as_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    return text in {"1", "true", "yes", "on"}


def _as_float(value: object, default: float = 0.0) -> float:
    if value is None or value == "":
        return default
    return float(value)


def _as_optional_float(value: object) -> float | None:
    if value is None or value == "":
        return None
    return float(value)


def load_flight_log(path: str | Path) -> FlightTrace:
    """Load an ``offboard_mission`` CSV into a ``FlightTrace``."""
    path = Path(path)
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"empty or headerless log: {path}")
        columns = tuple(reader.fieldnames)
        rows = list(reader)

    samples: list[FlightSample] = []
    day_offset = 0.0
    prev_raw: float | None = None
    t0: float | None = None

    for row in rows:
        raw = _parse_clock(row["time"], day_offset)
        if prev_raw is not None and raw + 1.0 < prev_raw:
            # Midnight wrap
            day_offset += 86400.0
            raw = _parse_clock(row["time"], day_offset)
        prev_raw = raw
        if t0 is None:
            t0 = raw
        t_s = raw - t0

        samples.append(
            FlightSample(
                t_s=t_s,
                state=str(row["state"]).strip().upper(),
                north=_as_float(row.get("north")),
                east=_as_float(row.get("east")),
                down=_as_float(row.get("down")),
                tgt_n=_as_float(row.get("tgt_n")),
                tgt_e=_as_float(row.get("tgt_e")),
                tgt_d=_as_float(row.get("tgt_d")),
                obstacle=str(row.get("obstacle") or "").strip(),
                wp_index=int(float(row.get("wp_index") or 0)),
                geocage=_as_bool(row.get("geocage", 0)),
                geofence=_as_bool(row.get("geofence", 0)),
                inside=_as_bool(row.get("inside", 1)),
                caged=_as_bool(row.get("caged", 0)),
                roll_deg=_as_float(row.get("roll_deg")),
                pitch_deg=_as_float(row.get("pitch_deg")),
                yaw_deg=_as_float(row.get("yaw_deg")),
                vn=_as_float(row.get("vn")),
                ve=_as_float(row.get("ve")),
                vd=_as_float(row.get("vd")),
                executive_mode=(
                    str(row["executive_mode"]).strip().upper()
                    if row.get("executive_mode")
                    else None
                ),
                battery_frac=_as_optional_float(row.get("battery_frac")),
                link_quality=_as_optional_float(row.get("link_quality")),
                propellant_s=_as_optional_float(row.get("propellant_s")),
                obstacle_source=(
                    str(row["obstacle_source"]).strip().lower()
                    if row.get("obstacle_source")
                    else None
                ),
                sensor_fresh=_as_bool(row.get("sensor_fresh", 0)),
                lidar_front_m=_as_optional_float(row.get("lidar_front_m")),
                lidar_left_m=_as_optional_float(row.get("lidar_left_m")),
                lidar_right_m=_as_optional_float(row.get("lidar_right_m")),
                mapped_clearance_m=_as_optional_float(row.get("mapped_clearance_m")),
                nominal_n=_as_optional_float(row.get("nominal_n")),
                nominal_e=_as_optional_float(row.get("nominal_e")),
                nominal_d=_as_optional_float(row.get("nominal_d")),
            )
        )

    return FlightTrace(samples=samples, source=str(path), columns=columns)


def write_flight_log(path: str | Path, samples: Sequence[FlightSample]) -> None:
    """Write samples in the mission CSV schema (for fixtures / synthetic runs)."""
    path = Path(path)
    fieldnames = [
        "time",
        "state",
        "north",
        "east",
        "down",
        "tgt_n",
        "tgt_e",
        "tgt_d",
        "obstacle",
        "obstacle_source",
        "sensor_fresh",
        "lidar_front_m",
        "lidar_left_m",
        "lidar_right_m",
        "mapped_clearance_m",
        "nominal_n",
        "nominal_e",
        "nominal_d",
        "wp_index",
        "geocage",
        "geofence",
        "inside",
        "caged",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "vn",
        "ve",
        "vd",
        "executive_mode",
        "battery_frac",
        "link_quality",
        "propellant_s",
    ]
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for sample in samples:
            hours = int(sample.t_s // 3600)
            minutes = int((sample.t_s % 3600) // 60)
            secs = sample.t_s % 60.0
            writer.writerow(
                {
                    "time": f"{hours:02d}:{minutes:02d}:{secs:06.3f}",
                    "state": sample.state,
                    "north": sample.north,
                    "east": sample.east,
                    "down": sample.down,
                    "tgt_n": sample.tgt_n,
                    "tgt_e": sample.tgt_e,
                    "tgt_d": sample.tgt_d,
                    "obstacle": sample.obstacle,
                    "obstacle_source": sample.obstacle_source or "",
                    "sensor_fresh": int(sample.sensor_fresh),
                    "lidar_front_m": "" if sample.lidar_front_m is None else sample.lidar_front_m,
                    "lidar_left_m": "" if sample.lidar_left_m is None else sample.lidar_left_m,
                    "lidar_right_m": "" if sample.lidar_right_m is None else sample.lidar_right_m,
                    "mapped_clearance_m": "" if sample.mapped_clearance_m is None else sample.mapped_clearance_m,
                    "nominal_n": "" if sample.nominal_n is None else sample.nominal_n,
                    "nominal_e": "" if sample.nominal_e is None else sample.nominal_e,
                    "nominal_d": "" if sample.nominal_d is None else sample.nominal_d,
                    "wp_index": sample.wp_index,
                    "geocage": int(sample.geocage),
                    "geofence": int(sample.geofence),
                    "inside": int(sample.inside),
                    "caged": int(sample.caged),
                    "roll_deg": sample.roll_deg,
                    "pitch_deg": sample.pitch_deg,
                    "yaw_deg": sample.yaw_deg,
                    "vn": sample.vn,
                    "ve": sample.ve,
                    "vd": sample.vd,
                    "executive_mode": sample.executive_mode or "",
                    "battery_frac": (
                        "" if sample.battery_frac is None else sample.battery_frac
                    ),
                    "link_quality": (
                        "" if sample.link_quality is None else sample.link_quality
                    ),
                    "propellant_s": (
                        "" if sample.propellant_s is None else sample.propellant_s
                    ),
                }
            )


def iter_transitions(trace: FlightTrace) -> Iterable[tuple[str, str, float]]:
    """Yield (from_state, to_state, t_s) for each state change."""
    prev: FlightSample | None = None
    for sample in trace.samples:
        if prev is not None and sample.state != prev.state:
            yield prev.state, sample.state, sample.t_s
        prev = sample
