"""Telemetry as the adapter reported it. Samples are copied from
``VehicleInterface.state`` after each tick and never synthesized or edited;
assertions and the report refer to them by index."""

from __future__ import annotations

import csv
import hashlib
import io
from dataclasses import astuple, dataclass, fields

from px4_offboard.vehicle.interface import VehicleState


@dataclass(frozen=True)
class Sample:
    index: int
    t_s: float  # adapter clock, seconds since connect
    step: int  # mission step index; -1 while initializing
    north_m: float
    east_m: float
    down_m: float
    vn_mps: float
    ve_mps: float
    vd_mps: float
    armed: bool
    # Velocity setpoint sent before this sample; None when none was sent (initialization).
    cmd_vn_mps: float | None
    cmd_ve_mps: float | None
    cmd_vd_mps: float | None

    @property
    def altitude_m(self) -> float:
        return -self.down_m


COLUMNS = tuple(f.name for f in fields(Sample))


class TelemetryLog:
    def __init__(self) -> None:
        self.samples: list[Sample] = []

    def record(self, state: VehicleState, step: int, command: tuple[float, float, float] | None) -> Sample:
        n, e, d = state.position_ned
        vn, ve, vd = state.velocity_ned
        sample = Sample(len(self.samples), state.t_s, step, n, e, d, vn, ve, vd, state.armed, *(command or (None,) * 3))
        self.samples.append(sample)
        return sample

    def to_csv(self) -> str:
        buf = io.StringIO()
        writer = csv.writer(buf, lineterminator="\n")
        writer.writerow(COLUMNS)
        for s in self.samples:
            writer.writerow(
                "" if v is None else repr(v) if isinstance(v, float) else int(v) if isinstance(v, bool) else v
                for v in astuple(s)
            )
        return buf.getvalue()

    @staticmethod
    def sha256(text: str) -> str:
        return hashlib.sha256(text.encode()).hexdigest()
