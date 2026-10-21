"""Simulation-only fleet experiment contract and preflight validation.

No subprocesses are launched here. This module validates configurations before
a future parameterized SITL adapter consumes them.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys

from px4_offboard.swarm_fleet_layout import make_fleet

SUPPORTED_SCENARIOS = frozenset({"cooperative_point_visits"})
MIN_SPAWN_SEPARATION_M = 3.5
BASE_MAVLINK_PORT = 18570
BASE_DDS_PORT = 8889


@dataclass(frozen=True)
class VehicleConfig:
    vehicle_id: str
    instance: int
    system_id: int
    mavlink_port: int
    spawn_north_m: float
    spawn_east_m: float


@dataclass(frozen=True)
class ExperimentConfig:
    scenario: str
    seed: int
    vehicles: tuple[VehicleConfig, ...]
    fault_schedule: tuple[dict, ...]
    output_dir: str

    def as_dict(self) -> dict:
        return {
            "scenario": self.scenario,
            "seed": self.seed,
            "vehicles": [vars(v) for v in self.vehicles],
            "fault_schedule": list(self.fault_schedule),
            "output_dir": self.output_dir,
        }


def _integer(value: object, label: str, minimum: int, maximum: int) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"{label} must be an integer in {minimum}..{maximum}")
    return value


def validate_config(raw: object) -> ExperimentConfig:
    if not isinstance(raw, dict):
        raise ValueError("experiment must be a JSON object")
    allowed = {"scenario", "seed", "fleet_size", "vehicles", "fault_schedule", "output_dir"}
    extra = set(raw) - allowed
    if extra:
        raise ValueError(f"unknown configuration keys: {sorted(extra)}")
    scenario = raw.get("scenario")
    if scenario not in SUPPORTED_SCENARIOS:
        raise ValueError(f"unsupported scenario: {scenario!r}")
    count = _integer(raw.get("fleet_size"), "fleet_size", 1, 4)
    seed = _integer(raw.get("seed"), "seed", 0, 2**32 - 1)
    output = raw.get("output_dir")
    if not isinstance(output, str) or not output.strip():
        raise ValueError("output_dir must be a nonempty path")
    if "\\x00" in output:
        raise ValueError("output_dir contains NUL")
    schedule = raw.get("fault_schedule", [])
    if not isinstance(schedule, list):
        raise ValueError("fault_schedule must be a list")
    if schedule:
        raise ValueError("fault schedules are not supported by the initial preflight contract")
    fleet = make_fleet(count)
    supplied = raw.get("vehicles")
    if supplied is None:
        supplied = [
            {"vehicle_id": name, "instance": i, "system_id": i + 1,
             "mavlink_port": BASE_MAVLINK_PORT + i,
             "spawn_north_m": point[0], "spawn_east_m": point[1]}
            for i, (name, point) in enumerate(fleet.homes.items(), start=1)
        ]
    if not isinstance(supplied, list) or len(supplied) != count:
        raise ValueError("vehicles must contain exactly fleet_size entries")
    vehicles = []
    for row in supplied:
        if not isinstance(row, dict) or set(row) != set(VehicleConfig.__dataclass_fields__):
            raise ValueError("vehicle fields must exactly match the vehicle contract")
        instance = _integer(row["instance"], "instance", 1, 4)
        system_id = _integer(row["system_id"], "system_id", 2, 5)
        port = _integer(row["mavlink_port"], "mavlink_port", 1024, 65535)
        name = row["vehicle_id"]
        if not isinstance(name, str) or name != f"px4_{instance}":
            raise ValueError("vehicle_id must match its PX4 instance")
        coords = (row["spawn_north_m"], row["spawn_east_m"])
        if any(type(x) not in (int, float) or not math.isfinite(x) for x in coords):
            raise ValueError("spawn coordinates must be finite numbers")
        vehicles.append(VehicleConfig(name, instance, system_id, port, *coords))
    for attr in ("vehicle_id", "instance", "system_id", "mavlink_port"):
        values = [getattr(v, attr) for v in vehicles]
        if len(values) != len(set(values)):
            raise ValueError(f"duplicate {attr}")
    if sorted(v.instance for v in vehicles) != list(range(1, count + 1)):
        raise ValueError("instances must be contiguous starting at 1")
    for v in vehicles:
        if v.system_id != v.instance + 1 or v.mavlink_port != BASE_MAVLINK_PORT + v.instance:
            raise ValueError("system IDs and ports must match existing PX4 runner assignments")
    for i, a in enumerate(vehicles):
        for b in vehicles[i + 1:]:
            if math.hypot(a.spawn_north_m - b.spawn_north_m,
                          a.spawn_east_m - b.spawn_east_m) < MIN_SPAWN_SEPARATION_M:
                raise ValueError(f"overlapping spawns: {a.vehicle_id}, {b.vehicle_id}")
    return ExperimentConfig(scenario, seed, tuple(vehicles), tuple(), output)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Validate fleet experiment JSON without launching SITL")
    parser.add_argument("config", type=Path)
    args = parser.parse_args(argv)
    try:
        config = validate_config(json.loads(args.config.read_text()))
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"preflight failed: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(config.as_dict(), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
