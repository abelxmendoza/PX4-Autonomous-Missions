"""Validate simulation experiment inputs without launching ROS/PX4 or touching hardware.

This is a preflight contract, not an N-vehicle SITL runner or a verification result.
The existing two-vehicle runner deliberately remains unchanged.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import itertools
import json
import math
from pathlib import Path

from .mission_logic import DEFAULT_OBSTACLE_COURSE
from .swarm_fleet_layout import make_fleet
from .swarm_logic import FENCE, GPS_DENIED_ZONE, Fleet, SurveyCoordinator


def _integer(value, name, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"{name} must be an integer in {low}..{high}")
    return value


def _keys(value, required, optional=()):
    if not isinstance(value, dict):
        raise ValueError("expected a JSON object")
    missing = set(required) - value.keys()
    unknown = value.keys() - set(required) - set(optional)
    if missing or unknown:
        raise ValueError(f"missing fields: {sorted(missing)}; unknown fields: {sorted(unknown)}")


@dataclass(frozen=True)
class VehicleBinding:
    vehicle: str
    namespace: str
    px4_instance: int
    target_system_id: int
    mavlink_gcs_port: int


@dataclass(frozen=True)
class Fault:
    vehicle: str
    kind: str
    after_ready_s: float


@dataclass(frozen=True)
class Experiment:
    scenario: str
    seed: int
    output_dir: str
    fleet: Fleet
    vehicles: tuple[VehicleBinding, ...]
    faults: tuple[Fault, ...]

    def to_dict(self):
        return {
            "schema": 1, "stage": "preflight_only", "simulation_only": True,
            "scenario": self.scenario, "seed": self.seed,
            "seed_scope": "experiment identity; no randomized physics is configured",
            "output_dir": self.output_dir,
            "vehicles": [asdict(v) for v in self.vehicles],
            "fleet": asdict(self.fleet), "faults": [asdict(f) for f in self.faults],
            "dds_agent_port": 8889,
            "fault_clock": "seconds after the target vehicle first becomes READY",
            "limitations": ["Not executed against PX4/Gazebo by this command",
                            "Shared ports must be checked/reserved by a future runner",
                            "GPS-loss, wind and airspace faults are not supported here"],
        }


def validate_layout(fleet: Fleet, vehicles: tuple[VehicleBinding, ...]):
    """Check layout and all identity/port bindings before any process is allowed."""
    expected = {f"px4_{i}" for i in range(1, len(vehicles) + 1)}
    if not 1 <= len(vehicles) <= 4 or set(fleet.homes) != expected:
        raise ValueError("fleet must contain 1..4 consecutive px4 IDs")
    if {v.vehicle for v in vehicles} != expected or len({v.vehicle for v in vehicles}) != len(vehicles):
        raise ValueError("duplicate or mismatched vehicle IDs")
    for binding in vehicles:
        i = int(binding.vehicle.split("_")[1])
        if (type(binding.px4_instance) is not int or binding.px4_instance != i
                or type(binding.target_system_id) is not int or binding.target_system_id != i + 1
                or binding.namespace != f"/px4_{i}"):
            raise ValueError("namespace/system ID must match PX4 instance")
        _integer(binding.mavlink_gcs_port, "MAVLink port", 1024, 65535)
        if binding.mavlink_gcs_port != 18570 + i:
            raise ValueError("MAVLink GCS port must match the existing PX4 runner convention")
    if len({v.mavlink_gcs_port for v in vehicles}) != len(vehicles):
        raise ValueError("duplicate MAVLink ports")
    clearance = SurveyCoordinator(fleet=fleet).obstacle_clearance
    reservation = SurveyCoordinator(fleet=fleet).reservation
    obstacles = [*DEFAULT_OBSTACLE_COURSE, GPS_DENIED_ZONE]
    groups = [list(fleet.homes.values()), list(fleet.landings.values()),
              [t.start for t in fleet.tasks if t.name.startswith("gate_")]]
    points = [*fleet.homes.values(), *fleet.landings.values(),
              *(p for t in fleet.tasks for p in (t.start, t.end))]
    for p in points:
        if (len(p) != 3 or any(type(x) not in (int, float) or not math.isfinite(x) for x in p)
                or not FENCE.north_min + .5 <= p[0] <= FENCE.north_max - .5
                or not FENCE.east_min + .5 <= p[1] <= FENCE.east_max - .5
                or not -FENCE.altitude_max <= p[2] <= 0):
            raise ValueError("layout point must be finite and inside the flight fence")
        if any(abs(p[0] - o.north) <= o.size_north / 2 + clearance
               and abs(p[1] - o.east) <= o.size_east / 2 + clearance for o in obstacles):
            raise ValueError("layout point intersects obstacle/keep-out clearance")
    if any(math.dist(a[:2], b[:2]) <= reservation
           for group in groups for a, b in itertools.combinations(group, 2)):
        raise ValueError("overlapping layout: spacing must exceed the reservation")


def parse_experiment(data: dict) -> Experiment:
    _keys(data, ("schema", "fleet_size", "scenario", "seed", "output_dir"), ("faults",))
    _integer(data["schema"], "schema", 1, 1)
    n = _integer(data["fleet_size"], "fleet_size", 1, 4)
    seed = _integer(data["seed"], "seed", 0, 2**32 - 1)
    if data["scenario"] not in ("gate_visit_nominal", "gate_visit_dropout"):
        raise ValueError("unsupported scenario (only gate_visit_nominal/dropout)")
    output = data["output_dir"]
    if not isinstance(output, str) or not output.strip() or "\x00" in output:
        raise ValueError("output_dir must be a nonempty path string")
    fleet = make_fleet(n)
    vehicles = tuple(VehicleBinding(f"px4_{i}", f"/px4_{i}", i, i + 1, 18570 + i)
                     for i in range(1, n + 1))
    validate_layout(fleet, vehicles)
    raw_faults = data.get("faults", [])
    if not isinstance(raw_faults, list):
        raise ValueError("faults must be a list")
    faults = []
    for fault in raw_faults:
        _keys(fault, ("vehicle", "kind", "after_ready_s"))
        delay = fault["after_ready_s"]
        if (not isinstance(fault["vehicle"], str) or fault["vehicle"] not in fleet.homes
                or fault["kind"] != "abort_and_land"
                or type(delay) not in (int, float) or not math.isfinite(delay) or delay <= 0):
            raise ValueError("fault requires a known vehicle, abort_and_land and a finite positive delay")
        faults.append(Fault(fault["vehicle"], fault["kind"], float(delay)))
    if data["scenario"] == "gate_visit_nominal" and faults:
        raise ValueError("nominal scenario cannot include faults")
    if data["scenario"] == "gate_visit_dropout" and (n < 2 or len(faults) != 1):
        raise ValueError("dropout scenario requires at least two vehicles and exactly one fault")
    return Experiment(data["scenario"], seed, output, fleet, vehicles, tuple(faults))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path)
    args = parser.parse_args(argv)
    try:
        experiment = parse_experiment(json.loads(args.config.read_text(encoding="utf-8")))
    except (OSError, ValueError, TypeError, OverflowError) as exc:
        parser.error(str(exc))
    print(json.dumps(experiment.to_dict(), sort_keys=True, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
