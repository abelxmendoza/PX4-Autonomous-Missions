"""Fail-closed verification of synchronized cooperative survey evidence."""
import argparse
import json
import math
from pathlib import Path

from .swarm_logic import SurveyCoordinator


def verify(path):
    errors = []
    minimum = math.inf
    samples = 0
    previous = None
    final = None
    expected = set()
    fleet = set()
    limit = None
    active = False
    geometry = {t.name: t for t in SurveyCoordinator().tasks}
    traversing = set()
    observed = set()
    try:
        with Path(path).open() as stream:
            header = json.loads(next(stream))
            if header.get("schema") != 1 or header.get("type") != "header":
                raise ValueError("missing schema header")
            expected = set(header["expected_tasks"])
            fleet = set(header["homes_ned"])
            if expected != set(geometry):
                raise ValueError("unknown survey specification")
            limit = float(header["minimum_separation_m"])
            if len(fleet) < 2 or not expected or not math.isfinite(limit) or limit <= 0:
                raise ValueError("invalid verification metadata")
            for line in stream:
                row = json.loads(line)
                if row.get("type") != "sample":
                    raise ValueError("invalid sample type")
                now = float(row["time"])
                if not math.isfinite(now) or (previous is not None and not 0 < now - previous <= 0.5):
                    errors.append("non-monotonic time or evidence gap")
                previous = now
                samples += 1
                final = row
                active = active or row["phase"] not in {"WAITING", "ABORTED"}
                if not active:
                    continue
                vehicles = row["vehicles"]
                if set(vehicles) != fleet:
                    errors.append("missing vehicle evidence")
                    continue
                for v in vehicles.values():
                    if v.get("valid") is not True or not 0 <= now - float(v["sent"]) <= 0.75:
                        errors.append("stale or invalid vehicle evidence")
                for name, task in geometry.items():
                    for vehicle, t in vehicles.items():
                        p = t["position"]
                        key = (name, vehicle)
                        if math.dist(p, task.start) < 0.6 and t["armed"]:
                            traversing.add(key)
                        if key in traversing:
                            if abs(p[1] - task.start[1]) > 0.8 or abs(p[2] - task.start[2]) > 0.8 or p[0] < task.start[0] - 0.8:
                                traversing.discard(key)
                            elif math.dist(p, task.end) < 0.6:
                                observed.add(name)
                positions = [v["position"] for v in vehicles.values()]
                if any(len(p) != 3 or not all(math.isfinite(x) for x in p) for p in positions):
                    raise ValueError("invalid position")
                for i, a in enumerate(positions):
                    for b in positions[i + 1:]:
                        minimum = min(minimum, math.dist(a[:2], b[:2]))
        if samples < 2 or final is None or final["phase"] != "COMPLETE":
            errors.append("mission not complete")
        if observed != expected:
            errors.append("survey transects not independently observed")
        if final is None or set(final.get("completed", [])) != expected:
            errors.append("survey coverage incomplete")
        if final is None or set(final.get("vehicles", {})) != fleet or not all(
            v.get("state") == "LANDED" and v.get("landed") is True and v.get("armed") is False
            for v in final.get("vehicles", {}).values()
        ):
            errors.append("landing and disarming not confirmed for every vehicle")
        if minimum == math.inf:
            errors.append("no separation evidence")
        elif minimum < limit:
            errors.append("minimum separation breached")
    except (OSError, ValueError, KeyError, TypeError, IndexError, AttributeError, StopIteration) as exc:
        errors.append(f"invalid evidence: {exc}")
    return {"passed": not errors, "errors": sorted(set(errors)), "samples": samples,
            "minimum_horizontal_separation_m": minimum if math.isfinite(minimum) else None,
            "reassignments": final.get("reassignments", 0) if final else 0}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log")
    args = parser.parse_args()
    report = verify(args.log)
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
