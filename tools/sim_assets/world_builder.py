"""Seeded, reproducible obstacle worlds from a scenario file.

Scenario (YAML or JSON):
  format: sim-scenario
  version: 1
  name: rack_corridor            # world name ([A-Za-z0-9_])
  seed: 42                       # same seed + same models -> same world
  area: {x: [-8, 8], y: [4, 36]} # ENU metres; footprints stay inside
  min_clearance_m: 1.0           # free gap between any two footprints
  keepout: [{x: 0, y: 0, radius_m: 3.0}]   # e.g. the vehicle spawn
  ground_size_m: 100
  placements:
    - {model: pallet_rack, count: 4, yaw: random}
    - {model: utility_pole, x: 0, y: 20, yaw: 0}

Footprints are circles of radius footprint_radius_m, read from each model's
asset_manifest.json (or given inline for hand-made models). Fixed
placements go first, then random ones in file order using one
random.Random(seed), so the result depends only on the scenario and the
model footprints. The placement record written beside the world lists
every pose and the manifest hashes it was built from.
"""
from __future__ import annotations

import hashlib
import json
import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

# tools/ is on sys.path (tools/sim_asset.py entry point, tests).
from realmforge_to_sdf import px4_world

MAX_ATTEMPTS = 5000


def load_scenario(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    validate(data)
    return data


def validate(data):
    if not isinstance(data, dict) or data.get("format") != "sim-scenario" or data.get("version") != 1:
        raise ValueError("expected format: sim-scenario, version: 1")
    name = data.get("name")
    if not isinstance(name, str) or not name or not all(c.isalnum() or c == "_" for c in name):
        raise ValueError("name must be non-empty [A-Za-z0-9_]")
    if type(data.get("seed")) is not int:
        raise ValueError("seed must be an integer")
    area = data.get("area")
    if (not isinstance(area, dict) or any(
            not isinstance(area.get(k), list) or len(area[k]) != 2
            or not all(_num(v) for v in area[k]) or area[k][0] >= area[k][1]
            for k in ("x", "y"))):
        raise ValueError("area needs x and y as [min, max] with min < max")
    if not isinstance(data.get("placements"), list) or not data["placements"]:
        raise ValueError("placements must be a non-empty list")
    for i, p in enumerate(data["placements"]):
        if not isinstance(p, dict) or not isinstance(p.get("model"), str):
            raise ValueError(f"placement {i}: model name required")
        fixed = "x" in p or "y" in p
        if fixed and not (_num(p.get("x")) and _num(p.get("y"))):
            raise ValueError(f"placement {i}: fixed placement needs numeric x and y")
        if not fixed and (type(p.get("count", 1)) is not int or p.get("count", 1) < 1):
            raise ValueError(f"placement {i}: count must be a positive integer")
        yaw = p.get("yaw", "random")
        if yaw != "random" and not _num(yaw):
            raise ValueError(f"placement {i}: yaw must be 'random' or radians")


def _num(v):
    return type(v) in (int, float) and math.isfinite(v)


def footprint(model, placement, models_dir):
    if _num(placement.get("footprint_radius_m")):
        return float(placement["footprint_radius_m"]), None
    manifest = Path(models_dir) / model / "asset_manifest.json"
    if not manifest.is_file():
        raise ValueError(f"{model}: no asset_manifest.json; give footprint_radius_m inline")
    raw = manifest.read_bytes()
    return float(json.loads(raw)["footprint_radius_m"]), hashlib.sha256(raw).hexdigest()


def place(data, models_dir):
    """Return the list of placed instances (deterministic for a seed)."""
    rng = random.Random(data["seed"])
    (x0, x1), (y0, y1) = data["area"]["x"], data["area"]["y"]
    clearance = float(data.get("min_clearance_m", 0.0))
    keepout = data.get("keepout", [])
    placed, hashes = [], {}

    def fits(x, y, r):
        if not (x0 + r <= x <= x1 - r and y0 + r <= y <= y1 - r):
            return False
        if any(math.hypot(x - k["x"], y - k["y"]) < r + k["radius_m"] for k in keepout):
            return False
        return all(math.hypot(x - o["x"], y - o["y"]) >= r + o["radius_m"] + clearance
                   for o in placed)

    def yaw_of(p):
        return rng.uniform(-math.pi, math.pi) if p.get("yaw", "random") == "random" else float(p["yaw"])

    ordered = ([p for p in data["placements"] if "x" in p] +
               [p for p in data["placements"] if "x" not in p])
    for p in ordered:
        radius, digest = footprint(p["model"], p, models_dir)
        if digest:
            hashes[p["model"]] = digest
        if "x" in p:
            if not fits(p["x"], p["y"], radius):
                raise ValueError(f"fixed {p['model']} at ({p['x']}, {p['y']}) collides or leaves the area")
            placed.append(_instance(p["model"], p["x"], p["y"], yaw_of(p), radius, placed))
            continue
        for _ in range(p.get("count", 1)):
            for _attempt in range(MAX_ATTEMPTS):
                x, y = rng.uniform(x0, x1), rng.uniform(y0, y1)
                if fits(x, y, radius):
                    placed.append(_instance(p["model"], x, y, yaw_of(p), radius, placed))
                    break
            else:
                raise ValueError(f"could not place {p['model']} after {MAX_ATTEMPTS} "
                                 "attempts: area too crowded for the clearance")
    return placed, hashes


def _instance(model, x, y, yaw, radius, placed):
    index = sum(1 for o in placed if o["model"] == model)
    return {"name": f"{model}_{index}", "model": model, "x": round(x, 4),
            "y": round(y, 4), "yaw": round(yaw, 4), "radius_m": radius}


def build(data, models_dir):
    """Return (world SDF text, placement record dict)."""
    validate(data)
    placed, hashes = place(data, models_dir)
    sdf, world = px4_world(data["name"], data.get("ground_size_m", 100))
    for inst in placed:
        inc = ET.SubElement(world, "include")
        ET.SubElement(inc, "uri").text = f"model://{inst['model']}"
        ET.SubElement(inc, "name").text = inst["name"]
        ET.SubElement(inc, "pose").text = f"{inst['x']} {inst['y']} 0 0 0 {inst['yaw']}"
    ET.indent(sdf, space="  ")
    xml = ET.tostring(sdf, encoding="unicode", xml_declaration=True) + "\n"
    record = {"format": "sim-scenario-record", "version": 1, "scenario": data,
              "manifest_sha256": hashes, "instances": placed}
    return xml, record
