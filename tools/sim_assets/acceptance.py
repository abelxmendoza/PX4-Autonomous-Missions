"""Acceptance checks for a generated Gazebo model (docs/sim_asset_experiments.md).

Each criterion returns PASS / FAIL / SKIP with the measured value, so a
report shows *why* an asset was accepted, not just that it was. Static
checks read asset_manifest.json; the gz checks run drop_test in a real
headless Gazebo.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

SCALE_TOL_REL = 0.02      # 2 % ...
SCALE_TOL_ABS_M = 0.01    # ... or 1 cm, whichever is larger
GROUND_TOL_M = 0.02
MAX_COLLISION_TRIS = 50000
STABLE_DRIFT_M = 0.02


def _c(name, ok, value, need):
    return {"criterion": name, "result": "PASS" if ok else "FAIL", "value": value, "required": need}


def _skip(name, why):
    return {"criterion": name, "result": "SKIP", "value": None, "required": why}


def static_checks(manifest, reference_dims=None):
    out = []
    ext = manifest["extents_m"]
    if reference_dims:
        errs = [abs(e - r) for e, r in zip(ext, reference_dims)]
        tol = [max(SCALE_TOL_ABS_M, SCALE_TOL_REL * r) for r in reference_dims]
        out.append(_c("scale_matches_reference", all(e <= t for e, t in zip(errs, tol)),
                      [round(v, 4) for v in ext], f"{reference_dims} within max(1 cm, 2 %)"))
    else:
        out.append(_skip("scale_matches_reference", "no reference dimensions given"))
    lo_z = manifest.get("bounds_m", [[0, 0, 0]])[0][2]
    out.append(_c("rests_on_ground", abs(lo_z) <= GROUND_TOL_M, round(lo_z, 4), f"|z_min| <= {GROUND_TOL_M} m"))
    if "links" in manifest:
        tris = sum(l["collision_triangles"] for l in manifest["links"])
    else:
        tris = manifest["checks"]["normalised"]["triangles"] if manifest["collision"] == "mesh" else 0
    out.append(_c("collision_budget", tris <= MAX_COLLISION_TRIS, tris, f"<= {MAX_COLLISION_TRIS} triangles"))
    joints = manifest.get("joints", [])
    bad = [j["name"] for j in joints if j["type"] != "fixed" and (
        abs(math.hypot(*j["axis"]) - 1) > 1e-3 or (not j.get("unlimited") and (
            not j.get("limits") or not all(math.isfinite(v) for v in j["limits"])
            or j["limits"][0] >= j["limits"][1])))]
    out.append(_c("joints_well_formed", not bad, f"{len(joints)} joints, bad: {bad}",
                  "unit axis; finite lower < upper unless unlimited") if joints else _skip("joints_well_formed", "no joints"))
    dynamic = manifest.get("mode") == "articulated" or (manifest.get("import_spec") or {}).get("static") is False
    if dynamic:
        masses = [l["mass"]["mass_kg"] for l in manifest.get("links", []) if l.get("mass")]
        if "inertial" in manifest and manifest["inertial"]:
            masses = [manifest["inertial"]["mass_kg"]]
        out.append(_c("has_mass", bool(masses) and all(m > 0 for m in masses), masses, "mass > 0 per link"))
    return out


def gz_checks(models_dir, name, manifest, probes=None):
    from . import drop_test
    report = drop_test.run(models_dir, name, probes)
    out = [_c("loads_in_gazebo", report["loaded"] and not report["errors"],
              {"loaded": report["loaded"], "errors": report["errors"][:3]}, "loads, no [Err] lines"),
           _c("probes_settle", report["settled"], report["probes"], "every probe at rest")]
    xyz = report.get("model_xyz")
    if manifest.get("mode") == "articulated" and xyz:
        drift = math.dist(xyz, [0, 0, 0])
        out.append(_c("dynamic_model_stable", drift <= STABLE_DRIFT_M, round(drift, 4),
                      f"model drift <= {STABLE_DRIFT_M} m after settling"))
    top = manifest.get("bounds_m", [[0, 0, 0], manifest["extents_m"]])[1][2]
    ok = report["settled"] and all(-GROUND_TOL_M <= p["surface_z"] <= top + GROUND_TOL_M
                                   for p in report["probes"])
    out.append(_c("contact_within_bounds", ok, [p["surface_z"] for p in report["probes"]],
                  f"0 <= surface <= top ({round(top, 3)} m)"))
    return out, report


def evaluate(models_dir, name, reference_dims=None, probes=None, run_gz=True):
    manifest = json.loads((Path(models_dir) / name / "asset_manifest.json").read_text())
    results = static_checks(manifest, reference_dims)
    if run_gz:
        gz, _ = gz_checks(models_dir, name, manifest, probes)
        results += gz
    verdict = "FAIL" if any(r["result"] == "FAIL" for r in results) else "PASS"
    return {"model": name, "verdict": verdict, "criteria": results}
