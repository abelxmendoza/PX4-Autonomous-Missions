"""Physics check: load a model in headless Gazebo and drop probe cubes on it.

A 10 cm probe cube is released 0.5 m above the model's top at each (x, y)
probe point and left to settle (a cube, not a ball, so it stays put on a
sloped surface such as a convex hull's instead of rolling off). Its rest
height says what collision surface
Gazebo actually built there: the top of the obstacle, a shelf, or the
ground (a gap). This tests the collision geometry Gazebo loaded, rather
than the geometry we meant to write.

Scope: this exercises COLLISION geometry. gpu_lidar and depth cameras ray-cast
the VISUAL mesh, which loads through the same gz-common mesh loader (same
frame handling) but is not checked here.

Runs in a private gz-transport partition so it cannot touch a running sim.
"""
from __future__ import annotations

import json
import os
import re
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from realmforge_to_sdf import px4_world

PROBE_HALF_M = 0.05  # cube half-size
POSE_RE = re.compile(r"\[\s*(-?[\d.e+-]+)\s+(-?[\d.e+-]+)\s+(-?[\d.e+-]+)\s*\]")


def _world(model, top_z, probes):
    sdf, world = px4_world("drop_test", ground_size_m=60)
    inc = ET.SubElement(world, "include")
    ET.SubElement(inc, "uri").text = f"model://{model}"
    ET.SubElement(inc, "name").text = model
    for i, (x, y) in enumerate(probes):
        probe = ET.SubElement(world, "model", name=f"probe_{i}")
        ET.SubElement(probe, "pose").text = f"{x} {y} {top_z + 0.5} 0 0 0"
        link = ET.SubElement(probe, "link", name="cube")
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "mass").text = "0.1"
        tensor = ET.SubElement(inertial, "inertia")
        moment = 0.1 * (2 * PROBE_HALF_M) ** 2 / 6
        for tag in ("ixx", "iyy", "izz"):
            ET.SubElement(tensor, tag).text = f"{moment:.3e}"
        col = ET.SubElement(link, "collision", name="c")
        ET.SubElement(ET.SubElement(ET.SubElement(col, "geometry"), "box"), "size").text = f"{2 * PROBE_HALF_M} " * 3
    ET.indent(sdf, space="  ")
    return ET.tostring(sdf, encoding="unicode", xml_declaration=True)


def _pose(name, env):
    out = subprocess.run(["gz", "model", "-m", name, "-p"], capture_output=True,
                         text=True, timeout=15, env=env).stdout
    match = POSE_RE.search(out.split("Pose", 1)[-1]) if "Pose" in out else None
    return tuple(float(v) for v in match.groups()) if match else None


def run(models_dir, model, probes=None, timeout_s=40.0, settle_tol_m=1e-3):
    """Return {"loaded", "probes": [{xy, rest_z}], "errors"} from a real gz run."""
    models_dir = Path(models_dir).resolve()
    manifest = json.loads((models_dir / model / "asset_manifest.json").read_text())
    top_z = manifest["bounds_m"][1][2] if "bounds_m" in manifest else manifest["extents_m"][2]
    probes = probes or [(0.0, 0.0)]
    env = dict(os.environ)
    env["GZ_SIM_RESOURCE_PATH"] = f"{models_dir}:{env.get('GZ_SIM_RESOURCE_PATH', '')}"
    env["GZ_PARTITION"] = f"sim_asset_drop_{os.getpid()}"
    with tempfile.TemporaryDirectory() as tmp:
        world = Path(tmp) / "drop_test.sdf"
        world.write_text(_world(model, top_z, probes), encoding="utf-8")
        log = open(Path(tmp) / "gz.log", "w+")
        proc = subprocess.Popen(["gz", "sim", "-s", "-r", str(world)], stdout=log,
                                stderr=subprocess.STDOUT, env=env, start_new_session=True)
        try:
            deadline = time.monotonic() + timeout_s
            last = None
            while time.monotonic() < deadline:
                time.sleep(1.0)
                now = [_pose(f"probe_{i}", env) for i in range(len(probes))]
                if all(now) and last and all(abs(a[2] - b[2]) < settle_tol_m for a, b in zip(now, last)):
                    break
                last = now if all(now) else None
            else:
                now = None
            model_pose = _pose(model, env)
            loaded = model_pose is not None
        finally:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        log.seek(0)
        errors = [ln.strip() for ln in log if "[Err]" in ln or "Error" in ln]
        log.close()
    return {
        "model": model,
        "loaded": loaded,
        "settled": now is not None,
        "model_xyz": list(model_pose) if loaded else None,
        "probes": [{"xy": list(xy), "rest_z": None if not now else round(p[2], 4),
                    "surface_z": None if not now else round(p[2] - PROBE_HALF_M, 4)}
                   for xy, p in zip(probes, now or [None] * len(probes))],
        "errors": errors,
    }
