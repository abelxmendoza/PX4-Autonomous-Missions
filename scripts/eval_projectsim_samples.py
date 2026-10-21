#!/usr/bin/env python3
"""E0: can real ProjectSim procedural output run in Gazebo Harmonic? (free)

Fetches pinned CC-BY-4.0 projectsim sample assets from Hugging Face,
converts each with tools/sim_assets/usd_import in both modes, and runs the
acceptance checks (including headless gz drop tests) against the sample's
own published dimensions. Writes evidence/sim_assets/projectsim_samples.json.

Costs no ProjectSim credits. Needs: usd-core, trimesh, Pillow, gz.
"""
import argparse
import json
import platform
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from sim_assets import acceptance, hf_samples, usd_import  # noqa: E402

SAMPLES = [("articulated-logistics-cages", f"cage_{i:04d}") for i in range(1, 6)] + \
          [("articulated-desks", f"desk_{i:04d}") for i in range(1, 6)]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--work", type=Path, default=ROOT / "sim_assets" / "samples")
    ap.add_argument("--out", type=Path, default=ROOT / "evidence" / "sim_assets" / "projectsim_samples.json")
    args = ap.parse_args()
    models = args.work / "_models"
    results = []
    for dataset, asset in SAMPLES:
        usd = hf_samples.fetch(dataset, asset, args.work)
        prov = json.loads((usd.parents[2] / "provenance.json").read_text())
        ref = prov["sample_manifest"].get("dims_m")
        for mode in ("static", "articulated"):
            name = f"{asset}_{mode}"
            entry = {"dataset": dataset, "asset": asset, "mode": mode,
                     "revision": prov["revision"], "reference_dims_m": ref,
                     "reference_mass_kg": prov["sample_manifest"].get("mass_kg"),
                     "reference_joints": len(prov["sample_manifest"].get("joints", []))}
            try:
                m = usd_import.convert(usd, models, name, mode=mode, provenance=prov, overwrite=True)
                entry["converted_joints"] = len(m["joints"])
                entry["converted_mass_kg"] = round(sum(l["mass"]["mass_kg"] for l in m["links"] if l["mass"]), 3)
                entry["warnings"] = m["warnings"]
                entry.update(acceptance.evaluate(models, name, ref))
            except Exception as err:  # record, keep going: this is a survey
                entry.update(verdict="FAIL", error=f"{type(err).__name__}: {err}")
            print(f"{asset:<12} {mode:<11} {entry['verdict']}"
                  + "".join(f"  {c['criterion']}={c['result']}" for c in entry.get("criteria", [])
                            if c["result"] != "PASS") + (f"  {entry.get('error', '')}"))
            results.append(entry)
    gz = subprocess.run(["gz", "sim", "--versions"], capture_output=True, text=True).stdout.strip()
    report = {"experiment": "E0 projectsim sample USD -> Gazebo compatibility",
              "credits_spent": 0, "gz_sim": gz, "python": platform.python_version(),
              "passed": sum(r["verdict"] == "PASS" for r in results), "total": len(results),
              "results": results}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + "\n")
    print(f"{report['passed']}/{report['total']} PASS -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
