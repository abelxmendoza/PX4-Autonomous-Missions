"""Asset inventory: every model manifest + plan + spend ledger in one table.

Read-only and offline. Columns follow docs/sim_asset_experiments.md: name,
source platform and id, endpoint, credits, format, dimensions, collision,
articulation, licence, validation, intended use.
"""
from __future__ import annotations

import json
from pathlib import Path


def _row_from_manifest(m, location):
    prov = m.get("provenance") or {}
    src = str(prov.get("source", ""))
    if "huggingface.co" in src or src.startswith("hf:"):
        platform, credits, endpoint = "projectsim sample (HF)", 0, "published sample"
    elif src == "projectsim":
        platform, credits, endpoint = "projectsim API", None, prov.get("plan_item")
    else:
        platform, credits, endpoint = src or "local", 0, "-"
    joints = m.get("joints", [])
    moving = [j for j in joints if j.get("type") != "fixed"]
    return {
        "name": m["name"], "location": location, "platform": platform,
        "source_id": prov.get("asset") or prov.get("asset_id") or prov.get("id") or prov.get("path"),
        "endpoint": endpoint, "credits": credits,
        "format": "USD->SDF (gltf+stl)" if m.get("importer") == "usd_import" else f"{m.get('source_file', '')}->SDF",
        "extents_m": [round(v, 3) for v in m["extents_m"]],
        "collision": m.get("collision"), "mode": m.get("mode", "static" if (m.get("import_spec") or {}).get("static", True) else "dynamic"),
        "moving_joints": len(moving),
        "license": prov.get("license") or "unknown",
        "warnings": len(m.get("warnings", [])),
        "description": prov.get("description") or (prov.get("sample_manifest") or {}).get("class"),
    }


def build(model_dirs, plan=None, ledger_entries=(), acceptance=None):
    rows = []
    for d in model_dirs:
        for manifest in sorted(Path(d).glob("*/asset_manifest.json")):
            rows.append(_row_from_manifest(json.loads(manifest.read_text()), str(Path(d))))
    accepted = {}
    if acceptance:
        for r in acceptance.get("results", []):
            if r.get("mode") == "static":
                accepted[f"ps_{r['asset']}"] = r["verdict"]
    for r in rows:
        r["validation"] = accepted.get(r["name"], "not run")
    spend = {}
    for e in ledger_entries:
        if e["event"] == "submitted":
            spend[e["item"]] = {"credits": e["credits"], "id": e.get("set_id") or e.get("asset_id"),
                                "status": "submitted"}
        if e["event"] == "downloaded" and e.get("item") in spend:
            spend[e["item"]]["status"] = "downloaded"
    planned = []
    for item in (plan or {}).get("items", []):
        from .projectsim import item_cost
        planned.append({"id": item["id"], "endpoint": item["endpoint"], "credits": item_cost(item),
                        "batch": item.get("batch") or item.get("experiment") or "candidate",
                        "status": spend.get(item["id"], {}).get("status", "not submitted"),
                        "remote_id": spend.get(item["id"], {}).get("id")})
    return {"models": rows, "generation": planned}


def to_markdown(inv):
    lines = ["| model | platform | source | extents (m) | collision | moving joints | licence | validation |",
             "|---|---|---|---|---|---|---|---|"]
    for r in inv["models"]:
        lines.append(f"| {r['name']} | {r['platform']} | {r['source_id']} | {r['extents_m']} | "
                     f"{r['collision']} | {r['moving_joints']} | {r['license']} | {r['validation']} |")
    lines += ["", "| generation item | endpoint | credits | batch | status | remote id |",
              "|---|---|---|---|---|---|"]
    for g in inv["generation"]:
        lines.append(f"| {g['id']} | {g['endpoint']} | {g['credits']} | {g['batch']} | {g['status']} | {g['remote_id'] or ''} |")
    return "\n".join(lines) + "\n"
