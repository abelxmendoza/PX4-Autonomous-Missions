#!/usr/bin/env python3
"""Simulation asset pipeline: mesh -> checked Gazebo model -> seeded world.

Local (no network, no credits):
  import-mesh MESH --name N --height 2.0 [--up y] [--collision mesh|convex_hull|box]
  import-usd USD --name N [--mode static|articulated]   UsdPhysics -> multi-link SDF
  fetch-sample DATASET ASSET                free CC-BY-4.0 projectsim samples (HF)
  drop-test NAME [--probe X,Y ...]          headless Gazebo collision check
  accept NAME [--ref-dims X,Y,Z]            acceptance criteria report (PASS/FAIL)
  build-world SCENARIO.yaml OUT.sdf         seeded, reproducible placement

ProjectSim (optional; PROJECTSIM_TOKEN in the environment):
  ps-plan PLAN.yaml                         cost + budget, no network
  ps-auth                                   free token check
  ps-submit PLAN.yaml ITEM --confirm-spend  SPENDS CREDITS
  ps-fetch ID [--set]                       download when complete (free)
  ps-import PLAN.yaml ITEM ID [--variant N] raw download -> Gazebo model

See docs/sim_asset_pipeline.md.
"""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODELS = ROOT / "models"
ASSETS = ROOT / "sim_assets"
LEDGER = ASSETS / "projectsim_ledger.jsonl"
RAW = ASSETS / "raw"
SAMPLES = ASSETS / "samples"


def _import(source, spec, provenance, out, overwrite):
    from sim_assets import gz_model, mesh_import
    result = mesh_import.normalise(mesh_import.load_mesh(source), spec)
    manifest = gz_model.write_model(out, source, spec, result, provenance, overwrite)
    print(json.dumps({"model": str(Path(out) / spec.name), "extents_m": manifest["extents_m"],
                      "scale": manifest["transform"]["scale"],
                      "collision": spec.collision, "warnings": manifest["warnings"]}, indent=2))


def _spec(args_or_dict):
    from sim_assets.mesh_import import ImportSpec
    d = args_or_dict if isinstance(args_or_dict, dict) else vars(args_or_dict)
    keys = ImportSpec.__dataclass_fields__
    return ImportSpec(**{k: d[k] for k in keys if k in d and d[k] is not None})


def cmd_import_mesh(a):
    spec = _spec({"name": a.name, "up_axis": a.up, "collision": a.collision,
                  "target_axis": a.axis, "target_size_m": a.size, "source_unit_m": a.unit,
                  "static": not a.dynamic, "mass_kg": a.mass, "density_kg_m3": a.density})
    provenance = {"source": "local", "path": str(a.mesh), "license": a.license,
                  "description": a.description}
    _import(a.mesh, spec, provenance, a.out, a.overwrite)


def cmd_import_usd(a):
    from sim_assets import usd_import
    manifest = usd_import.convert(a.usd, a.out, a.name, mode=a.mode, collision=a.collision,
                                  provenance={"source": a.source or "local", "path": str(a.usd),
                                              "license": a.license},
                                  max_texture_px=a.texture_px, overwrite=a.overwrite)
    print(json.dumps({k: manifest[k] for k in ("name", "mode", "collision", "extents_m",
                                               "joints", "warnings")}, indent=2))


def cmd_fetch_sample(a):
    from sim_assets import hf_samples
    print(hf_samples.fetch(a.dataset, a.asset, SAMPLES))


def cmd_drop_test(a):
    from sim_assets import drop_test
    probes = [tuple(float(v) for v in p.split(",")) for p in a.probe] or None
    report = drop_test.run(a.models, a.name, probes)
    print(json.dumps(report, indent=2))
    return 0 if report["loaded"] and report["settled"] else 1


def cmd_accept(a):
    from sim_assets import acceptance
    ref = [float(v) for v in a.ref_dims.split(",")] if a.ref_dims else None
    probes = [tuple(float(v) for v in p.split(",")) for p in a.probe] or None
    report = acceptance.evaluate(a.models, a.name, ref, probes, run_gz=not a.no_gz)
    print(json.dumps(report, indent=2))
    return 0 if report["verdict"] == "PASS" else 1


def cmd_build_world(a):
    from sim_assets import world_builder
    xml, record = world_builder.build(world_builder.load_scenario(a.scenario), a.models)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(xml, encoding="utf-8")
    rec = a.out.with_suffix(".placement.json")
    rec.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    print(f"{len(record['instances'])} instances -> {a.out} (record: {rec.name})")


def cmd_ps_plan(a):
    from sim_assets import projectsim as ps
    plan = ps.load_plan(a.plan)
    rows = [(i["id"], i["endpoint"], ps.item_cost(i)) for i in plan["items"]]
    for item_id, endpoint, cost in rows:
        print(f"  {item_id:<24} {endpoint:<9} {cost:>3} credits")
    print(f"  {'TOTAL':<34} {sum(r[2] for r in rows):>3} credits")
    print(json.dumps(ps.budget_status(plan, ps.Ledger(LEDGER)), indent=2))


def cmd_ps_auth(a):
    from sim_assets import projectsim as ps
    listing = ps.Client().auth_check()
    print(f"token OK; {len(listing.get('assets', []))} asset(s) on first page")


def cmd_ps_submit(a):
    from sim_assets import projectsim as ps
    plan = ps.load_plan(a.plan)
    item = next((i for i in plan["items"] if i["id"] == a.item), None)
    if item is None:
        sys.exit(f"no item {a.item!r} in {a.plan}")
    if not a.confirm_spend:
        print(f"DRY RUN: {a.item} would cost {ps.item_cost(item)} credits. Request body:")
        print(json.dumps(ps.request_body(plan, item), indent=2))
        print("Nothing sent. Re-run with --confirm-spend after approval.")
        return 2
    try:
        entry = ps.submit(ps.Client(), plan, a.item, ps.Ledger(LEDGER), confirm_spend=True)
    except (ps.BudgetExceeded, ps.ProjectSimError) as err:
        sys.exit(f"not submitted: {err}")
    print(json.dumps(entry, indent=2))


def cmd_ps_fetch(a):
    from sim_assets import projectsim as ps
    fetch = ps.fetch_set if a.set else ps.fetch
    print(json.dumps(fetch(ps.Client(), a.id, RAW, ps.Ledger(LEDGER)), indent=2))


def cmd_ps_import(a):
    from sim_assets import projectsim as ps
    plan = ps.load_plan(a.plan)
    item = next(i for i in plan["items"] if i["id"] == a.item)
    raw = RAW / a.id
    name = a.item
    if a.variant is not None:
        raw, name = raw / f"{a.variant:03d}", f"{a.item}_{a.variant:03d}"
    metadata = json.loads((raw / "projectsim_asset.json").read_text())
    settings = dict(item["import"])
    if settings.pop("kind", "mesh") == "usd":
        from sim_assets import usd_import
        usd = sorted(p for p in raw.rglob("*.usd*") if not p.stem.endswith("_scene"))
        if not usd:
            sys.exit(f"no USD under {raw}")
        manifest = usd_import.convert(
            usd[0], a.out, name, mode=settings.get("mode", "static"),
            collision=settings.get("collision"), overwrite=a.overwrite,
            provenance={"source": "projectsim", "id": a.id, "variant": a.variant,
                        "plan_item": a.item, "request": ps.request_body(plan, item),
                        "effective_params": metadata.get("effective_params"),
                        "license": plan.get("license_note"), "description": item.get("rationale")})
        print(json.dumps({k: manifest[k] for k in ("name", "extents_m", "joints", "warnings")}, indent=2))
        return
    glb, sidecar = ps.pick_files(metadata)
    provenance = {
        "source": "projectsim", "asset_id": a.id, "plan_item": a.item,
        "request": ps.request_body(plan, item),
        "effective_params": metadata.get("effective_params"),
        "dimensions": metadata.get("dimensions"),
        "physics_sidecar": json.loads((raw / sidecar).read_text()) if sidecar else None,
        "license": plan.get("license_note"),
        "description": item.get("rationale"),
    }
    _import(raw / glb, _spec({"name": name, **settings}), provenance, a.out, a.overwrite)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    m = sub.add_parser("import-mesh")
    m.add_argument("mesh", type=Path)
    m.add_argument("--name", required=True)
    m.add_argument("--up", default="y", choices=["y", "z"])
    size = m.add_mutually_exclusive_group(required=True)
    size.add_argument("--size", "--height", type=float, help="real-world size (m) along --axis")
    size.add_argument("--unit", type=float, help="metres per source unit (measured sources)")
    m.add_argument("--axis", default="z", choices=["x", "y", "z"])
    m.add_argument("--collision", default="mesh", choices=["mesh", "convex_hull", "box"])
    m.add_argument("--dynamic", action="store_true")
    m.add_argument("--mass", type=float)
    m.add_argument("--density", type=float)
    m.add_argument("--license", default="unknown")
    m.add_argument("--description")
    m.add_argument("--out", type=Path, default=MODELS)
    m.add_argument("--overwrite", action="store_true")
    m.set_defaults(func=cmd_import_mesh)

    u = sub.add_parser("import-usd")
    u.add_argument("usd", type=Path)
    u.add_argument("--name", required=True)
    u.add_argument("--mode", default="static", choices=["static", "articulated"])
    u.add_argument("--collision", choices=["usd", "convex_hull", "box"])
    u.add_argument("--texture-px", type=int, default=2048, help="downscale textures (0 = keep)")
    u.add_argument("--license", default="unknown")
    u.add_argument("--source")
    u.add_argument("--out", type=Path, default=MODELS)
    u.add_argument("--overwrite", action="store_true")
    u.set_defaults(func=cmd_import_usd)

    f = sub.add_parser("fetch-sample")
    f.add_argument("dataset", help="e.g. articulated-logistics-cages")
    f.add_argument("asset", help="e.g. cage_0002")
    f.set_defaults(func=cmd_fetch_sample)

    d = sub.add_parser("drop-test")
    d.add_argument("name")
    d.add_argument("--probe", action="append", default=[], help="X,Y in model frame")
    d.add_argument("--models", type=Path, default=MODELS)
    d.set_defaults(func=cmd_drop_test)

    c = sub.add_parser("accept")
    c.add_argument("name")
    c.add_argument("--ref-dims", help="reference extents X,Y,Z in metres")
    c.add_argument("--probe", action="append", default=[])
    c.add_argument("--no-gz", action="store_true", help="manifest checks only")
    c.add_argument("--models", type=Path, default=MODELS)
    c.set_defaults(func=cmd_accept)

    w = sub.add_parser("build-world")
    w.add_argument("scenario", type=Path)
    w.add_argument("out", type=Path)
    w.add_argument("--models", type=Path, default=MODELS)
    w.set_defaults(func=cmd_build_world)

    s = sub.add_parser("ps-plan"); s.add_argument("plan", type=Path); s.set_defaults(func=cmd_ps_plan)
    s = sub.add_parser("ps-auth"); s.set_defaults(func=cmd_ps_auth)
    s = sub.add_parser("ps-submit")
    s.add_argument("plan", type=Path); s.add_argument("item")
    s.add_argument("--confirm-spend", action="store_true")
    s.set_defaults(func=cmd_ps_submit)
    s = sub.add_parser("ps-fetch"); s.add_argument("id")
    s.add_argument("--set", action="store_true", help="id is a variation set")
    s.set_defaults(func=cmd_ps_fetch)
    s = sub.add_parser("ps-import")
    s.add_argument("plan", type=Path); s.add_argument("item"); s.add_argument("id")
    s.add_argument("--variant", type=int, help="variation-set member (variant_index)")
    s.add_argument("--out", type=Path, default=MODELS)
    s.add_argument("--overwrite", action="store_true")
    s.set_defaults(func=cmd_ps_import)

    args = p.parse_args(argv)
    return args.func(args) or 0


if __name__ == "__main__":
    sys.exit(main())
