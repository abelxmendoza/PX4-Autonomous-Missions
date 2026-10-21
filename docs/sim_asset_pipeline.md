# Simulation asset pipeline (mesh → Gazebo → seeded worlds)

`tools/sim_asset.py` turns any mesh into a checked Gazebo Harmonic model and
places models into reproducible PX4 worlds. ProjectSim is an **optional**
mesh source behind a spend guard; nothing else depends on it.

```
local mesh / ProjectSim GLB ─► mesh_import ─► gz_model ─┐
                                (axis, scale, checks,       ├─► models/NAME/ ─► world_builder ─► worlds/*.sdf
                                 collision, inertia)        │   (manifest)      (seeded, records poses)
UsdPhysics USD (ProjectSim ───► usd_import ─────────────────┘        │
 procedural, HF samples)       (links, joints, inertia, textures)   └─► acceptance (drop_test in headless gz)
```

## Verified facts this design depends on

| Fact | How it was established |
|---|---|
| gz-sim 8.15 loads GLB **and** STL vertices verbatim: no glTF Y-up → Z-up conversion | Headless drop test, 2026-10-21: a column 2 m tall along +Y was hit by a probe at z = 0.147 m, so it was lying flat along +Y. Repeated in `test_gazebo_orientation_and_scale`. |
| With the pipeline's +90° X rotation, a Y-up GLB stands upright at the requested scale | `test_gazebo_orientation_and_scale`: surfaces read 2.0 m and 0.8 m (expected 2.0 m and 0.8 m) |
| Convex-hull collision fills gaps between parts; mesh collision keeps them open | `test_gazebo_gap_survives_mesh_collision_only`: a probe between two towers lands on the ground (mesh) or at 2.0 m (hull) |
| The visual `<pose>` + `<scale>` applied to the raw file reproduces the analysed geometry | `test_visual_pose_reproduces_normalised_mesh` (numerical) |

**Not verified:** visual-mesh orientation inside a rendered scene. The visual
mesh goes through the same gz-common loader as the collision mesh, but no
camera or lidar has been pointed at it yet. That is the next check, and it
matters because `gpu_lidar` and depth cameras ray-cast the **visual** mesh,
not the collision mesh.

## Usage

```bash
# local mesh; a size is required for unmeasured meshes (generated, web, sketch)
python3 tools/sim_asset.py import-mesh chair.glb --name office_chair --height 1.0
python3 tools/sim_asset.py drop-test office_chair --probe 0,0 --probe 0.3,0
python3 tools/sim_asset.py build-world scenario.yaml worlds/chairs.sdf
```

`build-world` writes `worlds/NAME.placement.json` next to the world. It holds
the seed, every instance pose, and the sha256 of each model manifest used.
The same scenario with the same models always produces a byte-identical world.

### Rules the code enforces

* **Scale is an input, not a measurement.** `--height/--size` (unmeasured)
  or `--unit` (known units) is required, and the manifest records which.
* **Collision defaults to the mesh itself**, capped at 20k triangles. Use
  `convex_hull`/`box` only for solid objects: a hull over a rack, chair or
  table creates false obstacles (see the gap test).
* **Mass and inertia are labelled ASSUMED.** They come from density × volume
  (the convex hull if the mesh is not watertight) or a stated mass. Static
  models write no inertia at all.
* **The original visual file is copied byte-for-byte** and hashed, so the
  textures stay intact and the provenance can be checked.
* A hand-made model directory (no `asset_manifest.json`) is never overwritten.

## ProjectSim feasibility (docs read 2026-10-21)

| Question | Answer from docs.projectsim.ai |
|---|---|
| Cost | Standard, measured and high-fidelity assets cost 4 credits each; a procedural member costs 1. A failed job is refunded. Gift credits are spent first. |
| 120 gift credits | 30 standard/measured assets. The credits expire 2026-11-18, and an event code can also cap the account's asset count. |
| Gazebo output | **None.** `physics.simulator` picks the format: USD (Isaac/Genesis), MJCF, URDF (SAPIEN), or `agnostic` = **GLB + FBX + physics sidecar JSON**. We use `agnostic` + GLB. |
| Procedural sets (1 credit each, 10 to 500 per set) | USD only. Converted by `usd_import` (UsdPhysics → multi-link SDF); validated 20/20 on ProjectSim's own published samples (see [experiments](sim_asset_experiments.md)). |
| Articulated (doors, drawers) | **Standard** route: Isaac only, so not via `agnostic`. **Procedural** route: USD joints convert to SDF joints. |
| Scale | A standard asset is **unmeasured**. Measured assets need one image and a ChArUco video, and claim "up to millimetre" accuracy. |
| Gotcha | A standard asset **defaults to articulated generation**. The plan sends `articulated: false` and `collision_granularity: null` to get reconstruction. |
| Downloads | Presigned URLs live for 1 h. Stored metadata has them stripped (they are bearer credentials). |
| Licence | projectsim.ai/terms: use is limited to robotics simulation and to robot perception/planning/control training or evaluation; outputs may not train 3D/image/video generators. Gift-credit IP assignment is unconfirmed. Details in [experiments](sim_asset_experiments.md#licensing-projectsimaiterms-read-2026-10-21). |

### Spend guard (in code, tested with a mock transport)

* The token comes from `PROJECTSIM_TOKEN` or `~/.config/projectsim/token` (refused unless chmod 600). It is never logged and never sent to storage hosts.
* `ps-submit` without `--confirm-spend` prints the request body and cost, sends nothing, and exits 2.
* A request still containing `TODO` placeholders is refused.
* Failed set members are logged as per-member refunds.
* A submission is refused if it would cut into `reserve_credits`.
* An `intent` line is appended to `sim_assets/projectsim_ledger.jsonl` **before** the request. Unresolved intents count as spent.
* The `Idempotency-Key` is a hash of the request body, so a retry after a timeout replays the original request instead of buying the asset twice.
* The ledger is a local estimate. The dashboard's Billing page is the source of truth.

### Budget

See [sim_asset_experiments.md](sim_asset_experiments.md) for the experiments and
their acceptance criteria. The budget:

| Phase | Credits | What |
|---|---|---|
| E0 | 0 | ProjectSim's free CC-BY samples through the USD path: **done, 20/20 PASS** |
| E1 | 10 | Procedural shelving-bay family (10 members, the API minimum) |
| E2 | 4 | Standard pallet jack (low-profile forks, thin handle) |
| E3 | 4 | Measured object from Omega1's real course (needs your ChArUco capture) |
| B | ≤ 62 | Re-rolls, then variants of whatever proves useful; candidates are in the plan |
| Reserve | 40 | Measured re-captures and failures |

### Workflow per asset (after approval)

```bash
# free, no token: real ProjectSim USD output
python3 tools/sim_asset.py fetch-sample articulated-logistics-cages cage_0002
python3 tools/sim_asset.py import-usd sim_assets/samples/articulated-logistics-cages/cage_0002/usd/cage_0002/cage_0002.usda \
    --name cage_0002 --source "hf:projectsim/articulated-logistics-cages" --license CC-BY-4.0
python3 tools/sim_asset.py accept cage_0002 --ref-dims 1.2,1.0,0.9

export PROJECTSIM_TOKEN=...          # or a chmod-600 file at ~/.config/projectsim/token; never commit
python3 tools/sim_asset.py ps-auth                                   # free
python3 tools/sim_asset.py ps-submit sim_assets/plans/initial_batch.yaml e2_pallet_jack --confirm-spend
python3 tools/sim_asset.py ps-fetch <asset_id>                       # free; repeat until complete
python3 tools/sim_asset.py ps-import sim_assets/plans/initial_batch.yaml e2_pallet_jack <asset_id>
python3 tools/sim_asset.py accept e2_pallet_jack
# procedural sets: ps-fetch <set_id> --set ; ps-import PLAN ITEM <set_id> --variant N
```

## Tests

`tests/test_sim_assets.py`: the fast tests cover transform maths, specs,
the model contract, USD conversion (synthetic UsdPhysics stages), seeded
worlds and the spend guard (mocked HTTP). Four `integration` tests run real
headless gz-sim drop tests. The USD tests skip without `usd-core`.

```bash
python3 -m pytest tests/test_sim_assets.py -m "not integration"
python3 -m pytest tests/test_sim_assets.py -m integration    # needs gz
```
