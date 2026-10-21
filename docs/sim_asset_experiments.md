# ProjectSim experiments and acceptance criteria

Status 2026-10-21: **10 credits committed** (E1 submitted, generating). E0 is done
and free. E2 and E3 are
planned in [`sim_assets/plans/initial_batch.yaml`](../sim_assets/plans/initial_batch.yaml)
and need approval: 18 of the 120 gift credits, with 40 held in reserve. The
credits expire 2026-11-18.

ProjectSim is Kaedim's product. The docs, terms and Hugging Face datasets all say
"projectsim by Kaedim", and the MoltenHack promo code came from the Kaedim card.
These 120 credits are therefore the Kaedim hackathon resource, not a separate pool.

## Endpoint facts (docs.projectsim.ai, read 2026-10-21)

| Endpoint | Credits | Count | Output we can use | Joints | Scale |
|---|---|---|---|---|---|
| Standard | 4 | 1 | `agnostic`: GLB + physics sidecar | Articulated generation is **Isaac USD only** | unmeasured |
| Measured | 4 | 1 | `agnostic` GLB | none | ChArUco; "up to mm" |
| Procedural set | 1 per member | **10 to 500** | **USD only**; converted by `usd_import` | yes (rigid, joints) | prompt-guided, not enforced |
| High-fidelity set | 4 per member | 1 to 100 | `agnostic` GLB | none | unmeasured |

Failed jobs and failed set members are refunded. The smallest procedural
experiment costs 10 credits.

## E0: does real ProjectSim USD run in Gazebo Harmonic? (free, done)

Before spending anything, the USD path was tested on ProjectSim's own published
output: the CC-BY-4.0 datasets `projectsim/articulated-logistics-cages` and
`articulated-desks` on Hugging Face (revisions pinned in `tools/sim_assets/hf_samples.py`).

```bash
python3 scripts/eval_projectsim_samples.py   # -> evidence/sim_assets/projectsim_samples.json
```

**Result: 20/20 PASS** (5 cages and 5 desks, each in static and articulated mode)
on gz-sim 8.15.0:
- Joint count, total mass and dimensions match the published manifests exactly.
- The models load with no `[Err]` lines, and the probes settle within bounds.
- The articulated models stay put after settling.

**Answers to the compatibility questions:**

* **USD → Gazebo needs a converter, but not Blender.** gz-sim 8 has no USD loader
  here, and `usd2sdf` is not installed. `tools/sim_assets/usd_import.py` reads
  UsdPhysics with Pixar `usd-core` and writes multi-link SDF directly:
  * each rigid body becomes a link;
  * visual meshes go to a textured GLB per link;
  * the collision mesh goes to an STL per link;
  * `MassAPI` principal inertia becomes `<inertial>`;
  * revolute, prismatic and fixed joints carry their axes and limits.
* **Joint conventions are cross-checked.** The cage gate axis comes out as
  (0, 0, −1), matching the MJCF ProjectSim ships for the same asset. Gazebo
  reports the identical axis in its joint frame.
* **Two Gazebo pitfalls were found by the acceptance checks, not by reading:**
  1. `type="continuous"` silently becomes FIXED in gz-sim 8's dartsim. Unlimited
     wheels and casters are therefore written as `revolute` with no `<limit>`.
  2. ProjectSim's own MJCF collision is coarse (the whole cage is one box). Its USD
     carries a detailed collision mesh (26k triangles, `approximation=sdf` for
     Isaac). We use that mesh directly for static obstacles, and convex hulls for
     dynamic links.
* **Visuals verified by rendering.** `tools/sim_asset.py render` takes a headless
  ogre camera snapshot. The cages and desks render upright, at scale and textured
  ([wide](../evidence/sim_assets/warehouse_projectsim_render.png),
  [close](../evidence/sim_assets/warehouse_projectsim_closeup.png)).
* **Visual files are a .gltf per link with shared textures.** Embedding the
  texture atlas in a GLB per link duplicated it for every part: 387 MB for 10
  models, now 79 MB.
* **Not yet verified:** that joints actually *move* under load in Gazebo (only
  parsing and stability were checked).

## In use now: `worlds/warehouse_projectsim.sdf`

15 instances of the CC-BY samples (2 of each cage, 1 of each desk), placed by
[`sim_assets/scenarios/warehouse_projectsim.yaml`](../sim_assets/scenarios/warehouse_projectsim.yaml)
(seed 2026, 1.5 m clearance, spawn kept clear). Placement is recorded in
`worlds/warehouse_projectsim.placement.json`. The world loads in headless gz-sim
with all 15 models and no errors.

```bash
python3 scripts/eval_projectsim_samples.py --install models_generated   # free; models are gitignored
GZ_SIM_RESOURCE_PATH=$PWD/models_generated:$GZ_SIM_RESOURCE_PATH gz sim worlds/warehouse_projectsim.sdf
```

## Acceptance criteria (automated: `tools/sim_asset.py accept NAME`)

| # | Criterion | Threshold | Applies to |
|---|---|---|---|
| A1 | Scale vs reference | each extent within max(1 cm, 2 %) | measured (vs tape/caliper), procedural (vs prompt range), samples (vs manifest) |
| A2 | Rests on ground | \|z_min\| ≤ 2 cm | all |
| A3 | Collision budget | ≤ 50k collision triangles per model | all |
| A4 | Joints well formed | unit axis; finite lower < upper unless unlimited | articulated |
| A5 | Has mass | every link > 0 kg | dynamic |
| A6 | Loads in Gazebo | model present, no `[Err]` lines | all |
| A7 | Probes settle | all probe cubes at rest within 40 s | all |
| A8 | Dynamic stability | model drift ≤ 2 cm after settling | articulated |
| A9 | Contact within bounds | every probe surface between 0 and the model top | all |

Experiment-specific criteria follow. Each one targets what the experiment is for.

## E1: procedural warehouse shelving family (10 credits)

10 shelving bays from one prompt (height 1.5 to 2.5 m, width 0.9 to 1.8 m,
3 to 5 levels), converted by `usd_import` in static mode.

* A1 to A9 on every member, with A1 checked against the **prompt ranges**. Docs:
  "Prompts do not enforce exact clearances", so the out-of-range rate is reported
  rather than assumed.
* **Gap preserved:** a probe dropped between shelf levels lands on a lower deck,
  not on the top level. Same method as `test_gazebo_gap_survives_mesh_collision_only`.
* **Variety:** at least 6 of 10 members differ in height or width by at least
  10 cm, so the family actually varies.
* **Seeded worlds:** `build-world` places a random subset per seed. The same seed
  gives a byte-identical world (already tested).
* **Pass:** at least 8/10 members accepted. Failed members are refunded and logged.

## E2: standard industrial obstacle, pallet jack (4 credits)

* A1 to A9, with A1 on length: we scale to 1.55 m. Width and height are reported
  against typical pallet-jack specs (about 0.55 m wide; forks 8 cm or less).
* **Low profile kept:** a probe on a fork lands at 0.05 to 0.12 m, so the forks
  were not merged into a slab.
* **Thin handle kept:** the handle stays a separate thin shape above 0.8 m, and a
  probe beside it reaches the ground.
* **Physics sidecar:** archived verbatim in the manifest. Its fields are recorded,
  **not trusted**, until checked against the GLB.

## E3: measured physical object, ChArUco (4 credits; needs your capture)

The object must be one that sits on Omega1's real test course and fits the capture
constraints: Letter-size board, about 0.5 m from the camera, whole object in frame
(roughly 40 cm or smaller), matte, and contrasting with the surface.

* **A1 against your own measurements:** three extents with tape or caliper, each
  within max(3 mm, 1 %). This is ProjectSim's "up to mm" claim, tested
  independently.
* The board's 50 mm print-scale line must be confirmed before capture (their
  instruction). Otherwise the result is invalid, not a ProjectSim failure.
* **Privacy:** Kaedim hosts and processes inputs, and the capture video shows your
  room. Film on a clear surface.
* The `upload_id` placeholders stay `TODO` until then. `ps-submit` refuses to send
  a request that contains `TODO`.

## Navigation benchmark (after E1/E2; not built yet)

Seeded worlds from accepted assets. The planned metrics per run:
- success / collision;
- minimum clearance to any obstacle surface (from Gazebo ground-truth poses);
- path length ÷ straight-line length;
- time to goal;
- for the depth comparison: obstacle detection range by sensor (RealSense model,
  Depth Anything V2, ultrasonic).

PX4 (x500 with lidar) can run this first. **Omega1 cannot until its URDF and Gazebo
twin exist.** That is the blocking dependency, and it is not a ProjectSim problem.

## Licensing (projectsim.ai/terms, read 2026-10-21)

* **Use:** limited to robotics simulation and to training or evaluating robot
  perception, planning and control. That fits all of the above.
* **Training restriction:** outputs may not train, fine-tune or distil models whose
  main purpose is generating images, video or 3D assets. Anyone we give outputs to
  must receive the same restriction.
* **Kaedim's rights:** Kaedim keeps a broad licence to outputs.
* **Open question:** the IP assignment is "after payment", and it is unconfirmed
  whether gift-credit outputs qualify. Ask Kaedim before publishing generated
  assets. Raw downloads are gitignored.
* **The free HF samples are CC-BY-4.0:** usable now, with the attribution
  "projectsim by Kaedim" and the dataset URL. Both are recorded in every
  converted manifest.
