# RealmForge XR ↔ PX4 Missions: delivery roadmap

## Target outcome
Author a 3D world in RealmForge XR, configure a mission, simulate a two-UAV fault/recovery scenario using PX4/Gazebo, inspect independent verification evidence, and replay results on a hosted mobile browser while the development laptop is off.

RealmForge owns scene authoring and XR presentation; PX4 Missions owns physics integration, autonomy, test execution, evidence, and verification. Keep repositories separate with a versioned data contract.

## Release 0.1 — world import (in progress)
- [x] Read RealmForge v1 JSON and export primitive static obstacles to SDF.
- [x] Add deterministic fixture, unit tests, and GitHub Actions smoke checks.
- [x] Add `--world-file` to the existing swarm runner.
- [ ] Validate Gazebo loads the generated SDF on the configured Linux workstation.
- [ ] Confirm coordinate mapping, collision geometry, and vehicle spawn clearance visually.
- [ ] Confirm the two-drone recovery scenario completes or document failures in the imported scene.
- [ ] Make the adapter docs match the tested implementation; merge PR #3 after checks.

**Gate:** a repeatable command loads an exported world and produces a recorded PX4 SITL run. A passing XML test alone does not meet this gate.

## Release 0.2 — mission authoring
- [ ] Define versioned `px4-mission` JSON contract (world reference, spawn poses, waypoint coordinates, altitude, safety constraints, fault scenario).
- [ ] Export missions from RealmForge without exposing MAVLink/ROS to the browser.
- [ ] Validate bounds, coordinate transforms, spawn clearance, and route/obstacle intersection.
- [ ] Add an adapter from validated mission JSON to a supported simulation runner; retain existing golden mission unchanged.

**Gate:** edit waypoints in RealmForge, export, and reproduce the same mission in SITL.

## Release 0.3 — replay and verification
- [ ] Add a versioned telemetry/evidence API schema; reuse existing golden recording and independent verifier.
- [ ] Overlay world geometry, drone trajectories, events, and requirement results in the existing Three.js replay.
- [ ] Show fault, reassignment, and recovery in a timeline with scrub controls.
- [ ] Add regression tests for time alignment, axis conversion, and missing telemetry.

**Gate:** the existing golden run can be replayed in its authored environment with traceable verification evidence.

## Release 0.4 — hosted mobile results
- [ ] Make the replay and verification UI responsive and touch-accessible.
- [ ] Persist world exports and completed run artifacts in authenticated hosted storage.
- [ ] Add read-only mission history, run comparisons, and shareable permission-controlled result URLs.
- [ ] Test from a phone with the laptop physically off.

**Gate:** previously completed missions and their evidence are accessible from the phone with no local machine.

## Release 1.0 — remote execution
- [ ] Provision isolated Linux simulation worker with pinned PX4/Gazebo/ROS dependencies.
- [ ] Implement authenticated job submission, queue, resource quotas, timeout, cancellation, and artifact upload.
- [ ] Stream *read-only* job status and telemetry to the hosted UI.
- [ ] Prevent arbitrary shell execution and never expose MAVLink/ROS ports to the public internet.
- [ ] Run a full end-to-end cloud test from the phone while the laptop is off.

**Gate:** design → submit → simulate → verify → replay succeeds remotely and is reproducible.

## Later — scan-to-sim and immersive review
- [ ] Convert supported scan meshes with explicit scale, collision proxies, and origin alignment.
- [ ] Validate geometry/physics quality before using scanned spaces for navigation claims.
- [ ] Offer XR replay inspection using the same world and evidence contract.

## Risks and constraints
- RealmForge object positions are base-centered Y-up; Gazebo is Z-up. Validate transform and yaw with asymmetric rotated fixtures.
- Static SDF boxes are only coarse collision proxies, not photorealistic or physically validated scans.
- Existing swarm routes/spawn points are fixed and may intersect imported geometry.
- Browser-hosted static sites cannot run PX4/Gazebo; live simulation needs an always-on local or cloud Linux worker.
- No physical flight or hardware safety certification is implied by SITL verification.

## Recommended execution order
1. Finish PR #3 and verify a live custom-world SITL run.
2. Build a small mission schema and adapter, not a new simulator.
3. Extend the *existing* replay and verification UI.
4. Make recorded results mobile-friendly.
5. Add remote execution only after the local pipeline is reproducible.
