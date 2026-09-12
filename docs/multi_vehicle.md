# Cooperative two-UAV gate / landing-point mission

The current coordinator flies four **point visits**, not the older four-lane survey. Legacy filenames (`run_swarm_demo.py`, `swarm_survey.launch.py`) and phase names (`SURVEY`) remain for compatibility. No payload delivery or imagery/area coverage is implemented.

## Run

On the existing Linux PX4/Gazebo workstation, from the repository root:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/run_swarm_demo.py             # nominal
python3 scripts/run_swarm_demo.py --dropout   # flagship recovery
```

Headless is default; add `--gui` for Gazebo. `--px4-dir` selects an already-built PX4 checkout. This runner owns isolated simulation processes, uses local simulated MAVLink endpoints and does not connect to hardware. Do not run another instance using PX4 IDs 1/2 concurrently. It cleans up only its own process groups.

## Current geometry

Coordinates are shared-world NED (north, east, down), metres. Spawns: `px4_1` at `(0,-3,0)`, `px4_2` at `(0,7,0)`. Gates and landing-approach points are:

| Task | Initial owner | Position N/E/D |
| --- | --- | --- |
| gate_1 | px4_1 | 28 / 11 / -3 |
| land_1 | px4_1 | 50 / -2 / -3 |
| gate_2 | px4_2 | 28 / 16.5 / -3 |
| land_2 | px4_2 | 50 / 5 / -3 |

A* routes around the five known physical obstacles and the visible GPS-denied prism (used only as a keep-out in this launch). The fence is N[-5,55], E[-6,18], maximum altitude 6 m. Static obstacle clearance is 1.5 m; inter-vehicle reservation is 3.5 m; minimum sampled horizontal separation is 2.5 m. Task start=end means the verifier checks point visits; older nonzero transect logic is retained.

## Architecture and failure behavior

- `swarm_logic.py`: pure coordinator, A* transit, task ownership, measured/predicted separation and persistent segment reservations. Intermediate route acceptance is 1 m; terminal task acceptance is 0.5 m, inside the verifier's 0.6 m observation radius.
- `swarm_vehicle.py`: independent PX4 controller per vehicle, setpoint streaming, frame alignment, command freshness, geofence and landing checks. Coordinator command loss holds after 0.75 s and lands after 2 s.
- `swarm_coordinator.py`: ROS adapter; records synchronized telemetry, commands, assignments, retirement and completion.
- `swarm_verify.py`: fails incomplete missions, stale/missing evidence, time gaps, insufficient separation, unobserved tasks and unconfirmed final landing/disarming.

A commanded dropout lands vehicle 2 shortly after it becomes ready. The coordinator holds its peer until fresh landed/disarmed evidence persists for at least 1 s, then reassigns unfinished work. The retired vehicle remains a reserved obstacle. Missing telemetry is different: it aborts coordination rather than guessing that the absent vehicle landed.

Session IDs, increasing sequence numbers and same-host monotonic timestamps protect command freshness. Estimator frame resets invalidate alignment and trigger an abort. The coordinator timeout is 330 s; the local controller cap is 340 s; runner default is 390 s. Actual execution time depends on the simulator.

## Evidence and transport

[Golden recovery](golden_demo.md) passed in a fresh September 12 run: two reassignments, 641 synchronized samples and 8.095491 m minimum horizontal separation. [Curated evidence](../evidence/README.md) includes the raw compressed log and reproducible report. Prior lane-survey results are historical, not current-scenario certification.

Each run writes `demo_artifacts/swarm/<run>/` with process/PX4 logs, raw `swarm_<session>.jsonl` and `verification.json`. Exit 0 means its verifier passed. The runner uses ROS domain 88, DDS UDP 8889, a unique Gazebo partition, and PX4 system IDs 2/3. `swarm_survey.launch.py` starts ROS nodes only against already configured simulation dependencies; `two_vehicle.launch.py` is only the older independent hover/land smoke test.

## Limits

Fixed two vehicles on one host, known-map routing, sampled separation, no full dynamic-collision guarantee. No LiDAR, camera, GPS-aiding-loss handling, hardware or distributed swarm support in this cooperative launch. Landed/disarmed evidence does not establish touchdown accuracy on a visual pad. The standalone verifier does not independently audit reassignment dwell; direct tests and golden timing inspection cover that policy. Its task geometry comes from current code, so old lane recordings require their original scenario version.
