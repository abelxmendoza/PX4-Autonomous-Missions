# Cooperative two-UAV survey

The project now has a central survey coordinator, two independent PX4 flight
controllers, conservative route reservations, and confirmed-failure task
reassignment. The original single-drone mission remains available separately.

## Run the simulation

Prerequisites: the existing built PX4 SITL checkout, Gazebo Harmonic,
MicroXRCEAgent, ROS 2 Humble, built px4_msgs, and the Python dependencies in
`requirements.txt`. From the repository root:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/run_swarm_demo.py
```

Run the recovery scenario:

```bash
python3 scripts/run_swarm_demo.py --dropout
```

The runner owns Gazebo, two PX4 instances, a DDS agent, two controllers, and the
coordinator. It runs headless by default; `--gui` opens Gazebo. `--px4-dir`
selects another already-built checkout. It does not connect to hardware.
Do not run another demo with PX4 instances 1/2 concurrently.

Each run gets a directory under `demo_artifacts/swarm/` with process logs,
PX4 logs, synchronized `swarm_<session>.jsonl`, and `verification.json`.
Exit code 0 means verification passed. Interrupting the runner stops its own
simulation processes and records an incomplete run. It never kills unrelated
simulators.

The fixed scenario now flies in `worlds/obstacle_world.sdf` — the same
obstacle course and geofence area as the single-vehicle mission — with
spawns at Gazebo ENU `(0,0,0)` (px4_1, the existing launch pad) and
`(14,0,0)` (px4_2, a second mint-colored pad added to the world for this
scenario). It flies four northbound survey transects from N=4 to N=14 at
E=-2, 1, 4, and 7 m, at 3 m altitude. Initially each drone owns two lanes.
Survey completion describes the flight transects, not camera imagery or
mapped area.

The east offsets thread the gap between OB1 (east ~[-7.5,-4.5]) and OB2
(east ~[8.5,11.5]) with >=1.5 m margin on either side; the survey band
(north 4-14) brackets that obstacle pair and stops 1.5 m short of the
GPS-denied zone at north=15.5 — this swarm has no GPS-denial handling, so
flying into that zone would be a visually confusing no-op, not a real event.
Extending further north to also thread OB3/OB4/OB5 needs per-band east
offsets, since a single constant-east lane can't clear all three obstacle
bands at once — left for a future pass.

The launch pads and swarm homes sit at north=0, east=-3 and east=7,
aligned side by side inside the front of the course. Obstacle avoidance uses a dedicated `obstacle_clearance` (1.5 m),
smaller than the inter-vehicle `reservation` margin (3.5 m) — the larger
value guards live separation between two moving, uncertain vehicles, and
was too generous to leave any routable space between OB1 and OB2 when
applied to fixed geometry.

## Architecture and behavior

- `swarm_logic.py`: pure coordinator, task state, A* routing around retired
  vehicles and the physical obstacle course (`DEFAULT_OBSTACLE_COURSE` in
  `mission_logic.py` — the same obstacle definitions the single-vehicle
  mission avoids), and conservative horizontal segment reservations. It
  monitors measured separation and constant-velocity closest approach over
  one second. Obstacle avoidance only covers commute legs planned via A*
  (home->task, task->task, task->home); the straight survey transects
  themselves are hand-laid-out to clear the obstacles instead, since those
  legs bypass planning by design.
- `swarm_vehicle.py`: per-vehicle PX4 control loop, pre-streaming, takeoff,
  rate-limited position setpoints (3 m/s, 1.5 m setpoint lead), yaw facing
  the direction of travel (`yaw_toward`, same convention as the
  single-vehicle mission — NaN when already at the setpoint, which PX4
  reads as hold-current-yaw), geofence and tracking checks, and landing.
  Loss of coordinator commands holds locally after 0.75 s and lands after 2 s.
  The coordinator allows 330 s for a recovered single-drone route; the local
  watchdog caps flight at 340 s and the process runner at 390 s.
- `swarm_coordinator.py`: ROS transport adapter and synchronized evidence log.
- `swarm_verify.py`: rejects incomplete missions, stale/missing evidence,
  time gaps, insufficient separation, unobserved transects, and unconfirmed
  landing/disarming. Completion flags alone do not prove transect coverage.

Vehicle IDs are `px4_1` and `px4_2`, with MAVLink system IDs 2 and 3. Each
controller subscribes only to its own PX4 topics. Shared positions use world
NED, calculated from the configured spawn and the initial stationary local
position. Estimator frame resets invalidate that alignment and trigger an
abort; they are not silently treated as vehicle motion.

A commanded vehicle failure causes that drone to land. Its assignments remain
reserved until fresh telemetry confirms landed and disarmed for one second.
Then unfinished lanes are reassigned. The retired drone remains a reserved
obstacle, and transit routes can detour around it. An obstructed survey
transect stays incomplete rather than being replaced by a coverage-skipping
detour. Loss of telemetry is different: the coordinator aborts the mission
because the missing vehicle's location cannot be confirmed.

Commands carry coordinator and vehicle session IDs, increasing sequence
numbers, and expiring timestamps. This version requires a single host and its
shared monotonic clock. It is a fixed two-vehicle SITL implementation, not a
hardware-ready or general distributed swarm controller. Route reservations
and sampled separation checks are not a proof against all dynamic collisions.
The vehicles now fly through the same physical obstacle course as the
single-vehicle mission (avoided via the shared known-map A* planner, not
sensor evidence). The existing LiDAR, camera, VIO, and GPS-denial behaviors
are still not part of this cooperative launch — neither vehicle carries
those sensors or runs that logic.

## Transport and manual launches

The runner uses ROS domain 88, DDS-agent UDP port 8889, and a unique Gazebo
partition. Gazebo discovery is bound to `127.0.0.1`; ROS localhost-only mode is
disabled because the bare DDS agent does not follow that ROS setting. These
settings are applied only to the runner's child processes. MAVLink GCS
heartbeats go to the two simulated endpoints independently. Offboard RC-loss
exception is configured for the simulation; other PX4 failure handling remains.

After a package build, `swarm_survey.launch.py` launches the cooperative ROS
nodes against an existing, correctly configured two-vehicle simulation and
DDS agent. It does not launch those dependencies itself:

```bash
colcon build --symlink-install --packages-select px4_offboard
source install/setup.bash
ros2 launch px4_offboard swarm_survey.launch.py
```

Use matching ROS domain and DDS settings in all manual terminals. The older
`two_vehicle.launch.py` is only an independent hover/land smoke test using DDS
port 8888. It is not the cooperative survey launch. Both assume the documented
vehicle identities; the survey also requires the exact spawn offsets above.

## Verification

```bash
PYTHONPATH="src/px4_offboard:$PYTHONPATH" python3 -m pytest src/px4_offboard/test -q
python3 -m px4_offboard.swarm_verify path/to/swarm_session.jsonl
```

ROS node tests require the sourced environment. Pure coordinator tests run
without ROS and cover full kinematic missions, confirmed dropout/reassignment,
stale telemetry, process restarts, frame conversion, and geometric conflicts.
Kinematic tests are not PX4 flight evidence. CI runs both test groups.

The normal PX4/Gazebo run on September 7 completed all four lanes and landed
both vehicles; its verifier measured a minimum horizontal separation of
7.93 m across 1,648 samples. Local evidence is under
`demo_artifacts/swarm/20260907_180127_6cf541/`. Recovery flight verification is
being completed separately. Generated run artifacts are intentionally ignored
by Git.

## Open-source components

| Component | Use |
| --- | --- |
| [PX4 multi-vehicle ROS 2 guide](https://docs.px4.io/v1.16/en/ros/ros2_multi_vehicle) and [Gazebo guide](https://docs.px4.io/v1.16/en/sim_gazebo_gz/multi_vehicle_simulation) | Implemented instance IDs, command routing, and DDS namespaces using the installed PX4 stack |
| Existing repository A* planner | Reused for transit around a confirmed landed drone |
| [libMultiRobotPlanning](https://github.com/whoenig/libMultiRobotPlanning) (MIT) | Candidate for future timed multi-agent planning; not installed or copied |
| [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2) (MIT) | Architecture reference; its Crazyflie vehicle stack is not integrated |

Integration code is original. A future imported dependency should pin a
reviewed revision and retain its copyright and license notices.
