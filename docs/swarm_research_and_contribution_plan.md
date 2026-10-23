# Swarm engineering direction and open-source contribution plan

Research date: October 23, 2026. Target: a reproducible civilian fleet-testing
workflow, with evidence that engineers can inspect. No upstream code was copied
or new framework introduced in this change.

## What the repository actually does

At source revision `1763269895dd24c598b701c9a543e0ef564ab3df`, `swarm_logic.py`
contains the N-vehicle coordinator, all-pairs separation monitoring, assignment
recovery, route reservations, yielding, stationary-peer replanning and deadlock
reporting. Kinematic harnesses test 1/2/3/5/10 vehicles. This is materially different
from an N-vehicle PX4 demonstration.

The actual PX4 path is still a fixed pair:

- `scripts/run_swarm_demo.py`: fixed spawn/controller loops and terminal count.
- `swarm_coordinator.py`: default Fleet/HOMES for subscriptions and log headers.
- `swarm_vehicle.py`: default HOMES identities/origins; identity uses the last
  character of the vehicle name (must be changed before any future 10+ adapter).
- `swarm_verify.py`: specification derived from the default coordinator tasks.
- `scripts/export_swarm_replay.py`: samples require exactly two vehicles.

The existing GPS keep-out geometry is **not evidence of GPS sensor failure** in a
swarm mission. Seeding weather/airspace authorization inputs are **not a wind
dynamics model or a fleet airspace-compliance demonstration**. Independent raw
telemetry verification must precede any replay resampling.

`bugs/BUG-021.md` already records remaining dropout timeouts, predictor aborts and
deadlocks. Preserve these failures as useful evidence. Separation and completion
are separate metrics: a safely aborted mission did not finish its work.

## Relevant problems and code to study

| Upstream | Evidence inspected | What it teaches this project | Contribution candidate |
|---|---|---|---|
| [PX4 v1.15 multi-vehicle Gazebo](https://docs.px4.io/v1.15/en/sim_gazebo_gz/multi_vehicle_simulation) and [ROS 2 identities](https://docs.px4.io/main/en/ros2/multi_vehicle) | Official multi-instance, namespace and system-ID workflow | Reuse the flight stack and get per-vehicle identity/transport right; do not invent a competing autopilot | Reproduce a concrete multi-instance setup problem, then offer a focused tested example or docs correction if it is genuinely absent upstream |
| [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2), [`crazyflie_server.py`](https://github.com/IMRCLab/crazyswarm2/blob/main/crazyflie_sim/crazyflie_sim/crazyflie_server.py), [usage](https://imrclab.github.io/crazyswarm2/usage.html) | Server code builds enabled robot state from configuration and separately initializes simulation backends and visualization; docs describe local ROS isolation | Fleet configuration should drive simulation identities; visualization is separate from simulated dynamics. Hardware is Crazyflie, not PX4/X500, so it is a design reference rather than a drop-in replacement | A reproducible simulation/config validation test or example, after checking contribution rules and confirming maintainers want the change |
| [Open-RMF issue #108](https://github.com/open-rmf/rmf_traffic/issues/108) and [`Negotiation.cpp`](https://github.com/open-rmf/rmf_traffic/blob/main/rmf_traffic/src/rmf_traffic/schedule/Negotiation.cpp) | User report of long negotiation/oscillation when robots share a destination; source has versioned proposal tables, rejection and alternative timelines | Resource contention, liveness and idle-vs-active priority are practical fleet concerns. Ground-robot scheduling concepts are transferable; aircraft dynamics/safety claims are not | First reproduce a small shared-resource negotiation case and attach logs/metrics. A local benchmark is not a fix for the upstream issue |
| [NASA ICAROUS](https://github.com/nasa/icarous), [NASA onboard sense-and-avoid paper](https://shemesh.larc.nasa.gov/fm/papers/DASC2020-DBPSCGMM-draft.pdf) | Project architecture and paper discuss conformance monitoring, conflict resolution and return-to-mission behavior | Separate mission goals, safety monitoring and recovery. Learn from established work rather than presenting our heuristics as formal methods | Study the Python simulation examples and compare compatible scenarios only after coordinate/unit/assumption matching. Review upstream license before any reuse |

These sources establish concrete engineering problems and existing approaches;
they do not establish a commercial market gap or hiring demand. The Open-RMF item
is a user-reported issue, not a proven diagnosis of its current release. Upstream
branches and issue status can change. No issues/comments/messages were posted.

## Build faster by narrowing the claim

The useful demonstration is: **one aircraft aborts, the remaining fleet either
recovers its tasks or reports exactly why recovery cannot finish**. Keep simulator,
controller, independent verifier and viewer distinct. Before new algorithms, make
failures repeatable and measurable.

Delivery order:

1. **This change:** side-effect-free validated 1–4 vehicle experiment plans, with
   explicit supported faults, IDs, ports, layout checks and truthful stage labels.
2. **Next:** wire the contract through the existing PX4 adapters and introduce a
   versioned independent fleet specification. Retain the working two-vehicle path.
3. **Acceptance:** run 2/3-vehicle nominal and abort/land cases on the Linux SITL
   workstation. Same inputs do not guarantee deterministic physics; archive raw
   logs and versions so outcomes can be reproduced and investigated.
4. **Recovery improvement:** select one reproducible dropout failure from BUG-021.
   Distinguish physically blocked task points from scheduling deadlock. Compare
   baseline/candidate on identical seeds; require no regression in sampled
   separation and show any predictor aborts. Do not optimize the pass rate by
   removing hard scenarios or weakening the threshold.
5. **Demo:** 60–90 seconds showing fleet configuration, fault event, assignment
   change, safety decision, final task coverage and verifier verdict. Let viewers
   inspect the original evidence. No new dashboard until the execution gate works.

Candidate experiment matrix after adapter work:

| Dimension | Initial values | Primary measurement |
|---|---|---|
| Fleet size | 2, 3; 4 after those pass | Completion time, coverage, minimum sampled separation |
| Controlled abort | One vehicle after READY | Landing/disarm confirmation and reassignment latency |
| Communication fault (later) | Measured delay, missing/stale telemetry | Detection/hold/abort latency; no premature reassignment |
| Localization fault (later) | PX4-supported GPS aiding loss or estimator reset | Position validity and documented fallback; distinguish from keep-out routing |
| Environmental fault (later) | Actual supported simulator wind model | Cross-track error and abort thresholds; distinguish from a weather permission flag |

## Credibility and contribution gate

For each improvement retain a small diff, test that catches the original problem,
reproduction command, baseline/candidate outputs, source revision and limitations.
Be able to explain the coordinate frame, identity mapping, failure policy and what
the verifier can miss. Test count alone is not proof of flight safety.

For an upstream contribution: reproduce locally, inspect current tests and
CONTRIBUTING/license, make a minimal compatible regression test, and open a focused
PR only after confirming the issue still exists. Do not submit project-promotion
issues or claim maintainers endorse this work. No upstream contribution has been
made by this change.

Interview description supported now: “I am building a PX4 simulation and
verification platform. It has a recorded two-vehicle recovery case and a separate
larger-fleet kinematic harness. I am connecting those through validated experiment
configurations so that fleet failures can be reproduced and compared.”

This patch adds configuration infrastructure; it does not yet improve measured
flight completion rates, resolve BUG-021's remaining failures, or prove larger
PX4 fleets. ROS 2/PX4/Gazebo were unavailable in the editing environment.
