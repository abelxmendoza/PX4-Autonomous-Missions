# Autonomous Fleet Lab — Product direction and next acceptance gate

## Engineer problem
Existing tools already fly, simulate and display multiple aircraft. This project should not duplicate PX4, Gazebo or QGroundControl. The product hypothesis is that engineers need a repeatable, evidence-first workflow to **compare autonomous fleet behavior across mission configurations and failure conditions**, with clear traceability from requirements to recorded telemetry.

This is a hypothesis to validate with users, not a claim that the market lacks alternatives.

## Target workflow
1. Select a mission scenario and number of simulated vehicles.
2. Set reproducible parameters: seed, initial positions, task set, terrain/altitude reference, synthetic traffic and fault schedule.
3. Validate configuration and run the scenario against local PX4/Gazebo SITL.
4. Collect per-vehicle logs and synchronized mission events.
5. Evaluate independent requirements; report PASS / FAIL / SKIP with evidence and limits.
6. Compare two runs by mission coverage, completion time, fault recovery and sampled separation.

## MVP: fleet-size experiments
- Support a parameterized **simulation-only** scenario with 1, 2, 3, 4 vehicles initially.
- Preserve the existing two-vehicle recovery scenario and its golden evidence unchanged.
- Define stable IDs, spawn spacing, per-vehicle namespace/port assignments and deterministic task partitioning.
- Reject invalid fleet sizes or overlapping starts before launching.
- Test task ownership, retirement, reassignment, telemetry freshness and verifier failure behavior.
- Produce machine-readable run manifest and comparison summary.
- Expose configuration through a local CLI first; UI selection follows after the adapter works.

## Current vs planned
**Current:** a recorded two-vehicle PX4 cooperative point-visit mission; an N-vehicle coordinator exercised at 1/2/3/5/10 vehicles in a kinematic harness; recovery verification; guided browser replays; offline ULog and recorded-perception verification; a synthetic-depth prototype; and an experimental RealmForge world adapter. The larger fleet harness is not a larger PX4 SITL demonstration.

**Next:** connect the existing fleet layout/coordinator to a parameterized PX4 runner and reproducible run manifests. Synthetic weather, airspace and traffic inputs already exist for seeding; a unified fleet experiment configuration and comparison workflow does not. General coverage missions, terrain datasets, hardware perception and remote execution remain later work.

## Non-goals
- Building a replacement autopilot or GCS.
- Treating visualization as a validated physics model.
- Claiming real-world collision avoidance, airspace compliance, or certified flight safety.
- Claiming cloud-seeding effectiveness from simulated flight.

## First acceptance gate
- 1/2/3/4-vehicle configurations can be parsed and validated.
- At least two fleet sizes execute against SITL on a documented Linux setup.
- No regressions in pinned two-vehicle golden tests.
- Reproducible logs identify exact simulator/PX4 versions and scenario seed.
- At least one deliberately failed requirement produces FAIL, not a misleading PASS.

## User discovery
Interview 5 engineers or researchers who run PX4/Gazebo tests. Ask about repeated manual setup, failure reproduction, multi-run comparison, and auditability. Compare with existing QGroundControl and PX4 simulation workflows before claiming differentiation.

## Delivery order after consolidation

1. **Fleet experiment contract and preflight validation.** Reuse `swarm_fleet_layout.make_fleet`;
   accept counts 1–4 initially, a scenario ID, seed, fault schedule and output directory. Reject
   overlapping starts, invalid IDs/ports and unsupported combinations before launching a process.
   Test both accepted and rejected configurations. Do not add another coordinator.
2. **Parameterized local PX4 runner.** Derive namespaces, ports, vehicle IDs and cleanup ownership
   from that contract. Keep the current two-vehicle command and golden evidence unchanged.
   Completion must count the configured vehicles, not a hard-coded pair.
3. **Evidence and comparison.** Write config, seed, code revision, dirty state, PX4/Gazebo versions,
   artifact hashes and termination reason to a manifest. Compare coverage, completion time,
   recovery and sampled separation, with missing evidence explicitly incomplete.
4. **Acceptance campaign.** Run at least two fleet sizes in real PX4 SITL, including one nominal and
   one fault case. Preserve an intentionally failing verification and reproduce it offline.
5. **UI only after that gate passes.** Add experiment selection/results around the working CLI.
   Hosted completed results precede remote simulation; hardware work stays separately labelled.

The initial side-effect-free contract for item 1 is available in
[`fleet_experiments.md`](fleet_experiments.md). It validates and prints a plan;
it does not launch a larger PX4 fleet. The next integration task is item 2,
including the ROS adapters and independent verifier specification. Consolidation
and preflight validation do not mark the fleet execution gate passed.
See [the research and contribution plan](swarm_research_and_contribution_plan.md)
for upstream references, known gaps and the focused demonstration target.
See [the consolidation record](consolidation_20261020.md) for integrated work and validation.
