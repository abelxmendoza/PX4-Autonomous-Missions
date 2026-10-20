# Autonomous Fleet Lab — Product direction (draft)

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
**Current:** fixed two-vehicle cooperative point-visit mission, PX4 SITL recordings, recovery verifier, browser replay, other single-vehicle experiments.

**Planned:** configurable fleet counts; general cooperative coverage missions; altitude/terrain datasets; synthetic airspace/traffic; parameterized weather; comparative reporting. None should be represented as production-ready today.

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
