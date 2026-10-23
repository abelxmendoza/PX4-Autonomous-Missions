# Fleet experiment preflight (simulation only)

This implements item 1 of PRODUCT_DIRECTION.md: a strict experiment contract that
reuses `swarm_fleet_layout.make_fleet`. It validates configuration and prints a
normalized plan; **it does not launch PX4, execute faults or establish flight
success**. The current runner, ROS nodes, verifier, golden evidence and controller
behavior are unchanged.

```bash
PYTHONPATH=src/px4_offboard python3 -m px4_offboard.fleet_experiment \
  config/fleet_experiments/three_vehicle_dropout.json
python3 -m pytest src/px4_offboard/test/test_fleet_experiment.py -q
```

Schema 1 requires `fleet_size` (integer 1–4), `scenario`, `seed` (unsigned 32-bit
integer), and `output_dir`. Only `gate_visit_nominal` and `gate_visit_dropout`
exist. A dropout requires at least two vehicles and exactly one `abort_and_land`
fault, with a known vehicle and finite positive `after_ready_s`. Timing is relative
to that vehicle becoming READY, not launch/wall time. Nominal cannot contain faults.
Unrecognized fields are errors; a misspelled or unsupported wind/GPS option must
not silently become a nominal experiment.

The seed is retained as experiment identity, **not a claim of randomized or
repeatable simulator physics**. The output directory is recorded, not created.
Configuration validation never starts processes, opens flight links or writes
artifacts. The CLI exits 2 on invalid input and prints only the plan on success.

Generated bindings use the existing runner's convention: instances 1–N, namespaces
`/px4_1`…`/px4_N`, target systems instance+1 and MAVLink GCS UDP ports 18570+instance.
The shared DDS agent port is 8889. These are plans; a runner must check port
availability, reserve resources and isolate concurrent runs before use.

Preflight checks finite coordinates, fence margins, obstacle/GPS-keep-out
clearance, home/pad/gate spacing and identity/port consistency. Layout is generated,
not user-overridden. `validate_layout` is also independently tested using corrupted
layouts/bindings. Geometric preflight does not prove dynamic collision freedom,
route feasibility, estimator quality or mission liveness.

## Next executable gate

1. Pass this exact plan into runner, coordinator and vehicle nodes. Remove fixed
   HOMES assumptions consistently; do not edit only the launch loop.
2. Extend verification with an explicit versioned fleet/task specification while
   retaining the old two-vehicle schema and golden results unchanged.
3. Record simulator/PX4 versions, source revision/dirty state, input configuration,
   termination reason, raw artifact hashes and verifier version.
4. Run 2- and 3-vehicle nominal/dropout cases in actual PX4/Gazebo. Include an
   intentionally failing run. Report completion, coverage, recovery latency,
   sampled minimum separation, predictor aborts and deadlock/timeouts separately.

Do not call the larger-fleet SITL acceptance gate passed until those runs exist.

## Validation of this change (October 23, 2026)

Linux/Python 3.12, without ROS/PX4/Gazebo:

- 81 passed: experiment preflight, existing fleet layout, swarm logic and export tests.
  This includes 44 new preflight cases.
- 67 passed: existing fleet, invariants, gridlock and scaling tests (kinematic).
- `python3 scripts/verify_evidence.py`: 9/9 expected outcomes matched; deliberately
  failing cases still fail as expected. Includes byte-hash and replay/report checks.
- Example preflight CLI succeeded; `git diff --check` passed.

These two test groups are distinct (148 tests total). No new flight was executed,
and none of these results demonstrate improved flight performance.
