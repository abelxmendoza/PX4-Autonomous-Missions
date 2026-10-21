# Fleet experiment preflight (initial contract)

This is a **simulation-only configuration validator**, not a fleet launcher. It does not
start PX4, Gazebo, ROS 2, or any hardware connection. The existing two-vehicle
`scripts/run_swarm_demo.py` remains unchanged.

## Try it

From the repository root, with the CI dependencies installed:

```bash
PYTHONPATH=src/px4_offboard python3 -m px4_offboard.fleet_experiment config/fleet_experiment_example.json
PYTHONPATH=src/px4_offboard PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_fleet_experiment.py -q
```

The CLI prints normalized vehicle assignments as JSON and exits with code 2 for
invalid input. It does **not** create the output directory.

The initial contract accepts 1–4 vehicles, the `cooperative_point_visits`
scenario, an unsigned 32-bit integer seed, and an empty fault schedule. Nonempty
fault schedules are rejected until a validated fault schema and SITL adapter exist.

Default positions come from `swarm_fleet_layout.make_fleet`; default system IDs
and MAVLink ports match the existing two-vehicle runner. A custom `vehicles`
array may override spawn coordinates but must preserve those fixed assignments.
Spawns must be at least 3.5 m apart.

## Next integration gate

1. Wire validated configurations into a **separate** parameterized SITL runner.
2. Verify coordinate conventions and simulator spawn positions against the existing
   two-vehicle baseline before launching additional instances.
3. Emit manifests containing seed, code revision, PX4/Gazebo versions and artifact hashes.
4. Run nominal and fault scenarios at two fleet sizes; verify failures remain FAIL.
5. Preserve the existing golden evidence and compare regression results.

No SITL run, ROS integration, or hardware validation is claimed by this change.
