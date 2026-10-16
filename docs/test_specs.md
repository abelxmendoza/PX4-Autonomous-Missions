# Declarative test specifications

A test spec is a YAML file that names a vehicle adapter, an ordered mission, a fault list and a set of
assertions. The runner validates it strictly, drives the mission through an explicit state machine,
records the telemetry the adapter reports, evaluates the assertions and writes a machine-readable
verification report traced to [`requirements/registry.yaml`](../requirements/registry.yaml).

**Status: first vertical slice. Only the mock adapter exists.** It wraps `vehicle.sim.SimVehicle`, a
first-order velocity-tracking point mass on a `FakeClock`. A PASSED mock run shows that the spec,
the runner and the assertion logic work against a test double. It is not a PX4 SITL, HIL or flight
result, and every report says so in `environment` and `evidence_notice`.

```bash
PYTHONPATH=src/px4_offboard python3 -m px4_offboard.testspec --spec config/test_specs/basic_mission.yaml
python3 tools/run_test_spec.py --spec config/test_specs/basic_mission.yaml --validate-only
```

Exit status: 0 PASSED, 1 FAILED, 2 ERROR (including an invalid spec), 3 ABORTED. Ctrl-C cancels
cleanly and still writes the report. Artifacts go to `artifacts/test_runs/<run_id>/` (git-ignored):
`report.json`, `telemetry.csv`, `run.log.jsonl`.

## Spec

See [`config/test_specs/basic_mission.yaml`](../config/test_specs/basic_mission.yaml). The catalogues
in `testspec/schema.py` (`ADAPTERS`, `SUPPORTED_FAULTS`, `ACTIONS`, `ASSERTIONS`) are the only things
a spec can name. The parser rejects all of the following before anything runs:

- unknown keys at any level, and duplicate YAML keys (PyYAML would otherwise keep the last one)
- unknown actions, assertion types and adapters
- any fault the adapter does not support (for the mock adapter that is every fault)
- missing required parameters, non-numbers (including booleans), NaN/inf and out-of-range values
- impossible action orders, such as takeoff before arm or disarm while airborne
- malformed requirement ids, ids not in the registry, invalid keep-in polygons, and a spec with no assertions

The parser uses a `SafeLoader`, so Python object tags are not constructed. There is no expression
language, no `eval` and no hook that imports code from YAML.

| Action | Completes when | Bounded by |
| --- | --- | --- |
| `arm` / `disarm` | the adapter reports the new armed state | `timeout_s` |
| `takeoff`, `goto` | within `tolerance_m` of the target and slower than 0.2 m/s | `timeout_s` |
| `hold` | `duration_s` has elapsed (position hold) | its duration |
| `land` | height <= `tolerance_m` and vertical speed <= 0.1 m/s | `timeout_s` |

Timeouts are on the adapter clock. The run's whole wall-clock budget is `timeouts.run_wall_s`, and the
first telemetry sample must arrive within `timeouts.telemetry_s`.

| Assertion | Passes when |
| --- | --- |
| `geofence` | every sample is inside or approaching the local-NED keep-in, classified by `mission_geofence.evaluate_vehicle` |
| `max_altitude` | no sample is above `limit_m` |
| `max_speed` | no sample exceeds `horizontal_mps` and/or `vertical_mps` |
| `final_position` | the last sample is within `tolerance_m` of the point |
| `final_disarmed` | the last sample is disarmed |

## Lifecycle and outcomes

```
CREATED -> VALIDATED -> INITIALIZING -> READY -> RUNNING -> EVALUATING -> PASSED | FAILED
   any non-terminal state -> ERROR | ABORTED
```

- **FAILED**: the target ran and did not meet the spec. A step timed out or was rejected, or an
  assertion failed. A mission that stopped early is FAILED, and its remaining steps are `NOT_RUN`.
- **ERROR**: no valid verdict was possible. Causes are an invalid or unreadable spec, the registry
  being unavailable, an adapter failure, no telemetry, a step timeout while the vehicle link is not
  alive, an unexpected exception, the wall-clock budget being exceeded, or an assertion that could not
  be evaluated.
- **ABORTED**: the run was cancelled. Telemetry recorded up to that point is kept, and no assertion is evaluated.
- **PASSED**: reachable only from EVALUATING, and only when every step completed and every
  assertion was evaluated and passed.

## Report

`report.json` (`kind: test_spec_verification_report`, `schema: 1`) holds the run id and verdict with
its reason, the adapter's `environment` label (`kind: mock`, `px4_sitl: false`, `hil: false`,
`physical_flight: false`, model limits), the spec path, SHA-256 and parsed config, the git commit and
dirty flag, UTC timestamps, the full state history, step results with their telemetry sample ranges,
and assertion results with threshold, observed value and deciding sample. It also holds the telemetry
file's SHA-256 and sample count.

Requirement rows copy the title, level and threshold from the registry. A row's result is computed
only from this run's linked assertions: PASS, FAIL, INCOMPLETE (the linked assertions passed but the
mission did not finish) or NOT_EVALUATED. Every row has `evidence_scope` set to the environment
kind and `changes_registry_verdict: false`. The registry verdict still comes only from
`tools/validation_report.py`. The example links its keep-in assertion to REQ-GEOFENCE-007 and states
in the assertion's `note` that this coverage is partial. The basic mission has no dispensing phase
and no precision-landing marker.

## Roadmap (not implemented)

1. **Vehicle adapters.** Add a `px4_sitl` adapter over `vehicle.mavlink_vehicle.PX4SITLVehicle`, and
   later MAVSDK, reusing `px4_startup.assess` for a fresh-start gate. Its environment label would
   report `px4_sitl: true`. HIL needs hardware the repository does not have.
2. **Telemetry assertions and requirements.** Add assertions that call the existing `vv_harness`
   checks, plus a registry verification kind that reads a test-spec report the way `external` reads a
   results file. The verification would be scoped by environment so mock reports cannot satisfy it.
3. **Fault injection.** Add `SUPPORTED_FAULTS` per adapter that map to `mission_faults.MissionFaultPlan`
   (mission level) and `faults/injectors` (sensor/link) instead of adding a third fault catalogue.
4. **Reports.** Add Markdown rendering next to `validation/report.py` and a campaign roll-up in the style of
   `mission_verification.py`, where a missing report stays in the denominator.
5. **Dashboard.** Add a replay page under `web/replay` that loads `telemetry.csv` and `report.json`.
6. **CI.** Add a `make spec-mock` target in the `validation` job. SITL specs stay on the manual
   `sitl-campaign` workflow.
