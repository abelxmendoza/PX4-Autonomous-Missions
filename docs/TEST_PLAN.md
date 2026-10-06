# Test plan

Scope: the validation platform around a PX4 / ROS 2 offboard autonomy stack -- mission logic, stereo-VO + IMU
fusion, a velocity-PID outer loop, MAVLink communications, fault injection, and the requirement/evidence
tooling. Written to be read next to [`validation_report.md`](CI.md#artifacts) (generated) and
[REQUIREMENTS.md](REQUIREMENTS.md). Where this document says a test "has been run", the command and result are given;
where it has not, that is said.

## 1. System under test

| Part | What it is | Source |
| --- | --- | --- |
| Offboard autonomy | ROS 2 Humble nodes: mission state machine, A* course, reactive avoidance, geofence/geocage, velocity-PID loop | `src/px4_offboard/px4_offboard/` |
| Localization | Stereo VO (OpenCV block matching + LK + PnP) fused with PX4 `SensorCombined` in a 6-state EKF; runs as a side channel, not feeding PX4 | `stereo_depth.py`, `ekf_fusion.py`, `ekf_fusion_node.py` |
| Communications | MAVLink v2 codec, serial/UDP transports, link supervision, I2C/SPI stubs | `comms/` |
| Vehicle abstraction | `VehicleInterface` + Sim / PX4 SITL / PX4 hardware | `vehicle/` |
| Validation tooling | Flight-log verifier, requirement registry, fault injection, run comparison, evidence manifest | `vv_harness.py`, `validation/`, `faults/`, `run_compare.py`, `scripts/verify_evidence.py` |
| Firmware under test (indirect) | PX4 v1.16 SITL, via MAVLink and Micro XRCE-DDS | external |

PX4 firmware itself is not modified or tested here; the platform tests the software that talks to it.

## 2. Test levels

| Level | Purpose | Where | Run by | Executed? |
| --- | --- | --- | --- | --- |
| Unit | Logic in isolation: codec, parser, EKF, PID, schema, injectors | `src/px4_offboard/test/` | `make test-fast` | **Yes** (CI-equivalent run: 402 passed, 5 ROS modules skipped, ROS-free venv; 450 passed with ROS) |
| Component (mock hardware) | Link/vehicle code against mock serial, UDP loopback, pty, mock autopilot, mock I2C/SPI | same | `make test-fast` | **Yes** |
| Replay / evidence | Curated flight recordings produce pinned verdicts | `evidence/`, `scripts/verify_evidence.py` | `make evidence` | **Yes** (9/9) |
| Fault injection | Seeded scenarios against the real EKF and link code | `config/fault_scenarios/` | `make faults` | **Yes** (19 faults: 17 PASS, 2 declared known gaps) |
| Requirement validation | Registry -> Test -> Evidence -> Result, with gate | `requirements/registry.yaml` | `make validate` | **Yes** (17 PASS, 3 FAIL declared known-open, 1 PARTIAL, 1 NOT_RUN) |
| C++ | Codec vs shared golden vectors | `cpp/` | `make cpp-test` | **Yes** (41 checks, 12 vectors) |
| Browser | Replay viewer parsing/scene logic | `web/replay/test/` | `cd web/replay && npm test` | **Yes** (27 passed) |
| ROS integration | Node callbacks with real `px4_msgs` | `test_*_node*.py` | `pytest` with ROS sourced | **Yes** locally; **not yet on a CI runner** |
| SITL (simulated flight) | Full stack in Gazebo + PX4; missions scored by the verifier | `scripts/run_vo_validation.sh`, `scripts/run_full_stack.sh` | manual, workstation | **Yes**, earlier work: a series of live flights (seven are tabulated in [engineering results](engineering_results.md)) |
| SITL interop (MAVLink) | `PX4SITLVehicle` against live PX4 | `tools/sitl_smoke.py` | manual | **Yes**, 1 session |
| HIL | Real flight controller over UART/USB | `PX4HardwareVehicle` | manual | **No. Never run.** |
| Physical I2C/SPI | Real bus and peripheral | `comms/buses.py` | manual | **No. Never run.** |

## 3. Unit testing

Tests are written before the code where practical, and a test that guards a bug is confirmed to fail without the
fix (the frame-parser CRC check, the poll-drain bug, the UDP not-ready-peer bug, the frozen-VO rule were each
shown failing first). Rules:

- deterministic: injected clocks (`FakeClock`), seeded RNGs, no sleeps;
- adversarial inputs are first-class: bad CRC, garbage, truncation, unknown ids, signed frames, 200 random-noise
  chunks, NaN/absurd VO velocities, zero-progress writes, unplug mid-frame;
- cross-checked against a reference where one exists (pymavlink for the codec; the verifier's own definitions for
  `compare_runs`);
- pure-Python tests run without ROS; ROS node tests `importorskip` and are reported as skipped, not passed.

## 4. Integration testing

- **Replay:** `verify_evidence.py` re-runs the real verifier on pinned recordings and compares complete reports
  (hashes, verdicts, skipped checks). Two live stereo flights (K, L) are pinned as *expected failures* with the
  exact failing checks, so improving or regressing them is noticed either way.
- **Fault injection:** see [FAULT_INJECTION.md](FAULT_INJECTION.md).
- **Pipeline:** `test_validation_report.py` evaluates the whole registry (replays flights, runs scenarios) and checks
  the report traces every requirement.

## 5. SITL testing

Full-stack flights (PX4 SITL + Gazebo Harmonic + ROS 2 + stereo cameras) are run by hand on a workstation and
scored by the same verifier CI uses. They are not in CI (GPU/EGL, minutes per run, run-to-run variance). Results
so far, measured against **PX4's own estimate** (not ground truth):

| Flight | Drift (final / peak) | VO availability | Notes |
| --- | --- | --- | --- |
| K | 9.3% / 13.2% | 73% | drift passes, availability fails |
| L (same config as K) | 13.2% / 37.4% | 66% | both fail |

The K/L spread is why single-flight differences are not treated as regressions ([CI.md](CI.md#repeatability)).
`tools/sitl_smoke.py` additionally exercised `PX4SITLVehicle` over MAVLink/UDP against live PX4 (REQ-HIL-003).

## 6. HIL testing

Designed ([HIL_ARCHITECTURE.md](HIL_ARCHITECTURE.md)), **not executed**. What exists: `PX4HardwareVehicle`
(telemetry-only unless explicitly enabled), `PySerialTransport` (exercised over a pseudo-terminal only), a
bring-up procedure, and REQ-HIL-004 / REQ-COMMS-006 registered as `NOT_RUN` / `PARTIAL`. What a hardware run would
add: real UART timing and framing errors, PX4 firmware behaviour on a physical autopilot, sensor noise. None of that
is claimed.

## 7. Regression testing

- Test suites and the evidence manifest gate every push (see [CI.md](CI.md)).
- `tools/compare_runs.py baseline.csv candidate.csv [--baseline-extra ...] [--candidate-extra ...]` compares
  mission completion, drift, estimator error, VO availability, control tracking error, attitude excursion, minimum
  clearance, log rate, VO outage recovery, and CPU (only if the log has a `cpu_pct` column; otherwise `N/A`). With one
  run per side every verdict is tagged `single_run`; with repeated runs a difference inside the spread is
  `INCONCLUSIVE`. CPU/runtime is **not recorded by the current flight logger**, so that metric is unavailable.
- Bugs live in [`bugs/`](../bugs/README.md) with the failing test that now guards them.

## 8. Pass/fail criteria

- A test passes only if it ran and passed; a skipped test is never a pass.
- A requirement passes only if its verifications produced evidence that meets its threshold. Absent evidence is
  `NOT_RUN` or `FAIL`, never `PASS`. Thresholds are not changed to obtain a result.
- Build gate: no failing test; no unexplained requirement failure; no stale `known_open`/`known_gap` marker; no
  missing evidence other than hardware/live-SITL verifications that are explicitly `external`.
- Open items that do not block the build but do appear in every report: REQ-EST-001 (drift), REQ-EST-002
  (VO availability), REQ-EST-006 (frozen IMU).

## 9. Known limitations

1. No hardware: nothing here shows behaviour on a physical flight controller, bus, or sensor.
2. Flight evidence is a handful of SITL flights on one machine with a large run-to-run spread; "drift" is relative
   to PX4's estimate, not ground truth. VO drift/availability requirements are failing.
3. Fault injection is at the measurement level in a rig (no image processing, no live injection into SITL),
   one fault at a time.
4. `MockAutopilot` is not PX4; conformance results show host-side consistency only.
5. No frozen-IMU detector (BUG-020); no CPU/runtime metrics; no timing/latency or soak testing; no fuzzing beyond
   deterministic random-noise chunks.
6. Reactive obstacle avoidance remains unsafe (BUG-008/014/016 family); validated flights use the A* course mode.
7. The ROS 2 CI job has not run on a GitHub runner yet.
8. Only the MAVLink messages in `CRC_EXTRA` are decoded; signing is unsupported.

## 10. Reproducing

`make ci` (see [CI.md](CI.md#reproduce-it-locally)). SITL flights: [operations.md](operations.md). SITL interop:
start PX4 SITL, then `python tools/sitl_smoke.py`.
