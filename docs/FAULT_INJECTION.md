# Fault injection

Reusable, declarative fault injection for the validation system. A scenario is a short YAML file; the
framework injects the faults into a deterministic rig, compares each against a fault-free baseline of the
same seed, and writes evidence: **fault injected -> expected behaviour -> observed behaviour -> pass/fail ->
recovery time.**

```yaml
# config/fault_scenarios/vo_dropout_and_packet_loss.yaml
name: vo_dropout_and_packet_loss
duration_s: 60
seed: 1
faults:
  - type: vo_dropout
    start_s: 20
    duration_s: 5
  - type: mavlink_packet_loss
    start_s: 40
    duration_s: 3
    probability: 0.30
```

```bash
python tools/run_fault_scenarios.py config/fault_scenarios/*.yaml --out artifacts/faults   # or: make faults
```

Exit status is non-zero if any fault fails its expectations, except faults declared `known_gap`.

## Schema

Top level: `name`, `description`, `duration_s` (default 60), `seed` (default 1), `faults` (required, non-empty),
`background` (optional). Each fault: `type`, `start_s` (>= 0), `duration_s` (> 0; omitted for one-shot
`estimator_reset`), type parameters, optional `expect` overrides, optional `known_gap`. Unknown keys, unknown
types, missing or out-of-range parameters, and faults that end after the scenario are rejected with a message
naming the fault (`faults/schema.py`, 20 tests).

| Type | Domain | Parameters | What it does |
| --- | --- | --- | --- |
| `vo_dropout` | sensor | - | stereo VO delivers nothing |
| `dropped_messages` | sensor | `sensor` (vo\|imu), `probability` | random message loss |
| `delayed_messages` | sensor | `sensor`, `delay_s` | late delivery, original timestamp kept |
| `timestamp_jitter` | sensor | `sensor`, `std_s` | Gaussian jitter on timestamps (for VO this scales the velocity, as a wrong stamp interval would) |
| `corrupted_measurement` | sensor | `sensor`, `probability`, `magnitude` | random spikes (IMU: accelerometer only) |
| `frozen_sensor` | sensor | `sensor` | keeps repeating the first value in the window, with fresh timestamps |
| `estimator_reset` | sensor | - | EKF re-initialised in flight (position kept, velocity zeroed, covariance reset) |
| `mavlink_packet_loss` | link | `probability` | whole autopilot->host frames lost |
| `comm_disconnect` | link | - | transport disappears, then returns; the supervisor must reconnect |

**`background:`** faults are applied to the baseline *and* every fault run, and are not themselves evidence
records. It exists because 5 Hz VO updates mask almost any IMU fault (see
`sensor_faults.yaml`); `imu_faults_during_vo_outage.yaml` removes VO for the whole window so IMU faults have
somewhere to show up.

**Each fault runs alone** against its own baseline of the same seed, so every observed number is attributable to
exactly one cause. (Combined-fault interaction is not tested.)

## Expectations

Every fault type carries default expectations and a plain-language statement of expected behaviour
(`FAULT_TYPES` in `faults/schema.py`). A scenario may override them with `expect:`; overrides are copied into the
evidence record so they cannot be applied silently.

| Key | Meaning |
| --- | --- |
| `must_detect`, `detect_within_s` | health drops / link is seen down, within N s of fault start |
| `recover_within_s` | time after the fault ends until healthy **and** velocity error < 0.35 m/s (fusion), or connected, alive and fresh (link), sustained 1 s / 0.5 s |
| `max_err_growth_m` | peak horizontal error above the baseline's peak |
| `must_stay_alive`, `min_rx_rate_ratio` | link never declared stale; frames received in the window / baseline frames |

**Recovery time** is measured from the end of the fault window (from the start for the one-shot reset). `0 s`
means the system never left the "recovered" condition, which is the right answer for faults the estimator
absorbs; the evidence also lists how many messages were affected (`injected`) so "no effect" can be told apart
from "fault never fired".

## Evidence

Per fault, in `artifacts/faults/<scenario>.json` (plus `.md` and a time-series CSV per fault):

```json
{ "id": "vo_dropout_and_packet_loss/1-vo_dropout", "domain": "sensor",
  "fault": {"type": "vo_dropout", "start_s": 20, "duration_s": 5, "params": {}},
  "injected": {"in_window": 25, "dropped": 25},
  "expected": {"expected_text": "...", "criteria": {"must_detect": true, "detect_within_s": 1.0, ...}, "overrides": {}},
  "observed": {"detected": true, "detection_time_s": 0.4, "recovery_time_s": 0.0, "err_growth_m": 0.118, ...},
  "checks": [{"name": "must_detect", "expected": true, "observed": true, "passed": true}, ...],
  "passed": true, "status": "PASS", "known_gap": null, "recovery_time_s": 0.0 }
```

Statuses: `PASS`, `FAIL` (fails the gate), `KNOWN_GAP` (fails but declared; does not fail the gate and is never
reported as passing), `PASS_GAP_OBSOLETE` (declared gap that now passes; fails the gate until the marker is
removed). `test_the_only_known_gaps_are_the_documented_ones` pins the exact set.

## The rigs

**Fusion rig** (`faults/fusion_rig.py`): a synthetic 60 s flight (speed 2 m/s +- 1.25, heading swinging +-46
degrees) produces IMU samples at 100 Hz with bias and noise and VO velocity at 5 Hz. They pass through the
injector into the **real** `PoseVelocityEKF` with the same acceptance rules and `fusion_healthy()` the ROS node
uses. Truth is exact, unlike the SITL flights where only PX4's own estimate exists. *Not modelled:* image
processing (VO failures are injected at the measurement, never produced by feature tracking), attitude error in
the camera-to-body conversion.

**Link rig** (`faults/link_rig.py`): the real `PX4SITLVehicle`/`MavlinkLink` against `MockAutopilot` on a fake
clock, losing whole frames or pulling the transport. Host-side behaviour only.

Both are fully seeded: re-running a scenario reproduces the evidence exactly (tested).

## Results (seed 1, current commit)

Generated by `make faults`; thresholds are the type defaults unless a scenario file says otherwise.

| Scenario / fault | Detect | Recovery | Error growth vs baseline | Status |
| --- | --- | --- | --- | --- |
| `imu_faults_during_vo_outage/1-dropped_messages` dropped_messages sensor=imu, probability=0.3 @20+10 s | 0 s | 10 s | +0.62 m | **PASS** |
| `imu_faults_during_vo_outage/2-timestamp_jitter` timestamp_jitter sensor=imu, std_s=0.002 @20+10 s | 0 s | 10 s | -0.14 m | **PASS** |
| `imu_faults_during_vo_outage/3-corrupted_measurement` corrupted_measurement sensor=imu, probability=0.1, magnitude=5.0 @20+10 s | 0 s | 10 s | +0.44 m | **PASS** |
| `imu_faults_during_vo_outage/4-frozen_sensor` frozen_sensor sensor=imu @20+5 s | 0 s | 15 s | +32.09 m | **KNOWN_GAP (BUG-020)** |
| `link_faults/1-mavlink_packet_loss` mavlink_packet_loss probability=0.3 @20+5 s | - | 0 s | n/a (link) | **PASS** |
| `link_faults/2-mavlink_packet_loss` mavlink_packet_loss probability=1.0 @30+6 s | 3 s | 0.05 s | n/a (link) | **PASS** |
| `link_faults/3-comm_disconnect` comm_disconnect  @42+5 s | 0.05 s | 0.25 s | n/a (link) | **PASS** |
| `sensor_faults/1-dropped_messages` dropped_messages sensor=vo, probability=0.5 @20+10 s | 1 s | 0 s | +0.00 m | **PASS** |
| `sensor_faults/2-dropped_messages` dropped_messages sensor=imu, probability=0.3 @20+10 s | - | 0 s | -0.00 m | **PASS** |
| `sensor_faults/3-delayed_messages` delayed_messages sensor=vo, delay_s=0.4 @20+10 s | - | 0 s | +0.74 m | **PASS** |
| `sensor_faults/4-timestamp_jitter` timestamp_jitter sensor=imu, std_s=0.002 @20+10 s | - | 0 s | +0.00 m | **PASS** |
| `sensor_faults/5-timestamp_jitter` timestamp_jitter sensor=vo, std_s=0.02 @20+10 s | - | 0 s | -0.21 m | **PASS** |
| `sensor_faults/6-corrupted_measurement` corrupted_measurement sensor=vo, probability=0.3, magnitude=20.0 @20+10 s | 3.6 s | 0 s | +0.05 m | **PASS** |
| `sensor_faults/7-corrupted_measurement` corrupted_measurement sensor=imu, probability=0.1, magnitude=5.0 @20+10 s | - | 0 s | -0.02 m | **PASS** |
| `sensor_faults/8-frozen_sensor` frozen_sensor sensor=vo @20+5 s | 0.6 s | 0 s | +0.08 m | **PASS** |
| `sensor_faults/9-frozen_sensor` frozen_sensor sensor=imu @20+5 s | - | 0 s | +0.00 m | **KNOWN_GAP (BUG-020)** |
| `sensor_faults/10-estimator_reset` estimator_reset  @20+0 s | - | 0 s | +0.02 m | **PASS** |
| `vo_dropout_and_packet_loss/1-vo_dropout` vo_dropout  @20+5 s | 0.4 s | 0 s | +0.12 m | **PASS** |
| `vo_dropout_and_packet_loss/2-mavlink_packet_loss` mavlink_packet_loss probability=0.3 @40+3 s | - | 0.05 s | n/a (link) | **PASS** |

### What these results show -- and do not

- **Found and fixed (BUG-019):** a frozen VO stream was accepted as fresh. Before the fix, same seed: detected after
  5.4 s, +11.04 m error, health green for the first 5 s. After: detected in 0.6 s, +0.08 m.
- **Open (BUG-020):** a frozen IMU is not detected. With VO healthy it does no measurable harm (+0.003 m); with
  VO out, +32 m. Declared `known_gap`; not fixed because the false-positive rate of an exact-repeat rule cannot be
  measured without an IMU recording (the VO rule could, from five recorded flights).
- Many sensor faults show `0 s` recovery and near-zero error growth. That is real for this rig (5 Hz velocity updates
  re-anchor the filter and the accelerations are modest), not proof that the live system is robust to them.
- Passing here means "behaves as specified in a seeded model", not "survives that fault in flight". No fault was
  injected into a live SITL flight in this work.

## Adding a fault type

1. Add the type to `FAULT_TYPES` (domain, parameters, expected text, default expectations) with a schema test.
2. Implement it in `SensorFaultInjector` (and test it in isolation) or in the link rig.
3. Add it to a scenario file; run `make faults`; read the evidence before trusting the pass.
4. If it exposes a defect, write the failing test, fix, and add a `bugs/` entry with the before/after numbers.
