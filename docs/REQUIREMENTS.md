# Requirements catalog

Scope: existing software-in-the-loop behavior. These 26 stable catalog IDs are acceptance statements for specific configurations, not airworthiness claims. The 14 existing single-vehicle verifier IDs remain unchanged in code and saved logs; this table maps them to catalog IDs rather than breaking historical reports. Seven additional entries name existing cooperative behavior, and five more cover stereo/IMU localization, the outer-loop velocity controller, and loss-of-control / obstacle-contact detection.

**MUST** is mandatory within the stated scope. **SHOULD** retains the existing harness severity. A scenario can pass applicable checks while other requirements are unexercised. See the [verification matrix](verification_matrix.md) for evidence and gaps.

## Single-vehicle behavior

| Catalog ID | Level | Existing verifier ID | Requirement and acceptance boundary |
| --- | --- | --- | --- |
| REQ-TEL-001 | MUST | REQ-STATE-01 | Recorded mission state transitions shall be reconstructable and legal according to `LEGAL_TRANSITIONS`. Empty traces fail. Partial traces do not establish preflight or full-flight completion. |
| REQ-FLT-001 | MUST | REQ-ALT-01 | With geofence enabled, recorded altitude outside FAILSAFE shall not exceed configured maximum plus the verifier's 0.25 m tolerance. Default maximum: 12 m. This check excludes FAILSAFE flight. |
| REQ-FLT-002 | MUST | REQ-WP-01 | Recorded waypoint index shall never decrease. Monotonicity alone does not prove all waypoints were visited. |
| REQ-FLT-003 | MUST | REQ-TERM-01 | After LANDING or FAILSAFE, the mission shall not return to MOVE. A truncated log with no terminal state is not proof of landing. |
| REQ-GEO-001 | MUST | REQ-GEOCAGE-01 | Setpoints marked `caged` shall remain inside the configured fence inset. The verifier default inset is 1 m; current launch configuration uses 3 m. Replay must use the recording's configuration when known. |
| REQ-GEO-002 | MUST | REQ-GEOFENCE-01 | An airborne, geofence-enabled breach shall be followed by FAILSAFE or return inside within 1.5 s. Verification uses recorded `inside` flags; this is a response requirement, not unconditional containment. |
| REQ-OBS-001 | MUST | REQ-AVOID-SENSOR-01 | Direct front/left/right avoidance classifications shall have fresh sector evidence; sensor-only MOVE shall not use stale LiDAR. The -1 range sentinel means confirmed clear. Legacy logs without these fields are skipped. |
| REQ-OBS-002 | MUST | REQ-CLEARANCE-01 | Available mapped-clearance samples shall be positive. This checks sampled vehicle position against the configured map, not a swept airframe collision envelope. Missing legacy fields are skipped. |
| REQ-EXE-001 | SHOULD | REQ-EXEC-01 | An executive ABORT shall reach FAILSAFE or LANDING within 2 s. Simulated resource degradation and optional-work skipping are also tested in `test_mission_executive.py`; ABORT is the recorded-flight check. |
| REQ-GPS-001 | SHOULD | REQ-GPS-ZONE-01 | Logged denied-zone membership shall match the configured NED prism when localization columns exist. |
| REQ-GPS-002 | MUST | REQ-GPS-INJECT-01 | Logical injected denial shall not simultaneously report healthy GPS as the selected localization source. |
| REQ-GPS-003 | MUST | REQ-GPS-POLICY-01 | A recorded LOC_FAILSAFE event under the supported hold/land policy shall reach FAILSAFE or LANDING within 2 s. Healthy external odometry can permit continuation; this does not guarantee mission completion. |
| REQ-GPS-004 | MUST | REQ-GPS-ACTUAL-01 | During requested GPS-aiding loss, logged GPS navigation health shall be false after a 1.5 s settling interval. In the current bridge this reflects GNSS fusion flags during injection, not physical loss of the GPS sensor. |
| REQ-GPS-005 | MUST | REQ-VIO-FUSION-01 | After settling, the simulated external-odometry stream shall be healthy and PX4 external-position fusion active during GPS-aiding loss. The legacy ID says VIO; the implemented source is Gazebo pose plus modeled noise/drift. |

## Cooperative behavior

These requirements apply to the fixed two-vehicle gate/landing-point scenario in `SurveyCoordinator`. Legacy names `swarm_survey`, `SURVEY`, and the verifier error “survey transects” remain API compatibility names. Current tasks have identical start/end positions; they are point visits, not area or imagery coverage.

| Catalog ID | Level | Requirement and acceptance boundary |
| --- | --- | --- |
| REQ-SWM-001 | MUST | Both vehicles shall maintain the header's minimum sampled horizontal separation (default 2.5 m), including a retired vehicle. Coordinator reservations use 3.5 m and predict closest approach over 1 s. Sampled checks are not continuous collision proofs. |
| REQ-SWM-002 | MUST | Unfinished work shall be reassigned only after fresh LANDED, landed=true, armed=false evidence for a faulted vehicle persists for at least 1 s. Any interruption resets the confirmation interval. |
| REQ-SWM-003 | MUST | Every completed task shall have an independent armed-position observation within 0.6 m of its endpoint. Nonzero transects additionally require start/end and corridor evidence. The coordinator's terminal acceptance is 0.5 m, distinct from its 1 m transit-corner acceptance. |
| REQ-SWM-004 | MUST | Overall COMPLETE shall require all expected tasks completed and both vehicles observed LANDED and disarmed. This does not certify touchdown accuracy on a visual landing pad. |
| REQ-SWM-005 | MUST | Active cooperative evidence shall include both vehicles, valid telemetry age at most 0.75 s, and strictly increasing sample times with gaps no greater than 0.5 s. Stale telemetry shall abort coordination without releasing assignments. |
| REQ-SWM-006 | MUST | A vehicle shall hold on coordinator-command loss after 0.75 s and land after 2 s; stale/replayed commands shall not renew its watchdog. |
| REQ-TEL-002 | MUST | Browser export shall retain the exact final recorded cooperative sample, even between 10 Hz playback ticks, and compute the verdict from that input rather than trusting a separate report file. |

## Perception, control and safety additions

Added with the stereo-camera + IMU fusion estimator and the outer-loop velocity PID (`control_mode:=velocity_pid`). The fusion estimate is **measured against PX4's own estimate**, which is itself an estimate: "drift" below means disagreement with PX4's navigation solution, not error against ground truth. The estimate does not feed PX4 in these runs (`publish_to_px4` stays false); it is only scored, except when `position_source:=fusion`, where the velocity PID deliberately closes on it.

| Catalog ID | Level | Verifier ID | Requirement and acceptance boundary |
| --- | --- | --- | --- |
| REQ-VO-001 | MUST | REQ-VO-DRIFT-01 | While the fusion estimate is healthy, its horizontal position shall end the flight within 10% of distance travelled of PX4's estimate and never exceed 20% once 20 m have been flown. Skipped (not passed) when under 20 m of healthy path. Distance is PX4's own path length. |
| REQ-VO-002 | MUST | REQ-VO-AVAIL-01 | When the fusion node is running, the estimate shall be healthy in at least 80% of MOVE samples. "Healthy" means an accepted VO velocity update within the last 0.5 s, so rejected or failed frames count against availability. A run with dark cameras fails with 0%. |
| REQ-CTL-001 | MUST | REQ-CTRL-01 | In velocity_pid mode the commanded velocity shall be finite, at most 3.0 m/s horizontal and 1.5 m/s vertical, and only issued in MOVE. The single tick that hands MOVE over to LANDING/FAILSAFE is exempt because the row is logged after the state changes. |
| REQ-ATT-001 | MUST | REQ-ATT-01 | Roll and pitch shall stay within 60 degrees in TAKEOFF, HOVER and MOVE. Beyond that the vehicle is tumbling, which no requirement previously covered; geofence breaches and altitude spikes were only its downstream symptoms (see BUG-016). |
| REQ-OBS-003 | MUST | REQ-CLEARANCE-02 | Mapped clearance (vehicle centre to obstacle surface) shall stay at or above 0.35 m (the x500's half-diagonal plus margin). REQ-OBS-002 only forbids being inside the obstacle, so a 1 cm graze passed it. Sampled position against the configured map, not a swept airframe envelope. |

## Interpretation and known verification gaps

- `run_vv().passed` means no non-skipped MUST failures. SHOULD failures do not fail the overall result. Several checks pass vacuously when no triggering event exists. The report's details and skips are part of the result.
- The single-vehicle CSV does not establish arming/disarming or full-flight completeness. Some recordings start at MOVE, and some have substantial gaps. Do not advertise end-to-end success based only on their overall PASS.
- Geofence and altitude exceptions mean a FAILSAFE trajectory outside the boundary can still pass applicable response checks. The matrix must not relabel that as containment.
- REQ-SWM-002 and REQ-SWM-006 have direct automated tests; `swarm_verify.verify` does not independently check reassignment dwell or command watchdog response. Golden-run timing evidence supports the former, but is not an additional automated verifier.
- The cooperative verifier constructs task geometry from the current coordinator and checks names against the header. Historical four-lane recordings cannot be verified with the current gate/landing specification. Curated current recordings match that geometry; schema-versioned scenario metadata remains a limitation.
- No hardware-in-the-loop, real aircraft, camera localization, or visual SLAM validation is claimed.


## Requirement registry (structured, machine-evaluated)

The catalog above predates the registry and is kept for traceability to historical
reports. New work is registered in [`requirements/registry.yaml`](../requirements/registry.yaml),
which is the source of truth for **what is claimed, its threshold, how it is measured, and
which tests and evidence back it**. Each entry has: `id`, `title`, `level`, `description`,
`rationale`, `threshold`, `measurement`, `verification` (one or more of `flight`, `fault`,
`pytest`, `external`), `tests`, optional `legacy_ids` and `known_open`.

`python tools/validation_report.py` evaluates every entry and writes
`artifacts/validation/validation_report.{json,md}` with **Requirement -> Test -> Evidence ->
Result** for each ID. See [CI.md](CI.md) for how the CI gate uses it.

Rules the evaluator enforces, not conventions:

- **No evidence, no PASS.** A listed test that did not run (or was skipped) fails its requirement;
  an `external` verification with no results file is `NOT_RUN`, never a pass.
- **Statuses:** `PASS`, `FAIL`, `PARTIAL` (everything runnable passed, an `external` part was not run),
  `NOT_RUN`.
- **`known_open`** declares a requirement that is currently *not met* and where that is tracked.
  The gate tolerates exactly that, and fails if the marker is stale (the requirement now passes).
- Thresholds are not changed to obtain a pass. Where a measured result misses its threshold
  (REQ-EST-001/002) it is reported as FAIL.

| ID | Level | Requirement | Threshold | Verified by | Legacy IDs | Evidence status |
| --- | --- | --- | --- | --- | --- | --- |
| REQ-EST-001 | MUST | Fusion position drift is bounded relative to distance flown | final drift <= 10% of path; peak drift <= 20% (>= 20 m flown) | flight | REQ-VO-DRIFT-01, REQ-VO-001 | known open |
| REQ-EST-002 | MUST | Visual odometry is available while moving | healthy in >= 80% of MOVE samples (healthy = accepted VO update within 0.5 s and fresh IMU) | flight | REQ-VO-AVAIL-01, REQ-VO-002 | known open |
| REQ-EST-003 | MUST | A VO outage is detected and the dead-reckoned error stays bounded | detect <= 1.0 s; error growth <= 2.0 m over the fault-free baseline (5 s outage) | fault | - | evidenced |
| REQ-EST-004 | MUST | A frozen VO stream is rejected rather than trusted | detect <= 1.0 s; error growth <= 2.0 m; recovery <= 2.0 s | fault, pytest | - | evidenced |
| REQ-EST-005 | MUST | Corrupted VO measurements are gated | error growth <= 2.0 m; recovery <= 2.0 s | fault, pytest | - | evidenced |
| REQ-EST-006 | SHOULD | A frozen IMU is detected | detect <= 1.0 s; error growth <= 1.0 m | fault | - | known open |
| REQ-CTRL-001 | MUST | The velocity PID loop stays inside its envelope | \|v_xy\| <= 3.0 m/s, \|v_z\| <= 1.5 m/s, finite, every velocity_pid sample | flight, pytest | REQ-CTRL-01, REQ-CTL-001 | evidenced |
| REQ-CTRL-002 | MUST | No loss of attitude control while airborne | max(\|roll\|, \|pitch\|) <= 60 deg | flight | REQ-ATT-01, REQ-ATT-001 | evidenced |
| REQ-SAFE-001 | MUST | The airframe keeps clear of mapped obstacles | min mapped clearance > 0 and >= 0.35 m | flight | REQ-CLEARANCE-01, REQ-CLEARANCE-02, REQ-OBS-002, REQ-OBS-003 | evidenced |
| REQ-COMMS-001 | MUST | The MAVLink v2 codec is byte-identical to the reference implementation | 0 byte differences for HEARTBEAT, ATTITUDE, LOCAL_POSITION_NED, COMMAND_LONG, SET_POSITION_TARGET_LOCAL_NED; crc_extra table equal | pytest | - | evidenced |
| REQ-COMMS-002 | MUST | Malformed input never produces a false frame and never stops the stream | no exceptions; every malformed case counted; following good frame parsed; 200 random-noise chunks yield no frame | pytest | - | evidenced |
| REQ-COMMS-003 | MUST | Link loss is detected and the link is re-established with bounded backoff | detect <= 1.0 s; recover <= 4.0 s after the device returns; at most max_attempts opens per reconnect | fault, pytest | - | evidenced |
| REQ-COMMS-004 | MUST | Partial frame loss does not take the link down | link alive throughout; rx ratio >= 0.40 of baseline | fault | - | evidenced |
| REQ-COMMS-005 | MUST | Timeouts, short writes and burst reads are handled | no read hang; zero-progress write raises LinkDown; 30 queued frames drained by one poll | pytest | - | evidenced |
| REQ-COMMS-006 | SHOULD | I2C and SPI driver logic handles NACK, hang and register conventions | all mock-bus tests pass | external, pytest | - | needs hardware/SITL for part |
| REQ-RECOVERY-001 | MUST | Fusion recovers promptly after a VO outage | recovery <= 2.0 s | fault | - | evidenced |
| REQ-RECOVERY-002 | MUST | Telemetry resumes promptly after a link disconnect | recovery <= 4.0 s | fault | - | evidenced |
| REQ-RECOVERY-003 | MUST | The estimator re-converges after a mid-flight reset | recovery <= 3.0 s; error growth <= 1.5 m | fault | - | evidenced |
| REQ-HIL-001 | MUST | One behavioural contract holds for simulated, SITL and hardware vehicle classes | every conformance test passes for all three classes | pytest | - | evidenced |
| REQ-HIL-002 | MUST | A hardware vehicle cannot actuate unless explicitly enabled | 0 command bytes on the wire without allow_actuation=True | pytest | - | evidenced |
| REQ-HIL-003 | SHOULD | PX4SITLVehicle interoperates with a running PX4 SITL over MAVLink/UDP | telemetry received; arm and disarm acknowledged | external | - | needs hardware/SITL for part |
| REQ-HIL-004 | SHOULD | PX4HardwareVehicle interoperates with a real PX4 flight controller over UART/USB | telemetry received over serial from a real flight controller | external | - | needs hardware/SITL for part |

Evidence status describes what the registry can currently show, not what is true of an
aircraft: "evidenced" means a test, curated recording or fault scenario produced the result;
"needs hardware/SITL for part" means a real-flight-controller or running-SITL verification is
declared and has not been run; "known open" means the requirement is currently failing.
