# Requirements catalog

Scope: existing software-in-the-loop behavior. These 21 stable catalog IDs are acceptance statements for specific configurations, not airworthiness claims. The 14 existing single-vehicle verifier IDs remain unchanged in code and saved logs; this table maps them to catalog IDs rather than breaking historical reports. Seven additional entries name existing cooperative behavior.

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

## Interpretation and known verification gaps

- `run_vv().passed` means no non-skipped MUST failures. SHOULD failures do not fail the overall result. Several checks pass vacuously when no triggering event exists. The report's details and skips are part of the result.
- The single-vehicle CSV does not establish arming/disarming or full-flight completeness. Some recordings start at MOVE, and some have substantial gaps. Do not advertise end-to-end success based only on their overall PASS.
- Geofence and altitude exceptions mean a FAILSAFE trajectory outside the boundary can still pass applicable response checks. The matrix must not relabel that as containment.
- REQ-SWM-002 and REQ-SWM-006 have direct automated tests; `swarm_verify.verify` does not independently check reassignment dwell or command watchdog response. Golden-run timing evidence supports the former, but is not an additional automated verifier.
- The cooperative verifier constructs task geometry from the current coordinator and checks names against the header. Historical four-lane recordings cannot be verified with the current gate/landing specification. Curated current recordings match that geometry; schema-versioned scenario metadata remains a limitation.
- No hardware-in-the-loop, real aircraft, camera localization, or visual SLAM validation is claimed.
