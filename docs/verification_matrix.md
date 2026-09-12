# Verification matrix

Reproduce evidence results: `python3 scripts/verify_evidence.py`. It validates SHA-256, runs the real verifiers, checks expected outcomes (including the preserved failure), and compares complete JSON snapshots. No ROS or simulator is needed. [Manifest](../evidence/manifest.json) records origins; [catalog](requirements.md) defines acceptance scope.

Paths in implementation/function columns are relative to `src/px4_offboard/px4_offboard/`; test names below are in `src/px4_offboard/test/`. PASS is scoped to listed evidence. **UNIT PASS** is not flight evidence; **NOT EXERCISED** is not a pass claim for the physical behavior.

| Requirement | Implementation | Scenario / test | Evidence | Verification function | Status |
| --- | --- | --- | --- | --- | --- |
| REQ-TEL-001 | `mission_state.py`, `offboard_mission.py` | nominal excerpt; illegal-transition test | [nominal report](../evidence/nominal/report.json) | `vv_harness.check_legal_state_sequence` | PASS for logged MOVE→LANDING; early phases absent |
| REQ-FLT-001 | `offboard_mission.py`, `mission_logic.Fence` | nominal; GPS aiding loss | [GPS report](../evidence/gps_denied/report.json) | `check_altitude_limit` | PASS within check scope; GPS recording max 8.11 m |
| REQ-FLT-002 | `offboard_mission._waypoint_target` | nominal; decreasing-index negative test | nominal report | `check_wp_monotonic` | PASS; no full-coverage claim |
| REQ-FLT-003 | `mission_state.py` | nominal; terminal-state tests | nominal report | `check_terminal_finality` | PASS; does not prove disarm |
| REQ-GEO-001 | `offboard_mission._apply_geocage` | `test_caged_setpoint_outside_fails_vv` | Python test result | `check_geocage_setpoints` | UNIT PASS; curated flight trigger may be absent |
| REQ-GEO-002 | `offboard_mission._check_failsafes` | `test_geofence_breach_without_failsafe_fails`, `test_geofence_breach_then_failsafe_passes` | Python test result | `check_geofence_response` | UNIT PASS for fault response; curated good flights have no breach |
| REQ-OBS-001 | `lidar_sectors.py`, `offboard_mission._detect_obstacle` | fresh/stale/clear-sentinel tests | `test_flight_vv.py`; [avoidance report](../evidence/obstacle_avoidance/report.json) | `check_sensor_backed_avoidance` | UNIT PASS; legacy avoidance recording SKIP, not LiDAR proof |
| REQ-OBS-002 | `mission_logic.py`, `offboard_mission.py` | GPS course; zero-clearance negative test | GPS report | `check_obstacle_clearance` | PASS; minimum recorded mapped clearance 1.982 m |
| REQ-EXE-001 | `mission_executive.py`, `offboard_mission._apply_executive_decision` | `test_executive_abort_should_reach_terminal`; degradation tests | Python test result | `check_executive_abort` | UNIT PASS; ABORT not exercised in curated good flight |
| REQ-GPS-001 | `localization_logic.py` | GPS aiding loss; zone tests | GPS report | `check_gps_denied_zone` | PASS with current default prism |
| REQ-GPS-002 | `localization_logic.py` | logical-denial negative test | `test_gps_inject_rejects_healthy_gps_claim` | `check_gps_inject_source` | UNIT PASS; separate from actual EKF aiding-loss injection |
| REQ-GPS-003 | `localization_logic.py`, `offboard_mission._update_localization` | policy negative test; healthy-source continuation test | `test_flight_vv.py`, `test_localization_logic.py` | `check_gps_policy_response` | UNIT PASS; no LOC_FAILSAFE event in curated GPS recording |
| REQ-GPS-004 | `offboard_mission._set_gps_failure`, `_raw_gps_healthy` | GPS aiding loss | GPS report | `check_actual_gps_failure` | PASS; 91 settled samples, health proxy rather than physical sensor outage |
| REQ-GPS-005 | `vio_bridge.py`, `vio_noise.py` | simulated external odometry | GPS report | `check_vio_fusion` | PASS; 91 settled samples with external position fused |
| REQ-SWM-001 | `swarm_logic.SurveyCoordinator._step` | fresh SITL recovery | [golden report](../evidence/swarm/golden_report.json) | `swarm_verify.verify` pairwise separation | PASS; minimum 8.095491 m across 641 synchronized rows |
| REQ-SWM-002 | `SurveyCoordinator._step` retirement gate | dropout; airborne-fault test; interrupted-dwell test | golden raw log; `test_swarm_logic.py` | coordinator regression tests; golden timing inspection | UNIT PASS + observed 1.1 s landed/disarmed interval before release; standalone verifier gap |
| REQ-SWM-003 | `SurveyCoordinator._step` endpoint acceptance | original two recovery tests; near-miss regression | [before](../evidence/failure_case/before_report.json), [after](../evidence/failure_case/after_report.json), golden report | `swarm_verify.verify` independent task observations | EXPECTED FAIL before; PASS after in kinematic and fresh SITL evidence |
| REQ-SWM-004 | `SurveyCoordinator._step`, `swarm_vehicle.py` | SITL nominal and recovery | [nominal swarm](../evidence/swarm/nominal_report.json), golden report | `swarm_verify.verify` final coverage/landing checks | PASS; point visits and confirmed landing, not payload delivery |
| REQ-SWM-005 | `SurveyCoordinator.update`, `_step`, `swarm_coordinator._tick` | stale telemetry, replay, missing/truncated evidence tests | Python tests; golden report | `swarm_verify.verify` timestamps/vehicle freshness | PASS for golden; negative cases rejected |
| REQ-SWM-006 | `swarm_vehicle._command`, `_tick` | `test_coordinator_timeout_holds_then_lands`, bad/replayed-command tests | ROS test result | `test_swarm_nodes.py` | ROS UNIT PASS; golden uses commanded dropout, not communications loss |
| REQ-TEL-002 | `scripts/export_swarm_replay.py` | off-grid terminal sample, stale report sidecar | `test_swarm_export.py`; browser replay tests | `export`; `parseSwarmReplay` | PASS; exact final sample retained and verdict recomputed |

## Evidence levels

1. **Logic tests:** deterministic coordination/geometry/resource scenarios. Kinematic recovery evidence uses the real coordinator but no aircraft dynamics.
2. **ROS adapter tests:** real ROS/PX4 message types and node callbacks; do not establish an actual flight.
3. **Recorded SITL:** PX4/Gazebo telemetry, preserved unedited (gzip only), rechecked offline. The golden recovery is a fresh run after the fix.
4. **Browser tests:** CSV/coordinate parsing, scene export, synchronized tracks and final frame. Browser playback is a visualization of evidence, not a second physics simulation.

[Engineering results](engineering_results.md) contains measured counts and command outcomes. There is no hardware/HIL evidence tier in this repository.
