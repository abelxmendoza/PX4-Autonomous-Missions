# Claude-work integration validation — 2026-10-16

This review integrates the seeding replay and recovery-site staging, camera callback cleanup,
SITL estimator-start gate, and declarative mock test runner. Both previously unmerged Claude
branches are preserved in the Git ancestry. The untracked EKF2 audit is retained as
[ekf2_start_audit.md](ekf2_start_audit.md).

## Defects found and corrected

| Expected behavior | Observed problem / cause | Fix and regression coverage |
| --- | --- | --- |
| Replay reaches the recorded final mission state | Periodic pose sampling could stop before the terminal transition; nominal playback stayed at precision landing | Record an actual final pose after mission completion, bypassing only the sampling interval; `test_terminal_pose_is_recorded_even_between_periodic_samples` |
| Arm only on fresh health and plausible measured body rates | Missing rate telemetry looked like a quiet estimator; stale observations could persist after restart | Require finite rate and both telemetry streams newer than two seconds; clear old observations on restart; `test_sitl_arm_gate.py` |
| A requested hold lasts at least its duration | Rounding duration × tick rate could produce zero ticks | Wait against the adapter clock; `test_sub_tick_hold_still_waits_for_the_requested_duration` |
| Invalid telemetry cannot produce a passing assertion | A NaN between valid samples could disappear from a maximum calculation | Reject non-finite or non-advancing telemetry at collection; standalone assertions return NOT_EVALUATED for non-finite samples; regression cases cover NaN and infinities |
| Camera cleanup happens even when artifact output fails | An exception while writing report/trace bypassed subscription cleanup | Stop recording and close the camera in `finally`; preserve the existing callback-release fix |
| Validated replay publication is unambiguous | Duplicate scenario IDs silently overwrote earlier inputs; unexpected outcomes could replace the demo | Reject duplicates and unexpected results before writes; `test_seeding_export.py` |
| Common navigation links exist on every page | New seeding replay omitted the search-results link | Restore the link; existing site-link regression |

The original Python failure in `test_search_world` was a literal filename in a comment in the
recorder, not a ground-truth file read. The comment now describes the scoring boundary without
triggering that check. Flight guidance still does not read marker ground-truth coordinates.

## Automated checks

The retained [machine-derived totals](../evidence/validation_20261016/test_results.json) come from
local JUnit reports. `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` avoids incompatible installed ROS pytest hooks.

| Check | Result |
| --- | --- |
| `make ci` fast suite | 856 passed, 4 ROS-dependent module skips |
| `make ci` integration suite | 41 passed; same four module skips during collection |
| Full suite with ROS Humble and installed PX4 messages | 937 passed, no skips (includes the two exporter regressions added after the fast run) |
| `colcon build --packages-select px4_msgs px4_offboard` | Both packages built; installed messages, test-spec package and recorder import successfully |
| CMake Release build with warnings as errors; CTest | Passed; 1 CTest |
| Curated evidence, golden frames, generated world | Passed; expected failing recordings retained |
| Fault-injection and run-comparison checks | Passed their configured gates |
| Requirement registry | Gate OK: 29 PASS, 3 declared known-open FAIL, 1 PARTIAL, 2 NOT_RUN |
| Mock test-spec CLI | PASSED: 8/8 mission steps and 5/5 assertions |

The [requirement report](../evidence/validation_20261016/validation_report.json) retains the three
known-open estimator findings: VO drift (REQ-EST-001), VO availability (REQ-EST-002), and frozen-IMU
detection (REQ-EST-006). Hardware-dependent coverage remains incomplete. A green CI gate does not
turn those requirements into PASS.

## Scope and remaining limitations

- The YAML runner supports only the mock point-mass adapter. Its PASS does not validate PX4 or hardware.
- The arm gate mitigates some EKF2 bad starts; it does not detect every residual attitude error.
  The preserved audit describes four historical bad starts whose body rates were below its threshold.
- Live flights use PX4 SITL, simulated release authorization inputs and camera markers. No physical
  aircraft, operational airspace clearance or real cloud-seeding operation was tested.
- The Ogre 1 server config is a retained diagnostic experiment, not the renderer used for validation.
- The older `evidence/seeding_replay/dev-preview` recordings are unchanged, including the airspace
  and clearance recovery failures. Fresh recordings are separate evidence, not edits to those flights.

## Fresh PX4 SITL flights

The seven replay scenarios were flown again and published as
`evidence/seeding_replay/20261016T135125Z` (also `web/replay/data/seeding/`). Each ended in its
designed state. Touchdown error is horizontal, against the marker the vehicle locked, from PX4's
local position. These are single runs, not a statistical sample. `operational_approval` stays false.

| Scenario | Designed result | Touchdown error |
| --- | --- | --- |
| nominal | completed, recovery landed | 0.153 m |
| weather_denial | aborted_recovered | 0.044 m |
| airspace_denial | aborted_recovered | 0.086 m |
| traffic_conflict | aborted_recovered | 0.125 m |
| clearance_expired | aborted_recovered | 0.083 m |
| altitude_denial | aborted_recovered | 0.131 m |
| marker_loss | failed, `marker_lost_during_descent` | 0.104 m |

Two extra flights are not in that picker. `weather_during_dispense` closed the valve after the
fault and recovered (0.042 m). `geofence_boundary` held inside the tight fence until the wall clock
ended (`simulation_terminated`, fault `setpoint_outside`), which is that scenario's designed outcome.

`airspace_during_dispense` aborted the release as designed (one sample, 0.001 kg, valve closed,
reason `provider_failure`) and then failed recovery. The marker was last accepted at 12.36 s, about
8 m short of marker 9, and the unchanged 5 s timeout fired at 17.41 s during approach
(`marker_stale`, no descent samples). The same 2 m/s approach cap kept the marker on the
weather-during-dispense flight and on the published recoveries. This run is diagnostic evidence
only. It was not exported, and the marker timeout was not raised.
