# Bug Database

Real defects found, diagnosed, and fixed during development of this
project, each with symptom, root cause, fix, and (where one exists) the
regression test that now guards against it. Written retroactively from
git history and live SITL verification logs — nothing here is
hypothetical.

| ID | Summary | Found via | Regression test |
|----|---------|-----------|------------------|
| [BUG-001](BUG-001.md) | Missing barometer/magnetometer world plugins block arming | Live SITL preflight | None (SDF config) |
| [BUG-002](BUG-002.md) | Duplicate sensor plugin registration silently breaks the same sensors | Live SITL | None (SDF/launch config) |
| [BUG-003](BUG-003.md) | LiDAR sector label thrashes near obstacles, no debounce | Live SITL | None (open gap) |
| [BUG-004](BUG-004.md) | Sidestep avoidance crashes once every mapped obstacle is bypassed | Live SITL | None (open gap) |
| [BUG-005](BUG-005.md) | Lateral avoidance escapes toward a side without checking it's clear | Live SITL | None (open gap) |
| [BUG-006](BUG-006.md) | V&V harness false-positives on a safe "-1.0 = clear" LiDAR reading | Real flight-log replay | `test_sensor_backed_avoidance_passes_with_confirmed_clear_sector` |
| [BUG-007](BUG-007.md) | AABB hit-test false-positives on a path running parallel and clear | New unit test | `test_segment_parallel_and_clear_of_obstacle_does_not_hit` |
| [BUG-008](BUG-008.md) | Boxed-in avoidance logs a blocked escape side, then drives into it anyway | Live SITL (real collision) | None (open gap) |
| [BUG-009](BUG-009.md) | Pattern-based process cleanup kills another session's concurrent simulation | Live collateral damage | None (shell infra) |
| [BUG-010](BUG-010.md) | GPS-denied localization failsafe event never reaches the CSV log | Code review + live test | `test_gps_denied_failsafe_event_is_captured_in_the_log` |
| [BUG-011](BUG-011.md) | Headless Gazebo silently renders on the wrong GPU | New camera bring-up | None (host config) |
| [BUG-012](BUG-012.md) | Climb-avoidance decision recomputed every tick causes oscillation | Live SITL, reported by pilot | `test_climb_avoidance_target_is_locked_not_recomputed_each_tick` |
| [BUG-013](BUG-013.md) | Geocage margin too tight for real avoidance overshoot, geofence breach | Live SITL | None (config tuning) |
| [BUG-014](BUG-014.md) | Sensor-only bypass rigidly continues into an unanticipated second obstacle | Live SITL (near-collision) | `test_sensor_only_bypass_replans_on_emergency_close_reading` |
| [BUG-015](BUG-015.md) | The BUG-014 fix itself thrashes on ordinary bypass proximity | Live SITL re-verification | `test_sensor_only_bypass_does_not_thrash_on_expected_proximity` |
| [BUG-016](BUG-016.md) | **OPEN** — altitude spike + geofence breach at GPS-zone exit | Live SITL re-verification | Not yet fixed |

## Reading this list

A few things worth noticing across these entries:

- **BUG-014 → BUG-015** is a fix that introduced a new bug, caught by
  re-running the same live verification step used to check the original
  fix, before either was ever reported as done. Neither was committed
  until both were resolved and both had regression tests that
  demonstrably fail without their fix and pass with it restored.
- **BUG-006** is a bug in the verification harness itself (`vv_harness.py`),
  not the flight code — found by running the checker against a real
  flight log instead of only synthetic test fixtures, which is why CI's
  `vv-regression` job now replays real committed logs on every push.
- **BUG-009** is a bug in the *test infrastructure*, not the product —
  included because it was a real, live-confirmed failure with a genuine
  root cause and fix, even though nothing here is unit-testable.
- Several entries (BUG-003, 004, 005, 008, 011, 013) have **no automated
  regression test** — they were verified live via repeated SITL runs at
  the time, not encoded into the pytest suite. Left as an honest gap
  rather than backfilled with tests written after the fact to look more
  complete than the historical record supports.
- **BUG-016 is still open.** It's included here, unresolved, because a
  bug database that only ever shows fixed bugs isn't telling the whole
  story.
