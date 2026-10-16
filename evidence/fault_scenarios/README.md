# Fault scenarios: integrated seeding sortie in PX4 SITL

Each folder is one campaign flown on 2026-10-13 with `scripts/run_integrated_seeding_sitl.sh`
(headless PX4 SITL v1.16 + Gazebo Harmonic, `search_field`, `x500_search_cam`). Each run
starts its own PX4. A run folder holds `sitl_report.json`, `startup.json` and `exit_code.txt`
(0 = the mission result matched the scenario's designed outcome in `EXPECTED_RESULT`;
2 = it did not). Ground truth (`worlds/search_field_targets.json`) is read only after
landing, to score the touchdown.

These are single runs of each scenario, not a statistical sample.

| Campaign | Code | What it shows |
| --- | --- | --- |
| `20261013T174711Z` | before the fixes | Release denials and recovery for 4 of 6 faults. `altitude_denial` and `marker_loss` both reported `marker_lost_during_descent` and a 12 m "error" against marker 9. |
| `20261013T175610Z` | marker lock-on only | Lock-on fixed `altitude_denial`. `traffic_conflict` then failed (`marker_stale`): the approach outran the camera. |
| `20261013T180619Z` | lock-on + capped approach | **All seven runs gave their designed outcome.** |

## What was wrong, and the fixes

**Marker selection flipped between pads.** From the 18 m staging height the downward
camera sees several ArUco markers inside the recovery area (4 and 9 on `search_field`, 11 m
apart). The marker nearest the image centre was re-picked on every frame, so the target
jumped between them, the lander's 2 m jump filter dropped the fixes, and the descent ran on
stale fixes until the 5 s timeout. The report also scored the touchdown against the
last-seen id: the "12.3 m" altitude-denial touchdown was in fact 0.37 m from marker 4.
Fix: `track_landing_marker` (vision_marker_detect.py) locks onto the first marker acquired.

**The approach outran the camera.** Marker 4 is ~15 m from the staging point. One far
position setpoint flew that at cruise speed (5-6 m/s); the tilt took the marker out of view
and the fix aged out. The re-picking above had been masking this with fixes of other markers.
Fix: `PrecisionLandParams.approach_max_speed_mps` (opt-in; tracktor-beam behaviour is unchanged
by default), set to 2 m/s for this sortie: the approach setpoint is a carrot 2 m ahead.

## Final campaign (`20261013T180619Z`)

| Run | Release | Recovery | Touchdown from marker 4 |
| --- | --- | --- | --- |
| 00 nominal | authorized | landed (`completed`) | 0.034 m |
| 01 altitude_denial | denied: below the 1000 m test floor | landed (`aborted_recovered`) | 0.042 m |
| 02 weather_denial | denied: wind above limit | landed (`aborted_recovered`) | 0.027 m |
| 03 airspace_denial | denied: prohibited airspace | landed (`aborted_recovered`) | 0.020 m |
| 04 traffic_conflict | denied: traffic conflict | landed (`aborted_recovered`) | 0.036 m |
| 05 clearance_expired | denied: simulated clearance expired | landed (`aborted_recovered`) | 0.130 m |
| 06 marker_loss | authorized | `failed`: marker withheld during descent (designed) | 0.144 m |

**Known issue.** `06_marker_loss` exited 139 (segmentation fault) *after* writing its complete
report, during interpreter shutdown. It was 1 of 3 marker_loss runs that day; the report and
its outcome are unaffected. It is recorded as-is, not masked, until the teardown crash is found.

All weather, airspace, traffic and clearance inputs are simulated (`simulated: true` in each
report); none is an operational authorization. Physical flight: not validated.
