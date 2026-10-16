# EKF2 bad-start audit of the recorded SITL evidence

Audited 2026-10-16, after commit `6c0b909` added the arm gate to `tools/integrated_seeding_sitl.py`.
The question: did any committed evidence run arm with the corrupted EKF2 start that the gate
now catches? Evidence files were read, not changed.

**Result: no. None of the 40 seeding runs in `evidence/`, the `precision_land` touchdown,
the search sweep or the HIL smoke run started with a corrupted estimator.** Nothing needs
re-recording or annotating for this failure mode.

## The failure mode

PX4 SITL sometimes starts EKF2 while the spawned x500 is still settling. EKF2 then learns a
false gyro bias of about 0.1 to 0.17 rad/s. PX4 can still arm, and then flies with a 10 to
50 deg attitude error. The gate rejects arming when the bias-corrected body rate on the ground
is above 0.03 rad/s, and restarts ekf2.

## Method

Each evidence run was matched to its PX4 ulog
(`~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/`) by UTC time. Ulog names match the
report's `start_time` to within 2.2 s in every case. The `precision_land` report has no
timestamp, so it was matched by touchdown position instead: `2026-10-12/23_44_14.ulg` ends at
NED (14.092, -3.876), and the report gives (14.088, -3.877). It is also the last flight before
the report was committed.

For each ulog (pyulog), at the first `vehicle_status.arming_state == ARMED`:

- `estimator_sensor_bias.gyro_bias`, its norm at arm, and its peak before arm;
- `vehicle_angular_velocity` averaged over the 2 s before arm (the quantity the gate checks);
- the angle between `vehicle_attitude.q` and `vehicle_attitude_groundtruth.q`, at arm and its
  maximum after arm.

### Calibration on the 22 runs in `/tmp/segv` (2026-10-16)

| | healthy (10 runs) | bad start (12 runs) |
| --- | --- | --- |
| gyro bias at arm | ≤ 0.0003 rad/s | 0.0012 to 0.162 rad/s |
| peak gyro bias before arm | ≤ 0.0068 rad/s | 0.037 to 0.174 rad/s |
| attitude error at arm | 4.52 to 4.74 deg | 5.7 to 44.0 deg |
| max attitude error after arm | ≤ 5.81 deg | 11.8 to 180 deg |

By these numbers 12 of the 22 runs had a bad start, not the 8 counted earlier.

Healthy runs show a constant ~4.6 deg difference on the ground in this world, which drops below
1 deg (median) once flying. The bias **at the moment of arming** is not enough on its own: in
three bad runs, EKF2 had already pulled the bias down to 0.0012-0.0125 rad/s by arm time, but
they still had 6.3-12.4 deg error at arm and 11.8-44.6 deg in flight. The peak bias before arm
and the attitude error both separate the two groups cleanly, so the table below reports all of them.

### Finding for the arm gate (not about the evidence)

The gate in `6c0b909` checks the at-rest body rate against 0.03 rad/s. Across all 2026-10-16
ulogs up to `12_58_50` (34 runs), four bad starts were under that limit when they armed:

| Ulog | Peak bias before arm | Body rate at arm | Error at arm | Max error after arm |
| --- | --- | --- | --- | --- |
| 12_07_59 | 0.141 | 0.0147 | 12.4 | 44.6 |
| 12_26_52 | 0.051 | 0.0066 | 6.3 | 11.8 |
| 12_35_31 | 0.037 | 0.0051 | 9.0 | 21.4 |
| 12_42_40 | 0.039 | 0.0073 | 8.4 | 10.0 |

In these runs the bias estimate had mostly recovered, but the attitude had not, so the gate would
have let them arm. In this set, the peak bias before arm separates the groups: healthy runs peak
at ≤ 0.016 rad/s and bad starts at ≥ 0.037. This audit did not check which of these runs had the
gate in place.

## Results

All values at first arm. Bias in rad/s, attitude error in degrees.

| Run | Ulog | Bias at arm | Peak bias before arm | Error at arm | Max error after arm | Outcome |
| --- | --- | --- | --- | --- | --- | --- |
| fault_scenarios 174711Z (6 runs) | 2026-10-13/17_47_19 to 17_53_34 | ≤ 0.0001 | ≤ 0.0003 | 4.52 to 4.66 | ≤ 5.18 | as reported |
| fault_scenarios 175610Z (7 runs) | 2026-10-13/17_56_18 to 18_03_43 | ≤ 0.0002 | ≤ 0.0002 | 4.61 to 4.70 | ≤ 5.24 | as reported |
| fault_scenarios 180619Z (7 runs) | 2026-10-13/18_06_27 to 18_14_40 | ≤ 0.0002 | ≤ 0.0003 | 4.60 to 4.71 | ≤ 5.08 | as reported |
| integrated_seeding | 2026-10-13/02_39_07 | 0.0002 | 0.0003 | 4.68 | 4.81 | as reported |
| mission_verification (5 runs) | 2026-10-13/04_17_18 to 04_22_48 | ≤ 0.0002 | ≤ 0.0004 | 4.58 to 4.66 | ≤ 5.11 | as reported |
| recovery_campaign 051520Z (3) | 2026-10-13/05_15_28 to 05_18_04 | ≤ 0.0002 | ≤ 0.0003 | 4.62 to 4.73 | ≤ 5.26 | as reported |
| recovery_campaign 052703Z (3) | 2026-10-13/05_27_12 to 05_29_50 | ≤ 0.0002 | ≤ 0.0003 | 4.68 to 4.79 | ≤ 5.35 | as reported |
| recovery_campaign 054051Z (2) | 2026-10-13/05_40_59, 05_44_11 | ≤ 0.0002 | ≤ 0.0003 | 4.55, 4.66 | 4.92, 6.71 | see note 1 |
| recovery_campaign 054902Z (3) | 2026-10-13/05_49_11 to 05_49_34 | 0.0001 to 0.0002 | ≤ 0.0003 | 4.60 to 4.61 | never armed | see note 2 |
| recovery_campaign 055355Z (3) | 2026-10-13/05_54_03 to 05_56_48 | ≤ 0.0002 | ≤ 0.0002 | 4.57 to 4.74 | ≤ 5.87 | as reported |
| precision_land | 2026-10-12/23_44_14 | 0.0001 | 0.0003 | 4.62 | 7.85 | see note 3 |
| search sweep (2026-10-09) | 2026-10-09/21_03_22, 21_11_09, 21_13_45 | ≤ 0.0001 | ≤ 0.0004 | 4.55 to 5.73 | ≤ 6.30 | see note 4 |
| hil/sitl_smoke | 2026-10-06/05_16_20 | 0.0001 | 0.0001 | 6.69 | no flight | see note 5 |

Highest values across all 40 seeding runs: bias at arm 0.0002 rad/s, peak bias before arm
0.0004 rad/s, body rate before arm 0.0003 rad/s (gate limit 0.03), error at arm 4.79 deg.
The smallest bad start in the calibration set had a peak bias of 0.037 rad/s before arm and
5.7 deg error at arm. So the evidence also passes the stricter peak-bias test above, not just the gate's.

Notes:

1. `054051Z/01_traffic_conflict` reaches 6.71 deg in flight (median 2.38), above the
   calibration's healthy maximum of 5.81 deg. Its start is clean (bias 0.0001, 4.66 deg at
   arm). This was a 3-minute failed run that flew a different profile from the 1-minute
   calibration sortie, so this looks like flight dynamics, not a bias. It is far from the
   11.8 deg minimum of the bad starts.
2. `054902Z` never armed: each run ended after ~1 s as `simulation_terminated`. The estimator
   was healthy, so the bad start is not the cause.
3. `precision_land` reaches 7.85 deg in flight (median 3.26). The start is clean, and the
   flight profile differs from the seeding sortie.
4. The search sweep report records `2026-10-09T21:06:05Z` but no ulog name. All three
   candidate ulogs from that hour start clean, so the result holds whichever one it is.
5. The HIL smoke ulog shows a constant 6.69 deg ground offset instead of 4.6, with bias 0.0001
   rad/s and the same value from start to end. That is a different baseline (the smoke test
   does not run the `search_field` world), not a drifting bias.

## Not covered

- `slam/`, `stereo_vo/`, `gps_denied/`, `nominal/`, `obstacle_avoidance/`, `swarm/`, `bench/`
  and `failure_case/`. These are replays, CSV captures or non-seeding flights. They were out
  of scope for this audit.
- How often the bad start happens. It hit 12 of 22 runs in `/tmp/segv` on 2026-10-16 and
  0 of 40 on 2026-10-13. This audit does not explain the difference.

## Recommendation

No action is needed on the evidence. All 40 recorded runs armed with a healthy estimator, so
their outcomes are not artefacts of this failure mode. If the evidence README should record
that the runs predate the gate and were checked, add one line pointing to this file. That is
optional and is left for the owner to decide.
