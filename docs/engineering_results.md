# Engineering results — September 12, 2026

These are local command results and recorded measurements, not an assertion about an unseen hosted CI run. No hardware or HIL results are included. [Machine-derived test totals](../evidence/test_results.json) are retained alongside the flight evidence.

| Measurement | Result | Source / scope |
| --- | --- | --- |
| Original Python baseline | 131 passed, 2 failed, 3 skipped | Unsourced environment before the endpoint fix |
| Current lightweight Python suite | 138 passed, 3 skipped | `PYTHONPATH=src/px4_offboard python3 -m pytest src/px4_offboard/test -q`; skipped modules need ROS |
| Current ROS-enabled Python suite | 166 passed | Same suite after sourcing ROS Humble and installed PX4 messages; adapter tests do not fly aircraft |
| Browser/replay tests | 24 passed | `cd web/replay && npm test` |
| Curated recording expectations | 7/7 matched | `python3 scripts/verify_evidence.py`; one expected failure, six passing verifier outcomes |
| Existing legacy browser CSV regressions | 4 applicable-check passes | `vv_replay` on each `web/replay/data/*.csv`; sensor/GPS/clearance skips remain visible |
| Catalog | 21 requirements | 14 legacy single-vehicle verifier mappings plus 7 cooperative/telemetry acceptance entries |
| Documented defects in this pass | 2 | Coordinator endpoint acceptance; replay terminal-sample export |
| Fresh golden recovery | 641 synchronized rows; 2 reassignments; min horizontal separation 8.095491 m | [golden report](../evidence/swarm/golden_report.json), threshold 2.5 m |
| Golden playback duration | 63.900 s | First two-vehicle sample through exact final sample; raw log includes one earlier empty WAITING row |
| Curated nominal cooperative run | 503 samples; 0 reassignments; min separation 3.566418 m | [nominal report](../evidence/swarm/nominal_report.json) |
| Curated GPS-aiding-loss scenarios | 1 recording; 91 settled samples with external position fused | [GPS report](../evidence/gps_denied/report.json); 468 rows / 129 s total; contains gaps |
| Legacy avoidance excerpt | 410 samples; 214 avoidance-labelled rows; 41 s | [report](../evidence/obstacle_avoidance/report.json); not fresh-LiDAR proof |

## What “passing” means

The evidence runner validates input hashes, runs the actual verifiers, compares entire report snapshots and checks required non-skipped checks. A known-bad recording must fail for its original reason. It also compares the committed nominal/golden browser exports against regenerated raw evidence.

The single-vehicle verifier does not require complete preflight-to-disarm evidence. It excludes FAILSAFE altitude samples and allows absent events/legacy skips. Therefore some local failed or incomplete flights can have an overall applicable-check PASS. No such result is described here as successful full-flight containment. The golden cooperative verifier additionally requires independent task observations and final landed/disarmed flags.

## Re-run the checks

```bash
PYTHONPATH=src/px4_offboard python3 -m pytest src/px4_offboard/test -q
python3 scripts/verify_evidence.py
python3 scripts/export_gazebo_world.py --check
```

For the ROS-inclusive suite, source `/opt/ros/humble/setup.bash` and `install/setup.bash` first and preserve the resulting `PYTHONPATH`. Browser dependencies: `cd web/replay && npm ci && npm test`.

CI runs lightweight logic/verification, recorded evidence, browser tests and ROS adapter tests as separate jobs. It uploads test reports and writes a concise job summary. Full Gazebo/PX4 flights are intentionally manual; the fresh golden recording makes the important acceptance checks repeatable without the simulator.
