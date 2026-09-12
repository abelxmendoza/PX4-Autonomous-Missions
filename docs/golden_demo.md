# Golden demo: fault → confirmed recovery → verified evidence

One flagship scenario: **cooperative recovery after a commanded vehicle landing**. This uses existing PX4/ROS 2 coordination and known-map routes. It deliberately leaves LiDAR, the single-vehicle resource executive, and GPS-aiding-loss work out of the story. “Mission executive reacts” here means the cooperative coordinator's fault policy; it does not imply that `mission_executive.py` runs in this launch.

The fresh September 12 recording lasts **63.900 s** in browser playback (64.000 s including its first empty WAITING row). At 2× it is a 32-second recruiter overview. It is actual PX4/Gazebo SITL telemetry, not a kinematic fixture or animation invented for the presentation.

## Watch without installing the simulator

```bash
python3 -m http.server 8000 --directory web/replay
```

Open `http://localhost:8000/demo/`, choose **Watch recovery (64s)**. The flagship is the default recording. Play/pause, scrubbing, 2× speed, top-down and reconstructed FPV views remain available. The HUD labels its verdict **post-flight verification**, so a verdict visible early in playback is not presented as an online decision. The existing hosted replay must be redeployed to display this checkout's changes.

## Narration, keyed to recorded events

Times below are relative to the first sample containing both vehicles; they are not simulator startup wall time.

| Time | Observable event | Engineering point |
| --- | --- | --- |
| 0–11.2 s | WAITING with both vehicles | Preflight coordination waits for vehicle readiness; PX4 owns stabilization. |
| 11.3–17.6 s | TAKEOFF → READY | Each independent ROS controller streams targets to its PX4 instance. |
| 17.8 s | Vehicle 2 enters LANDING with `simulated vehicle dropout` | Existing fault injection exercises explicit retirement, not unexpected radio loss. |
| 17.8–28.3 s | Healthy vehicle held; failed vehicle lands | Work remains reserved while its owner may still be airborne. |
| 28.3–29.4 s | Failed vehicle continuously LANDED, landed=true, armed=false | Fresh confirmation persists 1.1 s before release; required dwell is 1 s. |
| 29.4 s | Reassignment count becomes 2 | Healthy vehicle takes remaining work; the landed peer stays an obstacle. |
| 41.1 / 44.1 / 50.4 / 53.8 s | Four point visits recorded | Independent positions must support completion, not just task flags. A* uses known map geometry; no reactive obstacle detection is claimed. |
| 54.0–63.9 s | Surviving vehicle lands; COMPLETE | Final evidence requires both vehicles landed/disarmed. |
| After run | Raw JSONL → verifier → report → replay | 641 rows, 2 reassignments, min separation 8.095491 m; PASS within documented scope. |

## Reproduce the evidence result

```bash
python3 scripts/verify_evidence.py
```

This also recomputes browser exports and rejects differences without overwriting them. To deliberately rebuild exports after review:

```bash
python3 scripts/verify_evidence.py --write-replays
```

Raw input: [golden_recovery.jsonl.gz](../evidence/swarm/golden_recovery.jsonl.gz). Result: [golden_report.json](../evidence/swarm/golden_report.json). Provenance/hashes: [manifest](../evidence/manifest.json). The recording was not edited or shortened for acceptance; only the browser representation is resampled and it retains the exact terminal observation.

## Repeat the flight on the configured Linux workstation

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/run_swarm_demo.py --dropout
```

The runner already records telemetry, executes verification, writes `verification.json`, and returns a failing exit code if acceptance fails. It reads the current Python source, so it tested the endpoint fix before installation. Do not replace the curated golden merely because a new run finished; inspect the new raw log, verifier result and configuration first.

Recorded environment: ROS 2 Humble; Gazebo Harmonic; PX4 checkout `d26cb57aca2cf9601bba76ba2524939f9d5b21b2`; local `px4_msgs` checkout `392e831c1f659429ca83902e66820d7094591410`. These identify the installed source checkouts, not a guarantee that external build artifacts are clean or byte-identical to those revisions. Command: `python3 scripts/run_swarm_demo.py --dropout`. Original run folder: `demo_artifacts/swarm/20260912_102427_4fb25c/`.

## What the demo does not prove

No camera/LiDAR navigation, GPS denial, payload delivery, surveyed imagery, distributed swarm operation, hardware/HIL validation or precise touchdown location. The visible landing pad is contextual scenery. The verifier checks sampled positions and terminal flags, not continuous aircraft-volume collision clearance. Read [requirements](requirements.md) for exact acceptance and remaining gaps.
