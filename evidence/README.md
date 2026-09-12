# Curated evidence

Run from the repository root:

```bash
python3 scripts/verify_evidence.py
```

The manifest pins seven cases by SHA-256 and provenance. Five contain historical or fresh PX4/Gazebo recordings; two are before/after **kinematic** recovery traces. A known failure must continue to fail for the exact expected reason. Reports are regenerated from the actual verifiers, not handwritten summaries.

| Folder | Contents and intended claim |
| --- | --- |
| `nominal/` | Report referencing the existing committed 21 s MOVE→LANDING CSV; no duplicate raw data. |
| `obstacle_avoidance/` | Report referencing the existing 41 s legacy replay, with 214 avoidance-labelled rows. Sensor-backed avoidance and clearance are not established by this schema. |
| `gps_denied/` | Exact compressed historical CSV, 468 samples over 129 s, and regenerated report. External odometry is Gazebo pose plus modeled noise/drift. Missing early phases and time gaps prevent claiming continuous complete-flight evidence. |
| `swarm/` | Exact compressed nominal and fresh golden recovery JSONL plus reports. Gate/landing tasks, not area survey or payload delivery. |
| `failure_case/` | Exact compressed before/after deterministic recovery fixture output. Only coordinator endpoint tolerance changes; tests and verifier acceptance are not relaxed. |

`manifest.json` is the source of truth for origins, hashes, expected verdicts and report paths. Uncompressed logs are inspectable with `gzip -dc evidence/swarm/golden_recovery.jsonl.gz`. Compression preserves raw bytes; recordings are neither resampled nor trimmed here. Browser data is separately resampled for presentation and must not replace raw verification input.

To refresh reports after reviewing verifier changes: `python3 scripts/verify_evidence.py --write-reports`. Do not update hashes or expected outcomes just to suppress a failure. The default command rejects changed report details, newly skipped checks, or a changed input hash.

Full simulator logs, PX4 logs and video remain in local ignored `demo_artifacts/`. They were not deleted. The small committed bundle is enough to re-run the named checks; it is not an archive of every simulator configuration or run. The exact historical PX4 firmware revisions are not present for old recordings.
