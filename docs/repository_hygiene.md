# Repository hygiene audit

No useful flight evidence was deleted during this pass. The curated bundle is described in [evidence/README.md](../evidence/README.md).

| Category | Finding | Treatment |
| --- | --- | --- |
| Generated workspace | `build/`, `install/`, `log/`, Python caches and package metadata | Remain ignored. Rebuild through colcon. |
| Flight outputs | Root `flight_log_*.csv`, `demo_artifacts/`, generated plots/video | Preserve locally; archive selected raw evidence with provenance and hashes. Ignore new root plots. |
| Dependency checkouts | `src/px4_msgs`, `src/px4_ros_com` | Remain ignored; external PX4 packages, not original project code. |
| Node/Python tools | `node_modules/`, virtual environments, test coverage/cache, local editor files | Broaden ignore rules without hiding evidence JSON/CSV or requirements. |
| Duplicate configuration | Root `config/` and installed ROS package `src/px4_offboard/config/` copies | Retain public paths for compatibility; CI checks byte equality to prevent drift. Package copy is the runtime source. |
| Overlapping mission scripts | Root MAVSDK takeoff, grid and avoidance scripts vs ROS mission node | Retain as earlier baseline examples; main README directs users to the current ROS stack. No evidence that deletion is safe or useful. |
| Old launches | `two_vehicle.launch.py` | Document as independent hover smoke test, not coordinated mission. |
| Stale docs | Four-transect description did not match current gate/landing tasks; GPS/VIO wording overstated sensing | Replace cooperative guide and qualify estimator/perception claims. Move long installation reference out of recruiter introduction. |
| Large tracked media | Project images, social-preview image, old `flight_plot.png`, browser telemetry | Preserve as existing presentation assets; no speculative deletion. Keep compressed new evidence small and avoid duplicating legacy CSVs. |
| Local assistant state | `.claude/`, `.codex/`, `.agents/` | Ignore local contents; do not remove or edit the user's local instructions. |

Historical recordings are evaluated under their documented limits. Old four-lane swarm runs are not silently relabeled as current gate/landing missions. Browser exports are derived assets; raw JSONL is the verification source.
