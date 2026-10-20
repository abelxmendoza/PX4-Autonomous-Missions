# Consolidation — October 20, 2026

This pass brings the fleet-product, offline-perception and RealmForge branches into one
reviewed checkout and preserves the existing local replay-guide work. It does not implement
remote fleet execution or certify imported worlds for flight.

## Integrated capabilities

- Event-timed guides for search, seeding and SLAM, including hide/resume and scrubbing.
- Initial ULog checks for finite local position and timestamp gaps, using pyulog.
- Recorded depth-policy report verification with explicit FAIL/SKIP handling.
- Standalone deterministic synthetic-depth prototype; no sensor or vehicle connection.
- Experimental RealmForge JSON-to-SDF conversion and opt-in custom-world launch support.
- A corrected [product direction](PRODUCT_DIRECTION.md) with a concrete fleet-experiment gate.

## Review fixes

The new root-level `tests/` directory was outside pytest's default discovery and the main
Autonomy verification job. Both now include it, so ULog, world-adapter and synthetic-depth
regressions run with the established suite. The adapter's standard-library unittest job remains.

Malformed ULog arrays now produce explicit input errors rather than indexing failures. World
and perception schemas reject boolean versions, invalid numeric transforms and unhashable
status/kind values. Synthetic-depth tests exercise missing/invalid pixels, thresholds, multiple
regions and malformed image shapes. No valid depth is not interpreted as proven clear space.

The seeding guide previously had no caption before the first mission transition. It now starts
with a takeoff caption; Chromium verifies it is visible even when opening a paused recording.

## Scope of runtime checks

- Headless Gazebo loaded the exported fixture with three obstacles and completed 1,000 iterations
  (exit 0). This is world-loading/physics startup evidence, not a PX4 mission or spawn-clearance proof.
- The ULog CLI parsed an actual local PX4 recording from October 16: 8,317 local-position samples,
  finite positions and maximum sample gap 0.011 s, below the configured 2 s threshold. Both checks
  passed. This does not establish position accuracy or safe flight. The raw log remains local.
- Chromium opened the home page and all three guided replays without JavaScript errors. Initial
  captions, hide/resume and end-of-recording scrubbing passed. No hosted deployment was made.

Machine-readable local results are retained in `evidence/consolidation_20261020/`.
The existing three known-open estimator requirements and the documented intermittent
mid-release airspace recovery failure remain unresolved; consolidation does not relabel them.

## Next implementation task

Implement a validated fleet-experiment configuration for counts 1–4 using the existing layout
and coordinator, before any simulator starts. Its acceptance tests must reject overlapping
spawns, invalid IDs/ports and unsupported scenarios. Then parameterize the local PX4 runner,
record reproducible manifests and execute at least two fleet sizes. The two-vehicle golden
recording and verifier must remain unchanged. See the ordered gate in PRODUCT_DIRECTION.md.

## Local validation results

| Check | Result |
| --- | --- |
| Fast suite | 892 passed, 4 ROS-dependent module skips |
| Integration suite | 41 passed |
| Full ROS-enabled suite | 971 passed, no skips |
| Replay tests | 94 passed |
| CMake/CTest | Build passed; 1 CTest passed |
| Recorded evidence, fault campaigns, requirement gate | Passed configured gates; known-open failures remain visible |
| World adapter | 5 tests passed; expanded malformed-input subtests also passed separately |

The local suites overlap and are not additive. The retained test totals identify the reviewed
code revision. Browser and ULog check outputs and the headless Gazebo log are retained alongside them.
