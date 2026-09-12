# Completion without an observed visit

## Problem / expected behavior

Two existing dropout/reassignment tests expected the surviving vehicle to finish all four assigned point visits, land, maintain separation, and pass independent flight-log verification. Both failed at `assert verify(path)["passed"]` while earlier completion/separation assertions passed.

Baseline: `662b72e14190e7eca2bed8fd06a41577b6c745c9`. Tests:
`test_complete_survey_and_confirmed_dropout_reassignment[True]` and
`test_recovery_budget_allows_measured_sitl_tracking_speed`.

## Why the normal visual demo did not reveal it

A route passing within roughly one metre of a waypoint looks plausible at course scale. The coordinator marked every task complete and eventually reported COMPLETE. Normal two-vehicle motion also happened to approach enough endpoints more closely. Recovery changed task ordering and approach direction, exposing the difference between a completion flag and an actual visit. A normal visual demonstration or a completion-counter-only test could miss this.

## Requirement violated

[REQ-SWM-003](../requirements.md): task completion requires an independent armed-position observation within 0.6 m. The verifier's corridor/start/end checks must not be weakened to match incorrect completion flags.

## Telemetry/evidence that exposed it

Both baseline traces failed only with `survey transects not independently observed` (legacy error wording; current tasks are point visits).

| Fixture | Samples | Reassignments | Min horizontal separation | Closest gate_1 approach | Closest gate_2 approach |
| --- | --- | --- | --- | --- | --- |
| Recovery at 1.0 m/s | 780 | 2 | 8.035714 m | 0.800824 m | 0.869309 m |
| Recovery at 0.45 m/s | 1,715 | 2 | 8.035581 m | 0.904871 m | 0.932215 m |

The preserved [before trace](../../evidence/failure_case/recovery_before.jsonl.gz) is the 1.0 m/s case, with its [verifier report](../../evidence/failure_case/before_report.json). These are deterministic kinematic fixtures, not PX4 flights. The slower original test remains regression coverage.

## Root cause and classification

**Coordinator/autonomy acceptance logic:** `SurveyCoordinator._step` consumed every route point within 1.0 m, including the final task endpoint. Once consumed, the task was marked completed and the drone turned toward another task before reaching the verifier's 0.6 m visit radius.

**Verifier assumptions:** the stricter radius was intentional evidence-based acceptance, not a false positive. **Telemetry collection:** complete, fresh fixed-step traces existed. **Timing/race conditions:** the failures were deterministic at two speeds, with no concurrency in the fixture. **Test fixture:** the unchanged fixture exposed the production coordinator's tolerance mismatch; it was not the source of the defect.

## Code fix

In `swarm_logic.py`, retain 1.0 m acceptance for intermediate transit corners and use 0.5 m when the last route point belongs to an active task. The 0.1 m difference from verifier acceptance provides observation margin before changing direction. No speed, timeout, separation threshold, expected result, or verifier tolerance was loosened.

## Regression coverage

- Both originally failing tests pass unchanged.
- `test_task_endpoint_requires_observable_visit`: a 0.8 m near miss remains incomplete; a 0.4 m visit completes.
- `test_verifier_rejects_completion_without_observed_task_visits`: forged completion flags with no travel remain rejected.
- The immutable before trace still fails for the original reason. The same fixture after the fix passes.
- A fresh PX4/Gazebo `--dropout` run independently exercises vehicle dynamics and transport after the code change.

## Final verification result

[Golden SITL report](../../evidence/swarm/golden_report.json): **PASS**, 641 synchronized samples, two task reassignments, minimum horizontal separation **8.095491 m**, all four task visits observed, both vehicles landed/disarmed. The recording lasts approximately 64 seconds; see [golden demo](../golden_demo.md).

Reproduce offline:

```bash
PYTHONPATH=src/px4_offboard python3 -m pytest src/px4_offboard/test/test_swarm_logic.py -q
python3 scripts/verify_evidence.py
```

The fix addresses endpoint observation, not general swarm scalability, landing-pad accuracy, or continuous collision avoidance guarantees.
