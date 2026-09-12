# A completed flight replayed as RETURNING

## Problem

The browser regression test expected the committed swarm replay's last phase to be COMPLETE, but observed RETURNING even though its attached report said PASS.

## Why a visual demo could miss it

The final approach looked plausible, and the exported report still advertised success. Only checking the final playback frame against the recording exposed that the display and its evidence disagreed.

## Requirement violated

REQ-TEL-002: preserve the final recorded cooperative observation and derive the verdict from the same raw input.

## Evidence and root cause

`export_swarm_replay.py` generated 10 Hz offsets using `int(duration * 10)`. If the terminal sample lay between ticks, the final generated timestamp preceded it. The sample selection loop therefore retained the preceding RETURNING row. The exporter also read a neighboring `verification.json`, which could be stale or belong to a different log.

This is a deterministic evidence-export boundary error. It does not indicate that PX4 failed to land.

## Code fix

Keep the existing 10 Hz playback samples, then append the exact last recorded offset when it is off-grid. Compute `swarm_verify.verify(path)` from the actual source rather than trusting the sidecar. Regenerate the nominal replay from its preserved raw input. The golden recovery uses the same exporter.

## Regression coverage and result

- `test_export_preserves_terminal_sample_between_playback_ticks` deliberately places COMPLETE 35 ms past a tick and requires the final time and landed states to survive.
- `test_export_recomputes_verdict_instead_of_trusting_sidecar` supplies a truncated log with a false passing sidecar; the export must report failure.
- The original browser nominal test passes unchanged after regeneration.
- The added golden browser test checks duration, two reassignments, completed tasks, valid final samples and both LANDED states.
- `verify_evidence.py` compares each curated swarm browser export to the raw source on every CI run.

Reproduce: `python3 scripts/verify_evidence.py` and `cd web/replay && npm test`.
