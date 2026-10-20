# Flight Log Verification MVP — initial CLI slice

This is an **early, limited implementation**, not a finished browser uploader or a flight-safety validator.

## Why this library
Use the BSD-3-Clause [PX4 pyulog](https://github.com/PX4/pyulog) parser instead of writing a ULog decoder. PX4 Flight Review and PlotJuggler already offer excellent visualization; this project focuses on explicit, evidence-linked checks and eventually comparing repeatable mission experiments.

## Install and run
```bash
python3 -m pip install pyulog
python3 tools/flight_log_verify.py path/to/flight.ulg --output flight-report.json
```

Exit codes: 0 = no FAIL (may include SKIP); 1 = at least one FAIL; 2 = invalid input or parsing error. **Exit 0 is not proof of safe flight.**

## Checks implemented
- Presence of `vehicle_local_position` instance 0 (SKIP if absent).
- Finite logged local x/y/z values (FAIL if nonfinite).
- Monotonic sample timestamps and maximum observed sample interval (FAIL if exceeded; SKIP if fewer than 2 samples).

No GPS accuracy, separation, terrain clearance, mission completion, airspace compliance or hardware safety assertions are made. Log sampling gaps are not automatically communication failures.

## Next steps
1. Test against known public PX4 ULog fixtures and recorded SITL logs.
2. Add topic-schema compatibility fixtures and unit tests.
3. Normalize verification results into the existing replay report model.
4. Provide local-first upload/import UX, with privacy and size limits.
5. Add baseline/candidate experiment comparison and mission-level checks.
6. Connect to the configurable multi-vehicle SITL runner.

## Sources
- https://github.com/PX4/pyulog
- https://github.com/PX4/flight_review
- https://docs.px4.io/main/en/log/flight_log_analysis
