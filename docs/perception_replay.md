# Recorded depth-perception verification

This project can verify version 1 `bench_replay_report` artifacts produced by
[`Jetson-RealSense-Perception`](https://github.com/abelxmendoza/Jetson-RealSense-Perception).
The perception repository remains the source of truth for CLEAR, STOP, and
SENSOR_FAULT decisions.

```bash
python3 tools/verify_perception_report.py /path/to/bench-report.json \
  --output /tmp/perception-verification.json
```

Exit `0` means all expectations passed with no SKIP events. Exit `1` means FAIL
or SKIP prevented a passing verification. Exit `2` means invalid input. The
`--allow-skip` option permits exploratory incomplete coverage but never turns
SKIP into PASS.

This is recorded-data verification only. It has no PX4, MAVLink, ROS, simulator,
or vehicle-command imports. It does not inject depth events into a controller,
prove obstacle avoidance, or establish physical safety.
