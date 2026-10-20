# Synthetic depth perception prototype

This standalone, ROS-free script is a **development seam** for later Jetson + RealSense integration, not a hardware demonstration.

Run with the existing CI dependency `numpy`:

```bash
python3 tools/synthetic_depth_perception.py
python3 tools/synthetic_depth_perception.py --threshold-m 0.8
```

The script creates a deterministic 120x160 depth image in **meters**, with a central 1.2 m rectangular region against a 5.0 m background. It reports whether pixels fall within the threshold, their bounding rectangle, and the minimum depth. No physical sensor data, calibrated intrinsics, ROS 2 messages, tracking, or actual obstacle avoidance are included.

## Next hardware-independent milestones
1. Add tests for missing depth, NaNs, no obstacle, and multiple regions.
2. Define a depth-frame interface and camera intrinsics contract.
3. Add an optional ROS 2 adapter with explicit timestamps and frame IDs.
4. When hardware is available, integrate RealSense SDK and validate depth scale, timestamps, camera calibration, and measured latency.
5. Record real and synthetic experiments separately; never label synthetic results as physical evidence.

This script is deliberately separate from PX4 flight verification until the evidence schema is validated.
