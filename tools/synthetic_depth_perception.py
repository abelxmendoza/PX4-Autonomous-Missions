#!/usr/bin/env python3
"""ROS-free deterministic depth-obstacle detection prototype.

Synthetic frames are NOT RealSense measurements. Distances are simulated meters.
"""
import argparse
import json
import math
import numpy as np


def synthetic_depth(width=160, height=120, obstacle_m=1.2, background_m=5.0):
    if width < 8 or height < 8 or not (0 < obstacle_m < background_m):
        raise ValueError("Invalid dimensions or obstacle distances")
    depth = np.full((height, width), background_m, dtype=np.float32)
    depth[height // 3:2 * height // 3, width // 3:2 * width // 3] = obstacle_m
    return depth


def detect_nearest_obstacle(depth_m, threshold_m=2.0, min_pixels=25):
    """Return a coarse obstacle ROI; no segmentation or 3D localization claim."""
    depth = np.asarray(depth_m)
    if depth.ndim != 2 or depth.size == 0:
        raise ValueError("Expected nonempty 2D depth image")
    if not math.isfinite(threshold_m) or threshold_m <= 0 or min_pixels < 1:
        raise ValueError("Invalid threshold or minimum pixel count")
    valid = np.isfinite(depth) & (depth > 0)
    near = valid & (depth <= threshold_m)
    count = int(np.count_nonzero(near))
    if count < min_pixels:
        return {"detected": False, "valid_pixel_count": int(np.count_nonzero(valid))}
    ys, xs = np.nonzero(near)
    return {
        "detected": True,
        "distance_m": float(np.min(depth[near])),
        "pixel_count": count,
        "bbox_xyxy": [int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1],
        "center_px": [float(xs.mean()), float(ys.mean())],
        "valid_pixel_count": int(np.count_nonzero(valid)),
        "limitations": "Coarse threshold ROI; multiple nearby objects may merge; no tracking or 3D pose",
    }


def main():
    parser = argparse.ArgumentParser(description="Synthetic depth obstacle smoke test (no hardware)")
    parser.add_argument("--threshold-m", type=float, default=2.0)
    args = parser.parse_args()
    try:
        frame = synthetic_depth()
        result = detect_nearest_obstacle(frame, args.threshold_m)
    except ValueError as exc:
        parser.error(str(exc))
    print(json.dumps({"source": "synthetic", "image_shape": list(frame.shape),
                      "obstacle": result}, indent=2))


if __name__ == "__main__":
    main()
