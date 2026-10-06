#!/usr/bin/env python3
"""Replay a recorded stereo dataset through the visual odometry and score it.

Dataset layout (written by ekf_fusion_node with VO_DEBUG_FRAMES=<dir>):
    <dir>/meta.csv                 idx, stamp_s, PX4 position/velocity/quaternion
    <dir>/left_00000.png ...       rectified-by-construction Gazebo stereo pair
    <dir>/right_00000.png ...

Why this exists: a live flight takes minutes and every VO idea needs one, so
VO tuning is done against a recorded flight instead. PX4's own estimate is the
reference -- it is used only to score, never fed into the VO.

    scripts/vo_offline_eval.py demo_artifacts/vo_dataset
    scripts/vo_offline_eval.py demo_artifacts/vo_dataset --set max_depth_m=8 --set reprojection_error_px=2
    scripts/vo_offline_eval.py demo_artifacts/vo_dataset --sweep
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "px4_offboard"))

from px4_offboard.ekf_fusion import quat_to_rotation_matrix  # noqa: E402
from px4_offboard.stereo_depth import StereoOdometry, VoConfig  # noqa: E402

OUTLIER_MPS = 3.0
ABSURD_MPS = 15.0  # no x500 in this scenario moves faster; such a VO output is a failure


def load_meta(root: Path) -> list[dict]:
    with (root / "meta.csv").open() as handle:
        return [
            {k: (int(v) if k == "idx" else float(v)) for k, v in row.items()}
            for row in csv.DictReader(handle)
        ]


def evaluate(root: Path, config: VoConfig, max_frames: int | None) -> dict:
    meta = load_meta(root)
    if max_frames:
        meta = meta[:max_frames]
    odo = StereoOdometry(config=config)
    samples = []  # (v_vo_world, v_ref_world, inliers, dt)
    rot_err_deg = []
    dead_reckoned = np.zeros(3)  # VO-only position, gated, started at PX4's first position
    start_pos = None
    path_len = 0.0
    last_ref = None
    final_err = 0.0
    t0 = time.time()
    prev = None
    for row in meta:
        left = cv2.imread(str(root / f"left_{row['idx']:05d}.png"), cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(str(root / f"right_{row['idx']:05d}.png"), cv2.IMREAD_GRAYSCALE)
        if left is None or right is None:
            continue
        ref_pos = np.array([row["px_n"], row["px_e"], row["px_d"]])
        ref_vel = np.array([row["pv_n"], row["pv_e"], row["pv_d"]])
        if start_pos is None:
            start_pos, dead_reckoned = ref_pos.copy(), ref_pos.copy()
        if last_ref is not None and np.all(np.isfinite(ref_pos)):
            path_len += float(np.linalg.norm(ref_pos - last_ref))
        last_ref = ref_pos
        motion = odo.process(left, right, row["stamp_s"])
        if motion is not None and prev is not None and np.all(np.isfinite(ref_vel)):
            _, trans_body = motion.in_body_frame()
            q_prev = np.array([prev["qw"], prev["qx"], prev["qy"], prev["qz"]])
            v_vo = quat_to_rotation_matrix(q_prev) @ (trans_body / motion.dt_s)
            v_ref = 0.5 * (ref_vel + np.array([prev["pv_n"], prev["pv_e"], prev["pv_d"]]))
            samples.append((v_vo, v_ref, motion.inlier_count, motion.dt_s))
            q_cur = np.array([row["qw"], row["qx"], row["qy"], row["qz"]])
            r_ref_rel = quat_to_rotation_matrix(q_prev).T @ quat_to_rotation_matrix(q_cur)
            rot_body, _ = motion.in_body_frame()
            cos_a = (np.trace(rot_body.T @ r_ref_rel) - 1.0) / 2.0
            rot_err_deg.append(float(np.degrees(np.arccos(np.clip(cos_a, -1.0, 1.0)))))
            if np.linalg.norm(v_vo) <= ABSURD_MPS:  # gated dead reckoning
                dead_reckoned = dead_reckoned + v_vo * motion.dt_s
        final_err = float(np.hypot(*(dead_reckoned[:2] - ref_pos[:2])))
        prev = row
    elapsed = time.time() - t0

    if not samples:
        return {"updates": 0}
    vo = np.array([s[0] for s in samples])
    ref = np.array([s[1] for s in samples])
    err = np.linalg.norm(vo - ref, axis=1)
    speed_ref = np.linalg.norm(ref, axis=1)
    moving = speed_ref > 0.8
    out = {
        "frames": len(meta),
        "updates": len(samples),
        "seconds": round(elapsed, 1),
        "ref_speed_median": float(np.median(speed_ref)),
        "median_err_mps": float(np.median(err)),
        "p90_err_mps": float(np.percentile(err, 90)),
        "outlier_pct": 100.0 * float(np.mean(err > OUTLIER_MPS)),
        "absurd_pct": 100.0 * float(np.mean(np.linalg.norm(vo, axis=1) > ABSURD_MPS)),
        "rot_err_med_deg": float(np.median(rot_err_deg)),
        "rot_err_p90_deg": float(np.percentile(rot_err_deg, 90)),
        "vo_only_end_err_m": final_err,
        "path_m": path_len,
        "vo_only_drift_pct": 100.0 * final_err / path_len if path_len > 1 else float("nan"),
    }
    for axis, name in enumerate("NED"):
        x, y = ref[moving, axis], vo[moving, axis]
        ok = np.abs(y - x) < OUTLIER_MPS
        if ok.sum() > 5 and np.dot(x[ok], x[ok]) > 0:
            out[f"scale_{name}"] = float(np.dot(x[ok], y[ok]) / np.dot(x[ok], x[ok]))
            out[f"corr_{name}"] = float(np.corrcoef(x[ok], y[ok])[0, 1])
    return out


SWEEP3 = {
    "sgbm (current default)": {},
    "bm block15": {"matcher": "bm", "block_size": 15},
    "bm block9": {"matcher": "bm", "block_size": 9},
    "bm block21": {"matcher": "bm", "block_size": 21},
}

SWEEP2 = {
    "reproj 2px": {"reprojection_error_px": 2.0},
    "reproj 1.5px": {"reprojection_error_px": 1.5},
    "reproj 1px": {"reprojection_error_px": 1.0},
    "reproj 1.5 minInl15": {"reprojection_error_px": 1.5, "min_inliers": 15},
    "reproj 1.5 corners400": {"reprojection_error_px": 1.5, "max_corners": 400},
}

SWEEP = {
    "baseline": {},
    "depth<=8": {"max_depth_m": 8.0},
    "depth<=6": {"max_depth_m": 6.0},
    "depth 1-8": {"min_depth_m": 1.0, "max_depth_m": 8.0},
    "reproj 2px": {"reprojection_error_px": 2.0},
    "depth<=8 reproj 2px": {"max_depth_m": 8.0, "reprojection_error_px": 2.0},
    "depth<=8 minInl 20": {"max_depth_m": 8.0, "min_inliers": 20},
    "depth<=8 reproj2 minInl20": {"max_depth_m": 8.0, "reprojection_error_px": 2.0, "min_inliers": 20},
    "disp128 block5": {"num_disparities": 128, "block_size": 5},
}


def fmt(name: str, r: dict) -> str:
    if not r.get("updates"):
        return f"{name:28s} no VO updates"
    sc = "/".join(f"{r.get('scale_' + a, float('nan')):.2f}" for a in "NED")
    cr = "/".join(f"{r.get('corr_' + a, float('nan')):.2f}" for a in "NED")
    return (
        f"{name:28s} n={r['updates']:4d} med_err={r['median_err_mps']:.2f} "
        f"p90={r['p90_err_mps']:.2f} outl={r['outlier_pct']:4.1f}% absurd={r['absurd_pct']:4.1f}% "
        f"scale(N/E/D)={sc} corr={cr} rot_err med/p90={r['rot_err_med_deg']:.2f}/{r['rot_err_p90_deg']:.2f}deg "
        f"VO-only drift={r['vo_only_drift_pct']:.0f}% "
        f"({r['seconds']:.0f}s)"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset", type=Path)
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--sweep3", action="store_true", help="stereo matcher comparison")
    ap.add_argument("--sweep2", action="store_true", help="refinement around the best first-sweep setting")
    ap.add_argument("--max-frames", type=int, default=None)
    args = ap.parse_args()

    fields = {f.name: f.type for f in dataclasses.fields(VoConfig)}
    base = {}
    for item in args.set:
        key, value = item.split("=", 1)
        if key not in fields:
            ap.error(f"unknown VoConfig field {key!r}; have {sorted(fields)}")
        base[key] = value if key == "matcher" else float(value) if "." in value or key.endswith(("_m", "_px")) else int(value)
    runs = SWEEP3 if args.sweep3 else SWEEP2 if args.sweep2 else SWEEP if args.sweep else {"custom" if base else "baseline": {}}
    for name, overrides in runs.items():
        cfg = VoConfig(**{**base, **overrides})
        print(fmt(name, evaluate(args.dataset, cfg, args.max_frames)), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
