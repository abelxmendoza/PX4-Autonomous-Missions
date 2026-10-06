"""Tests for stereo_depth.py.

Real math, synthetic inputs: the point of these tests is to verify the
geometry (disparity->depth scaling, and — most importantly — the PnP
inversion that turns "pose of prev-frame points as seen from the current
camera" into "how did the camera itself move, expressed in its own prior
frame") is actually correct, not to validate OpenCV's feature detector.
track_features (the one function that depends on real image content) gets
its own narrower test with a synthetic textured image.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from px4_offboard.stereo_depth import (  # noqa: E402
    IMAGE_HEIGHT_PX,
    IMAGE_WIDTH_PX,
    RelativeMotion,
    TrackedFeatures,
    camera_matrix,
    compute_disparity,
    disparity_to_depth,
    estimate_relative_motion,
    focal_length_px,
    track_features,
)


def test_camera_matrix_principal_point_tracks_image_size():
    cam = camera_matrix(width_px=640, height_px=480)
    assert cam[0, 2] == pytest.approx(320.0)
    assert cam[1, 2] == pytest.approx(240.0)
    assert cam[0, 0] == pytest.approx(focal_length_px(640, 1.74))


def test_stereo_odometry_rebuilds_intrinsics_from_frame_size():
    from px4_offboard.stereo_depth import StereoOdometry

    odom = StereoOdometry(camera_matrix_=camera_matrix(width_px=1280, height_px=960))
    assert odom.camera_matrix[0, 2] == pytest.approx(640.0)
    rng = np.random.default_rng(3)
    left = rng.integers(0, 256, size=(240, 320), dtype=np.uint8)
    right = np.roll(left, -4, axis=1)
    odom.process(left, right, 0.0)
    assert odom.camera_matrix[0, 2] == pytest.approx(160.0)
    assert odom.camera_matrix[1, 2] == pytest.approx(120.0)


def test_focal_length_rejects_invalid_inputs():
    with pytest.raises(ValueError):
        focal_length_px(0, 1.0)
    with pytest.raises(ValueError):
        focal_length_px(100, 0.0)


def test_disparity_to_depth_known_scaling():
    # depth = f * baseline / disparity — pick round numbers to check exactly.
    disparity = np.array([[10.0, 20.0, 0.1]], dtype=np.float32)
    depth = disparity_to_depth(disparity, focal_px=500.0, baseline_m=0.1)
    assert depth[0, 0] == pytest.approx(500.0 * 0.1 / 10.0)
    assert depth[0, 1] == pytest.approx(500.0 * 0.1 / 20.0)
    # Below MIN_DISPARITY_PX -> invalid (NaN), not a huge bogus depth.
    assert math.isnan(depth[0, 2])


def test_compute_disparity_recovers_a_known_pixel_shift():
    # A right image that is the left image shifted LEFT by N pixels means a
    # feature at column c in the left image appears at column c-N in the
    # right image -- i.e. disparity N, the near-camera case this module's
    # sign convention assumes (see compute_disparity's docstring).
    rng = np.random.default_rng(0)
    width, height, shift = 400, 300, 12
    base = rng.integers(0, 256, size=(height, width + shift), dtype=np.uint8)
    left = base[:, shift:]
    right = base[:, : width]
    disparity = compute_disparity(left, right, num_disparities=32, block_size=9)
    # Border regions are unreliable for block matching; check the interior.
    interior = disparity[40:-40, 60:-60]
    valid = interior[interior > 0]
    assert valid.size > 0
    assert np.median(valid) == pytest.approx(shift, abs=1.5)


def _project(camera_matrix_: np.ndarray, points_xyz: np.ndarray) -> np.ndarray:
    f = camera_matrix_[0, 0]
    cx, cy = camera_matrix_[0, 2], camera_matrix_[1, 2]
    x, y, z = points_xyz[:, 0], points_xyz[:, 1], points_xyz[:, 2]
    u = f * x / z + cx
    v = f * y / z + cy
    return np.stack([u, v], axis=1)


def _rotation_about_y(angle_rad: float) -> np.ndarray:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


def _synthetic_motion_case(rot_motion: np.ndarray, trans_motion: np.ndarray):
    """Build a TrackedFeatures + depth map whose ground truth is exactly
    (rot_motion, trans_motion) as *camera motion expressed in the previous
    camera's own frame* -- the convention estimate_relative_motion returns.
    """
    cam = camera_matrix()
    rng = np.random.default_rng(1)
    # Scattered points in front of the camera, prev-frame coordinates.
    n = 30
    x = rng.uniform(-1.5, 1.5, n)
    y = rng.uniform(-1.0, 1.0, n)
    z = rng.uniform(3.0, 8.0, n)
    points_prev = np.stack([x, y, z], axis=1)

    # Invert the module's own derivation to get solvePnP's (R, t) from the
    # ground-truth camera motion: rot_cam_motion = R.T, trans_cam_motion =
    # -R.T @ t  =>  R = rot_motion.T, t = -R @ trans_motion.
    r_solvepnp = rot_motion.T
    t_solvepnp = -r_solvepnp @ trans_motion
    points_curr = (r_solvepnp @ points_prev.T).T + t_solvepnp

    prev_px = _project(cam, points_prev)
    curr_px = _project(cam, points_curr)

    depth_map = np.full((IMAGE_HEIGHT_PX, IMAGE_WIDTH_PX), np.nan, dtype=np.float32)
    for (px, py), z_val in zip(prev_px, points_prev[:, 2]):
        xi, yi = int(round(px)), int(round(py))
        if 0 <= xi < IMAGE_WIDTH_PX and 0 <= yi < IMAGE_HEIGHT_PX:
            depth_map[yi, xi] = z_val

    tracked = TrackedFeatures(prev_points=prev_px, curr_points=curr_px)
    return tracked, depth_map, cam


def test_estimate_relative_motion_recovers_pure_translation():
    true_rot = np.eye(3)
    true_trans = np.array([0.2, 0.0, 0.0])  # moved 0.2m along the prev camera's x
    tracked, depth_map, cam = _synthetic_motion_case(true_rot, true_trans)
    result = estimate_relative_motion(tracked, depth_map, cam)
    assert result is not None
    assert np.allclose(result.rotation_matrix, true_rot, atol=1e-3)
    assert np.allclose(result.translation_m, true_trans, atol=1e-2)


def test_estimate_relative_motion_recovers_rotation_and_translation():
    true_rot = _rotation_about_y(math.radians(4.0))
    true_trans = np.array([0.1, 0.02, 0.15])
    tracked, depth_map, cam = _synthetic_motion_case(true_rot, true_trans)
    result = estimate_relative_motion(tracked, depth_map, cam)
    assert result is not None
    assert np.allclose(result.rotation_matrix, true_rot, atol=1e-3)
    assert np.allclose(result.translation_m, true_trans, atol=1e-2)


def test_estimate_relative_motion_returns_none_with_too_few_points():
    cam = camera_matrix()
    tracked = TrackedFeatures(
        prev_points=np.array([[100.0, 100.0], [200.0, 200.0]]),
        curr_points=np.array([[101.0, 100.0], [201.0, 200.0]]),
    )
    depth_map = np.full((IMAGE_HEIGHT_PX, IMAGE_WIDTH_PX), 5.0, dtype=np.float32)
    assert estimate_relative_motion(tracked, depth_map, cam) is None


def test_track_features_recovers_a_known_translation():
    # A textured (non-degenerate) synthetic frame, shifted by a known amount.
    rng = np.random.default_rng(2)
    canvas = rng.integers(0, 256, size=(300, 400), dtype=np.uint8)
    prev = canvas[50:250, 50:350]
    dx, dy = 6, 3
    curr = canvas[50 - dy : 250 - dy, 50 - dx : 350 - dx]
    tracked = track_features(prev, curr)
    assert tracked is not None
    # curr's window origin is (dx, dy) up-left of prev's in canvas
    # coordinates, so the same content shifts by (+dx, +dy) in local pixels.
    deltas = tracked.curr_points - tracked.prev_points
    assert np.median(deltas[:, 0]) == pytest.approx(dx, abs=1.0)
    assert np.median(deltas[:, 1]) == pytest.approx(dy, abs=1.0)


def test_track_features_returns_none_on_a_blank_frame():
    blank = np.zeros((200, 200), dtype=np.uint8)
    assert track_features(blank, blank) is None


def test_camera_forward_motion_becomes_body_forward_not_sideways():
    # The bug this guards against: VO returns motion in camera *optical*
    # axes (x right, y down, z forward). Forward flight is +z there, but the
    # EKF integrates body FRD (x forward). Without the relabel, flying north
    # at yaw 0 shows up as "down".
    from px4_offboard.stereo_depth import RelativeMotion

    motion = RelativeMotion(
        rotation_matrix=np.eye(3),
        translation_m=np.array([0.0, 0.0, 0.5]),  # 0.5 m along optical z
        inlier_count=20,
        dt_s=0.1,
    )
    rot_body, trans_body = motion.in_body_frame()
    assert np.allclose(trans_body, [0.5, 0.0, 0.0])
    assert np.allclose(rot_body, np.eye(3))


def test_camera_right_and_down_map_to_body_right_and_down():
    from px4_offboard.stereo_depth import RelativeMotion

    right = RelativeMotion(np.eye(3), np.array([0.3, 0.0, 0.0]), 20, 0.1)
    down = RelativeMotion(np.eye(3), np.array([0.0, 0.2, 0.0]), 20, 0.1)
    assert np.allclose(right.in_body_frame()[1], [0.0, 0.3, 0.0])
    assert np.allclose(down.in_body_frame()[1], [0.0, 0.0, 0.2])


def test_camera_yaw_rotation_is_a_body_yaw_rotation():
    # A turn about the camera's y (down) axis is a yaw: rotation about body z.
    from px4_offboard.stereo_depth import RelativeMotion

    angle = math.radians(10.0)
    rot_cam = _rotation_about_y(angle)
    motion = RelativeMotion(rot_cam, np.zeros(3), 20, 0.1)
    rot_body, _ = motion.in_body_frame()
    c, s = math.cos(angle), math.sin(angle)
    expected_yaw = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    assert np.allclose(rot_body, expected_yaw, atol=1e-9)


def test_stereo_odometry_motion_dt_comes_from_image_stamps(monkeypatch):
    # Frames can be dropped; the interval a motion spans is the stamp
    # difference, not however long processing took.
    import px4_offboard.stereo_depth as sd

    odo = sd.StereoOdometry()
    rng = np.random.default_rng(3)
    frame = rng.integers(0, 256, size=(480, 640), dtype=np.uint8)
    fake = sd.RelativeMotion(np.eye(3), np.array([0.0, 0.0, 0.1]), 30)
    monkeypatch.setattr(sd, "estimate_relative_motion", lambda *a, **k: fake)
    monkeypatch.setattr(
        sd,
        "track_features",
        lambda *a, **k: sd.TrackedFeatures(np.zeros((10, 2)), np.zeros((10, 2))),
    )
    assert odo.process(frame, frame, 10.00) is None  # first frame: nothing to compare
    motion = odo.process(frame, frame, 10.35)  # 3 frames dropped in between
    assert motion is not None
    assert motion.dt_s == pytest.approx(0.35)
    assert odo.process(frame, frame, 10.35) is None  # non-increasing stamp rejected
