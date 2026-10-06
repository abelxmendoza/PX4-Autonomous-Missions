"""Real stereo depth and feature-based visual odometry from rendered images.

Deliberately *not* full SLAM: no persistent map, no loop closure, no bundle
adjustment. This computes, from an actual pair of rendered stereo frames,
frame-to-frame relative motion the same way a real stereo camera would —
disparity -> metric depth -> 3D-lift tracked features -> PnP pose estimate.
Nothing here reads Gazebo ground truth; ``ekf_fusion.py`` is the only
consumer, and it only ever sees this module's output, never simulator state.

Geometry constants below must match ``models/x500_lidar_2d/model.sdf``
(mono_cam + mono_cam_right): both cameras share horizontal_fov/resolution,
separated by a fixed baseline along the mount's Y axis. If the SDF changes,
these must change with it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

try:
    import cv2

    _CV2_OK = True
except Exception:  # noqa: BLE001
    _CV2_OK = False

# Must match models/mono_cam and models/mono_cam_right (and the comments
# in models/x500_lidar_2d/model.sdf). StereoOdometry.process also rebuilds
# the pinhole matrix from the actual frame size, so a resolution mismatch
# with these defaults will not silently use the wrong focal length.
BASELINE_M = 0.06
IMAGE_WIDTH_PX = 640
IMAGE_HEIGHT_PX = 480
HORIZONTAL_FOV_RAD = 1.74

MIN_DISPARITY_PX = 0.5  # below this, depth blows up / is unreliable
MIN_TRACKED_POINTS = 8  # solvePnPRansac needs >=4; require margin for RANSAC


@dataclass(frozen=True)
class VoConfig:
    """Tunable stereo-VO parameters, chosen by offline evaluation against a
    recorded flight (scripts/vo_offline_eval.py), not guessed.

    Against PX4's velocity over 619 updates, moving from the original
    (4 px reprojection threshold, 8 min inliers) to the values below cut
    gross outliers from 12.3% to 5.3%, absurd (>15 m/s) outputs from 5.4% to
    0.9%, and p90 velocity error from 4.9 to 1.5 m/s. Restricting the depth
    range made every metric worse (it starves PnP of features), so no depth
    limit is applied. Tuned on a single recording -- treat the numbers as
    evidence for the direction, and re-check on any new flight.

    Matcher: block matching (window 9) was as accurate as semi-global
    matching on the same recording (median velocity error 0.24 vs 0.28 m/s,
    gross outliers 3.4% vs 5.3%) at roughly a third of the cost (~11 ms vs
    ~36 ms per frame). That matters live: under simulator load the slower
    matcher kept the node near 5 updates/s of a possible 10, and longer gaps
    between processed frames make feature tracking harder.
    """

    num_disparities: int = 64
    block_size: int = 9
    matcher: str = "bm"  # "bm" | "sgbm"; see the note on this class
    min_depth_m: float = 0.0
    max_depth_m: float = float("inf")
    reprojection_error_px: float = 1.5
    min_inliers: int = 15
    max_corners: int = 200
    quality_level: float = 0.01
    min_distance_px: float = 7.0


DEFAULT_VO_CONFIG = VoConfig()


def focal_length_px(width_px: int, horizontal_fov_rad: float) -> float:
    """Pinhole focal length in pixels from image width and horizontal FOV."""
    if width_px <= 0 or not (0.0 < horizontal_fov_rad < math.pi):
        raise ValueError("width_px and horizontal_fov_rad must be positive/valid")
    return (width_px / 2.0) / math.tan(horizontal_fov_rad / 2.0)


def camera_matrix(
    width_px: int = IMAGE_WIDTH_PX,
    height_px: int = IMAGE_HEIGHT_PX,
    horizontal_fov_rad: float = HORIZONTAL_FOV_RAD,
) -> np.ndarray:
    """3x3 pinhole intrinsics matrix, principal point at image center."""
    f = focal_length_px(width_px, horizontal_fov_rad)
    return np.array(
        [[f, 0.0, width_px / 2.0], [0.0, f, height_px / 2.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def compute_disparity(
    left_gray: np.ndarray,
    right_gray: np.ndarray,
    num_disparities: int = 64,
    block_size: int = 9,
    matcher: str = "sgbm",
) -> np.ndarray:  # noqa: D417
    """Dense disparity map (float32, pixels) via semi-global block matching.

    Positive disparity means the feature appears further right in the left
    image than in the right image (nearer objects -> larger disparity),
    which is the correct sign only if ``left_gray`` truly comes from the
    camera with the larger Y coordinate in the SDF (see module docstring) —
    ``stereo_depth_node.py`` is responsible for passing frames in that
    consistent order.
    """
    if not _CV2_OK:
        raise RuntimeError("OpenCV is required for compute_disparity")
    if left_gray.shape != right_gray.shape:
        raise ValueError("left/right frames must have matching shape")
    if matcher == "bm":
        bm = cv2.StereoBM_create(numDisparities=num_disparities, blockSize=block_size)
        return bm.compute(left_gray, right_gray).astype(np.float32) / 16.0
    if matcher != "sgbm":
        raise ValueError(f"unknown matcher {matcher!r}")
    matcher = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=num_disparities,
        blockSize=block_size,
        P1=8 * block_size**2,
        P2=32 * block_size**2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32,
    )
    # StereoSGBM returns fixed-point disparity scaled by 16.
    raw = matcher.compute(left_gray, right_gray)
    return raw.astype(np.float32) / 16.0


def disparity_to_depth(
    disparity_px: np.ndarray,
    focal_px: float,
    baseline_m: float = BASELINE_M,
) -> np.ndarray:
    """Metric depth (metres) from disparity; invalid pixels -> NaN."""
    depth = np.full(disparity_px.shape, np.nan, dtype=np.float32)
    valid = disparity_px > MIN_DISPARITY_PX
    depth[valid] = (focal_px * baseline_m) / disparity_px[valid]
    return depth


def depth_at(depth_map: np.ndarray, x_px: float, y_px: float) -> float | None:
    """Nearest-pixel depth lookup; None if out of bounds or invalid."""
    h, w = depth_map.shape
    xi, yi = int(round(x_px)), int(round(y_px))
    if not (0 <= xi < w and 0 <= yi < h):
        return None
    value = float(depth_map[yi, xi])
    return None if math.isnan(value) else value


@dataclass
class TrackedFeatures:
    prev_points: np.ndarray  # Nx2, pixel coords in the previous frame
    curr_points: np.ndarray  # Nx2, pixel coords in the current frame


def track_features(
    prev_gray: np.ndarray,
    curr_gray: np.ndarray,
    max_corners: int = 200,
    quality_level: float = 0.01,
    min_distance: float = 7.0,
    min_points: int = MIN_TRACKED_POINTS,
) -> TrackedFeatures | None:
    """Sparse frame-to-frame feature tracking (goodFeaturesToTrack + LK)."""
    if not _CV2_OK:
        raise RuntimeError("OpenCV is required for track_features")
    prev_pts = cv2.goodFeaturesToTrack(
        prev_gray,
        maxCorners=max_corners,
        qualityLevel=quality_level,
        minDistance=min_distance,
    )
    if prev_pts is None or len(prev_pts) < min_points:
        return None
    curr_pts, status, _err = cv2.calcOpticalFlowPyrLK(
        prev_gray, curr_gray, prev_pts, None
    )
    if curr_pts is None:
        return None
    mask = status.reshape(-1).astype(bool)
    if mask.sum() < min_points:
        return None
    return TrackedFeatures(
        prev_points=prev_pts.reshape(-1, 2)[mask],
        curr_points=curr_pts.reshape(-1, 2)[mask],
    )


# Camera optical axes (x right, y down, z forward) -> vehicle body FRD
# (x forward, y right, z down). The cameras are mounted forward-facing with
# no tilt (models/x500_lidar_2d/model.sdf), so this is a pure axis relabel.
CAMERA_TO_BODY = np.array(
    [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64
)


@dataclass
class RelativeMotion:
    rotation_matrix: np.ndarray  # 3x3, camera frame at t -> camera frame at t+dt
    translation_m: np.ndarray  # 3-vector, metres, in the t-frame camera axes
    inlier_count: int
    # Time between the two image stamps this motion spans. Zero until
    # StereoOdometry.process fills it in from the actual frame timestamps --
    # not from processing time, which differs whenever frames are dropped.
    dt_s: float = 0.0

    def in_body_frame(self) -> tuple[np.ndarray, np.ndarray]:
        """(rotation, translation) re-expressed in body FRD axes.

        The camera-frame result is in optical axes (z forward); the EKF
        integrates body-FRD quantities, so feeding it camera axes directly
        would turn forward motion into sideways/downward motion.
        """
        c = CAMERA_TO_BODY
        return c @ self.rotation_matrix @ c.T, c @ self.translation_m


def estimate_relative_motion(
    tracked: TrackedFeatures,
    prev_depth_map: np.ndarray,
    camera_matrix_: np.ndarray,
    config: VoConfig = DEFAULT_VO_CONFIG,
) -> RelativeMotion | None:
    """3D-2D PnP pose estimate: lift tracked points to 3D via stereo depth
    at the previous frame, solve for the pose of the current camera frame
    relative to the previous one. Real metric scale comes from the stereo
    baseline (via ``prev_depth_map``), not assumed or fudged.
    """
    if not _CV2_OK:
        raise RuntimeError("OpenCV is required for estimate_relative_motion")
    object_points = []
    image_points = []
    f = camera_matrix_[0, 0]
    cx, cy = camera_matrix_[0, 2], camera_matrix_[1, 2]
    for (px, py), (qx, qy) in zip(tracked.prev_points, tracked.curr_points):
        depth = depth_at(prev_depth_map, px, py)
        if depth is None or depth <= 0.0:
            continue
        # Stereo depth error grows with depth squared (disparity is only
        # ~1-2 px beyond ~8 m at this baseline), so distant points can be
        # excluded rather than allowed to corrupt the scale of the pose.
        if depth < config.min_depth_m or depth > config.max_depth_m:
            continue
        x = (px - cx) * depth / f
        y = (py - cy) * depth / f
        object_points.append((x, y, depth))
        image_points.append((qx, qy))
    if len(object_points) < config.min_inliers:
        return None
    object_points = np.array(object_points, dtype=np.float64)
    image_points = np.array(image_points, dtype=np.float64)
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        object_points,
        image_points,
        camera_matrix_,
        None,
        reprojectionError=config.reprojection_error_px,
        confidence=0.99,
        iterationsCount=200,
    )
    if not ok or inliers is None or len(inliers) < config.min_inliers:
        return None
    rot_matrix, _ = cv2.Rodrigues(rvec)
    # solvePnP gives the pose of the *previous* points' frame as seen from
    # the current camera; invert to get "how did the camera move" in the
    # previous frame's axes, which is what an EKF wants to integrate.
    rot_cam_motion = rot_matrix.T
    trans_cam_motion = -rot_matrix.T @ tvec.reshape(3)
    return RelativeMotion(
        rotation_matrix=rot_cam_motion,
        translation_m=trans_cam_motion,
        inlier_count=int(len(inliers)),
    )


class StereoOdometry:
    """Stateful frame-to-frame visual odometry over a stream of stereo pairs."""

    def __init__(
        self,
        camera_matrix_: np.ndarray | None = None,
        config: VoConfig = DEFAULT_VO_CONFIG,
    ):
        self.config = config
        self.camera_matrix = (
            camera_matrix_ if camera_matrix_ is not None else camera_matrix()
        )
        self._prev_left_gray: np.ndarray | None = None
        self._prev_depth: np.ndarray | None = None
        self._prev_stamp: float | None = None

    def reset(self) -> None:
        self._prev_left_gray = None
        self._prev_depth = None
        self._prev_stamp = None

    def process(
        self, left_gray: np.ndarray, right_gray: np.ndarray, stamp_s: float
    ) -> RelativeMotion | None:
        """Feed one new stereo pair; returns relative motion since the last
        call, or None if this is the first frame or tracking failed (e.g.
        too few features, degenerate scene) — callers must treat None as
        "no update available this tick," not as "zero motion."
        """
        height, width = left_gray.shape[:2]
        if (
            int(round(self.camera_matrix[0, 2] * 2.0)) != width
            or int(round(self.camera_matrix[1, 2] * 2.0)) != height
        ):
            self.camera_matrix = camera_matrix(width_px=width, height_px=height)

        disparity = compute_disparity(
            left_gray,
            right_gray,
            self.config.num_disparities,
            self.config.block_size,
            self.config.matcher,
        )
        depth = disparity_to_depth(disparity, self.camera_matrix[0, 0])

        motion: RelativeMotion | None = None
        if (
            self._prev_left_gray is not None
            and self._prev_depth is not None
            and self._prev_stamp is not None
        ):
            tracked = track_features(
                self._prev_left_gray,
                left_gray,
                self.config.max_corners,
                self.config.quality_level,
                self.config.min_distance_px,
                self.config.min_inliers,
            )
            if tracked is not None:
                motion = estimate_relative_motion(
                    tracked, self._prev_depth, self.camera_matrix, self.config
                )
                if motion is not None:
                    motion.dt_s = stamp_s - self._prev_stamp
                    if motion.dt_s <= 0.0:
                        motion = None

        self._prev_left_gray = left_gray
        self._prev_depth = depth
        self._prev_stamp = stamp_s
        return motion
