"""ArUco marker detection over a raw grayscale frame, decoupled from ROS.

Kept separate from vision_marker.py so that module stays free of the
cv2/numpy dependency; this one intentionally takes both on so the actual
pixel-detection path — not just the pinhole geometry — is unit-testable
without a running node or a real camera.

OpenCV's ArUco API changed shape across versions: pre-4.7 only has the
legacy free functions (Dictionary_get/DetectorParameters_create/drawMarker);
4.7+ deprecates those in favor of the ArucoDetector class and
generateImageMarker. This module detects which is available at import time
and uses it, so it works whether the environment has the old apt-packaged
OpenCV or a fresh pip-installed one.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from .ekf_fusion import rotation_matrix_to_quat
from .vision_marker import PinholeCamera

ARUCO_DICTIONARIES: dict[str, int] = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
}

_HAS_ARUCO_DETECTOR = hasattr(cv2.aruco, "ArucoDetector")


class UnknownArucoDictionary(ValueError):
    pass


def _dictionary(dictionary_name: str):
    try:
        dict_id = ARUCO_DICTIONARIES[dictionary_name]
    except KeyError as exc:
        raise UnknownArucoDictionary(
            f"unsupported dictionary {dictionary_name!r}; choose one of {sorted(ARUCO_DICTIONARIES)}"
        ) from exc
    if _HAS_ARUCO_DETECTOR:
        return cv2.aruco.getPredefinedDictionary(dict_id)
    return cv2.aruco.Dictionary_get(dict_id)


def _detect(gray_image: np.ndarray, dictionary):
    if _HAS_ARUCO_DETECTOR:
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        return detector.detectMarkers(gray_image)
    params = cv2.aruco.DetectorParameters_create()
    return cv2.aruco.detectMarkers(gray_image, dictionary, parameters=params)


@dataclass(frozen=True)
class MarkerDetection:
    marker_id: int
    center_x_px: float
    center_y_px: float
    side_px: float


def detect_markers(gray_image: np.ndarray, dictionary_name: str = "DICT_4X4_50") -> list[MarkerDetection]:
    """Every ArUco marker in the frame (the search mission needs all of them, not one)."""
    corners, ids, _rejected = _detect(gray_image, _dictionary(dictionary_name))
    if ids is None or len(corners) == 0:
        return []
    out = []
    for c, i in zip(corners, ids):
        pts = c[0]
        out.append(MarkerDetection(
            marker_id=int(np.ravel(i)[0]),
            center_x_px=float(pts[:, 0].mean()),
            center_y_px=float(pts[:, 1].mean()),
            side_px=float(np.linalg.norm(pts[0] - pts[1])),
        ))
    return out


def detect_largest_marker(
    gray_image: np.ndarray, dictionary_name: str = "DICT_4X4_50"
) -> MarkerDetection | None:
    """Detect all ArUco markers and return the largest (nearest/most reliable) one."""
    dictionary = _dictionary(dictionary_name)
    corners, ids, _rejected = _detect(gray_image, dictionary)
    if ids is None or len(corners) == 0:
        return None

    best_idx = max(range(len(corners)), key=lambda i: cv2.contourArea(corners[i][0]))
    pts = corners[best_idx][0]
    # ids[i] is shape (1,) on OpenCV <5.0, a bare scalar on 5.0+; np.ravel handles both.
    return MarkerDetection(
        marker_id=int(np.ravel(ids[best_idx])[0]),
        center_x_px=float(pts[:, 0].mean()),
        center_y_px=float(pts[:, 1].mean()),
        side_px=float(np.linalg.norm(pts[0] - pts[1])),
    )


@dataclass(frozen=True)
class MarkerPose:
    """Marker pose in the camera optical frame: x right, y down, z out of the lens."""

    marker_id: int
    position_xyz: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    corners_px: tuple[tuple[float, float], ...]
    side_px: float


def camera_matrix(width_px: int, height_px: int, horizontal_fov_rad: float) -> np.ndarray:
    """Pinhole intrinsics for an ideal camera (zero skew, principal point at the center)."""
    camera = PinholeCamera(width_px, height_px, horizontal_fov_rad)
    f = camera.focal_length_px
    return np.array(
        [[f, 0.0, camera.cx], [0.0, f, camera.cy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def estimate_marker_poses(
    gray_image: np.ndarray,
    intrinsics: np.ndarray,
    marker_size_m: float,
    dictionary_name: str = "DICT_4X4_50",
    dist_coeffs: np.ndarray | None = None,
) -> list[MarkerPose]:
    """solvePnP pose of every detected marker, in the camera optical frame.

    Object-point order matches OpenCV's clockwise corner order (top-left, top-right,
    bottom-right, bottom-left) and the tracktor-beam ArUco tracker.
    """
    if marker_size_m <= 0.0:
        raise ValueError(f"marker_size_m must be positive, got {marker_size_m}")
    corners, ids, _rejected = _detect(gray_image, _dictionary(dictionary_name))
    if ids is None or len(corners) == 0:
        return []
    dist = np.zeros((5, 1), dtype=np.float64) if dist_coeffs is None else np.asarray(dist_coeffs, dtype=np.float64)
    half = marker_size_m / 2.0
    object_points = np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
        dtype=np.float32,
    )
    flag = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)
    poses = []
    for corner, marker_id in zip(corners, ids):
        image_points = np.asarray(corner[0], dtype=np.float32).reshape(4, 2)
        ok, rvec, tvec = cv2.solvePnP(object_points, image_points, intrinsics, dist, flags=flag)
        if not ok:
            ok, rvec, tvec = cv2.solvePnP(
                object_points, image_points, intrinsics, dist, flags=cv2.SOLVEPNP_ITERATIVE
            )
        if not ok:
            continue
        rotation, _ = cv2.Rodrigues(rvec)
        quat_wxyz = rotation_matrix_to_quat(rotation)
        xyz = tuple(float(v) for v in np.ravel(tvec))
        if not all(math.isfinite(v) for v in xyz):
            continue
        pts = tuple((float(p[0]), float(p[1])) for p in image_points)
        poses.append(
            MarkerPose(
                marker_id=int(np.ravel(marker_id)[0]),
                position_xyz=(xyz[0], xyz[1], xyz[2]),
                orientation_xyzw=(
                    float(quat_wxyz[1]),
                    float(quat_wxyz[2]),
                    float(quat_wxyz[3]),
                    float(quat_wxyz[0]),
                ),
                corners_px=pts,
                side_px=float(np.linalg.norm(image_points[0] - image_points[1])),
            )
        )
    return poses


def generate_marker_image(
    marker_id: int, side_px: int, dictionary_name: str = "DICT_4X4_50"
) -> np.ndarray:
    """Render a marker's pixels — used for both tests and the world texture asset."""
    dictionary = _dictionary(dictionary_name)
    if _HAS_ARUCO_DETECTOR:
        return cv2.aruco.generateImageMarker(dictionary, marker_id, side_px)
    return cv2.aruco.drawMarker(dictionary, marker_id, side_px)
