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

from dataclasses import dataclass

import cv2
import numpy as np

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
    return MarkerDetection(
        marker_id=int(ids[best_idx][0]),
        center_x_px=float(pts[:, 0].mean()),
        center_y_px=float(pts[:, 1].mean()),
        side_px=float(np.linalg.norm(pts[0] - pts[1])),
    )


def generate_marker_image(
    marker_id: int, side_px: int, dictionary_name: str = "DICT_4X4_50"
) -> np.ndarray:
    """Render a marker's pixels — used for both tests and the world texture asset."""
    dictionary = _dictionary(dictionary_name)
    if _HAS_ARUCO_DETECTOR:
        return cv2.aruco.generateImageMarker(dictionary, marker_id, side_px)
    return cv2.aruco.drawMarker(dictionary, marker_id, side_px)
