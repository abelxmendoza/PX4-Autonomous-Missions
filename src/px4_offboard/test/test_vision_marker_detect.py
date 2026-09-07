"""Unit tests for ArUco marker detection against real, synthetic frames.

Uses actual cv2.aruco calls end to end (generate a marker, paste it into a
blank frame, detect it back) rather than mocking OpenCV — this is the
genuine pixel-detection path the live camera stream will exercise, not
just the geometry math in vision_marker.py.
"""

import numpy as np
import pytest

from px4_offboard.vision_marker_detect import (
    UnknownArucoDictionary,
    detect_largest_marker,
    generate_marker_image,
)


def _paste_marker(canvas_size: tuple[int, int], marker_id: int, side_px: int, top_left: tuple[int, int]):
    canvas = np.full(canvas_size, 200, dtype=np.uint8)  # light gray background
    marker = generate_marker_image(marker_id, side_px)
    x, y = top_left
    canvas[y : y + side_px, x : x + side_px] = marker
    return canvas


def test_detects_a_pasted_marker_at_the_expected_location():
    frame = _paste_marker((480, 640), marker_id=3, side_px=120, top_left=(100, 80))
    result = detect_largest_marker(frame)
    assert result is not None
    assert result.marker_id == 3
    # Center of a 120px marker placed at (100, 80) is (160, 140).
    assert result.center_x_px == pytest.approx(160.0, abs=2.0)
    assert result.center_y_px == pytest.approx(140.0, abs=2.0)
    assert result.side_px == pytest.approx(120.0, rel=0.05)


def test_blank_frame_has_no_detection():
    blank = np.full((480, 640), 200, dtype=np.uint8)
    assert detect_largest_marker(blank) is None


def test_largest_of_multiple_markers_is_returned():
    canvas = np.full((480, 640), 200, dtype=np.uint8)
    small = generate_marker_image(1, 40)
    big = generate_marker_image(2, 150)
    canvas[10:50, 10:50] = small
    canvas[200:350, 400:550] = big
    result = detect_largest_marker(canvas)
    assert result is not None
    assert result.marker_id == 2


def test_unknown_dictionary_raises():
    frame = np.full((480, 640), 200, dtype=np.uint8)
    with pytest.raises(UnknownArucoDictionary):
        detect_largest_marker(frame, dictionary_name="NOT_A_REAL_DICT")
