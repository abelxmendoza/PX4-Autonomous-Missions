"""The search drone's camera must look down, with the intrinsics the rest of the code assumes."""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MODEL = ET.parse(ROOT / "models" / "x500_search_cam" / "model.sdf").getroot().find("model")


def _camera_link():
    return next(l for l in MODEL.findall("link") if l.get("name") == "camera_down_link")


def _pose(text):
    return [float(v) for v in text.split()]


def test_model_name_matches_its_directory_and_config():
    cfg = ET.parse(ROOT / "models" / "x500_search_cam" / "model.config").getroot()
    assert MODEL.get("name") == "x500_search_cam" == cfg.findtext("name")


def test_camera_points_straight_down_from_under_the_body():
    x, y, z, roll, pitch, yaw = _pose(_camera_link().findtext("pose"))
    assert z < 0  # below base_link, so the body is not in the way
    assert math.isclose(pitch, math.pi / 2, abs_tol=1e-6) and roll == 0 and yaw == 0
    # A +90 deg pitch about y maps the sensor's +x (optical axis) onto -z: straight down.
    optical_axis_z = -math.sin(pitch)
    assert math.isclose(optical_axis_z, -1.0, abs_tol=1e-6)


def test_camera_is_fixed_to_the_airframe():
    joint = next(j for j in MODEL.findall("joint") if j.findtext("child") == "camera_down_link")
    assert joint.get("type") == "fixed" and joint.findtext("parent") == "base_link"


def test_intrinsics_match_the_forward_cameras_and_resolve_a_1m_marker_at_8m():
    cam = _camera_link().find("sensor/camera")
    width = int(cam.findtext("image/width"))
    hfov = float(cam.findtext("horizontal_fov"))
    assert (width, int(cam.findtext("image/height")), hfov) == (640, 480, 1.74)
    focal_px = width / (2 * math.tan(hfov / 2))
    marker_px_at_8m = focal_px * 1.0 / 8.0
    # ArUco 4x4 + border = 6 cells; ~5 px per cell is a practical detection floor.
    assert marker_px_at_8m >= 30, marker_px_at_8m


def test_tracking_aids_are_visual_only_and_hidden_from_the_search_camera():
    aids = next(l for l in MODEL.findall("link") if l.get("name") == "tracking_aids_link")
    assert aids.find("collision") is None  # cannot touch anything
    visuals = aids.findall("visual")
    assert {v.get("name") for v in visuals} >= {"geo_cage", "beacon"}
    mask = int(_camera_link().find("sensor/camera").findtext("visibility_mask"))
    for v in visuals:
        flags = int(v.findtext("visibility_flags"))
        assert mask & flags == 0, f"{v.get('name')} would appear in the downward camera"
    # ...while ordinary visuals (default flags: all bits) stay visible to it.
    assert mask & 0xFFFFFFFF != 0
