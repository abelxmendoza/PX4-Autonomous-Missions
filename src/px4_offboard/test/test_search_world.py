"""The search-and-find world: generated, valid, detectable, and its answers kept from the drones."""
from __future__ import annotations

import itertools
import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
WORLD = ROOT / "worlds" / "search_field.sdf"
TRUTH = ROOT / "worlds" / "search_field_targets.json"
MODELS = ROOT / "models"


@pytest.fixture(scope="module")
def truth() -> dict:
    return json.loads(TRUTH.read_text())


def test_committed_world_matches_the_generator():
    result = subprocess.run([sys.executable, str(ROOT / "tools" / "gen_search_world.py"), "--check"],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


def test_truth_file_describes_the_field_and_its_targets(truth):
    assert truth["world"] == "search_field"
    assert truth["dictionary"] == "DICT_4X4_50"
    assert truth["frame"].startswith("NED")
    assert len(truth["targets"]) >= 6
    assert len({t["id"] for t in truth["targets"]}) == len(truth["targets"])


def test_targets_are_inside_the_field_apart_and_off_the_launch_strip(truth):
    f = truth["field"]
    margin = truth["placement"]["edge_margin_m"]
    for t in truth["targets"]:
        assert f["north_min"] + margin <= t["north"] <= f["north_max"] - margin, t
        assert f["east_min"] + margin <= t["east"] <= f["east_max"] - margin, t
        assert t["north"] >= truth["placement"]["launch_strip_north_max_m"], t
    gaps = [math.dist((a["north"], a["east"]), (b["north"], b["east"]))
            for a, b in itertools.combinations(truth["targets"], 2)]
    assert min(gaps) >= truth["placement"]["min_spacing_m"]


def test_world_places_exactly_the_truth_targets_with_gazebo_axes(truth):
    world = ET.parse(WORLD).getroot().find("world")
    assert world.get("name") == "search_field"
    includes = {inc.findtext("uri"): inc for inc in world.findall("include")}
    for t in truth["targets"]:
        inc = includes.get(f"model://aruco_marker_{t['id']}")
        assert inc is not None, f"marker {t['id']} missing from the world"
        x, y, z = [float(v) for v in inc.findtext("pose").split()[:3]]
        # Gazebo world frame is ENU: x = east, y = north.
        assert (x, y) == pytest.approx((t["east"], t["north"]), abs=1e-6)
        assert 0.0 < z < 0.05  # flat on the ground, just above it to avoid z-fighting
    assert len([u for u in includes if u.startswith("model://aruco_marker_")]) == len(truth["targets"])


def test_world_declares_no_plugins_because_px4_server_config_provides_them():
    # Same rule as obstacle_world.sdf: duplicate system plugins break sensor topics.
    assert ET.parse(WORLD).getroot().find("world").find("plugin") is None


def test_every_marker_texture_is_read_back_as_its_own_id_by_our_detector(truth):
    cv2 = pytest.importorskip("cv2")
    from px4_offboard.vision_marker_detect import detect_largest_marker

    for t in truth["targets"]:
        model = MODELS / f"aruco_marker_{t['id']}"
        assert (model / "model.config").is_file() and (model / "model.sdf").is_file()
        texture = model / "materials" / "textures" / f"aruco_{t['id']}.png"
        image = cv2.imread(str(texture), cv2.IMREAD_GRAYSCALE)
        assert image is not None, texture
        found = detect_largest_marker(image, truth["dictionary"])
        assert found is not None and found.marker_id == t["id"], (texture, found)


def test_flight_code_never_reads_the_answers():
    # The drones must find the markers; only the verifier/tests may open the truth file.
    offenders = [p for p in (ROOT / "src" / "px4_offboard" / "px4_offboard").rglob("*.py")
                 if "search_field_targets" in p.read_text()]
    assert offenders == [], offenders
