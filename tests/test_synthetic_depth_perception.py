"""Synthetic data only: invalid depths must not be mistaken for nearby obstacles."""
import importlib.util
from pathlib import Path
import numpy as np
import pytest

spec = importlib.util.spec_from_file_location('synthetic_depth', Path(__file__).resolve().parents[1] / 'tools/synthetic_depth_perception.py')
depth = importlib.util.module_from_spec(spec)
spec.loader.exec_module(depth)


def test_deterministic_scene_and_threshold():
    frame = depth.synthetic_depth()
    np.testing.assert_array_equal(frame, depth.synthetic_depth())
    found = depth.detect_nearest_obstacle(frame)
    assert found['detected'] and found['distance_m'] == pytest.approx(1.2)
    assert found['bbox_xyxy'] == [53, 40, 106, 80]
    assert not depth.detect_nearest_obstacle(frame, threshold_m=0.8)['detected']


def test_missing_depth_does_not_claim_clear_space():
    result = depth.detect_nearest_obstacle(np.array([[np.nan, np.inf, 0, -1]]))
    assert result == {'detected': False, 'valid_pixel_count': 0}


def test_multiple_regions_are_reported_as_one_coarse_box():
    frame = np.full((12, 12), 5.)
    frame[0:3, 0:3] = 1.
    frame[9:12, 9:12] = 0.5
    result = depth.detect_nearest_obstacle(frame, min_pixels=10)
    assert result['bbox_xyxy'] == [0, 0, 12, 12]
    assert result['pixel_count'] == 18
    assert 'multiple nearby objects may merge' in result['limitations']


@pytest.mark.parametrize('frame', [[], np.zeros((2, 2, 2))])
def test_invalid_frame_shape(frame):
    with pytest.raises(ValueError):
        depth.detect_nearest_obstacle(frame)
