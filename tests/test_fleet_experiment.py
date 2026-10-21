"""Preflight contract regression tests; no ROS/PX4 installation needed."""
import pytest
from px4_offboard.fleet_experiment import validate_config


def config(n=2):
    return {"scenario": "cooperative_point_visits", "seed": 42,
            "fleet_size": n, "output_dir": "artifacts/fleet/experiment"}


@pytest.mark.parametrize("n", [1, 2, 3, 4])
def test_supported_fleet_sizes(n):
    result = validate_config(config(n))
    assert len(result.vehicles) == n
    assert len({v.vehicle_id for v in result.vehicles}) == n
    assert len({v.mavlink_port for v in result.vehicles}) == n


@pytest.mark.parametrize("bad", [0, 5, True, 2.0, "2", None])
def test_invalid_fleet_size(bad):
    data = config()
    data["fleet_size"] = bad
    with pytest.raises(ValueError):
        validate_config(data)


def test_repeatable():
    assert validate_config(config(4)).as_dict() == validate_config(config(4)).as_dict()


@pytest.mark.parametrize("field,value", [
    ("scenario", "unknown"), ("scenario", []), ("scenario", {}), ("seed", -1), ("seed", True),
    ("output_dir", ""), ("fault_schedule", [{"kind": "dropout"}]),
    ("unexpected", 123),
])
def test_reject_invalid_contract(field, value):
    data = config()
    data[field] = value
    with pytest.raises(ValueError):
        validate_config(data)


@pytest.mark.parametrize("field,value", [
    ("vehicle_id", "wrong"), ("instance", 1), ("system_id", 2),
    ("mavlink_port", 18571), ("spawn_north_m", float("nan")),
])
def test_reject_invalid_vehicle(field, value):
    data = config()
    data["vehicles"] = [vars(v).copy() for v in validate_config(data).vehicles]
    data["vehicles"][1][field] = value
    with pytest.raises(ValueError):
        validate_config(data)


def test_reject_overlap():
    data = config()
    data["vehicles"] = [vars(v).copy() for v in validate_config(data).vehicles]
    data["vehicles"][1]["spawn_north_m"] = data["vehicles"][0]["spawn_north_m"]
    data["vehicles"][1]["spawn_east_m"] = data["vehicles"][0]["spawn_east_m"]
    with pytest.raises(ValueError, match="overlapping"):
        validate_config(data)
