"""Barometric, AMSL, AGL, and local altitude stay in separate frames."""

from __future__ import annotations

import pytest

from px4_offboard.altitude_reference import (
    AltitudeBand,
    AltitudeFrame,
    AltitudeMonitor,
    Px4AltitudeSnapshot,
)


def _field_snapshot(**overrides) -> Px4AltitudeSnapshot:
    """4500 m field elevation, vehicle 72 m above it: 4572 m AMSL, 72 m local and AGL."""
    values = dict(
        time_s=10.0,
        baro_alt_m=4572.0,
        local_z_m=-72.0,
        local_vz_mps=-1.5,
        z_valid=True,
        vz_valid=True,
        dist_bottom_m=72.0,
        dist_bottom_valid=True,
        ref_alt_m=4500.0,
        gps_alt_amsl_m=None,
        baro_fault=False,
    )
    values.update(overrides)
    return Px4AltitudeSnapshot(**values)


def test_each_frame_uses_its_own_px4_source():
    report = AltitudeMonitor().update(_field_snapshot(), 10.0)

    assert report.barometric.altitude_m == pytest.approx(4572.0)
    assert report.barometric.source == "vehicle_air_data.baro_alt_meter"
    assert report.local.altitude_m == pytest.approx(72.0)
    assert report.local.source == "vehicle_local_position.z"
    assert report.amsl.altitude_m == pytest.approx(4572.0)
    assert report.amsl.source == "vehicle_local_position.ref_alt"
    assert report.agl.altitude_m == pytest.approx(72.0)
    assert report.agl.source == "vehicle_local_position.dist_bottom"
    assert report.vertical_up_mps == pytest.approx(1.5)
    assert report.fresh and report.estimator_ok
    assert report.baro_amsl_delta_m == pytest.approx(0.0)


def test_gps_amsl_is_reported_separately_from_the_local_origin():
    report = AltitudeMonitor().update(_field_snapshot(gps_alt_amsl_m=4580.0), 10.0)
    assert report.amsl.altitude_m == pytest.approx(4580.0)
    assert report.amsl.source == "sensor_gps.alt"
    assert report.local.altitude_m == pytest.approx(72.0)


def test_agl_stays_unavailable_without_a_distance_to_ground():
    report = AltitudeMonitor().update(
        _field_snapshot(dist_bottom_m=None, dist_bottom_valid=False),
        10.0,
    )
    assert report.agl.altitude_m is None
    assert report.agl.reason == "AGL unavailable"
    assert report.local.altitude_m == pytest.approx(72.0)

    band = AltitudeBand(AltitudeFrame.AGL, 50.0, 100.0)
    decision = band.decide(report)
    assert decision.authorized is False
    assert decision.reason == "AGL unavailable"
    assert decision.inside_band is None


def test_an_amsl_release_band_rejects_the_same_numbers_read_as_local_height():
    report = AltitudeMonitor().update(_field_snapshot(), 10.0)
    amsl = AltitudeBand(AltitudeFrame.AMSL, 4422.0, 4722.0).decide(report)
    local = AltitudeBand(AltitudeFrame.LOCAL, 4422.0, 4722.0).decide(report)

    assert amsl.authorized is True
    assert amsl.reason == ""
    assert amsl.inside_band is True
    assert local.authorized is False
    assert local.reason == "local altitude 72.0 m is below the 4422.0 m floor"


def test_a_barometer_fault_removes_only_the_barometric_frame():
    report = AltitudeMonitor().update(_field_snapshot(baro_fault=True), 10.0)
    assert report.barometric.reason == "barometer fault"
    assert report.local.altitude_m == pytest.approx(72.0)
    assert report.baro_amsl_delta_m is None

    decision = AltitudeBand(AltitudeFrame.BAROMETRIC, 4500.0, 4700.0).decide(report)
    assert decision.authorized is False
    assert decision.reason == "barometer fault"
    assert decision.cross_check == "not_compared"


def test_stale_and_future_samples_are_not_a_release_reference():
    monitor = AltitudeMonitor(max_age_s=0.5)
    stale = monitor.update(_field_snapshot(time_s=0.0), 2.0)
    assert stale.fresh is False
    decision = AltitudeBand(AltitudeFrame.BAROMETRIC, 4500.0, 4700.0).decide(stale)
    assert decision.authorized is False
    assert decision.reason == "altitude sample is stale"
    assert decision.altitude_m == pytest.approx(4572.0)

    future = monitor.update(_field_snapshot(time_s=5.0), 4.0)
    assert AltitudeBand(AltitudeFrame.LOCAL, 50.0, 100.0).decide(future).reason == (
        "altitude sample is from the future"
    )


def test_vertical_velocity_has_to_be_valid_before_a_release():
    report = AltitudeMonitor().update(_field_snapshot(vz_valid=False), 10.0)
    decision = AltitudeBand(AltitudeFrame.LOCAL, 50.0, 100.0).decide(report)
    assert decision.authorized is False
    assert decision.reason == "vertical velocity invalid"
    assert decision.estimator_ok is False


def test_barometric_and_amsl_disagreement_blocks_both_absolute_frames():
    # Baro sits inside the release band. The local origin puts AMSL 100 m away.
    report = AltitudeMonitor().update(_field_snapshot(baro_alt_m=4572.0, ref_alt_m=4600.0), 10.0)
    assert report.amsl.altitude_m == pytest.approx(4672.0)
    assert report.baro_amsl_delta_m == pytest.approx(-100.0)

    baro = AltitudeBand(AltitudeFrame.BAROMETRIC, 4500.0, 4600.0).decide(report)
    amsl = AltitudeBand(AltitudeFrame.AMSL, 4600.0, 4700.0).decide(report)
    local = AltitudeBand(AltitudeFrame.LOCAL, 50.0, 100.0).decide(report)

    assert baro.inside_band is True
    assert baro.authorized is False
    assert baro.cross_check == "diverge"
    assert baro.reason == "barometric altitude and AMSL differ by 100.0 m"
    assert amsl.authorized is False
    assert amsl.reason == baro.reason
    assert local.authorized is True
    assert local.cross_check == "diverge"


def test_the_same_snapshot_produces_the_same_decision_record():
    snapshot = _field_snapshot()
    band = AltitudeBand(AltitudeFrame.AMSL, 4422.0, 4722.0)
    first = band.decide(AltitudeMonitor().update(snapshot, 10.0)).as_dict()
    second = band.decide(AltitudeMonitor().update(snapshot, 10.0)).as_dict()
    assert first == second
    assert first["authorized"] is True
    assert first["source"] == "vehicle_local_position.ref_alt"
