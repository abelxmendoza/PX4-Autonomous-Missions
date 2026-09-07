"""Unit tests for GPS-denied Pass 1 localization scaffolding."""

from px4_offboard.localization_logic import (
    DEFAULT_GPS_DENIED_ZONE,
    DEFAULT_GPS_FAILURE_EXIT_DWELL_S,
    GpsDeniedZone,
    LocalizationEvent,
    LocalizationInputs,
    LocalizationPhase,
    LocalizationSource,
    LocalizationStateMachine,
    classify_source,
    gps_failure_desired,
    policy_requires_failsafe,
)


def test_zone_containment_aabb():
    zone = GpsDeniedZone(
        north_min=18.0,
        north_max=28.0,
        east_min=-5.0,
        east_max=5.0,
        down_min=-12.0,
        down_max=0.5,
    )
    assert zone.contains(23.0, 0.0, -5.0)
    assert not zone.contains(10.0, 0.0, -5.0)
    assert not zone.contains(23.0, 8.0, -5.0)
    assert not zone.contains(23.0, 0.0, -15.0)
    assert DEFAULT_GPS_DENIED_ZONE.contains(20.0, 0.0, -5.0)


def test_classify_source_priority():
    assert (
        classify_source(
            gps_injected_deny=True,
            dead_reckoning=True,
            raw_gps_xy_valid=True,
            non_gps_healthy=False,
        )
        == LocalizationSource.GPS_DENIED_INJECTED
    )
    assert (
        classify_source(
            gps_injected_deny=False,
            dead_reckoning=True,
            raw_gps_xy_valid=True,
            non_gps_healthy=False,
        )
        == LocalizationSource.DEAD_RECKONING
    )
    assert (
        classify_source(
            gps_injected_deny=False,
            dead_reckoning=False,
            raw_gps_xy_valid=True,
            non_gps_healthy=False,
        )
        == LocalizationSource.GPS
    )
    assert (
        classify_source(
            gps_injected_deny=False,
            dead_reckoning=False,
            raw_gps_xy_valid=False,
            non_gps_healthy=False,
        )
        == LocalizationSource.UNKNOWN
    )


def test_policy_failsafe_hold_and_land_only():
    assert policy_requires_failsafe("hold", True, False)
    assert policy_requires_failsafe("land", True, False)
    assert not policy_requires_failsafe("continue", True, False)
    assert not policy_requires_failsafe("hold", True, True)
    assert not policy_requires_failsafe("hold", False, False)


def test_transition_sequence_inject_then_restore():
    sm = LocalizationStateMachine()
    # Outside zone, healthy GPS
    snap = sm.update(
        LocalizationInputs(
            north_m=5.0,
            east_m=-13.0,
            down_m=-5.0,
            gps_xy_valid=True,
            inject_deny=True,
            action="continue",
        )
    )
    assert snap.phase == LocalizationPhase.GPS_OK
    assert not snap.in_zone
    assert not snap.gps_injected_deny
    assert snap.source == LocalizationSource.GPS

    # Enter denied zone with inject
    snap = sm.update(
        LocalizationInputs(
            north_m=23.0,
            east_m=0.0,
            down_m=-5.0,
            gps_xy_valid=True,
            inject_deny=True,
            action="continue",
        )
    )
    assert snap.in_zone
    assert snap.gps_injected_deny
    assert snap.source == LocalizationSource.GPS_DENIED_INJECTED
    assert not snap.gps_xy_valid  # effective invalid under inject
    assert snap.raw_gps_xy_valid
    assert snap.event in (
        LocalizationEvent.ENTER_DENIED_ZONE,
        LocalizationEvent.GPS_INVALID,
    )
    assert snap.phase in (
        LocalizationPhase.IN_DENIED_ZONE,
        LocalizationPhase.GPS_INVALID,
    )

    # Leave zone → restored
    snap = sm.update(
        LocalizationInputs(
            north_m=36.0,
            east_m=8.0,
            down_m=-5.0,
            gps_xy_valid=True,
            inject_deny=True,
            action="continue",
        )
    )
    assert not snap.in_zone
    assert not snap.gps_injected_deny
    assert snap.phase == LocalizationPhase.GPS_OK
    assert snap.event == LocalizationEvent.GPS_RESTORED
    assert snap.source == LocalizationSource.GPS


def test_inject_vs_raw_gps_without_inject():
    sm = LocalizationStateMachine()
    snap = sm.update(
        LocalizationInputs(
            north_m=23.0,
            east_m=0.0,
            down_m=-5.0,
            gps_xy_valid=True,
            inject_deny=False,
            action="hold",
        )
    )
    assert snap.in_zone
    assert not snap.gps_injected_deny
    assert snap.gps_xy_valid
    assert snap.source == LocalizationSource.GPS
    assert snap.phase == LocalizationPhase.IN_DENIED_ZONE
    assert snap.event == LocalizationEvent.ENTER_DENIED_ZONE
    assert not snap.failsafe


def test_failsafe_policy_on_inject_hold():
    sm = LocalizationStateMachine()
    snap = sm.update(
        LocalizationInputs(
            north_m=23.0,
            east_m=0.0,
            down_m=-5.0,
            gps_xy_valid=True,
            inject_deny=True,
            action="hold",
            non_gps_healthy=False,
        )
    )
    assert snap.failsafe
    assert snap.phase == LocalizationPhase.LOC_FAILSAFE
    assert snap.event == LocalizationEvent.LOC_FAILSAFE
    assert snap.failsafe_reason is not None

    # Sticky
    snap2 = sm.update(
        LocalizationInputs(
            north_m=5.0,
            east_m=0.0,
            down_m=-5.0,
            gps_xy_valid=True,
            inject_deny=True,
            action="hold",
        )
    )
    assert snap2.phase == LocalizationPhase.LOC_FAILSAFE
    assert snap2.failsafe


def test_non_gps_healthy_avoids_failsafe():
    sm = LocalizationStateMachine()
    snap = sm.update(
        LocalizationInputs(
            north_m=23.0,
            east_m=0.0,
            down_m=-5.0,
            gps_xy_valid=True,
            inject_deny=True,
            action="land",
            non_gps_healthy=True,
        )
    )
    assert not snap.failsafe
    assert snap.phase == LocalizationPhase.NON_GPS_ACTIVE
    assert snap.event == LocalizationEvent.NON_GPS_ACTIVE
    assert snap.source == LocalizationSource.VIO


def test_disabled_skips_zone():
    sm = LocalizationStateMachine()
    snap = sm.update(
        LocalizationInputs(
            north_m=23.0,
            east_m=0.0,
            down_m=-5.0,
            gps_xy_valid=True,
            inject_deny=True,
            action="hold",
            enabled=False,
        )
    )
    assert not snap.in_zone
    assert not snap.gps_injected_deny
    assert snap.phase == LocalizationPhase.GPS_OK
    assert not snap.failsafe


def test_gps_failure_waits_for_ev_before_inject():
    active, since = gps_failure_desired(
        in_zone=True,
        ev_pos_fused=False,
        currently_active=False,
        outside_since=None,
        now=10.0,
    )
    assert not active
    assert since is None


def test_gps_failure_injects_when_in_zone_with_ev():
    active, since = gps_failure_desired(
        in_zone=True,
        ev_pos_fused=True,
        currently_active=False,
        outside_since=None,
        now=10.0,
    )
    assert active
    assert since is None


def test_gps_failure_stays_latched_in_zone_if_ev_drops():
    active, since = gps_failure_desired(
        in_zone=True,
        ev_pos_fused=False,
        currently_active=True,
        outside_since=None,
        now=10.0,
    )
    assert active
    assert since is None


def test_gps_failure_exit_hysteresis_ignores_south_face_and_home_snap():
    dwell = DEFAULT_GPS_FAILURE_EXIT_DWELL_S
    north_max = DEFAULT_GPS_DENIED_ZONE.north_max

    # South-face flicker (N≈15) must not restore GNSS
    active, since = gps_failure_desired(
        in_zone=False,
        ev_pos_fused=True,
        currently_active=True,
        outside_since=None,
        now=20.0,
        exit_dwell_s=dwell,
        north_m=14.0,
        north_max=north_max,
    )
    assert active
    assert since is None

    # EKF snap toward home must not restore GNSS
    active, since = gps_failure_desired(
        in_zone=False,
        ev_pos_fused=True,
        currently_active=True,
        outside_since=None,
        now=20.5,
        exit_dwell_s=dwell,
        north_m=0.5,
        north_max=north_max,
    )
    assert active
    assert since is None

    # Past the far face starts the exit timer
    active, since = gps_failure_desired(
        in_zone=False,
        ev_pos_fused=True,
        currently_active=True,
        outside_since=None,
        now=21.0,
        exit_dwell_s=dwell,
        north_m=north_max + 2.5,
        north_max=north_max,
    )
    assert active
    assert since == 21.0
    active, since = gps_failure_desired(
        in_zone=False,
        ev_pos_fused=True,
        currently_active=True,
        outside_since=since,
        now=21.0 + dwell,
        exit_dwell_s=dwell,
        north_m=north_max + 3.0,
        north_max=north_max,
    )
    assert not active
    assert since is None
