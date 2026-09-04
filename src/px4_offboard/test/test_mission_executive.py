"""Unit tests for the resource-aware mission executive."""

from px4_offboard.mission_executive import (
    ExecutiveAction,
    MissionExecutive,
    MissionExecutiveConfig,
    MissionMode,
    ResourceBudgets,
)


def _exec(**overrides) -> MissionExecutive:
    cfg = MissionExecutiveConfig(
        science_waypoints=(2, 4, 6),
        initial_battery=1.0,
        initial_propellant_s=180.0,
        **overrides,
    )
    return MissionExecutive(cfg)


def test_starts_nominal():
    ex = _exec()
    d = ex.evaluate(current_wp_index=0, remaining_waypoints=9)
    assert d.mode is MissionMode.NOMINAL
    assert d.action is ExecutiveAction.CONTINUE


def test_battery_drain_while_moving():
    ex = _exec()
    start = ex.resources.battery_frac
    ex.update(1.0, airborne=True, moving=True, avoiding=False, link_ok=True)
    assert ex.resources.battery_frac < start
    assert ex.resources.propellant_time_s < 180.0


def test_degraded_skips_science_waypoint():
    ex = _exec()
    ex.resources.battery_frac = 0.40  # below degraded threshold 0.45
    d = ex.evaluate(current_wp_index=2, remaining_waypoints=5)
    assert d.mode is MissionMode.DEGRADED
    assert d.action is ExecutiveAction.SKIP_SCIENCE
    assert d.skipped_waypoints == [2]


def test_degraded_does_not_skip_non_science():
    ex = _exec()
    ex.resources.battery_frac = 0.40
    d = ex.evaluate(current_wp_index=1, remaining_waypoints=5)
    assert d.mode is MissionMode.DEGRADED
    assert d.action is ExecutiveAction.CONTINUE


def test_safe_mode_holds():
    ex = _exec()
    ex.resources.battery_frac = 0.20
    d = ex.evaluate(current_wp_index=3, remaining_waypoints=4)
    assert d.mode is MissionMode.SAFE
    assert d.action is ExecutiveAction.HOLD_SAFE


def test_abort_on_critical_battery():
    ex = _exec()
    ex.resources.battery_frac = 0.10
    d = ex.evaluate(current_wp_index=3, remaining_waypoints=4)
    assert d.mode is MissionMode.ABORT
    assert d.action is ExecutiveAction.ABORT_LAND


def test_abort_on_sustained_link_loss():
    ex = _exec()
    for _ in range(10):
        ex.update(1.0, airborne=True, moving=True, avoiding=False, link_ok=False)
    d = ex.evaluate(current_wp_index=3, remaining_waypoints=4)
    assert d.mode is MissionMode.ABORT
    assert "link lost" in d.reason


def test_compute_load_triggers_degraded():
    # compute_load feeds _classify_mode alongside battery/link/propellant
    # (mission_executive.py's DEGRADED/SAFE branches both check it) but had
    # no test coverage. Note: under the default drain-rate constants,
    # update()'s compute EMA converges to ~0.70 steady-state even at max
    # avoid+lidar load, so it can never naturally cross compute_degraded
    # (0.80) through update() alone — hence direct injection here, matching
    # this file's existing convention for the battery/link threshold tests.
    ex = _exec()
    ex.resources.compute_load = 0.85  # above compute_degraded (0.80), below compute_safe (0.92)
    d = ex.evaluate(current_wp_index=1, remaining_waypoints=5)
    assert d.mode is MissionMode.DEGRADED


def test_compute_load_triggers_safe():
    ex = _exec()
    ex.resources.compute_load = 0.95  # above compute_safe (0.92)
    d = ex.evaluate(current_wp_index=1, remaining_waypoints=5)
    assert d.mode is MissionMode.SAFE
    assert d.action is ExecutiveAction.HOLD_SAFE


def test_link_recovers():
    ex = _exec()
    ex.resources.link_quality = 0.2
    ex.update(1.0, airborne=True, moving=False, avoiding=False, link_ok=True)
    assert ex.resources.link_quality > 0.2
    assert ex.resources.link_loss_timer_s == 0.0


def test_mode_history_records_transitions():
    ex = _exec()
    ex.resources.battery_frac = 0.40
    ex.evaluate(current_wp_index=0, remaining_waypoints=5)
    assert ex.mode is MissionMode.DEGRADED
    assert ex._mode_history[-1][0] == "DEGRADED"


def test_disabled_executive_always_continues():
    ex = _exec(enable=False)
    ex.resources.battery_frac = 0.05
    d = ex.evaluate(current_wp_index=2, remaining_waypoints=3)
    assert d.mode is MissionMode.NOMINAL
    assert d.action is ExecutiveAction.CONTINUE


def test_custom_budgets():
    budgets = ResourceBudgets(battery_degraded=0.90)
    ex = MissionExecutive(
        MissionExecutiveConfig(budgets=budgets, science_waypoints=(0,))
    )
    ex.resources.battery_frac = 0.85
    d = ex.evaluate(current_wp_index=0, remaining_waypoints=1)
    assert d.mode is MissionMode.DEGRADED
    assert d.action is ExecutiveAction.SKIP_SCIENCE
