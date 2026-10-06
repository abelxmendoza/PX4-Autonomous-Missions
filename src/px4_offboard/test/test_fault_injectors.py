"""Sensor-stream injectors, one fault at a time, fully seeded."""
from __future__ import annotations

import numpy as np
import pytest

from px4_offboard.faults.injectors import SensorFaultInjector
from px4_offboard.faults.schema import load_scenario_text


def inj(body: str, seed: int = 1) -> SensorFaultInjector:
    sc = load_scenario_text("duration_s: 100\nfaults:\n" + body)
    return SensorFaultInjector(sc.faults, seed=seed)


V = np.array([1.0, 2.0, 3.0])


def test_outside_the_window_everything_passes_through_unchanged():
    i = inj("  - {type: vo_dropout, start_s: 10, duration_s: 5}")
    out = i.process("vo", 9.9, 9.9, V)
    assert len(out) == 1 and out[0].deliver_t == 9.9 and np.array_equal(out[0].value, V)
    assert len(i.process("vo", 15.0, 15.0, V)) == 1  # window is [start, end)


def test_vo_dropout_drops_vo_but_not_imu():
    i = inj("  - {type: vo_dropout, start_s: 10, duration_s: 5}")
    assert i.process("vo", 12.0, 12.0, V) == []
    assert len(i.process("imu", 12.0, 12.0, V)) == 1


def test_dropped_messages_follow_the_configured_probability():
    i = inj("  - {type: dropped_messages, start_s: 0, duration_s: 100, probability: 0.3, sensor: imu}")
    kept = sum(len(i.process("imu", t * 0.01, t * 0.01, V)) for t in range(5000))
    assert 0.65 < kept / 5000 < 0.75
    assert len(i.process("vo", 1.0, 1.0, V)) == 1  # other sensor untouched


def test_probability_zero_and_one_are_exact():
    never = inj("  - {type: dropped_messages, start_s: 0, duration_s: 10, probability: 0.0}")
    always = inj("  - {type: dropped_messages, start_s: 0, duration_s: 10, probability: 1.0}")
    assert all(len(never.process("vo", t, t, V)) == 1 for t in np.arange(0, 10, 0.2))
    assert all(always.process("vo", t, t, V) == [] for t in np.arange(0, 10, 0.2))


def test_delay_postpones_delivery_but_keeps_the_original_stamp():
    i = inj("  - {type: delayed_messages, start_s: 0, duration_s: 10, delay_s: 0.4}")
    d = i.process("vo", 3.0, 3.0, V)[0]
    assert d.deliver_t == pytest.approx(3.4) and d.stamp_s == 3.0


def test_timestamp_jitter_perturbs_stamps_with_the_requested_spread():
    i = inj("  - {type: timestamp_jitter, start_s: 0, duration_s: 100, std_s: 0.002, sensor: imu}")
    offsets = [i.process("imu", t * 0.01, t * 0.01, V)[0].stamp_s - t * 0.01 for t in range(3000)]
    assert abs(np.mean(offsets)) < 2e-4
    assert np.std(offsets) == pytest.approx(0.002, rel=0.15)


def test_corruption_adds_a_spike_of_the_requested_magnitude():
    i = inj("  - {type: corrupted_measurement, start_s: 0, duration_s: 10, probability: 1.0, magnitude: 20.0}")
    d = i.process("vo", 1.0, 1.0, V)[0]
    assert np.linalg.norm(d.value - V) == pytest.approx(20.0)
    assert d.corrupted


def test_frozen_sensor_repeats_the_first_value_seen_in_the_window():
    i = inj("  - {type: frozen_sensor, start_s: 5, duration_s: 5}")
    assert np.array_equal(i.process("vo", 4.9, 4.9, V)[0].value, V)
    first = i.process("vo", 5.0, 5.0, V * 2)[0]
    later = i.process("vo", 6.0, 6.0, V * 9)[0]
    assert np.array_equal(first.value, V * 2) and np.array_equal(later.value, V * 2)
    assert later.stamp_s == 6.0  # still looks like fresh data
    assert np.array_equal(i.process("vo", 10.0, 10.0, V * 9)[0].value, V * 9)  # thaws


def test_same_seed_gives_the_same_faults_and_different_seed_does_not():
    body = "  - {type: dropped_messages, start_s: 0, duration_s: 100, probability: 0.5}"
    a, b = inj(body, 7), inj(body, 7)
    seq = lambda i: [len(i.process("vo", t, t, V)) for t in np.arange(0, 20, 0.2)]  # noqa: E731
    assert seq(a) == seq(b)
    assert seq(inj(body, 7)) != seq(inj(body, 8))


def test_resets_are_reported_once_when_their_time_passes():
    i = inj("  - {type: estimator_reset, start_s: 10}")
    assert i.resets_due(0.0, 9.99) == 0
    assert i.resets_due(9.99, 10.01) == 1
    assert i.resets_due(10.01, 20.0) == 0
