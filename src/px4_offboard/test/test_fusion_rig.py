"""The offline fusion rig: a deterministic IMU + VO source feeding the real EKF."""
from __future__ import annotations

import numpy as np
import pytest

from px4_offboard.faults.fusion_rig import FusionRig, RigConfig


def test_baseline_run_tracks_truth_and_stays_healthy():
    trace = FusionRig(RigConfig(duration_s=40.0)).run([])
    healthy = np.mean([s.healthy for s in trace.samples if s.t > 2.0])
    assert healthy > 0.99
    assert max(s.err_horiz_m for s in trace.samples) < 1.0
    assert trace.samples[-1].err_horiz_m < 0.5


def test_runs_are_exactly_reproducible():
    a = FusionRig(RigConfig(duration_s=15.0, seed=5)).run([])
    b = FusionRig(RigConfig(duration_s=15.0, seed=5)).run([])
    assert [s.err_horiz_m for s in a.samples] == [s.err_horiz_m for s in b.samples]


def test_different_seeds_give_different_noise_but_similar_accuracy():
    a = FusionRig(RigConfig(duration_s=15.0, seed=5)).run([])
    b = FusionRig(RigConfig(duration_s=15.0, seed=6)).run([])
    assert [s.err_horiz_m for s in a.samples] != [s.err_horiz_m for s in b.samples]
    assert abs(a.samples[-1].err_horiz_m - b.samples[-1].err_horiz_m) < 0.5


def test_without_any_vo_the_estimate_is_unhealthy_and_drifts():
    from px4_offboard.faults.schema import load_scenario_text

    sc = load_scenario_text("duration_s: 40\nfaults:\n  - {type: vo_dropout, start_s: 5, duration_s: 30}")
    trace = FusionRig(RigConfig(duration_s=40.0)).run(sc.faults)
    during = [s for s in trace.samples if 6.0 < s.t < 35.0]
    assert not any(s.healthy for s in during)
    assert during[-1].err_horiz_m > 3.0 * trace.samples[int(5.0 / RigConfig().sample_dt_s)].err_horiz_m
