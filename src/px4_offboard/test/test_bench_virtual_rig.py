"""Virtual test-rig physics and fault behavior."""
from __future__ import annotations

import pytest

from px4_offboard.bench.virtual_rig import FaultWindow, RigConfig, VirtualTestRig


def run(rig: VirtualTestRig, command_pct: float, n: int):
    return [rig.step(command_pct) for _ in range(n)]


def test_thrust_rises_and_settles_toward_the_modeled_curve_at_full_command():
    cfg = RigConfig(dt_s=0.05, seed=1)
    rig = VirtualTestRig(cfg)
    samples = run(rig, 100.0, 400)  # 20 s, well past the RPM lag time constant
    expected_steady = cfg.thrust_k * (cfg.max_rpm / 1000.0) ** 2
    assert samples[0].thrust_n < 0.05 * expected_steady  # barely moved after one tick
    assert samples[-1].thrust_n == pytest.approx(expected_steady, rel=0.02)
    # monotonic climb (noise-free field, so this must hold exactly)
    assert all(b.rpm >= a.rpm - 1e-9 for a, b in zip(samples, samples[1:]))


def test_thrust_is_never_negative_across_a_throttle_sweep():
    rig = VirtualTestRig(RigConfig(seed=3))
    samples = [rig.step(p) for p in [0, 10, 40, 80, 100, 60, 20, 0, 0, 0] for _ in range(20)]
    assert all(s.thrust_n >= 0.0 for s in samples)


def test_command_is_clamped_to_0_100():
    rig = VirtualTestRig()
    assert rig.step(150.0).command_pct == 100.0
    assert rig.step(-20.0).command_pct == 0.0


def test_load_cell_dropout_reads_zero_only_during_its_window():
    cfg = RigConfig(dt_s=0.1, seed=2)
    rig = VirtualTestRig(cfg)
    rig.add_fault(FaultWindow("load_cell_dropout", start_s=1.0, duration_s=1.0))
    samples = run(rig, 100.0, 30)  # 3 s
    before = [s for s in samples if s.t < 1.0]
    during = [s for s in samples if 1.0 <= s.t < 2.0]
    after = [s for s in samples if s.t >= 2.0]
    assert all(s.load_cell_n != 0.0 for s in before[2:])  # past the initial near-zero ramp-up
    assert all(s.load_cell_n == 0.0 and s.fault_active == "load_cell_dropout" for s in during)
    assert all(s.fault_active == "" for s in after)
    assert all(s.load_cell_n != 0.0 for s in after)


def test_frozen_temp_sensor_latches_a_single_value_for_the_whole_window():
    cfg = RigConfig(dt_s=0.1, seed=4)
    rig = VirtualTestRig(cfg)
    rig.add_fault(FaultWindow("frozen_temp_sensor", start_s=0.5, duration_s=2.0))
    samples = run(rig, 100.0, 40)  # 4 s, well into the thermal climb
    during = [s for s in samples if 0.5 <= s.t < 2.5]
    assert len({s.temp_c for s in during}) == 1
    after = [s for s in samples if s.t >= 2.5]
    assert len({s.temp_c for s in after}) > 1  # unfreezes and keeps climbing/varying


def test_voltage_brownout_overrides_the_reading_to_the_declared_value():
    rig = VirtualTestRig(RigConfig(seed=5))
    rig.add_fault(FaultWindow("voltage_brownout", start_s=0.0, duration_s=0.5, params={"voltage_v": 7.4}))
    sample = rig.step(100.0)
    assert sample.voltage_v == 7.4
    assert sample.fault_active == "voltage_brownout"


def test_unknown_fault_type_is_rejected_at_construction():
    with pytest.raises(ValueError):
        FaultWindow("warp_core_breach", start_s=0.0, duration_s=1.0)


def test_same_seed_is_deterministic():
    a = run(VirtualTestRig(RigConfig(seed=9)), 70.0, 50)
    b = run(VirtualTestRig(RigConfig(seed=9)), 70.0, 50)
    assert [s.load_cell_n for s in a] == [s.load_cell_n for s in b]
