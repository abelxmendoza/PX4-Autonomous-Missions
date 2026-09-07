"""Unit tests for the bounded-drift VIO noise model (vio_noise.py)."""

from px4_offboard.vio_noise import VioDriftConfig, VioDriftModel


def test_seeded_model_is_deterministic():
    cfg = VioDriftConfig()
    a = VioDriftModel(cfg, seed=42)
    b = VioDriftModel(cfg, seed=42)
    for _ in range(50):
        a.step(0.05)
        b.step(0.05)
    assert a.bias_m == b.bias_m
    assert a.apply([1.0, 2.0, 3.0]) == b.apply([1.0, 2.0, 3.0])


def test_drift_stays_within_configured_bound():
    cfg = VioDriftConfig(
        position_std_m=0.05,
        drift_std_m_per_sqrt_s=0.5,  # aggressive walk to try to break the clamp
        drift_revert_rate_hz=0.0,  # no mean-reversion, worst case for the clamp
        max_bias_m=1.0,
    )
    model = VioDriftModel(cfg, seed=7)
    for _ in range(5000):
        bias = model.step(0.05)
        assert all(abs(v) <= cfg.max_bias_m + 1e-9 for v in bias)


def test_zero_dt_is_a_no_op():
    model = VioDriftModel(VioDriftConfig(), seed=1)
    model.step(1.0)
    before = model.bias_m
    assert model.step(0.0) == before
    assert model.step(-0.5) == before


def test_apply_adds_bias_and_noise_on_top_of_truth():
    cfg = VioDriftConfig(position_std_m=0.0, drift_std_m_per_sqrt_s=0.0)
    model = VioDriftModel(cfg, seed=3)
    # No drift and no measurement noise configured: apply() must return the
    # true position unchanged (regression guard for accidental bias terms).
    assert model.apply([10.0, -5.0, 2.0]) == [10.0, -5.0, 2.0]


def test_reported_variance_grows_with_accumulated_bias():
    cfg = VioDriftConfig(
        position_std_m=0.05,
        drift_std_m_per_sqrt_s=0.3,
        drift_revert_rate_hz=0.0,
        max_bias_m=2.0,
    )
    model = VioDriftModel(cfg, seed=11)
    base_variance = model.reported_variance()
    assert base_variance == [cfg.position_std_m**2] * 3

    for _ in range(200):
        model.step(0.1)
    drifted_variance = model.reported_variance()

    # At least one axis must reflect the accumulated bias as extra reported
    # uncertainty — a fixed covariance regardless of actual drift is exactly
    # the dishonest-confidence bug this model exists to fix.
    assert any(
        drifted > base for drifted, base in zip(drifted_variance, base_variance)
    )
    assert any(abs(b) > 1e-6 for b in model.bias_m)
