"""Bounded random-walk drift model for simulated VIO position noise.

``vio_bridge.py`` used to publish exact Gazebo ground-truth position as
"visual odometry" with a fixed, sensor-spec covariance that never actually
matched the (zero) error in the data. That meant the GPS-denied pipeline
was never exercised against a position source that could plausibly be
wrong — only against perfect truth wearing a VIO costume.

Real VIO drifts: it integrates small per-frame errors into a slowly
wandering bias between re-localizations, instead of holding a fixed error
forever. This model adds a mean-reverting random walk (clamped to
``max_bias_m``, since real VIO re-anchors rather than diverging forever) on
top of white measurement noise, and reports a covariance that grows with
the actual accumulated bias so the number PX4's EKF sees is honest.
"""

from __future__ import annotations

import random
from dataclasses import dataclass


@dataclass(frozen=True)
class VioDriftConfig:
    position_std_m: float = 0.05
    drift_std_m_per_sqrt_s: float = 0.02
    drift_revert_rate_hz: float = 0.05
    max_bias_m: float = 1.5


class VioDriftModel:
    """Per-axis (e.g. NED) bounded random-walk position drift."""

    def __init__(self, config: VioDriftConfig, seed: int | None = None):
        self._cfg = config
        self._rng = random.Random(seed)
        self._bias = [0.0, 0.0, 0.0]

    @property
    def bias_m(self) -> list[float]:
        return list(self._bias)

    def step(self, dt_s: float) -> list[float]:
        """Advance the drift by ``dt_s`` and return the updated bias (m)."""
        if dt_s <= 0.0:
            return self.bias_m
        cfg = self._cfg
        for i in range(3):
            reverting = -cfg.drift_revert_rate_hz * self._bias[i] * dt_s
            walk = cfg.drift_std_m_per_sqrt_s * dt_s**0.5 * self._rng.gauss(0.0, 1.0)
            updated = self._bias[i] + reverting + walk
            self._bias[i] = max(-cfg.max_bias_m, min(cfg.max_bias_m, updated))
        return self.bias_m

    def apply(self, true_position: list[float]) -> list[float]:
        """Return ``true_position`` plus current bias and fresh white noise."""
        noise_std = self._cfg.position_std_m
        return [
            true_position[i] + self._bias[i] + self._rng.gauss(0.0, noise_std)
            for i in range(3)
        ]

    def reported_variance(self) -> list[float]:
        """Variance estimate that grows honestly with accumulated drift."""
        base = self._cfg.position_std_m**2
        return [base + self._bias[i] ** 2 for i in range(3)]
