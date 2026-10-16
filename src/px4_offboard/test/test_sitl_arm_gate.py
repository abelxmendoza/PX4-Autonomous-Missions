"""Readiness must be based on fresh observations, never missing/stale telemetry."""
import asyncio
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("mavsdk")
spec = importlib.util.spec_from_file_location(
    "integrated_seeding_sitl", Path(__file__).resolve().parents[3] / "tools/integrated_seeding_sitl.py")
sitl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sitl)


@pytest.mark.parametrize("mode", ["missing_rate", "stale_rate", "stale_health", "nan_rate", "healthy", "bad_rate"])
def test_gate_requires_fresh_finite_health_and_rate(monkeypatch, mode):
    clock = [100.0]
    watch = sitl._ArmWatch()
    watch.health = SimpleNamespace(is_global_position_ok=True, is_home_position_ok=True, is_armable=True)
    watch.health_at = watch.rate_at = clock[0]
    watch.rest_rate = 0.0
    if mode == "missing_rate":
        watch.rest_rate = None
    elif mode == "nan_rate":
        watch.rest_rate = float("nan")
    elif mode == "stale_rate":
        watch.rate_at -= 10
    elif mode == "stale_health":
        watch.health_at -= 10
    elif mode == "bad_rate":
        watch.rest_rate = 0.1
        watch.rate_high_since = clock[0] - sitl.REST_RATE_HOLD_S

    async def advance(dt):
        clock[0] += dt
        if mode in ("healthy", "bad_rate"):
            watch.health_at = watch.rate_at = clock[0]

    monkeypatch.setattr(sitl.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(sitl.asyncio, "sleep", advance)
    restart, problem = asyncio.run(sitl._wait_until_armable(watch, 104.0))
    assert (problem is None) == (mode == "healthy")
    assert restart == (mode == "bad_rate")


def test_estimator_restart_discards_old_health_and_rates():
    watch = sitl._ArmWatch()
    watch.health = object()
    watch.health_at = watch.rate_at = 100
    watch.rest_rate = 0.1
    watch.rate_high_since = 90
    watch.reset()
    assert watch.health is None and watch.rest_rate is None
    assert watch.health_at == watch.rate_at == -float("inf")
    assert watch.rate_high_since is None
