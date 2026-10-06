"""Link-domain rig: PX4SITLVehicle against a mock autopilot, with link faults."""
from __future__ import annotations

import pytest

from px4_offboard.faults.link_rig import LinkRig, LinkRigConfig
from px4_offboard.faults.schema import load_scenario_text


def faults(body: str):
    return load_scenario_text("duration_s: 40\nfaults:\n  - " + body).faults


def test_baseline_link_is_alive_and_delivers_the_nominal_frame_rate():
    trace = LinkRig(LinkRigConfig(duration_s=20.0)).run([])
    assert all(s.alive for s in trace.samples if s.t > 4.0)
    assert trace.frames_between(5.0, 15.0) == pytest.approx(10 * (20 + 20 + 1), rel=0.05)


def test_unplug_is_detected_at_once_and_recovery_follows_the_replug():
    trace = LinkRig(LinkRigConfig(duration_s=30.0)).run(
        faults("{type: comm_disconnect, start_s: 10, duration_s: 5}")
    )
    during = [s for s in trace.samples if 10.3 <= s.t < 15.0]
    assert not any(s.connected for s in during)
    first_down = next(s.t for s in trace.samples if s.t >= 10.0 and not s.connected)
    assert first_down - 10.0 < 0.3
    after = [s for s in trace.samples if s.t >= 17.0]
    assert all(s.alive and s.connected for s in after)
    assert trace.reconnects == 1


def test_total_packet_loss_makes_the_link_go_stale_then_recover():
    trace = LinkRig(LinkRigConfig(duration_s=30.0)).run(
        faults("{type: mavlink_packet_loss, start_s: 10, duration_s: 8, probability: 1.0}")
    )
    assert not any(s.alive for s in trace.samples if 14.0 < s.t < 18.0)  # link_timeout 3 s
    assert all(s.alive for s in trace.samples if s.t > 20.0)
    assert trace.reconnects == 0  # the cable never came out


def test_partial_packet_loss_keeps_the_link_alive_and_reduces_throughput():
    cfg = LinkRigConfig(duration_s=30.0)
    base = LinkRig(cfg).run([])
    lossy = LinkRig(cfg).run(faults("{type: mavlink_packet_loss, start_s: 10, duration_s: 10, probability: 0.3}"))
    assert all(s.alive for s in lossy.samples if s.t > 4.0)
    ratio = lossy.frames_between(10, 20) / base.frames_between(10, 20)
    assert 0.55 < ratio < 0.85
    assert lossy.dropped_frames > 0


def test_same_seed_same_losses():
    cfg = LinkRigConfig(duration_s=20.0, seed=4)
    f = faults("{type: mavlink_packet_loss, start_s: 5, duration_s: 5, probability: 0.4}")
    assert LinkRig(cfg).run(f).dropped_frames == LinkRig(cfg).run(f).dropped_frames
