"""Link-domain fault rig: the real PX4SITLVehicle/MavlinkLink code against the
mock autopilot, on a deterministic clock, with packet loss and disconnects.

Models the *host-side* behaviour only (parser, supervision, reconnect). It says
nothing about radio/UART physical-layer errors or PX4's own link handling.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Sequence

from px4_offboard.comms.clock import FakeClock
from px4_offboard.comms.link import ReconnectPolicy
from px4_offboard.comms.serial_transport import MockSerialDevice, SerialConfig
from px4_offboard.vehicle.mavlink_vehicle import PX4SITLVehicle
from px4_offboard.vehicle.mock_autopilot import MockAutopilot

from .schema import FaultSpec

LINK_FAULTS = {"mavlink_packet_loss", "comm_disconnect"}


@dataclass(frozen=True)
class LinkRigConfig:
    duration_s: float = 60.0
    seed: int = 1
    tick_s: float = 0.05
    sample_dt_s: float = 0.1
    link_timeout_s: float = 3.0
    retry_period_s: float = 0.5


@dataclass(frozen=True)
class LinkSample:
    t: float
    connected: bool
    alive: bool
    last_rx_age_s: float | None


@dataclass
class LinkTrace:
    config: LinkRigConfig
    samples: list[LinkSample] = field(default_factory=list)
    frame_times: list[float] = field(default_factory=list)  # arrival time of each valid frame
    reconnects: int = 0
    dropped_frames: int = 0
    last_reconnect_s: float | None = None

    def frames_between(self, t0: float, t1: float) -> int:
        return sum(1 for t in self.frame_times if t0 <= t < t1)


class _LossyDevice(MockSerialDevice):
    """Drops whole autopilot->host frames inside loss windows."""

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.loss_probability = lambda t: 0.0
        self.rng = random.Random(0)
        self.dropped = 0

    def inject_rx_after(self, delay_s: float, data: bytes) -> None:
        if self.rng.random() < self.loss_probability(self._clock.now()):
            self.dropped += 1
            return
        super().inject_rx_after(delay_s, data)


class LinkRig:
    def __init__(self, config: LinkRigConfig | None = None) -> None:
        self.config = config or LinkRigConfig()

    def run(self, faults: Sequence[FaultSpec]) -> LinkTrace:
        cfg = self.config
        faults = [f for f in faults if f.type in LINK_FAULTS]
        clock = FakeClock()
        device = _LossyDevice(SerialConfig(device="mock0", baud=921600), clock)
        device.rng = random.Random(cfg.seed)
        loss = [f for f in faults if f.type == "mavlink_packet_loss"]
        device.loss_probability = lambda t: max(
            (f.params["probability"] for f in loss if f.start_s <= t < f.end_s), default=0.0
        )
        autopilot = MockAutopilot(device, clock)
        vehicle = PX4SITLVehicle(
            transport=device,
            clock=clock,
            link_timeout_s=cfg.link_timeout_s,
            policy=ReconnectPolicy(initial_delay_s=0.1, max_delay_s=1.0, multiplier=2.0, max_attempts=3),
        )
        trace = LinkTrace(cfg)
        vehicle.connect()

        outages = [f for f in faults if f.type == "comm_disconnect"]
        unplugged = False
        next_retry = 0.0
        next_sample = 0.0
        last_t = clock.now()
        seen_frames = 0

        while clock.now() < cfg.duration_s:
            clock.advance(cfg.tick_s)
            now = clock.now()
            want_unplugged = any(f.start_s <= now < f.end_s for f in outages)
            if want_unplugged and not unplugged:
                device.unplug()
                unplugged = True
            elif not want_unplugged and unplugged:
                device.plug()
                unplugged = False

            autopilot.step(now - last_t)
            last_t = now
            vehicle.update(0.0)
            frames_now = vehicle.link.stats.rx_frames
            trace.frame_times += [now] * (frames_now - seen_frames)
            seen_frames = frames_now

            if not vehicle.link_health().connected and now >= next_retry:
                if vehicle.reconnect():
                    trace.reconnects += 1
                    trace.last_reconnect_s = vehicle.link.stats.last_reconnect_s
                next_retry = clock.now() + cfg.retry_period_s
                last_t = last_t  # autopilot catches up next tick (it kept running)

            if now >= next_sample:
                next_sample += cfg.sample_dt_s
                health = vehicle.link_health()
                trace.samples.append(
                    LinkSample(round(now, 6), health.connected, health.alive, health.last_rx_age_s)
                )
        trace.dropped_frames = device.dropped
        return trace
