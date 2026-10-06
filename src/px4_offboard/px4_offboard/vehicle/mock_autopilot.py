"""A MAVLink autopilot stand-in on the far end of a mock transport.

Speaks just enough of the protocol (heartbeat, attitude, local position, arm
command + ack, velocity setpoints) for host-side software to be exercised
deterministically. It is NOT PX4: it has no EKF, no mode logic, no failsafes,
and its dynamics are a first-order velocity follower.
"""

from __future__ import annotations

import math

from px4_offboard.comms import messages as m
from px4_offboard.comms.clock import Clock
from px4_offboard.comms.mavlink_frame import (
    MSG_COMMAND_LONG,
    MSG_HEARTBEAT,
    MSG_SET_POSITION_TARGET_LOCAL_NED,
    FrameParser,
    ParseEvent,
    encode_frame,
)
from px4_offboard.comms.serial_transport import MockSerialDevice
from px4_offboard.comms.mavlink_frame import (
    MSG_ATTITUDE,
    MSG_COMMAND_ACK,
    MSG_LOCAL_POSITION_NED,
)


class MockAutopilot:
    def __init__(
        self,
        device: MockSerialDevice,
        clock: Clock,
        heartbeat_hz: float = 1.0,
        telemetry_hz: float = 20.0,
        velocity_tau_s: float = 0.3,
    ) -> None:
        self.device = device
        self._clock = clock
        self._hb_period = 1.0 / heartbeat_hz
        self._tel_period = 1.0 / telemetry_hz
        self._tau = velocity_tau_s
        self._since_hb = self._hb_period  # emit immediately on first step
        self._since_tel = self._tel_period
        self._seq = 0
        self._parser = FrameParser()
        self.armed = False
        self.pos = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.yaw_rad = 0.0
        self._vel_cmd = [0.0, 0.0, 0.0]
        self.arm_result = m.MAV_RESULT_ACCEPTED
        self.respond_to_commands = True
        self.commands_received: list[m.CommandLong] = []
        self.setpoints_received: list[m.SetPositionTarget] = []
        device.on_write = self._on_host_bytes

    # --- host -> autopilot ---------------------------------------------
    def _on_host_bytes(self, data: bytes) -> None:
        for event in self._parser.feed(data):
            if event.kind != ParseEvent.FRAME:
                continue
            frame = event.frame
            if frame.msgid == MSG_COMMAND_LONG:
                cmd = m.unpack_command_long(frame.payload)
                self.commands_received.append(cmd)
                if cmd.command == m.MAV_CMD_COMPONENT_ARM_DISARM:
                    if self.arm_result == m.MAV_RESULT_ACCEPTED:
                        self.armed = cmd.params[0] >= 0.5
                    if self.respond_to_commands:
                        self._send(MSG_COMMAND_ACK, m.pack_command_ack(cmd.command, self.arm_result))
            elif frame.msgid == MSG_SET_POSITION_TARGET_LOCAL_NED:
                sp = m.unpack_set_position_target(frame.payload)
                self.setpoints_received.append(sp)
                self._vel_cmd = [sp.vx, sp.vy, sp.vz]

    # --- autopilot -> host ------------------------------------------------
    def _send(self, msgid: int, payload: bytes) -> None:
        if not self.device.plugged:
            return
        self.device.inject_rx(encode_frame(msgid, payload, seq=self._seq))
        self._seq = (self._seq + 1) & 0xFF

    def step(self, dt: float) -> None:
        target = self._vel_cmd if self.armed else [0.0, 0.0, 0.0]
        alpha = min(1.0, dt / self._tau)
        for i in range(3):
            self.vel[i] += alpha * (target[i] - self.vel[i])
            self.pos[i] += self.vel[i] * dt
        self._since_hb += dt
        self._since_tel += dt
        if self._since_hb >= self._hb_period:
            self._since_hb -= self._hb_period
            base = m.MAV_MODE_FLAG_SAFETY_ARMED | 0x01 if self.armed else 0x01
            self._send(MSG_HEARTBEAT, m.pack_heartbeat(base_mode=base))
        if self._since_tel >= self._tel_period:
            self._since_tel -= self._tel_period
            t_ms = int(self._clock.now() * 1000) & 0xFFFFFFFF
            self._send(MSG_ATTITUDE, m.pack_attitude(t_ms, 0.0, 0.0, self.yaw_rad))
            self._send(
                MSG_LOCAL_POSITION_NED,
                m.pack_local_position_ned(t_ms, *self.pos, *self.vel),
            )
