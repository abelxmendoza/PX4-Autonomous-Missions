"""MAVLink-backed vehicles: PX4 SITL and a real PX4 flight controller.

Both share every line of protocol handling in :class:`MavlinkVehicle`; they
differ only in kind, default endpoint, and actuation policy. A hardware
vehicle is telemetry-only unless the caller opts in with
``allow_actuation=True``, so pointing a test at a real controller cannot arm
it by accident.
"""

from __future__ import annotations

from collections import deque

from px4_offboard.comms import messages as m
from px4_offboard.comms.clock import Clock, SystemClock
from px4_offboard.comms.link import LinkDown, LinkState, MavlinkLink, ReconnectPolicy
from px4_offboard.comms.mavlink_frame import (
    MSG_ATTITUDE,
    MSG_COMMAND_ACK,
    MSG_COMMAND_LONG,
    MSG_HEARTBEAT,
    MSG_LOCAL_POSITION_NED,
    MSG_SET_POSITION_TARGET_LOCAL_NED,
    Frame,
)
from px4_offboard.comms.serial_transport import PySerialTransport, SerialConfig, Transport
from px4_offboard.comms.udp_transport import UdpTransport

from .connection import parse_connection
from .interface import (
    ActuationNotPermitted,
    CommandRejected,
    CommandTimeout,
    LinkHealth,
    VehicleError,
    VehicleInterface,
    VehicleKind,
    VehicleState,
)

RATE_WINDOW_S = 2.0


def transport_for(connection: str) -> Transport:
    spec = parse_connection(connection)
    if spec.kind == "serial":
        return PySerialTransport(SerialConfig(device=spec.device, baud=spec.baud))
    return UdpTransport(spec)


class MavlinkVehicle(VehicleInterface):
    default_connection: str | None = None
    default_allow_actuation = True

    def __init__(
        self,
        connection: str | None = None,
        *,
        transport: Transport | None = None,
        clock: Clock | None = None,
        allow_actuation: bool | None = None,
        command_timeout_s: float = 2.0,
        link_timeout_s: float = 3.0,
        target_system: int = 1,
        target_component: int = 1,
        policy: ReconnectPolicy | None = None,
    ) -> None:
        if transport is None:
            connection = connection or self.default_connection
            if connection is None:
                raise ValueError(f"{type(self).__name__} needs an explicit connection string")
            transport = transport_for(connection)
        self._clock = clock or SystemClock()
        self.link = MavlinkLink(
            transport, self._clock, policy=policy, link_timeout_s=link_timeout_s
        )
        self.allow_actuation = (
            self.default_allow_actuation if allow_actuation is None else allow_actuation
        )
        self.command_timeout_s = command_timeout_s
        self._target = (target_system, target_component)
        self._armed = False
        self._attitude: m.Attitude | None = None
        self._position: m.LocalPositionNed | None = None
        self._acks: dict[int, int] = {}
        self._rx_times: deque[float] = deque()
        self._state: VehicleState | None = None

    # --- lifecycle -----------------------------------------------------
    def connect(self) -> None:
        self.link.connect()

    def reconnect(self) -> bool:
        return self.link.reconnect()

    def disconnect(self) -> None:
        self.link.disconnect()

    # --- telemetry -------------------------------------------------------
    @property
    def state(self) -> VehicleState | None:
        return self._state

    def update(self, timeout_s: float = 0.1) -> None:
        for frame in self.link.poll(timeout_s):
            self._handle(frame)

    def _handle(self, frame: Frame) -> None:
        now = self._clock.now()
        self._rx_times.append(now)
        while self._rx_times and now - self._rx_times[0] > RATE_WINDOW_S:
            self._rx_times.popleft()
        if frame.msgid == MSG_HEARTBEAT:
            self._armed = m.unpack_heartbeat(frame.payload).armed
        elif frame.msgid == MSG_ATTITUDE:
            self._attitude = m.unpack_attitude(frame.payload)
        elif frame.msgid == MSG_LOCAL_POSITION_NED:
            self._position = m.unpack_local_position_ned(frame.payload)
        elif frame.msgid == MSG_COMMAND_ACK:
            ack = m.unpack_command_ack(frame.payload)
            self._acks[ack.command] = ack.result
        if self._position is not None and self._attitude is not None:
            p, a = self._position, self._attitude
            self._state = VehicleState(
                t_s=p.time_boot_ms / 1000.0,
                position_ned=(p.x, p.y, p.z),
                velocity_ned=(p.vx, p.vy, p.vz),
                roll_deg=a.roll_deg,
                pitch_deg=a.pitch_deg,
                yaw_deg=a.yaw_deg,
                armed=self._armed,
            )

    def link_health(self) -> LinkHealth:
        now = self._clock.now()
        recent = [t for t in self._rx_times if now - t <= RATE_WINDOW_S]
        age = (now - self._rx_times[-1]) if self._rx_times else None
        return LinkHealth(
            connected=self.link.state is LinkState.CONNECTED,
            alive=self.link.is_alive(),
            rx_rate_hz=len(recent) / RATE_WINDOW_S,
            last_rx_age_s=age,
        )

    # --- commands ----------------------------------------------------------
    def _require_actuation(self) -> None:
        if not self.allow_actuation:
            raise ActuationNotPermitted(
                f"{type(self).__name__} is telemetry-only; construct it with "
                "allow_actuation=True to send commands"
            )

    def _send(self, msgid: int, payload: bytes) -> None:
        try:
            self.link.send(msgid, payload)
        except LinkDown as exc:
            raise VehicleError(f"link down: {exc}") from exc

    def _command(self, command: int, *params: float) -> None:
        self._require_actuation()
        self._acks.pop(command, None)
        self._send(MSG_COMMAND_LONG, m.pack_command_long(command, *self._target, *params))
        deadline = self._clock.now() + self.command_timeout_s
        while self._clock.now() < deadline:
            self.update(0.05)
            if command in self._acks:
                result = self._acks.pop(command)
                if result != m.MAV_RESULT_ACCEPTED:
                    raise CommandRejected(f"command {command} rejected (result {result})")
                return
        raise CommandTimeout(f"no ack for command {command} within {self.command_timeout_s}s")

    def arm(self) -> None:
        self._command(m.MAV_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self) -> None:
        self._command(m.MAV_CMD_COMPONENT_ARM_DISARM, 0.0)

    def set_velocity(self, vn: float, ve: float, vd: float) -> None:
        self._require_actuation()
        self._send(
            MSG_SET_POSITION_TARGET_LOCAL_NED,
            m.pack_velocity_setpoint(*self._target, vn, ve, vd),
        )


class PX4SITLVehicle(MavlinkVehicle):
    """PX4 SITL: MAVLink offboard port on UDP 14540 by default."""

    kind = VehicleKind.SITL
    default_connection = "udpin:0.0.0.0:14540"


class PX4HardwareVehicle(MavlinkVehicle):
    """A real PX4 flight controller over UART/USB.

    Telemetry-only by default and no default device: the caller must name the
    port, and must pass ``allow_actuation=True`` to arm or command it. Nothing
    in this repository's CI has run this class against real hardware.
    """

    kind = VehicleKind.HARDWARE
    default_connection = None
    default_allow_actuation = False
