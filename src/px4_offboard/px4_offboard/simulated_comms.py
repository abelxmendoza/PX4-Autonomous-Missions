"""Simulated mission communications.

Messages stay inside this process. Nothing is transmitted on a radio, and
nothing is sent to ATC. A simulated clearance is scoped to a mission, a
region, an altitude band, and a time window. It cannot set
``operational_approval``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from enum import Enum

from .airspace import offset_latlon
from .weather import Provenance, horizontal_distance_m

SIM_MISSION_ID = "sim-seeding"
_SECRET_KEYS = frozenset({"password", "secret", "api_key", "token", "credential"})


class MessageType(Enum):
    REQUEST = "request"
    ACKNOWLEDGMENT = "acknowledgment"
    ADVISORY = "advisory"
    SIMULATED_CLEARANCE = "simulated_clearance"
    DENIAL = "denial"
    CANCELLATION = "cancellation"
    EXPIRATION = "expiration"
    COMMUNICATION_FAILURE = "communication_failure"


class Reason:
    SIMULATED_CLEARANCE_VALID = "simulated_clearance_valid"
    SIMULATED_CLEARANCE_MISSING = "simulated_clearance_missing"
    SIMULATED_CLEARANCE_EXPIRED = "simulated_clearance_expired"
    SIMULATED_CLEARANCE_DENIED = "simulated_clearance_denied"
    SIMULATED_CLEARANCE_WRONG_MISSION = "simulated_clearance_wrong_mission"
    SIMULATED_CLEARANCE_WRONG_AREA = "simulated_clearance_wrong_area"
    SIMULATED_CLEARANCE_WRONG_ALTITUDE = "simulated_clearance_wrong_altitude"
    COMMUNICATION_UNAVAILABLE = "communication_unavailable"


@dataclass(frozen=True)
class SimMessage:
    message_id: str
    scenario_id: str
    sender: str
    recipient: str
    message_type: MessageType
    created_at_s: float
    received_at_s: float | None
    expires_at_s: float | None
    latitude_deg: float | None
    longitude_deg: float | None
    radius_m: float | None
    altitude_floor_amsl_m: float | None
    altitude_ceiling_amsl_m: float | None
    correlation_id: str | None
    mission_id: str | None
    payload: tuple[tuple[str, object], ...] = ()
    provenance: Provenance = Provenance.SIMULATED
    non_operational: bool = True

    def __post_init__(self) -> None:
        object.__setattr__(self, "non_operational", True)
        object.__setattr__(self, "payload", _freeze_payload(self.payload))

    def as_dict(self) -> dict:
        return {
            "message_id": self.message_id,
            "scenario_id": self.scenario_id,
            "sender": self.sender,
            "recipient": self.recipient,
            "message_type": self.message_type.value,
            "created_at_s": self.created_at_s,
            "received_at_s": self.received_at_s,
            "expires_at_s": self.expires_at_s,
            "latitude_deg": self.latitude_deg,
            "longitude_deg": self.longitude_deg,
            "radius_m": self.radius_m,
            "altitude_floor_amsl_m": self.altitude_floor_amsl_m,
            "altitude_ceiling_amsl_m": self.altitude_ceiling_amsl_m,
            "correlation_id": self.correlation_id,
            "mission_id": self.mission_id,
            "payload": {key: value for key, value in self.payload},
            "provenance": self.provenance.value,
            "non_operational": True,
            "operational_approval": False,
            "units": {"radius": "m", "altitude": "m", "time": "s"},
        }


def _freeze_payload(payload: object) -> tuple[tuple[str, object], ...]:
    if isinstance(payload, tuple):
        items = payload
    elif isinstance(payload, dict):
        items = tuple(payload.items())
    else:
        return ()
    frozen: list[tuple[str, object]] = []
    for key, value in items:
        if str(key).lower() in _SECRET_KEYS:
            continue
        if isinstance(value, (str, int, float, bool)) or value is None:
            frozen.append((str(key), value))
    return tuple(sorted(frozen))


@dataclass(frozen=True)
class _Pending:
    deliver_at_s: float
    message: SimMessage
    drop: bool


class SimCommsBroker:
    """In-process delivery. Delay, loss, duplicates, and outages are explicit."""

    def __init__(self, max_retained: int = 128) -> None:
        if max_retained < 1:
            raise ValueError("max_retained must be positive")
        self.max_retained = max_retained
        self._pending: list[_Pending] = []
        self._delivered: list[SimMessage] = []
        self._outage_until_s: float | None = None
        self._events: list[dict] = []

    def publish(self, message: SimMessage, *, deliver_at_s: float | None = None, drop: bool = False) -> None:
        when = message.created_at_s if deliver_at_s is None else deliver_at_s
        self._pending.append(_Pending(when, message, drop))
        self._events.append(
            {
                "event": "queued",
                "message_id": message.message_id,
                "message_type": message.message_type.value,
                "deliver_at_s": when,
                "drop": drop,
                "provenance": message.provenance.value,
                "non_operational": True,
            }
        )

    def set_outage(self, until_s: float | None) -> None:
        self._outage_until_s = until_s
        self._events.append({"event": "outage_set", "until_s": until_s, "non_operational": True})

    def outage_active(self, now_s: float) -> bool:
        return self._outage_until_s is not None and now_s < self._outage_until_s

    def poll(self, now_s: float) -> tuple[SimMessage, ...]:
        if self.outage_active(now_s):
            self._events.append({"event": "outage", "time_s": now_s, "non_operational": True})
            return tuple(self._delivered)
        due = [item for item in self._pending if item.deliver_at_s <= now_s]
        self._pending = [item for item in self._pending if item.deliver_at_s > now_s]
        for item in sorted(due, key=lambda pending: (pending.deliver_at_s, pending.message.message_id)):
            if item.drop:
                self._events.append(
                    {"event": "dropped", "message_id": item.message.message_id, "time_s": now_s, "non_operational": True}
                )
                continue
            if any(existing.message_id == item.message.message_id for existing in self._delivered):
                self._events.append(
                    {
                        "event": "duplicate_ignored",
                        "message_id": item.message.message_id,
                        "time_s": now_s,
                        "non_operational": True,
                    }
                )
                continue
            delivered = replace(item.message, received_at_s=now_s)
            self._delivered.append(delivered)
            if len(self._delivered) > self.max_retained:
                self._delivered = self._delivered[-self.max_retained :]
            self._events.append(
                {
                    "event": "delivered",
                    "message_id": delivered.message_id,
                    "message_type": delivered.message_type.value,
                    "time_s": now_s,
                    "provenance": delivered.provenance.value,
                    "non_operational": True,
                }
            )
        return tuple(self._delivered)

    def events(self) -> list[dict]:
        return list(self._events)


@dataclass(frozen=True)
class CommsPolicy:
    mission_id: str = SIM_MISSION_ID
    origin_latitude_deg: float = 47.0
    origin_longitude_deg: float = -122.0


@dataclass(frozen=True)
class CommsDecision:
    time_s: float
    authorized: bool
    reason_code: str
    status: str
    source: str
    provenance: str | None
    age_s: float | None
    mission_id: str
    clearance: dict | None
    operational_approval: bool = False

    def as_dict(self) -> dict:
        return {
            "gate": "simulated_communications",
            "time_s": self.time_s,
            "authorized": self.authorized,
            "reason_code": self.reason_code,
            "status": self.status,
            "source": self.source,
            "provenance": self.provenance,
            "age_s": self.age_s,
            "mission_id": self.mission_id,
            "clearance": self.clearance,
            "operational_approval": False,
            "non_operational": True,
            "transmitted": False,
        }


class CommsAuthorizer:
    def __init__(self, broker: SimCommsBroker, policy: CommsPolicy | None = None) -> None:
        self.broker = broker
        self.policy = policy or CommsPolicy()

    def evaluate(
        self,
        now_s: float,
        north_m: float,
        east_m: float,
        release_altitude_amsl_m: float | None,
    ) -> CommsDecision:
        latitude, longitude = offset_latlon(
            self.policy.origin_latitude_deg,
            self.policy.origin_longitude_deg,
            north_m,
            east_m,
        )
        if self.broker.outage_active(now_s):
            self.broker.poll(now_s)
            return _decision(now_s, self.policy, Reason.COMMUNICATION_UNAVAILABLE, "unavailable", None, None)
        messages = self.broker.poll(now_s)
        return _from_messages(messages, self.policy, now_s, latitude, longitude, release_altitude_amsl_m)


def simulation_clearance(**overrides: object) -> SimMessage:
    values: dict = {
        "message_id": "CLR-SIM-1",
        "scenario_id": "sim-seeding",
        "sender": "sim-atc",
        "recipient": "sim-uav",
        "message_type": MessageType.SIMULATED_CLEARANCE,
        "created_at_s": 0.0,
        "received_at_s": None,
        "expires_at_s": 1_000_000_000.0,
        "latitude_deg": 47.0,
        "longitude_deg": -122.0,
        "radius_m": 100_000.0,
        "altitude_floor_amsl_m": 0.0,
        "altitude_ceiling_amsl_m": 20_000.0,
        "correlation_id": "REQ-SIM-1",
        "mission_id": SIM_MISSION_ID,
        "payload": (("statement", "simulated clearance"),),
        "provenance": Provenance.SIMULATED,
    }
    values.update(overrides)
    return SimMessage(**values)


def simulation_comms_authorizer(
    broker: SimCommsBroker | None = None,
    policy: CommsPolicy | None = None,
    clearance: SimMessage | None = None,
) -> CommsAuthorizer:
    owned = broker if broker is not None else SimCommsBroker()
    if broker is None:
        owned.publish(clearance if clearance is not None else simulation_clearance())
    return CommsAuthorizer(owned, policy)


def _from_messages(
    messages: tuple[SimMessage, ...],
    policy: CommsPolicy,
    now_s: float,
    latitude: float,
    longitude: float,
    release_amsl: float | None,
) -> CommsDecision:
    ordered = tuple(sorted(messages, key=lambda item: (item.received_at_s or 0.0, item.message_id)))
    clearance: SimMessage | None = None
    denied = False
    other_mission = False
    for message in ordered:
        if message.message_type is MessageType.SIMULATED_CLEARANCE and message.mission_id != policy.mission_id:
            other_mission = True
            continue
        if message.mission_id not in (None, policy.mission_id) and message.message_type in (
            MessageType.DENIAL,
            MessageType.CANCELLATION,
            MessageType.EXPIRATION,
        ):
            continue
        if message.message_type is MessageType.DENIAL:
            denied = True
            clearance = None
        elif message.message_type in (MessageType.CANCELLATION, MessageType.EXPIRATION):
            clearance = None
            denied = False
        elif message.message_type is MessageType.SIMULATED_CLEARANCE:
            denied = False
            clearance = message
    if denied:
        return _decision(now_s, policy, Reason.SIMULATED_CLEARANCE_DENIED, "denied", None, None)
    if clearance is None:
        reason = Reason.SIMULATED_CLEARANCE_WRONG_MISSION if other_mission else Reason.SIMULATED_CLEARANCE_MISSING
        return _decision(now_s, policy, reason, "missing", None, None)
    if clearance.expires_at_s is not None and now_s >= clearance.expires_at_s:
        return _decision(now_s, policy, Reason.SIMULATED_CLEARANCE_EXPIRED, "expired", clearance, None)
    if (
        clearance.latitude_deg is None
        or clearance.longitude_deg is None
        or clearance.radius_m is None
        or not math.isfinite(clearance.radius_m)
        or clearance.radius_m <= 0.0
    ):
        return _decision(now_s, policy, Reason.SIMULATED_CLEARANCE_WRONG_AREA, "unscoped", clearance, None)
    distance = horizontal_distance_m(latitude, longitude, clearance.latitude_deg, clearance.longitude_deg)
    if distance > clearance.radius_m:
        return _decision(now_s, policy, Reason.SIMULATED_CLEARANCE_WRONG_AREA, "outside", clearance, distance)
    if (
        release_amsl is None
        or not math.isfinite(release_amsl)
        or clearance.altitude_floor_amsl_m is None
        or clearance.altitude_ceiling_amsl_m is None
        or not (clearance.altitude_floor_amsl_m <= release_amsl <= clearance.altitude_ceiling_amsl_m)
    ):
        return _decision(now_s, policy, Reason.SIMULATED_CLEARANCE_WRONG_ALTITUDE, "altitude", clearance, None)
    age = None if clearance.received_at_s is None else now_s - clearance.received_at_s
    return _decision(now_s, policy, "", "simulated_clearance_valid", clearance, age)


def _decision(
    now_s: float,
    policy: CommsPolicy,
    reason: str,
    status: str,
    clearance: SimMessage | None,
    age_s: float | None,
) -> CommsDecision:
    return CommsDecision(
        time_s=now_s,
        authorized=reason == "",
        reason_code=reason,
        status=status,
        source="sim-comms-broker",
        provenance=None if clearance is None else clearance.provenance.value,
        age_s=age_s,
        mission_id=policy.mission_id,
        clearance=None if clearance is None else clearance.as_dict(),
    )
