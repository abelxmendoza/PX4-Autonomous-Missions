"""Simulated communications stay in process and cannot approve a real flight."""

from __future__ import annotations

from px4_offboard.simulated_comms import (
    SIM_MISSION_ID,
    CommsAuthorizer,
    MessageType,
    Reason,
    SimCommsBroker,
    SimMessage,
    simulation_clearance,
    simulation_comms_authorizer,
)
from px4_offboard.weather import Provenance


def _message(message_type, message_id, **overrides) -> SimMessage:
    values = dict(
        message_id=message_id,
        scenario_id="sim-seeding",
        sender="sim-atc",
        recipient="sim-uav",
        message_type=message_type,
        created_at_s=0.0,
        received_at_s=None,
        expires_at_s=None,
        latitude_deg=47.0,
        longitude_deg=-122.0,
        radius_m=100_000.0,
        altitude_floor_amsl_m=0.0,
        altitude_ceiling_amsl_m=20_000.0,
        correlation_id="REQ-SIM-1",
        mission_id=SIM_MISSION_ID,
        payload=(("statement", "simulated"),),
        provenance=Provenance.SIMULATED,
    )
    values.update(overrides)
    return SimMessage(**values)


def _evaluate(authorizer, now=10.0):
    return authorizer.evaluate(now, 30.0, 10.0, 15.0)


def test_valid_simulated_clearance_authorizes_without_operational_approval():
    decision = _evaluate(simulation_comms_authorizer())
    assert decision.authorized is True
    assert decision.status == "simulated_clearance_valid"
    assert decision.operational_approval is False
    assert decision.as_dict()["operational_approval"] is False
    assert decision.as_dict()["transmitted"] is False
    assert decision.as_dict()["non_operational"] is True
    assert decision.provenance == "simulated"
    assert decision.clearance["non_operational"] is True


def test_missing_clearance_denies_release():
    decision = _evaluate(CommsAuthorizer(SimCommsBroker()))
    assert decision.reason_code == Reason.SIMULATED_CLEARANCE_MISSING


def test_expired_clearance_denies_release():
    authorizer = simulation_comms_authorizer(clearance=simulation_clearance(expires_at_s=5.0, message_id="CLR-EXP"))
    assert _evaluate(authorizer, now=5.0).reason_code == Reason.SIMULATED_CLEARANCE_EXPIRED


def test_clearance_for_another_mission_does_not_authorize_this_one():
    clearance = simulation_clearance(message_id="CLR-OTHER", mission_id="other-mission")
    decision = _evaluate(simulation_comms_authorizer(clearance=clearance))
    assert decision.reason_code == Reason.SIMULATED_CLEARANCE_WRONG_MISSION


def test_clearance_outside_its_region_does_not_authorize_release():
    clearance = simulation_clearance(message_id="CLR-FAR", radius_m=1.0)
    decision = _evaluate(simulation_comms_authorizer(clearance=clearance))
    assert decision.reason_code == Reason.SIMULATED_CLEARANCE_WRONG_AREA


def test_clearance_outside_its_altitude_band_does_not_authorize_release():
    clearance = simulation_clearance(message_id="CLR-HIGH", altitude_floor_amsl_m=100.0, altitude_ceiling_amsl_m=200.0)
    decision = _evaluate(simulation_comms_authorizer(clearance=clearance))
    assert decision.reason_code == Reason.SIMULATED_CLEARANCE_WRONG_ALTITUDE


def test_duplicate_delivery_keeps_the_first_message():
    broker = SimCommsBroker()
    clearance = simulation_clearance(message_id="CLR-DUP")
    broker.publish(clearance)
    broker.publish(clearance, deliver_at_s=1.0)
    authorizer = CommsAuthorizer(broker)
    decision = _evaluate(authorizer, now=2.0)
    assert decision.authorized is True
    assert [item["event"] for item in broker.events()].count("delivered") == 1
    assert "duplicate_ignored" in [item["event"] for item in broker.events()]


def test_receipt_order_wins_over_creation_order():
    broker = SimCommsBroker()
    broker.publish(_message(MessageType.DENIAL, "DEN-1", created_at_s=1.0), deliver_at_s=1.0)
    broker.publish(simulation_clearance(message_id="CLR-LATE", created_at_s=10.0), deliver_at_s=5.0)
    authorizer = CommsAuthorizer(broker)
    assert _evaluate(authorizer, now=2.0).reason_code == Reason.SIMULATED_CLEARANCE_DENIED
    assert _evaluate(authorizer, now=6.0).authorized is True


def test_communication_outage_denies_release():
    authorizer = simulation_comms_authorizer()
    assert _evaluate(authorizer, now=1.0).authorized is True
    authorizer.broker.set_outage(100.0)
    decision = _evaluate(authorizer, now=2.0)
    assert decision.reason_code == Reason.COMMUNICATION_UNAVAILABLE
    assert decision.operational_approval is False


def test_a_dropped_acknowledgment_does_not_authorize_release():
    broker = SimCommsBroker()
    broker.publish(_message(MessageType.ACKNOWLEDGMENT, "ACK-1"), drop=True)
    decision = _evaluate(CommsAuthorizer(broker), now=1.0)
    assert decision.reason_code == Reason.SIMULATED_CLEARANCE_MISSING
    assert any(item["event"] == "dropped" for item in broker.events())


def test_delayed_delivery_is_absent_until_the_deliver_time():
    broker = SimCommsBroker()
    broker.publish(simulation_clearance(message_id="CLR-DELAY"), deliver_at_s=10.0)
    authorizer = CommsAuthorizer(broker)
    assert _evaluate(authorizer, now=5.0).reason_code == Reason.SIMULATED_CLEARANCE_MISSING
    assert _evaluate(authorizer, now=10.0).authorized is True


def test_a_simulated_clearance_cannot_become_operational_approval():
    leaked = simulation_clearance(payload={"token": "do-not-store", "statement": "simulated"})
    assert all(key != "token" for key, _value in leaked.payload)
    decision = _evaluate(simulation_comms_authorizer(clearance=leaked))
    assert decision.operational_approval is False
    assert decision.clearance["operational_approval"] is False
    assert "token" not in decision.clearance["payload"]


def test_broker_retains_a_bounded_number_of_messages():
    broker = SimCommsBroker(max_retained=8)
    for index in range(20):
        broker.publish(simulation_clearance(message_id=f"CLR-{index}", created_at_s=float(index)), deliver_at_s=float(index))
    delivered = broker.poll(100.0)
    assert len(delivered) == 8
    assert delivered[0].message_id == "CLR-12"
