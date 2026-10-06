"""Embedded-communications test subsystem: framing, serial transport, link
supervision and bus stubs. Pure Python, no ROS; every hardware dependency sits
behind an interface with a deterministic mock so CI exercises the failure
paths without a device."""
