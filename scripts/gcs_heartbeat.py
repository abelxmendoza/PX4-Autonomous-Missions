#!/usr/bin/env python3
"""Minimal MAVLink GCS so PX4 SITL can arm without QGroundControl.

Also disables datalink-loss / RC-loss failsafes that block SITL arming.
"""

from __future__ import annotations

import time

from pymavlink import mavutil


PARAMS = {
    "NAV_DLL_ACT": 0,   # datalink loss action: disabled
    "NAV_RCL_ACT": 0,   # RC loss action: disabled
    "COM_RCL_EXCEPT": 4,  # ignore RC loss in offboard (bitmask; version-dependent)
}


def set_param(conn, name: str, value: float):
    conn.mav.param_set_send(
        conn.target_system,
        conn.target_component,
        name.encode("utf-8"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )


def main():
    # Connect as GCS on the PX4 SITL GCS port
    conn = mavutil.mavlink_connection(
        "udp:127.0.0.1:14550",
        source_system=255,
        source_component=190,
    )
    print("GCS heartbeat → udp:127.0.0.1:14550")

    # Wait briefly for PX4 to accept the link
    for _ in range(15):
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        time.sleep(1.0)

    try:
        conn.wait_heartbeat(timeout=5)
        print(f"PX4 heartbeat sys={conn.target_system} comp={conn.target_component}")
        for name, value in PARAMS.items():
            set_param(conn, name, value)
            print(f"  param {name}={value}")
    except Exception as exc:  # noqa: BLE001 — best-effort for SITL bring-up
        print(f"Param set skipped ({exc}) — continuing heartbeats")

    while True:
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        time.sleep(1.0)


if __name__ == "__main__":
    main()
