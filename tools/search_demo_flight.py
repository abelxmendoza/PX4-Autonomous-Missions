#!/usr/bin/env python3
"""Scripted demo flight over the search field: takeoff, a lawnmower sweep, land.

    python tools/search_demo_flight.py [--altitude 8] [--speed 4]

A DEMO, not the search mission: the lanes are fixed in advance (they cover the field
outline from worlds/search_field.sdf), nothing reacts to what the camera sees, and the
ground-truth target file is never read. It exists so the world, the downward camera and
PX4 can be seen working together. Talks to PX4 SITL with MAVSDK on UDP 14540.
"""
from __future__ import annotations

import argparse
import asyncio
import math

from mavsdk import System

# Lanes over the 60 x 60 m field (NED metres from home). At 8 m the camera footprint is
# about 19 m across, so 12 m lane spacing overlaps; east +-24 m plus that reaches the edges.
LANE_NORTH_M = (14.0, 26.0, 38.0, 50.0)
LANE_EAST_M = 24.0
EARTH_M_PER_DEG = 111_320.0


def lawnmower() -> list[tuple[float, float]]:
    points = []
    for i, n in enumerate(LANE_NORTH_M):
        a, b = (-LANE_EAST_M, LANE_EAST_M) if i % 2 == 0 else (LANE_EAST_M, -LANE_EAST_M)
        points += [(n, a), (n, b)]
    return points


def offset(lat: float, lon: float, north: float, east: float) -> tuple[float, float]:
    return (lat + north / EARTH_M_PER_DEG,
            lon + east / (EARTH_M_PER_DEG * math.cos(math.radians(lat))))


async def fly(altitude: float, speed: float) -> None:
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    print("waiting for PX4 ...", flush=True)
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            break
    home = await anext(drone.telemetry.home())
    print(f"home {home.latitude_deg:.6f}, {home.longitude_deg:.6f}", flush=True)

    await drone.action.set_takeoff_altitude(altitude)
    await drone.action.arm()
    await drone.action.takeoff()
    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m > altitude - 0.5:
            break
    await drone.action.set_current_speed(speed)

    target_amsl = home.absolute_altitude_m + altitude
    for k, (n, e) in enumerate(lawnmower(), start=1):
        lat, lon = offset(home.latitude_deg, home.longitude_deg, n, e)
        yaw = 90.0 if e > 0 else 270.0
        print(f"leg {k}: to north {n:+.0f} m, east {e:+.0f} m", flush=True)
        await drone.action.goto_location(lat, lon, target_amsl, yaw)
        async for pos in drone.telemetry.position():
            dn = (pos.latitude_deg - lat) * EARTH_M_PER_DEG
            de = (pos.longitude_deg - lon) * EARTH_M_PER_DEG * math.cos(math.radians(lat))
            if math.hypot(dn, de) < 1.5:
                break

    print("sweep done, landing", flush=True)
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            break
    print("landed", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--altitude", type=float, default=8.0)
    parser.add_argument("--speed", type=float, default=4.0)
    args = parser.parse_args()
    asyncio.run(fly(args.altitude, args.speed))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
