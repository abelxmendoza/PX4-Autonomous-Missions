import asyncio
from mavsdk import System

async def main():
    drone = System(mavsdk_server_address="localhost", port=50051)
    await drone.connect()

    print("Connecting to MAVSDK server (localhost:50051)...")
    try:
        async with asyncio.timeout(10):
            async for state in drone.core.connection_state():
                if state.is_connected:
                    print("Drone connected via MAVSDK server!")
                    break
    except asyncio.TimeoutError:
        print("ERROR: Could not reach MAVSDK server. Is 'mavsdk_server udpout://127.0.0.1:14580' running?")
        return

    print("Waiting for position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position OK")
            break

    await asyncio.sleep(2)

    print("Arming...")
    await drone.action.arm()

    print("Taking off...")
    await drone.action.takeoff()

    await asyncio.sleep(10)

    print("Landing...")
    await drone.action.land()

asyncio.run(main())