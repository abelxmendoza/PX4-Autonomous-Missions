import asyncio
from mavsdk import System

async def main():
    drone = System(mavsdk_server_address="localhost", port=50051)
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

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