# PX4 Autonomous Missions — Gazebo Harmonic + ROS 2 + MAVSDK

![PX4 Autonomous Mission Overview](PX4ProjectImage2.png)

Autonomous UAV control stack using **PX4 v1.15+**, **Gazebo Harmonic**, **ROS 2 Humble**, **px4_msgs + Micro XRCE-DDS**, and **MAVSDK-Python**. Includes a custom obstacle world, pre-planned and reactive obstacle avoidance, a full state machine flight controller, and QGroundControl integration.

---

## Stack

| Component | Version |
|-----------|---------|
| PX4 Autopilot (SITL) | v1.15+ |
| Gazebo Harmonic | gz-sim 8.x |
| ROS 2 | Humble |
| px4_msgs + Micro XRCE-DDS | native PX4 bridge |
| MAVROS2 | ros-humble-mavros (optional GCS relay) |
| MAVSDK-Python | 2.0+ |
| QGroundControl | Daily / Stable |
| Ubuntu | 22.04 |

---

## Architecture

```
offboard_mission.py (ROS 2)        MAVSDK scripts
  px4_offboard package               mission.py
  px4_msgs + XRCE-DDS                obstacle_avoidance.py
        │                            offboard_avoidance.py
        │ UDP 8888                          │
        ↓                                   │ gRPC :50051
  MicroXRCEAgent                     MAVSDK Server
        │                                   │
        └─────────── UDP :14540 ────────────┘
                           │
                      PX4 SITL
                    + Gazebo Harmonic
                    + obstacle_world.sdf
                           │
                    UDP :14550
                           │
                   QGroundControl
```

---

## What's Included

### ROS 2 Package — `src/px4_offboard/`

| Node | Run command | Description |
|------|-------------|-------------|
| `offboard_mission` | `ros2 run px4_offboard offboard_mission` | Full autonomy engine: state machine + trajectory + obstacle avoidance |
| `offboard_control` | `ros2 run px4_offboard offboard_control` | Minimal hover node — baseline / sanity check |

### MAVSDK Scripts

| File | Description |
|------|-------------|
| `mission.py` | Lawnmower grid scan — 3×3 GPS waypoint mission, CSV telemetry |
| `obstacle_avoidance.py` | Pre-planned 8-waypoint path around all 5 obstacles at 20 m |
| `offboard_avoidance.py` | Reactive OFFBOARD loop — geometry-based detection, NED control, smoothing |
| `fly.py` | Minimal takeoff / 10 s hover / land test |
| `plot_flight.py` | Post-flight telemetry visualization (4-panel PNG) |

### Simulation & Config

| File | Description |
|------|-------------|
| `worlds/obstacle_world.sdf` | Gazebo Harmonic world — 5 color-coded static obstacles |
| `launch/simulation.launch.py` | ROS 2 launch: PX4 SITL + Gazebo Harmonic + MAVROS2 |
| `config/mavros_params.yaml` | MAVROS2 FCU URL, plugin allowlist, TF config |

---

## Obstacle World Layout

Five static obstacles in `worlds/obstacle_world.sdf` (GPS origin `47.397742°N, 8.545594°E`):

```
N (north)
^
50 |                          ★ WP8 destination
46 |              ● WP7
38 |         [OB5 purple wall  18m east]
36 |                   ● WP6 (east of OB5)
32 |              ● WP5
24 |    [OB3 green]   ● WP4   [OB4 blue]
15 |    ● WP2    ● WP3
10 |    [OB1 red  12m east]   [OB2 orange  28m east]
 5 |    ● WP1
 0 |  ★ SPAWN
   +---------------------------------------------> E (east)
        0    10   18   28        meters
```

| Name | Position (E, N) | Size | Height |
|------|-----------------|------|--------|
| OB1 | (12m, 10m) | 3×3m | 4m |
| OB2 | (28m, 10m) | 3×3m | 6m |
| OB3 | (10m, 24m) | 4×3m | 4m |
| OB4 | (24m, 24m) | 2×2m | 5m |
| OB5 | (18m, 38m) | 5×3m | 4m |

---

## State Machine — `offboard_mission`

```
PREFLIGHT → ARMING → TAKEOFF → HOVER → MOVE → LANDING
```

| Transition | Condition |
|-----------|-----------|
| PREFLIGHT → ARMING | 2 s setpoint pre-stream complete |
| ARMING → TAKEOFF | `nav_state=14` AND `arming_state=2` (vehicle_status) |
| TAKEOFF → HOVER | `\|z_err\|` < 0.2 m |
| HOVER → MOVE | 3 s stabilization hold |
| MOVE → LANDING | All waypoints reached |

In the **MOVE** state, every setpoint passes through the full avoidance pipeline:

```
trajectory_generator()
    → _detect_obstacle()    # geometry check vs. SDF obstacle map
    → _apply_avoidance()    # sidestep + exponential smooth (α=0.2)
    → publish_setpoint()
```

### Trajectory modes

Set `TRAJECTORY_MODE` at the top of `offboard_mission.py`:

```python
TRAJECTORY_MODE = TrajectoryMode.WAYPOINTS   # 4-point square at 3m AGL
TRAJECTORY_MODE = TrajectoryMode.CIRCLE      # 8m radius orbit, 20s period
```

### Obstacle detection

Bearing computed relative to **direction of travel** — "front" means in the way of the current waypoint, not world north.

```python
DETECTION_RADIUS = 4.0   # metres
FRONT_ANGLE      = 30    # degrees — cone ahead
SIDE_ANGLE       = 60    # degrees — flanks
```

To connect real sensors (LiDAR, depth camera, ROS 2 topic), replace `_detect_obstacle()` only — everything downstream is unchanged.

---

## Installation

### 1 — PX4 + Gazebo Harmonic

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
cd ~/PX4-Autopilot
bash Tools/setup/ubuntu.sh

# Copy custom obstacle world
cp worlds/obstacle_world.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
```

### 2 — Micro XRCE-DDS Agent

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

### 3 — ROS 2 workspace

```bash
sudo apt install ros-humble-desktop python3-colcon-common-extensions

cd ~/Desktop/px4-autonomous-mission
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 4 — MAVROS2 (optional — for GCS relay / MAVROS topics)

```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh
```

### 5 — Python dependencies (MAVSDK scripts)

```bash
pip install -r requirements.txt
```

### 6 — QGroundControl

Download from [qgroundcontrol.com](https://qgroundcontrol.com/). Auto-connects via UDP 14550 — no config needed.

---

## Running

### Full autonomy — state machine + obstacle avoidance (ROS 2)

```bash
# Terminal 1 — PX4 SITL + Gazebo Harmonic
cd ~/PX4-Autopilot && rm -f build/px4_sitl_default/dataman
GZ_SIM_RESOURCE_PATH=~/Desktop/px4-autonomous-mission/worlds \
PX4_GZ_WORLD=obstacle_world make px4_sitl gz_x500

# Terminal 2 — Micro XRCE-DDS agent
MicroXRCEAgent udp4 -p 8888

# Terminal 3 — autonomy node
source /opt/ros/humble/setup.bash
source ~/Desktop/px4-autonomous-mission/install/setup.bash
ros2 run px4_offboard offboard_mission
```

### MAVSDK missions

```bash
# Start MAVSDK server first
mavsdk_server udpout://127.0.0.1:14580

# Then run any script
python3 mission.py              # lawnmower scan
python3 obstacle_avoidance.py   # pre-planned avoidance
python3 offboard_avoidance.py   # reactive OFFBOARD loop
python3 fly.py                  # basic test
```

### Simulation via ROS 2 launch (PX4 + Gazebo + MAVROS2)

```bash
source /opt/ros/humble/setup.bash
ros2 launch launch/simulation.launch.py
```

---

## Monitor

```bash
# PX4 shell — confirm OFFBOARD + ARMED
listener vehicle_status
listener vehicle_local_position

# ROS 2 — live position
ros2 topic echo /fmu/out/vehicle_local_position

# ROS 2 — odometry
ros2 topic echo /fmu/out/vehicle_odometry

# MAVROS2 topics (if running)
ros2 topic echo /mavros/state
ros2 topic echo /mavros/local_position/pose
```

---

## Port Reference

| Port | Protocol | Purpose |
|------|----------|---------|
| 8888 | UDP | Micro XRCE-DDS agent |
| 14540 | UDP | MAVLink offboard (MAVSDK / MAVROS) |
| 14550 | UDP | GCS — QGroundControl auto-connects |
| 50051 | TCP | MAVSDK gRPC server |

---

## Telemetry & Visualization

MAVSDK scripts log timestamped CSV on every run:
```
time,latitude,longitude,alt_abs_m,alt_rel_m
15:14:01,47.3977508,8.5456073,488.13,15.01
```

Plot the last flight:
```bash
python3 plot_flight.py   # → flight_plot.png
```

---

## Real Hardware

Swap `fcu_url` in `config/mavros_params.yaml` and `offboard_mission.py`:
```yaml
fcu_url: "serial:///dev/ttyTHS1:921600"   # UART (Jetson / RPi)
fcu_url: "serial:///dev/ttyUSB0:57600"    # USB serial
```

---

## Notes

- Run `rm -f ~/PX4-Autopilot/build/px4_sitl_default/dataman` before each SITL restart
- Simulated battery drains — full sim restart required between flights
- World SDF contains no plugin declarations — all sensor plugins come from PX4's `server.config` via `GZ_SIM_SERVER_CONFIG_PATH`
- Sensor names in `x500_base/model.sdf` (`air_pressure_sensor`, `magnetometer_sensor`) match `GZBridge.cpp` hardcoded topic paths — do not rename

---

## Author

Abel — UAV autonomy stack development
GitHub: github.com/abelxmendoza
