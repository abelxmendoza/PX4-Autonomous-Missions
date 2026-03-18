# PX4 Autonomous Missions — Gazebo Harmonic + MAVROS2

![PX4 Autonomous Mission Overview](PX4ProjectImage2.png)

Autonomous UAV mission stack using **PX4 v1.15+**, **Gazebo Harmonic**, **MAVROS2 (ROS 2 Humble)**, and **MAVSDK-Python**. Includes a custom obstacle world for obstacle avoidance simulation and full QGroundControl integration.

---

## Stack

| Component | Version |
|-----------|---------|
| PX4 Autopilot (SITL) | v1.15+ |
| Gazebo Harmonic | gz-sim 8.x |
| ROS 2 | Humble |
| MAVROS2 | ros-humble-mavros |
| MAVSDK-Python | 2.0+ |
| QGroundControl | Daily / Stable |
| Ubuntu | 22.04 |

---

## What's Included

| File | Description |
|------|-------------|
| `worlds/obstacle_world.sdf` | Gazebo Harmonic world with 5 static obstacles |
| `launch/simulation.launch.py` | ROS 2 launch: PX4 SITL + Gazebo Harmonic + MAVROS2 |
| `config/mavros_params.yaml` | MAVROS2 configuration (FCU URL, plugins, TF) |
| `obstacle_avoidance.py` | Mission script that navigates around all 5 obstacles |
| `mission.py` | Lawnmower scan pattern (3×3 grid, 9 waypoints) |
| `fly.py` | Minimal takeoff/hover/land test script |
| `plot_flight.py` | Post-flight telemetry visualization |

---

## Obstacle World Layout

Five static obstacles in `worlds/obstacle_world.sdf`, positioned relative to the PX4 SITL GPS origin (`47.397742°N, 8.545594°E`):

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

The `obstacle_avoidance.py` mission routes around all obstacles with ≥5 m lateral clearance at 20 m altitude.

---

## Installation

### 1 — PX4 + Gazebo Harmonic

```bash
# Clone PX4 (v1.15 or later supports gz_x500 for Gazebo Harmonic)
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
cd ~/PX4-Autopilot
bash Tools/setup/ubuntu.sh

# Copy custom obstacle world
cp worlds/obstacle_world.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
```

### 2 — ROS 2 Humble + MAVROS2

```bash
# ROS 2 Humble (if not already installed)
sudo apt install ros-humble-desktop python3-colcon-common-extensions

# MAVROS2 + GeographicLib datasets
sudo apt install ros-humble-mavros ros-humble-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh
```

### 3 — Python dependencies

```bash
pip install -r requirements.txt
```

### 4 — QGroundControl

Download from [qgroundcontrol.com](https://qgroundcontrol.com/). QGC auto-connects via UDP 14550 when PX4 SITL is running — no extra config needed.

---

## Running the Simulation

### Option A — ROS 2 launch (recommended)

Starts PX4 SITL + Gazebo Harmonic + MAVROS2 in one command:

```bash
source /opt/ros/humble/setup.bash
ros2 launch launch/simulation.launch.py
```

Then open QGroundControl (auto-connects on port 14550).

### Option B — Manual terminals

**Terminal 1 — PX4 SITL + Gazebo Harmonic:**
```bash
cd ~/PX4-Autopilot
rm -f build/px4_sitl_default/dataman
GZ_SIM_RESOURCE_PATH=/path/to/px4-autonomous-mission/worlds \
PX4_GZ_WORLD=obstacle_world make px4_sitl gz_x500
```
Wait for `Ready for takeoff!`

**Terminal 2 — MAVROS2:**
```bash
source /opt/ros/humble/setup.bash
ros2 run mavros mavros_node \
  --ros-args \
  -p fcu_url:=udp://:14540@ \
  -p gcs_url:=udp://@localhost:14550 \
  --params-file config/mavros_params.yaml
```

**Terminal 3 — MAVSDK server:**
```bash
mavsdk_server udpin://0.0.0.0:14540
```

---

## Running Missions

### Obstacle Avoidance Mission

```bash
python3 obstacle_avoidance.py
```

Flies 8 pre-planned waypoints around all 5 obstacles at 20 m altitude, then returns to launch.

### Lawnmower Scan Mission

```bash
python3 mission.py
```

Flies a 3×3 lawnmower grid pattern (9 waypoints, ~20 m step, 15 m altitude).

### Basic Flight Test

```bash
python3 fly.py
```

Takeoff, 10-second hover, land.

---

## MAVROS2 ROS Topics

Once running, MAVROS2 exposes the full MAVLink interface over ROS 2:

```bash
# Vehicle state
ros2 topic echo /mavros/state

# GPS position
ros2 topic echo /mavros/global_position/global

# Local position (ENU)
ros2 topic echo /mavros/local_position/pose

# Set OFFBOARD mode
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'OFFBOARD'}"

# Arm
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

---

## Port Reference

| Port | Protocol | Purpose |
|------|----------|---------|
| 14540 | UDP | MAVLink offboard API (MAVSDK / MAVROS) |
| 14550 | UDP | GCS link — QGroundControl auto-connects here |
| 50051 | TCP | MAVSDK gRPC server |

---

## Telemetry & Visualization

Each mission run generates a timestamped CSV:
```
time,latitude,longitude,alt_abs_m,alt_rel_m
15:14:01,47.3977508,8.5456073,488.13,15.01
...
```

Plot the last flight:
```bash
python3 plot_flight.py
```
Outputs `flight_plot.png` with flight path, altitude, and lat/lon over time.

---

## Real Hardware

Replace UDP with serial in `config/mavros_params.yaml`:
```yaml
fcu_url: "serial:///dev/ttyTHS1:921600"   # UART (Jetson / RPi)
# or
fcu_url: "serial:///dev/ttyUSB0:57600"    # USB
```

Delete `dataman` only applies to SITL. On real hardware just power-cycle between flights.

---

## Notes

- Delete `dataman` before each SITL restart: `rm -f ~/PX4-Autopilot/build/px4_sitl_default/dataman`
- Simulated battery drains — full sim restart required between flights
- Obstacle world GPS origin is `47.397742°N, 8.545594°E` (default PX4 Gazebo Harmonic home)
- `obstacle_avoidance.py` uses pre-planned waypoints; for reactive avoidance integrate PX4's Collision Prevention with a companion computer distance sensor publisher

---

## Author

Abel — UAV autonomy stack development
GitHub: github.com/abelxmendoza
