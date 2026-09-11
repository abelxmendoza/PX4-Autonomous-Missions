# Developing this repo from macOS

No ROS 2 / Gazebo / PX4 SITL here — that's Linux-only. This covers the parts
of the stack that run fully on macOS, mirroring the `pure-python-tests`,
`web-tests`, and `vv-regression` jobs in `.github/workflows/unit-tests.yml`.

## One-time setup

```bash
python3 -m venv .venv-mac
source .venv-mac/bin/activate
pip install pytest opencv-contrib-python-headless numpy pandas matplotlib mavsdk pymavlink
pip install -e src/px4_offboard --no-deps

cd web/replay && npm install
```

## Day to day

Pure-Python logic + avoidance/state-machine tests (no ROS import needed):

```bash
source .venv-mac/bin/activate
cd src/px4_offboard
python3 -m pytest -q test/test_mission_logic.py test/test_mission_state.py \
  test/test_mission_executive.py test/test_flight_vv.py test/test_path_planner.py \
  test/test_lidar_sectors.py test/test_localization_logic.py test/test_vio_bridge.py \
  test/test_vio_noise.py test/test_camera_frame.py test/test_vision_marker.py \
  test/test_vision_marker_detect.py
```

`test/test_offboard_mission_node.py` is skipped here on purpose — it needs a
real rclpy + built px4_msgs (Linux only); it self-skips via
`pytest.importorskip`.

V&V regression against the recorded flight logs (validates state-machine /
fence / avoidance requirements without a live SITL run):

```bash
source .venv-mac/bin/activate
for log in web/replay/data/*.csv; do
  PYTHONPATH=src/px4_offboard python3 -m px4_offboard.vv_replay "$log"
done
```

Web flight-replay viewer + its unit tests:

```bash
cd web/replay
npm test              # vitest, lib.js logic (parseCsv, toWorld, etc.)
npx vite               # local dev server for the replay/demo pages, if wanted
```

Plotting an existing telemetry CSV (no PX4 connection required):

```bash
source .venv-mac/bin/activate
python3 plot_flight.py <path-to-csv>
```

## What still needs the Linux box

`run_demo.sh`, `full_stack.launch.py`, anything importing `rclpy` at module
level as a live node (`offboard_mission.py`, `lidar_sectors.py`, etc. when
actually run as nodes rather than imported for their logic), Gazebo, Micro
XRCE-DDS, and PX4 SITL itself.

## Fixed while setting this up

`vision_marker_detect.py`'s `detect_largest_marker` indexed `ids[i][0]`,
which assumed OpenCV's ArUco `ids` array is shape `(N,1)`. OpenCV 5.0 (what
`pip install opencv-contrib-python-headless` gives you today) returns shape
`(N,)` instead, so that raised `IndexError`. Now uses `np.ravel(ids[i])[0]`,
which handles both shapes.
