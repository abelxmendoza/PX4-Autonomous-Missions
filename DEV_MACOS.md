# Developing this repo from macOS

No ROS 2 / Gazebo / PX4 SITL here — that's Linux-only. This covers the parts
of the stack that run fully on macOS, mirroring the `unit-and-verification`,
`browser-replay`, and `recorded-flight-regression` jobs in `.github/workflows/unit-tests.yml`.

## One-time setup

```bash
python3 -m venv .venv-mac
source .venv-mac/bin/activate
pip install pytest opencv-contrib-python-headless numpy pandas matplotlib mavsdk pymavlink
pip install -e src/px4_offboard --no-deps

cd web/replay && npm install
```

## Day to day

Logic and verifier tests (ROS-dependent modules explicitly skip):

```bash
source .venv-mac/bin/activate
PYTHONPATH=src/px4_offboard python3 -m pytest src/px4_offboard/test -q
python3 scripts/verify_evidence.py
```

Run these from the repository root. The evidence command needs only the Python standard library and includes preserved before/after failures, GPS-aiding-loss evidence and current cooperative recordings. It compares raw input hashes, verifier reports and browser exports. Skips are not flight validation.

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
