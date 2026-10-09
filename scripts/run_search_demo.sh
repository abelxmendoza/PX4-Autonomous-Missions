#!/usr/bin/env bash
# Watch one drone sweep the search field: PX4 SITL + Gazebo (window, opened by PX4) + the downward
# camera with live ArUco detections + a scripted lawnmower flight (tools/search_demo_flight.py).
#
#   scripts/run_search_demo.sh            # needs a display; Ctrl-C stops everything
#
# Like run_vo_validation.sh, it copies the world and the drone model into the PX4
# checkout, because PX4 loads both from there. It adds new files only (search_field.sdf,
# models/x500_search_cam); PX4's stock models are left untouched.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4="${PX4_DIR:-$HOME/PX4-Autopilot}"
LOG_DIR="${LOG_DIR:-/tmp/search_demo}"
mkdir -p "$LOG_DIR"
[[ -x "$PX4/build/px4_sitl_default/bin/px4" ]] || { echo "build PX4 SITL first: make px4_sitl" >&2; exit 2; }

install -m 0644 "$ROOT/worlds/search_field.sdf" "$PX4/Tools/simulation/gz/worlds/search_field.sdf"
mkdir -p "$PX4/Tools/simulation/gz/models/x500_search_cam"
cp -r "$ROOT/models/x500_search_cam/." "$PX4/Tools/simulation/gz/models/x500_search_cam/"

export GZ_IP=127.0.0.1   # every Gazebo consumer must match PX4's server (BUG-017)
export GZ_SIM_RESOURCE_PATH="$ROOT/models:$ROOT/worlds:$PX4/Tools/simulation/gz/models:$PX4/Tools/simulation/gz/worlds"
NVIDIA_EGL=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
[[ -f "$NVIDIA_EGL" ]] && export __EGL_VENDOR_LIBRARY_FILENAMES="$NVIDIA_EGL"

PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
  sleep 2
  pkill -f "px4_sitl_default/bin/[p]x4" 2>/dev/null || true   # [x] patterns never match this shell
  pkill -f "[g]z sim" 2>/dev/null || true
  pkill -f "tools/[s]earch_camera_view.py" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== PX4 SITL in search_field (logs: $LOG_DIR)"
( cd "$PX4" && rm -f build/px4_sitl_default/dataman && \
  PX4_GZ_WORLD=search_field PX4_SYS_AUTOSTART=4014 PX4_SIM_MODEL=gz_x500_search_cam \
  ./build/px4_sitl_default/bin/px4 -d > "$LOG_DIR/px4.log" 2>&1 ) &
PIDS+=($!)

for _ in $(seq 1 90); do grep -q "Startup script returned successfully" "$LOG_DIR/px4.log" 2>/dev/null && break; sleep 1; done
grep -q "Startup script returned successfully" "$LOG_DIR/px4.log" || { echo "PX4 did not start; see $LOG_DIR/px4.log" >&2; exit 1; }

# No `gz sim -g` here: PX4's own start script opens the Gazebo window unless HEADLESS=1
# (a second GUI would just duplicate it).

echo "== downward camera window"
python3 "$ROOT/tools/search_camera_view.py" --snapshot "$LOG_DIR/camera_latest.png" > "$LOG_DIR/camera.log" 2>&1 &
PIDS+=($!)

sleep 10
echo "== demo flight"
python3 "$ROOT/tools/search_demo_flight.py" 2>&1 | tee "$LOG_DIR/flight.log"
sleep 3
echo "== camera summary: $(tail -1 "$LOG_DIR/camera.log")"
