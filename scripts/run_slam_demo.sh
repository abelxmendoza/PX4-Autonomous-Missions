#!/usr/bin/env bash
# Watch the LiDAR drone map the obstacle course: PX4 SITL + Gazebo (window opened by PX4) +
# tools/slam_flight.py, which runs 2-D SLAM and a layered costmap on the real Gazebo scans and
# draws them in the Gazebo window (costmap tiles, white map points, a red dead-reckoning trail
# and a green SLAM trail).
#
#   scripts/run_slam_demo.sh            # needs a display; Ctrl-C stops everything
#   HEADLESS=1 scripts/run_slam_demo.sh # no window (CI-style run)
#
# The world and the drone model are copied into the PX4 checkout first, because PX4 loads
# both from there (as run_search_demo.sh does).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4="${PX4_DIR:-$HOME/PX4-Autopilot}"
LOG_DIR="${LOG_DIR:-/tmp/slam_demo}"
mkdir -p "$LOG_DIR"
[[ -x "$PX4/build/px4_sitl_default/bin/px4" ]] || { echo "build PX4 SITL first: make px4_sitl" >&2; exit 2; }

install -m 0644 "$ROOT/worlds/obstacle_world.sdf" "$PX4/Tools/simulation/gz/worlds/obstacle_world.sdf"
for m in x500_lidar_2d lidar_2d_v2; do
  mkdir -p "$PX4/Tools/simulation/gz/models/$m"
  cp -r "$ROOT/models/$m/." "$PX4/Tools/simulation/gz/models/$m/"
done

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
}
trap cleanup EXIT INT TERM

echo "== PX4 SITL in obstacle_world with the 2-D LiDAR x500 (logs: $LOG_DIR)"
( cd "$PX4" && rm -f build/px4_sitl_default/dataman && \
  PX4_GZ_WORLD=obstacle_world PX4_SYS_AUTOSTART=4013 PX4_SIM_MODEL=gz_x500_lidar_2d \
  ./build/px4_sitl_default/bin/px4 -d > "$LOG_DIR/px4.log" 2>&1 ) &
PIDS+=($!)

for _ in $(seq 1 90); do grep -q "Startup script returned successfully" "$LOG_DIR/px4.log" 2>/dev/null && break; sleep 1; done
grep -q "Startup script returned successfully" "$LOG_DIR/px4.log" || { echo "PX4 did not start; see $LOG_DIR/px4.log" >&2; exit 1; }

sleep 10
echo "== SLAM flight"
python3 "$ROOT/tools/slam_flight.py" --report "$LOG_DIR/slam_report.json" --trace "$LOG_DIR/slam_trace.json" "$@" 2>&1 | tee "$LOG_DIR/flight.log"
