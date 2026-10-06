#!/usr/bin/env bash
# Fly one headless mission with the stereo+IMU fusion node running, then
# store the flight log and replay it through the offline verifier.
#
#   scripts/run_vo_validation.sh <tag> [control_mode] [position_source]
#
#   control_mode     position (default) | velocity_pid
#   position_source  px4 (default)      | fusion   (only matters for velocity_pid)
#
# Output: demo_artifacts/vo_validation/<tag>.csv + <tag>.launch.log
# Fusion never feeds PX4 here (publish_to_px4 stays false); its estimate is
# only measured against PX4's own, so a run cannot be "helped" by the thing
# being validated -- except in position_source=fusion, where the outer PID
# loop deliberately closes on it.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=./_stack_cleanup.sh
source "$ROOT/scripts/_stack_cleanup.sh"

TAG="${1:?usage: $0 <tag> [control_mode] [position_source]}"
CONTROL_MODE="${2:-position}"
POSITION_SOURCE="${3:-px4}"
PX4_CHECKOUT="${PX4_DIR:-${HOME}/PX4-Autopilot}"
MAX_RUN_S="${MAX_RUN_S:-260}"
HEADLESS="${HEADLESS:-true}"
# sensor_only = LiDAR only (what run_demo.sh flies); hybrid = LiDAR + known map.
OBSTACLE_SOURCE="${OBSTACLE_SOURCE:-sensor_only}"
# course = A* route around the mapped obstacles (what the GPS-denied demo flies);
# waypoints = reactive path through the field, which is dominated by the
# avoidance weaknesses recorded in bugs/ rather than by what is being validated.
TRAJECTORY_MODE="${TRAJECTORY_MODE:-course}"
DETECTION_MARGIN="${DETECTION_MARGIN:-2.5}"
OUT_DIR="$ROOT/demo_artifacts/vo_validation"
RUN_MARKER="$OUT_DIR/.run_started_$TAG"
LAUNCH_PID=""
FINALIZED=0

mkdir -p "$OUT_DIR"
touch "$RUN_MARKER"

finish() {
  [[ "$FINALIZED" -eq 1 ]] && return
  FINALIZED=1
  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    capture_stack_pids "$LAUNCH_PID"
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait_with_timeout "$LAUNCH_PID" 30
  fi
  [[ -n "$LAUNCH_PID" ]] && capture_stack_pids "$LAUNCH_PID"
  kill_stack_pids
  local log
  log="$(find "$ROOT" -maxdepth 1 -type f -name 'flight_log_mission_*.csv' -newer "$RUN_MARKER" -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)"
  if [[ -n "$log" ]]; then
    mv "$log" "$OUT_DIR/$TAG.csv"
    echo "flight log: $OUT_DIR/$TAG.csv"
  else
    echo "no flight log produced" >&2
  fi
}
trap finish EXIT INT TERM

cd "$ROOT"
[[ -d "$PX4_CHECKOUT/Tools/simulation/gz/worlds" ]] || { echo "PX4 checkout not found: $PX4_CHECKOUT" >&2; exit 2; }

# PX4 resolves the world and the spawned model from inside its own checkout.
install -m 0644 "$ROOT/worlds/obstacle_world.sdf" "$PX4_CHECKOUT/Tools/simulation/gz/worlds/obstacle_world.sdf"
for m in lidar_2d_v2 x500_lidar_2d mono_cam mono_cam_right; do
  mkdir -p "$PX4_CHECKOUT/Tools/simulation/gz/models/$m"
  cp -r "$ROOT/models/$m/." "$PX4_CHECKOUT/Tools/simulation/gz/models/$m/"
done

set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --symlink-install --packages-select px4_msgs px4_offboard >/dev/null
set +u
source "$ROOT/install/setup.bash"
set -u

echo "== $TAG: control_mode=$CONTROL_MODE position_source=$POSITION_SOURCE obstacle_source=$OBSTACLE_SOURCE trajectory=$TRAJECTORY_MODE =="
gz_sim_snapshot_before
ros2 launch px4_offboard full_stack.launch.py \
  "px4_dir:=$PX4_CHECKOUT" \
  "trajectory_mode:=$TRAJECTORY_MODE" \
  "avoidance_strategy:=climb" \
  "obstacle_source:=$OBSTACLE_SOURCE" \
  "detection_margin_m:=$DETECTION_MARGIN" \
  "demo_mode:=true" \
  "use_camera:=true" \
  "use_stereo:=true" \
  "use_sensor_fusion_vio:=true" \
  "control_mode:=$CONTROL_MODE" \
  "position_source:=$POSITION_SOURCE" \
  "headless:=$HEADLESS" >"$OUT_DIR/$TAG.launch.log" 2>&1 &
LAUNCH_PID=$!
gz_sim_track_new 45

# Run until the mission reaches a terminal state, plus a short settle, or the
# wall-clock cap -- whichever comes first.
START=$(date +%s)
while kill -0 "$LAUNCH_PID" 2>/dev/null; do
  sleep 3
  NOW=$(date +%s)
  if (( NOW - START > MAX_RUN_S )); then echo "wall-clock cap reached"; break; fi
  if grep -q "MISSION PHASE | .* -> \(LANDING\|FAILSAFE\)" "$OUT_DIR/$TAG.launch.log" 2>/dev/null; then
    echo "terminal state reached; settling"
    sleep 12
    break
  fi
done
grep "MISSION PHASE" "$OUT_DIR/$TAG.launch.log" | sed 's/.*MISSION PHASE/MISSION PHASE/' || true
