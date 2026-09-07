#!/usr/bin/env bash
# Recruiter-ready demo: launch the visible simulation and create a flight report.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=./_stack_cleanup.sh
source "$ROOT/scripts/_stack_cleanup.sh"
MODE="${1:-waypoints}"
PX4_CHECKOUT="${PX4_DIR:-${HOME}/PX4-Autopilot}"
ARTIFACT_DIR="$ROOT/demo_artifacts"
RUN_MARKER="$ARTIFACT_DIR/.run_started"
LAUNCH_PID=""
FINALIZED=0

case "$MODE" in
  waypoints|course|circle|gps-denied) ;;
  *)
    echo "Usage: $0 [waypoints|course|circle|gps-denied]" >&2
    exit 2
    ;;
esac

mkdir -p "$ARTIFACT_DIR"
export MPLCONFIGDIR="$ARTIFACT_DIR/.matplotlib"
mkdir -p "$MPLCONFIGDIR"
touch "$RUN_MARKER"

finish_demo() {
  if [[ "$FINALIZED" -eq 1 ]]; then
    return
  fi
  FINALIZED=1

  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    # Bounded: ros2 launch can hang here indefinitely if even one child
    # doesn't respond to SIGINT (observed with lidar_sectors/flight_trail/
    # offboard_mission Node actions and the gcs_heartbeat.py TimerAction),
    # which would otherwise stop stack_cleanup below from ever running.
    wait_with_timeout "$LAUNCH_PID" 10
  fi
  # ros2 launch's shutdown cascade doesn't reliably reach every spawned
  # child (e.g. gcs_heartbeat.py can survive it) — clean up explicitly,
  # regardless of whether the graceful wait above actually finished.
  stack_cleanup
  # stack_cleanup only targets the children; if ros2 launch itself is
  # still around (e.g. stuck on an internal wait of its own), take it too.
  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -KILL "$LAUNCH_PID" 2>/dev/null || true
  fi

  latest_log="$(find "$ROOT" -maxdepth 1 -type f -name 'flight_log_mission_*.csv' -newer "$RUN_MARKER" -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)"
  if [[ -n "$latest_log" ]]; then
    report="$ARTIFACT_DIR/flight_report_$(date +%Y%m%d_%H%M%S).png"
    python3 "$ROOT/plot_flight.py" --log "$latest_log" --output "$report" --no-show || true
    echo
    echo "Demo artifacts"
    echo "  Telemetry: $latest_log"
    echo "  Report:    $report"
  else
    echo "No new flight log was created; report generation skipped."
  fi
}

trap finish_demo EXIT INT TERM

cd "$ROOT"
if [[ ! -d "$PX4_CHECKOUT/Tools/simulation/gz/worlds" ]]; then
  echo "PX4 checkout not found at: $PX4_CHECKOUT" >&2
  echo "Set PX4_DIR to your PX4-Autopilot path and retry." >&2
  exit 2
fi

# PX4's startup script resolves named worlds only inside its own checkout.
# Keep that runtime copy synchronized with this project's canonical world.
install -m 0644 \
  "$ROOT/worlds/obstacle_world.sdf" \
  "$PX4_CHECKOUT/Tools/simulation/gz/worlds/obstacle_world.sdf"

# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
set -u
echo "Preparing demo package..."
colcon build --symlink-install --packages-select px4_msgs px4_offboard
# shellcheck disable=SC1091
set +u
source "$ROOT/install/setup.bash"
set -u

echo "============================================================"
echo " PX4 AUTONOMOUS MISSION — RECRUITER DEMO"
echo " Mission: $MODE | Gazebo: visible | Safety: geofence enabled"
echo " Optional: open QGroundControl before launch for split-screen telemetry."
echo " Press Ctrl+C after landing to generate the flight report."
echo "============================================================"

TRAJECTORY_MODE="$MODE"
USE_VIO=false
GPS_FAILURE=false
DETECTION_MARGIN=5.0
if [[ "$MODE" == "gps-denied" ]]; then
  TRAJECTORY_MODE=course
  USE_VIO=true
  GPS_FAILURE=true
  # The course already provides mapped clearance. A tighter live-sensor
  # margin avoids turning toward a neighboring obstacle before the VIO demo.
  DETECTION_MARGIN=2.5
  echo " GPS-denied mode: PX4 GPS failure + external-vision EKF fusion"
fi

ros2 launch px4_offboard full_stack.launch.py \
  "px4_dir:=$PX4_CHECKOUT" \
  "trajectory_mode:=$TRAJECTORY_MODE" \
  "avoidance_strategy:=climb" \
  "obstacle_source:=sensor_only" \
  "detection_margin_m:=$DETECTION_MARGIN" \
  "demo_mode:=true" \
  "use_vio:=$USE_VIO" \
  "gps_px4_failure_inject:=$GPS_FAILURE" \
  "headless:=false" &
LAUNCH_PID=$!
wait "$LAUNCH_PID"
