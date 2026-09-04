#!/usr/bin/env bash
# Recruiter-ready demo: launch the visible simulation and create a flight report.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-waypoints}"
PX4_CHECKOUT="${PX4_DIR:-${HOME}/PX4-Autopilot}"
ARTIFACT_DIR="$ROOT/demo_artifacts"
RUN_MARKER="$ARTIFACT_DIR/.run_started"
LAUNCH_PID=""
FINALIZED=0

case "$MODE" in
  waypoints|course|circle) ;;
  *)
    echo "Usage: $0 [waypoints|course|circle]" >&2
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
    wait "$LAUNCH_PID" 2>/dev/null || true
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
colcon build --symlink-install --packages-select px4_offboard
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

ros2 launch px4_offboard full_stack.launch.py \
  "px4_dir:=$PX4_CHECKOUT" \
  "trajectory_mode:=$MODE" \
  "avoidance_strategy:=climb" \
  "detection_margin_m:=5.0" \
  "demo_mode:=true" \
  "headless:=false" &
LAUNCH_PID=$!
wait "$LAUNCH_PID"
