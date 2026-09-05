#!/usr/bin/env bash
# One-shot: build (if needed) + launch full autonomy stack.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
# shellcheck source=./_stack_cleanup.sh
source "$ROOT/scripts/_stack_cleanup.sh"

if [[ ! -f install/setup.bash ]]; then
  echo "Building workspace..."
  # shellcheck disable=SC1091
  set +u; source /opt/ros/humble/setup.bash; set -u
  colcon build --symlink-install
fi

# ROS setup scripts reference unset vars under `set -u` (e.g. AMENT_TRACE_SETUP_FILES).
# shellcheck disable=SC1091
set +u; source /opt/ros/humble/setup.bash; set -u
# shellcheck disable=SC1091
set +u; source "$ROOT/install/setup.bash"; set -u

MODE="${1:-waypoints}"
EXTRA=()
if [[ "${2:-}" == "headless" ]]; then
  EXTRA+=(headless:=true)
fi

LAUNCH_PID=""
FINALIZED=0
finish_stack() {
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
}
trap finish_stack EXIT INT TERM

echo "Launching full stack (trajectory_mode=$MODE)..."
ros2 launch px4_offboard full_stack.launch.py \
  "trajectory_mode:=${MODE}" \
  "${EXTRA[@]}" &
LAUNCH_PID=$!
wait "$LAUNCH_PID"
