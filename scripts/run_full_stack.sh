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
    # Snapshot the launch tree's PIDs before signaling it — a child only
    # becomes an orphan (reparented to init) once shutdown starts falling
    # apart, so capturing now still sees the complete, correct tree. This
    # is what lets kill_stack_pids reach orphans later without resorting
    # to matching on process name/command line (which would also catch a
    # concurrent session's identically-named processes on a shared box).
    capture_stack_pids "$LAUNCH_PID"
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    # Bounded: ros2 launch can hang here indefinitely if even one child
    # doesn't respond to SIGINT (observed with lidar_sectors/flight_trail/
    # offboard_mission Node actions and the gcs_heartbeat.py TimerAction),
    # which would otherwise stop kill_stack_pids below from ever running.
    # 30s comfortably exceeds full_stack.launch.py's longest TimerAction
    # delay (offboard_mission at 24s) — shutting down soon after launch
    # would otherwise miss that child entirely: it hasn't spawned yet at
    # the capture above, so waiting for it to actually exist (even though
    # we're about to kill it) is what lets the re-capture below see it.
    wait_with_timeout "$LAUNCH_PID" 30
  fi
  # A child that was still just a pending TimerAction at the capture above
  # may have spawned during the wait — re-capture so kill_stack_pids can
  # reach it too, not just what existed at shutdown-request time.
  if [[ -n "$LAUNCH_PID" ]]; then
    capture_stack_pids "$LAUNCH_PID"
  fi
  # ros2 launch's shutdown cascade doesn't reliably reach every spawned
  # child (e.g. gcs_heartbeat.py can survive it) — clean up explicitly,
  # regardless of whether the graceful wait above actually finished.
  kill_stack_pids
}
trap finish_stack EXIT INT TERM

echo "Launching full stack (trajectory_mode=$MODE)..."
gz_sim_snapshot_before
ros2 launch px4_offboard full_stack.launch.py \
  "trajectory_mode:=${MODE}" \
  "${EXTRA[@]}" &
LAUNCH_PID=$!
# gz sim detaches from the process tree almost immediately, so this has to
# catch it in the narrow window right after our own launch spawns it —
# see the comment on gz_sim_track_new in _stack_cleanup.sh. Returns as
# soon as it's found, so this rarely adds real delay (PX4/Gazebo boot
# takes at least this long anyway).
gz_sim_track_new 45
wait "$LAUNCH_PID"
