#!/usr/bin/env bash
# Shared cleanup helper for scripts that launch full_stack.launch.py.
#
# Why this exists: ros2 launch's SIGINT/SIGTERM shutdown does not reliably
# cascade to every spawned child. In practice, gcs_heartbeat.py (started via
# a delayed ExecuteProcess action) has been observed surviving for hours as
# an orphan after the parent `ros2 launch` process was gone, even when the
# launch was stopped the "correct" way (SIGINT to the launch PID + wait).
# Source this file and call stack_cleanup after stopping the main launch
# process — it kills every known stack process by pattern, independent of
# whatever launch's own cascade did or didn't reach.


# Wait up to $2 seconds for PID $1 to exit; returns as soon as it does.
# Never blocks past the timeout — ros2 launch's own graceful shutdown can
# hang indefinitely if any one of its children doesn't respond to SIGINT,
# so this bounds how long we ever wait for it before forcing cleanup.
wait_with_timeout() {
  local pid="$1" timeout="$2" waited=0
  while kill -0 "$pid" 2>/dev/null && (( waited < timeout )); do
    sleep 1
    ((waited++)) || true
  done
}

stack_cleanup() {
  local root
  root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  local patterns=(
    "px4_sitl_default/bin/px4"
    "gz sim .*obstacle_world"
    "MicroXRCEAgent"
    "install/px4_offboard/lib/px4_offboard/"
    "${root}/scripts/gcs_heartbeat.py"
  )
  for pat in "${patterns[@]}"; do
    pkill -TERM -f "$pat" 2>/dev/null || true
  done
  sleep 1
  for pat in "${patterns[@]}"; do
    pkill -KILL -f "$pat" 2>/dev/null || true
  done
}
