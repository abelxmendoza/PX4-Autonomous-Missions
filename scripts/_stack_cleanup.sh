#!/usr/bin/env bash
# Shared cleanup helper for scripts that launch full_stack.launch.py.
#
# Why this exists: ros2 launch's SIGINT/SIGTERM shutdown does not reliably
# cascade to every spawned child. In practice, gcs_heartbeat.py (started via
# a delayed ExecuteProcess action) has been observed surviving for hours as
# an orphan after the parent `ros2 launch` process was gone, even when the
# launch was stopped the "correct" way (SIGINT to the launch PID + wait).
#
# This is PID-scoped, not pattern-scoped. An earlier version of this helper
# used `pkill -f <pattern>` (e.g. matching on "px4_sitl_default/bin/px4"),
# which is unsafe on a machine where more than one session may have its own
# full_stack.launch.py running at the same time — it kills every process
# whose command line matches, including a concurrent session's, not just
# this script's own launch tree. Capturing the exact descendant PIDs before
# shutdown begins (while they're still attached to $LAUNCH_PID) and killing
# only those specific PIDs avoids that collateral damage, while still
# reaching orphans that get reparented to init once launch's own shutdown
# starts falling apart.

# Recursively print every descendant PID of $1 (not including $1 itself).
_stack_collect_descendants() {
  local pid="$1" child
  for child in $(pgrep -P "$pid" 2>/dev/null); do
    echo "$child"
    _stack_collect_descendants "$child"
  done
}

# Snapshot every current descendant of $1 into the global STACK_PIDS array,
# merging into whatever's already captured rather than resetting it — call
# this once BEFORE sending any shutdown signal to $1, and again AFTER
# wait_with_timeout, since a TimerAction-delayed child (e.g. offboard_mission
# on a 24s delay) can spawn during the shutdown grace period itself and
# would otherwise never appear in a single earlier snapshot. A child only
# becomes an orphan (reparented to init) once launch's own shutdown starts
# failing to reap it, so each snapshot still has the correct tree as of
# the moment it's taken.
capture_stack_pids() {
  local root_pid="$1"
  STACK_PIDS=("${STACK_PIDS[@]:-}" "$root_pid")
  local pid
  while IFS= read -r pid; do
    if [[ -n "$pid" ]] && [[ ! " ${STACK_PIDS[*]} " == *" $pid "* ]]; then
      STACK_PIDS+=("$pid")
    fi
  done < <(_stack_collect_descendants "$root_pid")
}

# gz sim double-forks and detaches from our process tree almost immediately
# (observed: reparented to the user's systemd --user session within moments
# of spawning) — a PID-tree walk can never find it as a descendant, no
# matter how early it's taken. Diff gz-sim PIDs before/after our own launch
# starts it instead, in the narrow window right after spawn: call
# gz_sim_snapshot_before right before starting `ros2 launch`, then
# gz_sim_track_new right after backgrounding it. This is far safer than
# pattern-matching "gz sim" at shutdown time (whenever that happens to be,
# possibly hours later) — it only has to avoid a race with a concurrent
# session's gz sim spawning in this same few-second window, not at any
# point in the entire run.
GZ_SIM_PIDS=()
_GZ_SIM_BEFORE=""

gz_sim_snapshot_before() {
  # `|| true` matters here: under pipefail, pgrep finding zero matches (the
  # normal case — this runs before gz sim exists yet) makes the pipeline's
  # exit status 1 even though tr itself succeeds, which set -e would then
  # treat as this whole assignment failing and abort the script.
  _GZ_SIM_BEFORE=" $(pgrep -f 'gz sim ' 2>/dev/null | tr '\n' ' ' || true) "
}

gz_sim_track_new() {
  local timeout="${1:-45}" waited=0 pid after
  while (( waited < timeout )); do
    sleep 3
    ((waited+=3)) || true
    after=" $(pgrep -f 'gz sim ' 2>/dev/null | tr '\n' ' ' || true) "
    for pid in $after; do
      if [[ "$_GZ_SIM_BEFORE" != *" $pid "* ]] \
        && [[ ! " ${GZ_SIM_PIDS[*]:-} " == *" $pid "* ]]; then
        GZ_SIM_PIDS+=("$pid")
      fi
    done
    if (( ${#GZ_SIM_PIDS[@]} > 0 )); then
      return
    fi
  done
}

# Kill every PID captured by capture_stack_pids or gz_sim_track_new,
# regardless of whether it has since been reparented — this is what
# actually reaches orphans like gcs_heartbeat.py that ros2 launch's own
# shutdown cascade misses. Never touches a process just because its
# command line matches a pattern outside the narrow gz-sim spawn check.
kill_stack_pids() {
  local pid
  for pid in "${STACK_PIDS[@]:-}" "${GZ_SIM_PIDS[@]:-}"; do
    [[ -n "$pid" ]] && kill -TERM "$pid" 2>/dev/null || true
  done
  sleep 1
  for pid in "${STACK_PIDS[@]:-}" "${GZ_SIM_PIDS[@]:-}"; do
    [[ -n "$pid" ]] && kill -KILL "$pid" 2>/dev/null || true
  done
}

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
