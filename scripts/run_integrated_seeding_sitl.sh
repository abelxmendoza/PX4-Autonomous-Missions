#!/usr/bin/env bash
# Headless PX4 SITL run of the integrated seeding sortie.
# Starts search_field with the downward camera, then tools/integrated_seeding_sitl.py.
# Writes /tmp/integrated_seeding_sitl/report.json.
# Do not start the ROS precision_land node: it would also publish trajectory setpoints.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# One PX4 SITL at a time on this machine: two instances fight over the same UDP ports, and
# two Gazebo servers running search_field publish on the same camera topic, so a drone can
# "see" frames from the other simulation. Runs from any checkout or session queue here.
# PX4 and Gazebo inherit the lock, so it is held until the whole simulation has exited.
SITL_LOCK="${SITL_LOCK:-/tmp/px4_sitl.lock}"
exec 9>"$SITL_LOCK"
if ! flock -n 9; then
  echo "== waiting for another PX4 SITL run to finish (lock: $SITL_LOCK)"
  flock 9
fi
PX4="${PX4_DIR:-$HOME/PX4-Autopilot}"
LOG_DIR="${LOG_DIR:-/tmp/integrated_seeding_sitl}"
mkdir -p "$LOG_DIR"
[[ -x "$PX4/build/px4_sitl_default/bin/px4" ]] || { echo "build PX4 SITL first" >&2; exit 2; }

install -m 0644 "$ROOT/worlds/search_field.sdf" "$PX4/Tools/simulation/gz/worlds/search_field.sdf"
mkdir -p "$PX4/Tools/simulation/gz/models/x500_search_cam"
cp -a "$ROOT/models/x500_search_cam/." "$PX4/Tools/simulation/gz/models/x500_search_cam/"
for marker in "$ROOT"/models/aruco_marker_*; do
  name="$(basename "$marker")"
  mkdir -p "$PX4/Tools/simulation/gz/models/$name"
  cp -a "$marker/." "$PX4/Tools/simulation/gz/models/$name/"
done

export GZ_IP=127.0.0.1
export HEADLESS=1
export PX4_DIR="$PX4"
export GZ_SIM_RESOURCE_PATH="$ROOT/models:$ROOT/worlds:${PX4}/Tools/simulation/gz/models:${PX4}/Tools/simulation/gz/worlds"
NVIDIA_EGL=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
[[ -f "$NVIDIA_EGL" ]] && export __EGL_VENDOR_LIBRARY_FILENAMES="$NVIDIA_EGL"

PX4_PID=""
kill_owned() {
  local pid="$1"
  local child
  while read -r child; do
    [[ -n "$child" ]] && kill_owned "$child"
  done < <(pgrep -P "$pid" 2>/dev/null || true)
  kill "$pid" 2>/dev/null || true
}
cleanup() {
  # Only the process group started by this script. Do not signal other PX4 instances.
  if [[ -n "${PX4_PID}" ]]; then
    kill -- "-${PX4_PID}" 2>/dev/null || true
    kill_owned "${PX4_PID}"
  fi
}
trap cleanup EXIT INT TERM

echo "== PX4 SITL search_field (headless, logs: $LOG_DIR)"
LAUNCHED_AT="$(date +%s)"
: > "$LOG_DIR/px4.log"
export PX4_GZ_WORLD=search_field
export PX4_SYS_AUTOSTART=4014
export PX4_SIM_MODEL=gz_x500_search_cam
# The child calls setsid() so it is not a process-group leader inherited from
# the shell. Its pid is the session we are allowed to stop.
PX4_PID="$(
  python3 - "$PX4" "$PX4/build/px4_sitl_default/bin/px4" "$LOG_DIR/px4.log" <<'PY'
import os, sys
root, binary, log_path = sys.argv[1:]
log = os.open(log_path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
pid = os.fork()
if pid > 0:
    print(pid, flush=True)
    os._exit(0)
os.setsid()
os.chdir(root)
os.dup2(log, 1)
os.dup2(log, 2)
os.close(log)
dataman = os.path.join(root, "build/px4_sitl_default/dataman")
if os.path.exists(dataman):
    os.remove(dataman)
os.execv(binary, [binary, "-d"])
PY
)"
echo "$PX4_PID" > "$LOG_DIR/px4.pid"

startup_state() {
  PYTHONPATH="$ROOT/src/px4_offboard${PYTHONPATH:+:$PYTHONPATH}" python3 - "$PX4_PID" "$LOG_DIR/px4.log" "$LAUNCHED_AT" <<'PY'
import os, sys
from pathlib import Path
from px4_offboard.px4_startup import process_gate
pid = int(sys.argv[1])
log = Path(sys.argv[2])
launched = float(sys.argv[3])
try:
    os.kill(pid, 0)
    alive = True
except OSError:
    alive = False
text = log.read_text(errors="replace") if log.exists() else ""
mtime = log.stat().st_mtime if log.exists() else None
print(process_gate(pid_alive=alive, log_text=text, log_mtime_s=mtime, launched_at_s=launched))
PY
}

state="NOT_STARTED"
for _ in $(seq 1 120); do
  state="$(startup_state)"
  [[ "$state" == "READY" ]] && break
  [[ "$state" == "FAILED" ]] && break
  sleep 1
done
printf '%s\n' "{\"state\":\"$state\",\"pid\":$PX4_PID,\"launched_at_s\":$LAUNCHED_AT}" > "$LOG_DIR/startup.json"
if [[ "$state" != "READY" ]]; then
  echo "PX4 startup $state; see $LOG_DIR/px4.log" >&2
  tail -40 "$LOG_DIR/px4.log" >&2 || true
  exit 1
fi

SCENARIO="${SCENARIO:-nominal}"
TIMEOUT="${TIMEOUT:-360}"
echo "== integrated seeding flight ($SCENARIO, timeout ${TIMEOUT}s)"
set +e
python3 "$ROOT/tools/integrated_seeding_sitl.py" --scenario "$SCENARIO" --timeout "$TIMEOUT" --report "$LOG_DIR/report.json" --trace "$LOG_DIR/trace.json" | tee "$LOG_DIR/flight.log"
status=${PIPESTATUS[0]}
set -e
echo "== flight exit $status, report $LOG_DIR/report.json"
exit "$status"
