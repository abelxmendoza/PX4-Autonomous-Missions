#!/usr/bin/env bash
# Headless PX4 SITL touchdown check for precision landing.
# Starts search_field with the downward camera, flies until a marker is seen,
# then runs tools/precision_land_sitl.py. Writes /tmp/precision_land_sitl/report.json.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4="${PX4_DIR:-$HOME/PX4-Autopilot}"
LOG_DIR="${LOG_DIR:-/tmp/precision_land_sitl}"
mkdir -p "$LOG_DIR"
[[ -x "$PX4/build/px4_sitl_default/bin/px4" ]] || { echo "build PX4 SITL first" >&2; exit 2; }

install -m 0644 "$ROOT/worlds/search_field.sdf" "$PX4/Tools/simulation/gz/worlds/search_field.sdf"
mkdir -p "$PX4/Tools/simulation/gz/models/x500_search_cam"
cp -a "$ROOT/models/x500_search_cam/." "$PX4/Tools/simulation/gz/models/x500_search_cam/"
# Marker models are referenced by the world as model://aruco_marker_<id>.
for marker in "$ROOT"/models/aruco_marker_*; do
  name="$(basename "$marker")"
  mkdir -p "$PX4/Tools/simulation/gz/models/$name"
  cp -a "$marker/." "$PX4/Tools/simulation/gz/models/$name/"
done

export GZ_IP=127.0.0.1
export HEADLESS=1
export GZ_SIM_RESOURCE_PATH="$ROOT/models:$ROOT/worlds:${PX4}/Tools/simulation/gz/models:${PX4}/Tools/simulation/gz/worlds"
NVIDIA_EGL=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
[[ -f "$NVIDIA_EGL" ]] && export __EGL_VENDOR_LIBRARY_FILENAMES="$NVIDIA_EGL"

PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
  sleep 1
  pkill -f "px4_sitl_default/bin/[p]x4" 2>/dev/null || true
  pkill -f "[g]z sim" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== PX4 SITL search_field (headless, logs: $LOG_DIR)"
( cd "$PX4" && rm -f build/px4_sitl_default/dataman && \
  PX4_GZ_WORLD=search_field PX4_SYS_AUTOSTART=4014 PX4_SIM_MODEL=gz_x500_search_cam \
  ./build/px4_sitl_default/bin/px4 -d > "$LOG_DIR/px4.log" 2>&1 ) &
PIDS+=($!)

for _ in $(seq 1 120); do
  grep -q "Startup script returned successfully" "$LOG_DIR/px4.log" 2>/dev/null && break
  sleep 1
done
grep -q "Startup script returned successfully" "$LOG_DIR/px4.log" || {
  echo "PX4 did not start; see $LOG_DIR/px4.log" >&2
  tail -40 "$LOG_DIR/px4.log" >&2 || true
  exit 1
}

echo "== precision land flight"
set +e
python3 "$ROOT/tools/precision_land_sitl.py" --report "$LOG_DIR/report.json" | tee "$LOG_DIR/flight.log"
status=${PIPESTATUS[0]}
set -e
echo "== flight exit $status, report $LOG_DIR/report.json"
exit "$status"
