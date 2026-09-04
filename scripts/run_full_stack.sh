#!/usr/bin/env bash
# One-shot: build (if needed) + launch full autonomy stack.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

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

echo "Launching full stack (trajectory_mode=$MODE)..."
exec ros2 launch px4_offboard full_stack.launch.py \
  "trajectory_mode:=${MODE}" \
  "${EXTRA[@]}"
