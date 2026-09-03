#!/usr/bin/env bash
# One-shot: build (if needed) + launch full autonomy stack.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if [[ ! -f install/setup.bash ]]; then
  echo "Building workspace..."
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/install/setup.bash"

MODE="${1:-waypoints}"
EXTRA=()
if [[ "${2:-}" == "headless" ]]; then
  EXTRA+=(headless:=true)
fi

echo "Launching full stack (trajectory_mode=$MODE)..."
exec ros2 launch px4_offboard full_stack.launch.py \
  "trajectory_mode:=${MODE}" \
  "${EXTRA[@]}"
