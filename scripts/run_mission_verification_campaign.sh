#!/usr/bin/env bash
# Headless PX4 SITL verification campaign.
#
# The full target is 17 runs (5 nominal, 3 weather, 3 airspace, 3 geofence,
# 3 marker-loss). CAMPAIGN=subset (the default) flies one of each, five runs.
# CAMPAIGN=full flies all 17. Each run starts its own PX4 SITL. A failed run
# is recorded and the campaign continues.
#
# Reports land in evidence/mission_verification/<index>_<scenario>/sitl_report.json.
# The previous integrated_seeding report is not copied.
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${OUT:-$ROOT/evidence/mission_verification}"
mkdir -p "$OUT"

if [[ "${CAMPAIGN:-subset}" == "full" ]]; then
  RUNS=(
    nominal nominal nominal nominal nominal
    weather_during_dispense weather_during_dispense weather_during_dispense
    airspace_during_dispense airspace_during_dispense airspace_during_dispense
    geofence_boundary geofence_boundary geofence_boundary
    marker_loss marker_loss marker_loss
  )
else
  RUNS=(
    nominal
    weather_during_dispense
    airspace_during_dispense
    geofence_boundary
    marker_loss
  )
fi

for index in "${!RUNS[@]}"; do
  scenario="${RUNS[$index]}"
  name="$(printf '%02d_%s' "$index" "$scenario")"
  log_dir="/tmp/mission_verification/${name}"
  dest="$OUT/$name"
  mkdir -p "$dest" "$log_dir"
  if [[ "$scenario" == "geofence_boundary" ]]; then
    timeout_s=50
  else
    timeout_s=360
  fi
  echo "== campaign run $name"
  set +e
  SCENARIO="$scenario" TIMEOUT="$timeout_s" LOG_DIR="$log_dir" \
    bash "$ROOT/scripts/run_integrated_seeding_sitl.sh"
  status=$?
  set +e
  if [[ -f "$log_dir/report.json" ]]; then
    # Compressed: each report logs every control tick (~2.5 MB as JSON).
    gzip -9 -c "$log_dir/report.json" > "$dest/sitl_report.json.gz"
  fi
  printf '%s\n' "$status" > "$dest/exit_code.txt"
  echo "== $name exit $status"
done

python3 "$ROOT/tools/mission_verification_summary.py" \
  --campaign "$OUT" \
  --out "$OUT/campaign_summary.json"
echo "== summary $OUT/campaign_summary.json"
