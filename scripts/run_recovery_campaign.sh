#!/usr/bin/env bash
# Abort-to-recovery SITL campaign.
#
# Reuses scripts/run_integrated_seeding_sitl.sh. Each run has its own log
# directory and report. Earlier reports are not overwritten.
#
# CAMPAIGN=subset (default) flies one nominal, one traffic-conflict, and one
# clearance-expiration run. CAMPAIGN=full flies 5 nominal, 5 traffic-conflict,
# 5 clearance-expiration, and 3 marker-loss runs (18). A separate
# marker-acquisition-failure scenario is not implemented.
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT="${OUT:-$ROOT/evidence/recovery_campaign/$STAMP}"
mkdir -p "$OUT"

if [[ "${CAMPAIGN:-subset}" == "full" ]]; then
  # Acquisition-failure is not a separate simulator scenario. marker_loss is
  # the implemented descent-loss case. Do not count it twice.
  RUNS=(
    nominal nominal nominal nominal nominal
    traffic_conflict traffic_conflict traffic_conflict traffic_conflict traffic_conflict
    clearance_expired clearance_expired clearance_expired clearance_expired clearance_expired
    marker_loss marker_loss marker_loss
  )
else
  RUNS=(nominal traffic_conflict clearance_expired)
fi

for index in "${!RUNS[@]}"; do
  scenario="${RUNS[$index]}"
  name="$(printf '%02d_%s' "$index" "$scenario")"
  log_dir="/tmp/recovery_campaign/${STAMP}_${name}"
  dest="$OUT/$name"
  mkdir -p "$dest" "$log_dir"
  echo "== recovery campaign $name"
  set +e
  SCENARIO="$scenario" TIMEOUT="${TIMEOUT:-360}" LOG_DIR="$log_dir" \
    bash "$ROOT/scripts/run_integrated_seeding_sitl.sh"
  status=$?
  set +e
  if [[ -f "$log_dir/report.json" ]]; then
    # Compressed: each report logs every control tick (~2.5 MB as JSON).
    gzip -9 -c "$log_dir/report.json" > "$dest/sitl_report.json.gz"
  fi
  if [[ -f "$log_dir/startup.json" ]]; then
    cp "$log_dir/startup.json" "$dest/startup.json"
  fi
  printf '%s\n' "$status" > "$dest/exit_code.txt"
  echo "== $name exit $status"
done

echo "== reports $OUT"
