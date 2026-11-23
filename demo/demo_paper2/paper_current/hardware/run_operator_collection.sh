#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 || $# -gt 4 ]]; then
  echo "Usage: ./run_operator_collection.sh SEED REPEAT SESSION_ID [NOTES]" >&2
  exit 2
fi

seed=$1
repeat=$2
session_id=$3
operator_notes=${4:-"marked flat indoor floor"}

case "$seed" in
  1|2|3|4) ;;
  *) echo "SEED must be 1, 2, 3, or 4" >&2; exit 2 ;;
esac
if ! [[ "$repeat" =~ ^[1-9][0-9]*$ ]]; then
  echo "REPEAT must be a positive integer" >&2
  exit 2
fi
if ! [[ "$session_id" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
  echo "SESSION_ID contains unsupported characters" >&2
  exit 2
fi

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
policy="$script_dir/policies/sire_seeds_1_4_20260924/sire_seed_${seed}_model_1200_jit.pt"
output_dir="$script_dir/operator_hardware_logs/$session_id"

if compgen -G "$output_dir/*.metadata.json" >/dev/null; then
  echo "SESSION_ID already contains a collection; choose a new ID: $output_dir" >&2
  exit 1
fi

exec python3 "$script_dir/deploy_go2_operator_collection.py" eth0 \
  --config "$script_dir/deploy_sire_1200.yaml" \
  --policy "$policy" \
  --coverage "$script_dir/go2_operator_coverage_v1.yaml" \
  --seed "$seed" \
  --repeat "$repeat" \
  --session-id "$session_id" \
  --output-dir "$output_dir" \
  --motor-lost-abort-increment 10 \
  --pretrial-settle-min-s 2.0 \
  --pretrial-settle-timeout-s 5.0 \
  --pretrial-stable-window-s 0.5 \
  --pretrial-max-abs-roll-pitch 0.20 \
  --pretrial-max-body-rate 0.25 \
  --pretrial-max-joint-speed-rms 0.25 \
  --notes "operator coverage v1; session=$session_id; $operator_notes"
