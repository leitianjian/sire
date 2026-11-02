#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: ./run_paper_seed_session.sh SEED REPEATS SESSION_ID [NOTES]"
}

if [[ $# -lt 3 || $# -gt 4 ]]; then
  usage >&2
  exit 2
fi

seed=$1
repeats=$2
session_id=$3
operator_notes=${4:-"marked flat indoor floor"}

case "$seed" in
  1|2|3|4) ;;
  *) echo "SEED must be 1, 2, 3, or 4" >&2; exit 2 ;;
esac
if ! [[ "$repeats" =~ ^[1-9][0-9]*$ ]]; then
  echo "REPEATS must be a positive integer" >&2
  exit 2
fi
if ! [[ "$session_id" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
  echo "SESSION_ID may contain only letters, numbers, dot, underscore, and dash" >&2
  exit 2
fi

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
policy="$script_dir/policies/sire_seeds_1_4_20260924/sire_seed_${seed}_model_1200_jit.pt"
output_dir="$script_dir/paper_hardware_logs/$session_id"

exec python3 "$script_dir/deploy_go2_paper_session.py" eth0 \
  --config "$script_dir/deploy_sire_1200.yaml" \
  --policy "$policy" \
  --protocol "$script_dir/go2_paper_protocol_v3.yaml" \
  --seed "$seed" \
  --repeats "$repeats" \
  --session-id "$session_id" \
  --checkpoint-iteration 1200 \
  --robot-id go2 \
  --motor-lost-abort-increment 10 \
  --pretrial-settle-min-s 2.0 \
  --pretrial-settle-timeout-s 5.0 \
  --pretrial-stable-window-s 0.5 \
  --pretrial-max-abs-roll-pitch 0.20 \
  --pretrial-max-body-rate 0.25 \
  --pretrial-max-joint-speed-rms 0.25 \
  --output-dir "$output_dir" \
  --notes "paper protocol v3; session=$session_id; $operator_notes"
