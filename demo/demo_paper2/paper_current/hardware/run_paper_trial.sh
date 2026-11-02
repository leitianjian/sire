#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: ./run_paper_trial.sh SEED SEGMENT REPEAT [NOTES]

SEED: 1, 2, 3, or 4
SEGMENT:
  paper_v3_stand_08
  paper_v3_fore_aft_050
  paper_v3_lateral_pair_050
  paper_v3_yaw_pair_080
REPEAT: positive integer
EOF
}

if [[ $# -lt 3 || $# -gt 4 ]]; then
  usage >&2
  exit 2
fi

seed=$1
segment=$2
repeat=$3
operator_notes=${4:-"marked flat indoor floor"}

case "$seed" in
  1|2|3|4) ;;
  *) echo "SEED must be 1, 2, 3, or 4" >&2; exit 2 ;;
esac

case "$segment" in
  paper_v3_stand_08|paper_v3_fore_aft_050|paper_v3_lateral_pair_050|paper_v3_yaw_pair_080) ;;
  *) echo "Unknown paper segment: $segment" >&2; usage >&2; exit 2 ;;
esac

if ! [[ "$repeat" =~ ^[1-9][0-9]*$ ]]; then
  echo "REPEAT must be a positive integer" >&2
  exit 2
fi

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
policy="$script_dir/policies/sire_seeds_1_4_20260924/sire_seed_${seed}_model_1200_jit.pt"
config="$script_dir/deploy_sire_1200.yaml"
protocol="$script_dir/go2_paper_protocol_v3.yaml"
output_dir=${PAPER_OUTPUT_DIR:-"$script_dir/paper_hardware_logs"}

for required in "$policy" "$config" "$protocol"; do
  if [[ ! -f "$required" ]]; then
    echo "Required file not found: $required" >&2
    exit 1
  fi
done

echo "Formal hardware trial"
echo "  seed:       $seed"
echo "  segment:    $segment"
echo "  repeat:     $repeat"
echo "  policy:     $policy"
echo "  output:     $output_dir"

exec python3 "$script_dir/deploy_go2_segmented.py" eth0 \
  --config "$config" \
  --policy "$policy" \
  --protocol "$protocol" \
  --segment "$segment" \
  --repeat "$repeat" \
  --policy-seed "$seed" \
  --checkpoint-iteration 1200 \
  --robot-id go2 \
  --foot-contact-source none \
  --output-dir "$output_dir" \
  --notes "paper protocol v3; $operator_notes"
