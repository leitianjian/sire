#!/usr/bin/env bash
set -euo pipefail
SIM2SIM_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SIM2SIM_PYTHON="${SIM2SIM_PYTHON:-/home/lqf/miniconda3/envs/unitree-rl/bin/python}"
if [[ ! -x "$SIM2SIM_PYTHON" ]]; then
    echo "Python not found: $SIM2SIM_PYTHON; set SIM2SIM_PYTHON to an environment with torch, mujoco, pygame and pyyaml." >&2
    exit 1
fi
# One local inference thread; do not compete with the background batch job.
export OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export PYGAME_HIDE_SUPPORT_PROMPT=1 SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS=1
if [[ "${1:-}" == "export" ]]; then
    shift
    exec "$SIM2SIM_PYTHON" "$SIM2SIM_DIR/export_policy.py" "$@"
fi
exec "$SIM2SIM_PYTHON" "$SIM2SIM_DIR/deploy_mujoco.py" "$@"
