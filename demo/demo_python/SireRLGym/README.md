# SireRLGym

`SireRLGym` trains a GO2 policy on CPU using one independent Sire simulator per
environment. Physics advances through the native `SireRLBatchStepper`, whose
persistent worker pool is reused for the lifetime of the environment.

## 1. Create the Python environment

From the repository root:

```bash
python -m venv .venv
.venv/bin/pip install -r demo/demo_python/SireRLGym/requirements.txt
```

## 2. Build Sire

Initialize the repository submodules and build the native Python module with
`BUILD_PYTHON=ON`. Dependency locations are ordinary CMake cache paths and may
be changed for the local machine:

```bash
git submodule update --init --recursive
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_PYTHON=ON \
  -DPython3_EXECUTABLE="$PWD/.venv/bin/python" \
  -DTARGET_ARIS_PATH=/path/to/aris/install \
  -DTARGET_HPP_FCL_PATH=/path/to/hpp-fcl/install \
  -DTARGET_STDUUID_PATH=/path/to/stduuid/install
cmake --build build --parallel
mkdir -p python/src/sire/native
cp build/python_modules/sire.so python/src/sire/native/sire.so
```

The dependency paths are examples; point them at the corresponding local
install prefixes. Verify the binding before starting training:

```bash
PYTHONPATH=python/src .venv/bin/python -c \
  "import sire; print(sire.SireRLBatchStepper)"
```

The requirements pin the upstream `rsl_rl` revision used by this integration;
no external source-tree path is required.

## 3. Train

The default GO2 task uses flat ground, a 1 kHz physics step and a 50 Hz policy
step. A 1024-environment, 16-thread run is:

```bash
PYTHONPATH=python/src:demo/demo_python .venv/bin/python \
  demo/demo_python/SireRLGym/scripts/train.py \
  --task go2 --num_envs 1024 --sire_batch_threads 16 \
  --max_iterations 1000 --save_interval 50 --flat_terrain
```

Checkpoints and TensorBoard events are written below `logs/flat_go2/expN/`.
Use `--resume auto` to resume the newest run, or pass a checkpoint path to
`--resume`.

## 4. Regression and throughput checks

```bash
PYTHONPATH=python/src:demo/demo_python .venv/bin/python \
  demo/demo_python/SireRLGym/test/test_sire_batch_training.py

PYTHONPATH=python/src:demo/demo_python .venv/bin/python \
  demo/demo_python/SireRLGym/test/benchmark_sire_batch.py \
  --num-envs 1024 --threads 16 --steps 20 --flat-terrain
```

The regression compares native batch stepping with the retained legacy path,
checks per-environment reset/timeout behavior, and verifies that native errors
include the failing environment and its state.

## 5. sim2sim playback

Export a checkpoint to TorchScript, then run the standalone Sire simulator:

```bash
PYTHONPATH=python/src:demo/demo_python .venv/bin/python \
  demo/demo_python/SireRLGym/scripts/export_policy_jit.py \
  /path/to/model_1000.pt --out /tmp/go2_policy.pt

PYTHONPATH=python/src .venv/bin/python demo/demo_python/dogRL/dog_can_work.py \
  go2_can_work.yaml --policy /tmp/go2_policy.pt --cmd 1 0 0
```

Add `--no-viz` for a headless ten-second sim2sim regression. Policy observations
use base-frame linear and angular velocity, matching training.
