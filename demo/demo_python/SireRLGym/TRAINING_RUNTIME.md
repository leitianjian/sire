# Training runtime and memory checks

Use a **Release** Sire build configured with `SIRE_ENABLE_TRACY=OFF`, and
rebuild/install the Python extension together with the library. The new
`sire.tracyEnabled` flag reports the instrumentation in the loaded extension.
`train.py` rejects a Tracy build unless `--allow_profile` is supplied explicitly.
Do not reuse an older installed extension with these Python changes.

Training defaults:

- No recorder history in any environment. Latest contact results remain
  available for rewards and termination. `--visualize_interval N` enables
  history for env 0 only during the selected rollout and clears it afterwards.
- No per-step reward diagnostics or solver JSON traces. `--solver_trace`
  explicitly permits the `SIRE_SOLVER_TRACE_DIR` environment variable.
- Statistics are written to TensorBoard and printed every 10 PPO iterations.
  `--log_interval 0` disables these statistics; checkpoints remain enabled.
  Sampled episode statistics describe that sampled rollout, not all intervening
  rollouts. Training/scheduler cumulative time and step counters still advance.
- Memory sampling is off. `--memory_interval 10` appends four small JSON lines
  every 10 iterations, with no in-memory sample history. Windows records RSS
  and private bytes; Linux records RSS, anonymous RSS and virtual size.

The native and legacy RL loops stop at the next control boundary, leaving its
event pending for the next action. `dt_actual` is the whole interval elapsed.
`contact_forces` contains the latest physical-substep force in newtons, without
timestep scaling. It is not the control-period average; short contacts that end
before the final substep are not represented by that snapshot.

## Verification after rebuilding

In the Python environment that contains Sire, torch and rsl_rl, from PowerShell:

```powershell
Set-Location D:\code\sire\demo\demo_python
python -c "import sire; print(sire.__file__); print('Tracy:', sire.tracyEnabled)"
python -m SireRLGym.test.test_sire_batch_training
python SireRLGym\scripts\train.py --task go2 --max_iterations 120 --log_interval 10 --memory_interval 10
```

Keep the same environment count and terrain as the failing run when comparing
memory. Inspect `memory.jsonl` in the reported log directory:

- Compare the same stage across iterations 10, 20, ..., 120. A startup rise
  followed by a plateau differs from continuing growth at every iteration.
- Growth primarily after rollout points toward simulation or rollout storage;
  growth after update points toward PPO/optimizer allocations. These are
  localization clues, not proof of a leak.
- A recorder-clear drop suggests history/export allocations. Allocators may
  retain freed memory, so a lack of RSS drop alone does not prove a leak.
- These samples describe the trainer process, not the Tracy viewer. If the
  process exits, retain the exception/OS termination message with the samples.

For a normal run without memory sampling or per-iteration statistics:

```powershell
python SireRLGym\scripts\train.py --task go2 --log_interval 0
```

Changing control-boundary timing, reset observations and contact-force scaling
can change learning curves. Compare a fresh run with fixed seed before deciding
whether to continue an older checkpoint.
