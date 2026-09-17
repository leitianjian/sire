# Go2 paper experiment logger

`deploy_go2_paper.py` is an additive entry point. It imports the existing
`deploy_go2.py`; it does not replace or edit deployment, configuration,
policy-export, or build scripts.

```bash
cd /home/unitree/deploy_sire
python3 deploy_go2_paper.py eth0 \
  --method sire_full \
  --task velocity_sweep \
  --trial-id 01 \
  --expected-duration 30 \
  --notes "flat indoor floor"
```

Select a policy without modifying the shared YAML, for example:

```bash
python3 deploy_go2_paper.py eth0 --method mujoco --task stand --trial-id 01 \
  --policy model_1000_exp10_mujoco_jit.pt --expected-duration 20
```

Use one fixed method label per group: `sire_full`, either `sire_unshifted` or
`sire_no_event` (do not mix both), and `mujoco`. Recommended task labels are
`stand`, `trot`, `velocity_sweep`, and `accel_stop`. START, A, and SELECT keep
their original safety meanings.

Each run creates an append-only `.jsonl` and a `.metadata.json` under
`paper_logs/`. The sidecar stores the complete YAML snapshot, policy SHA-256,
group/task/trial labels, thresholds, exit status, and dropped-sample count.

The log retains synchronized clocks, loop/inference timing, command, joystick,
observation, action, target joint position, joint position/velocity/estimated
torque, IMU, raw and estimated foot forces, hysteretic contact labels, battery,
and read-only `rt/sportmodestate` position/velocity/yaw/body/foot states.

The paper entry point uses an absolute `time.monotonic_ns()` schedule. Policy
computation is included inside `control_dt`; it does not sleep for a full
`control_dt` after computation. Each sample reports schedule lateness, command
deadline overrun, deadline miss, and skipped deadlines. When severely late the
scheduler skips expired ticks instead of rapidly issuing catch-up commands.

Reject a trial with dropped samples, exception exit, motor loss, stale sport
state, or sustained deadline misses. SportModeState is an onboard estimate, not
independent ground truth; use it identically for all groups and state this
limitation. Recompute contact events offline from raw forces with one common
analysis script and report threshold sensitivity.
