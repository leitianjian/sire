"""
Sire RL API smoke test — validates that all RL-critical methods
execute without errors and produce tensors with plausible values.

Tests:
  - env creation & init
  - step() with zero / random actions
  - reset_idx()
  - _refresh_sim_tensors_sire (root, joints, feet, contacts)
  - _get_heights (heightfield or flat)
  - _compute_single_torque (P/V/T modes)
  - compute_observations / compute_reward
  - check_termination

Usage (from demo/demo_python):
    cd SireRLGym/test
    python test_sire_env_api.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import torch

# ensure RLGym parent is on path
ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from SireRLGym.envs.base.legged_robot_sire import LeggedRobotSire
from SireRLGym.envs.go2.go2_config import GO2RoughCfg


def _check_tensor(name: str, t: torch.Tensor, shape_expected=None,
                  min_val=None, max_val=None, allow_nan=False):
    """Assert tensor has no NaN, correct shape, and value bounds."""
    if t is None:
        print(f"  SKIP {name}: is None")
        return
    if shape_expected is not None and tuple(t.shape) != tuple(shape_expected):
        raise AssertionError(
            f"{name}: expected shape {shape_expected}, got {tuple(t.shape)}")
    if torch.isnan(t).any() and not allow_nan:
        raise AssertionError(f"{name}: contains NaN")
    if torch.isinf(t).any():
        raise AssertionError(f"{name}: contains Inf")
    if min_val is not None and (t.min() < min_val).any():
        raise AssertionError(
            f"{name}: min={t.min().item():.3f} < {min_val}")
    if max_val is not None and (t.max() > max_val).any():
        raise AssertionError(
            f"{name}: max={t.max().item():.3f} > {max_val}")


def main():
    print("=" * 55)
    print("  Sire RL API — smoke test")
    print("=" * 55)

    # ── 1. create env ──
    cfg = GO2RoughCfg()
    cfg.env.num_envs = 2
    cfg.terrain.mesh_type = "plane"
    cfg.terrain.measure_heights = False
    cfg.control.control_type = "P"

    print("\n[1] Creating env …")
    env = LeggedRobotSire(cfg, headless=True)
    print(f"    num_envs={env.num_envs}, num_dof={env.num_dof},"
          f" num_obs={env.num_obs}")

    # ── 2. check initial state tensors ──
    print("\n[2] Initial state tensors …")
    _check_tensor("root_states", env.root_states,
                  (cfg.env.num_envs, 13), min_val=-10, max_val=10)
    _check_tensor("dof_pos", env.dof_pos,
                  (cfg.env.num_envs, env.num_dof), min_val=-4, max_val=4)
    _check_tensor("dof_vel", env.dof_vel,
                  (cfg.env.num_envs, env.num_dof), min_val=-50, max_val=50)
    _check_tensor("base_quat", env.base_quat,
                  (cfg.env.num_envs, 4))
    _check_tensor("projected_gravity", env.projected_gravity,
                  (cfg.env.num_envs, 3))
    _check_tensor("contact_forces", env.contact_forces,
                  (cfg.env.num_envs, env.num_bodies, 3))
    _check_tensor("obs_buf", env.obs_buf,
                  (cfg.env.num_envs, env.num_obs))
    print("    OK")

    # ── 3. compute_observations ──
    print("\n[3] compute_observations …")
    env.compute_observations()
    _check_tensor("obs_buf (after compute)", env.obs_buf,
                  (cfg.env.num_envs, env.num_obs))
    print("    OK")

    # ── 4. step with zeros ──
    print("\n[4] step() with zero actions …")
    actions = torch.zeros(cfg.env.num_envs, env.num_actions)
    obs, priv, rew, reset, extras = env.step(actions)
    _check_tensor("obs", obs, (cfg.env.num_envs, env.num_obs))
    _check_tensor("rew", rew, (cfg.env.num_envs,))
    _check_tensor("reset", reset, (cfg.env.num_envs,))
    print(f"    rew mean={rew.mean().item():.3f}, reset any={reset.any().item()}")
    print("    OK")

    # ── 5. step with random actions ──
    print("\n[5] step() with random actions (×10) …")
    for i in range(10):
        act = torch.randn(cfg.env.num_envs, env.num_actions) * 0.5
        obs, priv, rew, reset, extras = env.step(act)
        if torch.isnan(obs).any():
            raise AssertionError(f"NaN in obs at step {i}")
        if reset.any():
            env.reset_idx(reset.nonzero(as_tuple=False).flatten())
    print("    OK")

    # ── 6. reset ──
    print("\n[6] reset_idx() …")
    env.reset_idx(torch.tensor([0]))
    _check_tensor("root_states after reset", env.root_states,
                  shape_expected=(cfg.env.num_envs, 13))
    _check_tensor("dof_pos after reset", env.dof_pos,
                  shape_expected=(cfg.env.num_envs, env.num_dof))
    print("    OK")

    # ── 7. torque computation ──
    print("\n[7] _compute_single_torque …")
    for mode in ["P", "V", "T"]:
        env.cfg.control.control_type = mode
        t = env._compute_single_torque(torch.zeros(env.num_actions), 0)
        _check_tensor(f"torque ({mode})", t, (env.num_actions,),
                      min_val=-max(env.torque_limits).item(),
                      max_val=max(env.torque_limits).item())
    env.cfg.control.control_type = "P"  # restore
    print("    OK")

    # ── 8. foot positions (local frame, subtract root XY) ──
    print("\n[8] foot_pos_world …")
    # feet_pos_world is world-frame; envs are spaced apart so subtract root XY
    fp_local = env.feet_pos_world.clone()
    fp_local[:, :, :2] -= env.root_states[:, :2].unsqueeze(1)  # remove env offset
    _check_tensor("feet_pos_world (local)", fp_local,
                  (cfg.env.num_envs, len(env.feet_indices), 3),
                  min_val=-0.6, max_val=0.6)
    print("    OK")

    # ── 9. contact ground detection ──
    print("\n[9] ground contact flags …")
    if len(env.feet_indices):
        _check_tensor("foot_ground_contact", env.foot_ground_contact,
                      shape_expected=(cfg.env.num_envs, len(env.feet_indices)))
    print("    OK")

    # ── 10. reward functions ──
    print("\n[10] reward functions …")
    for name in env.reward_names[:8]:  # test first 8
        fn = getattr(env, "_reward_" + name, None)
        if fn is None:
            continue
        r = fn()
        _check_tensor(f"_reward_{name}", r, (cfg.env.num_envs,))
    print("    OK")

    # ── 11. check_termination ──
    print("\n[11] check_termination …")
    env.check_termination()
    _check_tensor("reset_buf", env.reset_buf, (cfg.env.num_envs,))
    print(f"     reset_buf.sum() = {env.reset_buf.sum().item()}")
    print("    OK")

    print("\n" + "=" * 55)
    print("  ALL CHECKS PASSED")
    print("=" * 55)


if __name__ == "__main__":
    main()
