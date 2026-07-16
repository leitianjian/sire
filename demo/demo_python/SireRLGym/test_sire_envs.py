"""
Test script for Sire-based LeggedRobot environment creation.
Compares Sire simulation output with MuJoCo to validate correctness.

Usage:
    cd demo/demo_python/SireRLGym
    python test_sire_envs.py
"""

import sys
import time
from pathlib import Path

import numpy as np
import torch

# Add parent to path for imports
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from RLGym.envs.base.legged_robot import LeggedRobot
from RLGym.envs.base.legged_robot_sire import LeggedRobotSire
from RLGym.envs.base.legged_robot_config import LeggedRobotCfg


def short_test():
    """Minimal smoke test: create Sire envs and run a few steps."""
    print("=" * 60)
    print("SMOKE TEST: Sire environment creation")
    print("=" * 60)

    # Build a minimal config
    cfg = LeggedRobotCfg()

    # --- Phase 1: Create LeggedRobotSire (includes both MuJoCo + Sire) ---
    print("\n[1] Creating LeggedRobotSire (MuJoCo + Sire backends)...")
    t0 = time.time()
    env = LeggedRobotSire(cfg, headless=True)
    t1 = time.time()
    print(f"    Created in {t1 - t0:.2f}s")
    print(f"    num_envs={env.num_envs}, num_sire_simulators={len(getattr(env, 'sire_simulators', []))}")

    # --- Phase 2: Compare initial state ---
    print("\n[3] Comparing initial states (MuJoCo vs Sire)...")
    env._refresh_sim_tensors()       # MuJoCo
    mj_root = env.root_states[0].clone()
    mj_dof = env.dof_pos[0].clone()

    env._refresh_sim_tensors_sire()  # Sire
    si_root = env.root_states[0].clone()
    si_dof = env.dof_pos[0].clone()

    print(f"    MuJoCo root state:  pos={mj_root[:3].tolist()},  quat={mj_root[3:7].tolist()}")
    print(f"    Sire   root state:  pos={si_root[:3].tolist()},  quat={si_root[3:7].tolist()}")
    print(f"    MuJoCo dof_pos[:6]: {mj_dof[:6].tolist()}")
    print(f"    Sire   dof_pos[:6]: {si_dof[:6].tolist()}")
    print(f"    Root pos diff:  {torch.norm(mj_root[:3] - si_root[:3]):.6f}")
    print(f"    Dof pos diff:   {torch.norm(mj_dof - si_dof):.6f}")

    # --- Phase 3: Run a few simulation steps ---
    print("\n[4] Running simulation steps (both engines)...")
    actions = torch.randn(env.num_envs, env.num_actions) * 0.5

    # MuJoCo env
    env_mj = LeggedRobot(cfg, headless=True)
    obs_mj, _, rew_mj, reset_mj, _ = env_mj.step(actions)

    # Sire env
    obs_si, _, rew_si, reset_si, _ = env.step(actions)

    print(f"    MuJoCo obs[:6]: {obs_mj[0, :6].tolist()}")
    print(f"    Sire   obs[:6]: {obs_si[0, :6].tolist()}")
    print(f"    MuJoCo rew[0]:  {rew_mj[0]:.6f}")
    print(f"    Sire   rew[0]:  {rew_si[0]:.6f}")

    # --- Phase 4: Contact force comparison ---
    print("\n[5] Contact force comparison...")
    # Re-read MuJoCo
    env._refresh_sim_tensors()
    mj_contacts = env.contact_forces[0].clone()
    nz_mj = torch.nonzero(torch.norm(mj_contacts, dim=1) > 1e-6).squeeze(-1)

    # Re-read Sire
    env._refresh_sim_tensors_sire()
    si_contacts = env.contact_forces[0].clone()
    nz_si = torch.nonzero(torch.norm(si_contacts, dim=1) > 1e-6).squeeze(-1)

    print(f"    MuJoCo non-zero contact bodies: {nz_mj.tolist() if nz_mj.numel() > 0 else '[]'}")
    print(f"    Sire   non-zero contact bodies: {nz_si.tolist() if nz_si.numel() > 0 else '[]'}")

    if nz_mj.numel() > 0:
        print(f"    MuJoCo max contact force: {mj_contacts[nz_mj].norm(dim=1).max():.2f} N")
    if nz_si.numel() > 0:
        print(f"    Sire   max contact force: {si_contacts[nz_si].norm(dim=1).max():.2f} N")

    # --- Phase 5: Ground contact comparison ---
    print("\n[6] Ground contact comparison...")
    env._refresh_sim_tensors()
    mj_gc = env.body_ground_contact[0].clone()
    env._refresh_sim_tensors_sire()
    si_gc = env.body_ground_contact[0].clone()

    mj_gc_bodies = torch.nonzero(mj_gc).squeeze(-1).tolist()
    si_gc_bodies = torch.nonzero(si_gc).squeeze(-1).tolist()
    print(f"    MuJoCo ground-contact bodies: {mj_gc_bodies if mj_gc_bodies else '[]'}")
    print(f"    Sire   ground-contact bodies: {si_gc_bodies if si_gc_bodies else '[]'}")

    print("\n" + "=" * 60)
    print("SMOKE TEST COMPLETE")
    print("=" * 60)


def multi_step_stability_test(num_steps=100):
    """Run many steps to check for divergence or crashes."""
    print("\n" + "=" * 60)
    print(f"MULTI-STEP STABILITY TEST ({num_steps} steps)")
    print("=" * 60)

    cfg = LeggedRobotCfg()
    env = LeggedRobotSire(cfg, headless=True)

    actions = torch.zeros(env.num_envs, env.num_actions)

    pos_history = []
    t0 = time.time()
    for step_i in range(num_steps):
        # Slightly random actions to exercise contact
        if step_i % 20 == 0:
            actions = torch.randn(env.num_envs, env.num_actions) * 0.3

        obs, _, rew, reset, _ = env.step(actions)

        pos_history.append(env.root_states[0, 2].item())  # track base height

        if reset[0]:
            print(f"    Step {step_i}: env[0] reset triggered")
            env.reset_idx(torch.tensor([0]))

        if step_i % 10 == 0:
            print(f"    Step {step_i:4d}: base_z={env.root_states[0,2]:.4f}, "
                  f"max_contact={env.contact_forces[0].norm(dim=1).max():.2f}")

    t1 = time.time()
    print(f"\n    {num_steps} steps in {t1 - t0:.2f}s "
          f"({num_steps / (t1 - t0):.1f} steps/s)")
    print(f"    Base height range: [{min(pos_history):.4f}, {max(pos_history):.4f}]")

    # Check for NaN
    if np.any(np.isnan(pos_history)):
        print("    FAIL: NaN detected in base height!")
    else:
        print("    PASS: No NaN detected")

    print("=" * 60)


def timing_comparison():
    """Compare per-step timing of MuJoCo vs Sire."""
    print("\n" + "=" * 60)
    print("TIMING COMPARISON")
    print("=" * 60)

    cfg = LeggedRobotCfg()
    env_mj = LeggedRobot(cfg, headless=True)
    env_si = LeggedRobotSire(cfg, headless=True)

    actions = torch.randn(env_mj.num_envs, env_mj.num_actions) * 0.3
    warmup = 10
    measure = 30

    # Warmup MuJoCo
    for _ in range(warmup):
        env_mj.step(actions)

    t0 = time.time()
    for _ in range(measure):
        env_mj.step(actions)
    t_mj = (time.time() - t0) / measure

    # Warmup Sire
    for _ in range(warmup):
        env_si.step(actions)

    t0 = time.time()
    for _ in range(measure):
        env_si.step(actions)
    t_si = (time.time() - t0) / measure

    print(f"    MuJoCo avg step time: {t_mj*1000:.2f} ms")
    print(f"    Sire   avg step time: {t_si*1000:.2f} ms")
    print(f"    Ratio (Sire/MuJoCo):  {t_si/t_mj:.2f}x")
    print("=" * 60)


if __name__ == "__main__":
    short_test()
    multi_step_stability_test(num_steps=50)
    timing_comparison()
