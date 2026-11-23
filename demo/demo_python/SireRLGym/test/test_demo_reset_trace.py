"""
Diagnostic: trace WHY episodes reset — is it physics crash, base contact,
or state sanity check?  Runs with RANDOM actions (like training init).

Usage:
    python test/test_demo_reset_trace.py
"""
import sys, os, traceback
import numpy as np
import torch

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_TEST_DIR))
sys.path.insert(0, os.path.dirname(os.path.dirname(_TEST_DIR)))

import sire
from SireRLGym.utils.task_registry import make_env_cfg, make_env_from_cfg


def trace_resets():
    env_cfg = make_env_cfg('go2')
    env_cfg.env.num_envs = 16
    env_cfg.terrain.mesh_type = 'plane'
    env_cfg.sim.dt = 0.001
    env_cfg.control.decimation = 10

    print("Creating 16 Sire environments...")
    env = make_env_from_cfg('go2', env_cfg, headless=True)

    obs = env.get_observations()
    total_steps = 200
    reset_reasons = {}  # env_id → list of (step, reason)

    print(f"\nRunning {total_steps} control steps with RANDOM actions (simulating training init)")
    print(f"{'step':>5s} {'resets':>6s} {'alive':>5s} details")
    print("-" * 100)

    for step in range(total_steps):
        # Random actions like training (init_noise_std=1.0, clipped to [-1,1])
        actions = torch.clip(torch.randn(env.num_envs, 12), -1.0, 1.0)

        # Check pre-step state
        pre_failures = len(getattr(env, '_sire_physics_failures', []))

        obs, _, rew, reset_buf, extras = env.step(actions)

        # Collect reset reasons
        reset_ids = reset_buf.nonzero(as_tuple=False).flatten().tolist()
        if reset_ids:
            for eid in reset_ids:
                # Determine reason
                sl = env.sire_sim_loops[eid]
                m = env.sire_models[eid]
                base = m.partPool()[1]
                pq = base.pq
                vs = base.vs

                # Check if it was a physics crash
                post_failures = getattr(env, '_sire_physics_failures', [])
                is_physics_crash = len(post_failures) > pre_failures and eid in post_failures

                # Check state sanity
                state_bad = (abs(pq[0]) > 100 or abs(pq[1]) > 100
                             or pq[2] < -5 or pq[2] > 50
                             or any(abs(v) > 200 for v in vs))

                # Check base contact
                base_contact_force = float(torch.norm(
                    env.contact_forces[eid, env.termination_contact_indices, :]
                ).sum())

                # Check timeout
                is_timeout = bool(env.time_out_buf[eid].item())

                reasons = []
                if is_physics_crash:
                    reasons.append("PHYSICS_CRASH(NaN)")
                if state_bad:
                    reasons.append(f"STATE_BAD(z={pq[2]:.1f},vmax={max(abs(v) for v in vs):.0f})")
                if base_contact_force > 1.0:
                    reasons.append(f"BASE_CONTACT(f={base_contact_force:.1f})")
                if is_timeout:
                    reasons.append("TIMEOUT")
                if not reasons:
                    reasons.append("UNKNOWN")

                reason_str = "+".join(reasons)
                if eid not in reset_reasons:
                    reset_reasons[eid] = []
                reset_reasons[eid].append((step, reason_str))

        # Print summary every 20 steps
        if step % 20 == 0 or len(reset_ids) > 0:
            alive = env.num_envs - len(reset_ids)
            detail = ""
            if reset_ids:
                detail = f"  reset_ids={reset_ids}"
                # Show first env's state
                m0 = env.sire_models[0]
                base0 = m0.partPool()[1]
                sl0 = env.sire_sim_loops[0]
                try:
                    cr = sl0.lastContactPairResults()
                    n_contacts = len(cr) if cr else 0
                except Exception:
                    n_contacts = -1
                detail += (f"  env0: z={base0.pq[2]:.3f} vz={base0.vs[2]:.2f} "
                          f"contacts={n_contacts} rew={rew[0].item():.4f}")
            print(f"{step:5d} {len(reset_ids):6d} {alive:5d}{detail}")

    # ---- Summary ----
    print("\n" + "=" * 100)
    print("RESET REASONS SUMMARY")
    print("=" * 100)

    reason_counts = {}
    total_resets = 0
    for eid, events in sorted(reset_reasons.items()):
        print(f"\nEnv {eid}: {len(events)} resets")
        for step_num, reason in events:
            print(f"  step {step_num:4d}: {reason}")
            total_resets += 1
            reason_counts[reason] = reason_counts.get(reason, 0) + 1

    print(f"\nTotal resets: {total_resets}")
    print("By reason:")
    for reason, count in sorted(reason_counts.items(), key=lambda x: -x[1]):
        print(f"  {reason}: {count}")

    # ---- Check remaining alive envs ----
    print("\n" + "=" * 100)
    print("FINAL STATE OF ALL ENVS")
    print("=" * 100)
    for i in range(env.num_envs):
        m = env.sire_models[i]
        sl = env.sire_sim_loops[i]
        base = m.partPool()[1]
        try:
            cr = sl.lastContactPairResults()
            n_contacts = len(cr) if cr else 0
        except Exception:
            n_contacts = -1
        ep_len = env.episode_length_buf[i].item()
        reset = env.reset_buf[i].item()
        print(f"Env {i:2d}: z={base.pq[2]:.3f} vz={base.vs[2]:.2f} "
              f"contacts={n_contacts} ep_len={ep_len} reset={reset}")

    # ---- Also check: does MuJoCo-style check_termination trigger from BASE contact? ----
    print("\n" + "=" * 100)
    print("TERMINATION CHECK (base contact threshold = 1.0N)")
    print("=" * 100)
    for i in range(env.num_envs):
        cf = env.contact_forces[i, env.termination_contact_indices, :]
        force_norm = torch.norm(cf).item()
        if force_norm > 0.01:
            print(f"Env {i:2d}: base_contact_force={force_norm:.3f}N  "
                  f"{'WOULD TERMINATE!' if force_norm > 1.0 else 'ok'}")


if __name__ == '__main__':
    trace_resets()
