"""
Demo: Run trained Sire policy and show robot behavior.

Usage:
    python test/test_demo_trained.py
"""
import sys, os, torch
import numpy as np

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_SIREGYM_DIR = os.path.dirname(_TEST_DIR)
_DEMO_PY = os.path.dirname(_SIREGYM_DIR)
sys.path.insert(0, _DEMO_PY)

from SireRLGym.utils.task_registry import make_env_cfg, make_env_from_cfg
from SireRLGym.envs.base.legged_robot_sire import LeggedRobotSire
from rsl_rl.modules import ActorCritic


def demo():
    # ---- Setup env ----
    env_cfg = make_env_cfg('go2')
    env_cfg.env.num_envs = 1
    env_cfg.terrain.mesh_type = 'plane'
    env_cfg.sim.dt = 0.001
    env_cfg.control.decimation = 10
    env = make_env_from_cfg('go2', env_cfg, headless=True)

    sl = env.sire_sim_loops[0]
    m = env.sire_models[0]

    # ---- Load trained policy ----
    log_dir = r'd:\code\sire\SireRLGym\logs\rough_go2\exp8'
    ckpt = torch.load(os.path.join(log_dir, 'model_500.pt'), map_location='cpu')

    num_obs = env_cfg.env.num_observations
    num_privileged_obs = env_cfg.env.num_privileged_obs
    num_actions = env_cfg.env.num_actions

    policy = ActorCritic(
        num_obs, num_privileged_obs, num_actions,
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
    )
    policy.load_state_dict(ckpt['model_state_dict'])
    policy.eval()

    print("=" * 60)
    print("  Trained Policy Demo — 500 iterations, Sire backend")
    print("=" * 60)
    print(f"  Model params: {sum(v.numel() for v in policy.parameters()):,}")
    print(f"  Iteration:    {ckpt.get('iteration', 'N/A')}")
    print(f"  NaN weights:  {any(torch.isnan(v).any() for v in policy.parameters())}")
    print()

    # ---- Run policy for 500 steps ----
    obs = env.get_observations()
    header = f"{'step':>5s}  {'simTime':>8s}  {'base_z':>7s}  {'base_vz':>8s}  {'contacts':>9s}  {'rew':>8s}"
    print(header)
    print("-" * len(header))

    ep_rew = 0.0
    for step_i in range(500):
        with torch.no_grad():
            actions = policy.act(obs).detach()
        obs, _, rew, dones, _ = env.step(actions)
        ep_rew += rew[0].item()

        base = m.partPool()[1]
        z = base.pq[2]
        vz = base.vs[2]
        cr = sl.lastContactPairResults()

        if step_i % 25 == 0 or dones[0]:
            n_contacts = len(cr) if cr else 0
            print(f"{step_i:5d}  {sl.simTime():8.4f}  {z:7.3f}  {vz:8.3f}  "
                  f"{n_contacts:9d}  {rew[0].item():8.3f}")

        if torch.isnan(obs).any():
            print(f"\n  >>> NaN at step {step_i}!")
            break

        if dones[0]:
            print(f"\n  >>> Episode ended at step {step_i} (terminated or timeout)")
            break

        if z < -5 or abs(vz) > 200 or abs(z) > 100:
            print(f"\n  >>> Physics blowup at step {step_i}: z={z:.2f} vz={vz:.2f}")
            break

    else:
        print(f"\n  >>> Completed 500 steps. Total reward: {ep_rew:.2f}")

    # Final stats
    cr = sl.lastContactPairResults()
    print(f"\n  Final: z={m.partPool()[1].pq[2]:.3f}  "
          f"contacts={len(cr) if cr else 0}  "
          f"total_reward={ep_rew:.2f}")


if __name__ == '__main__':
    demo()
