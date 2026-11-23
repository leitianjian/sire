"""
Demo: Run trained Sire RL policy with MeshCat visualization.

Usage:
    python test/test_demo_visual.py
"""
import sys, os, time
import numpy as np
import torch

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_SIREGYM_DIR = os.path.dirname(_TEST_DIR)
_DEMO_PY = os.path.dirname(_SIREGYM_DIR)
sys.path.insert(0, _DEMO_PY)

import sire
from SireRLGym.utils.task_registry import make_env_cfg, make_env_from_cfg
from SireRLGym.envs.go2.go2_config import GO2RoughCfg
from rsl_rl.modules import ActorCritic


def demo():
    # ---- Build env ----
    env_cfg = make_env_cfg('go2')
    env_cfg.env.num_envs = 1
    env_cfg.terrain.mesh_type = 'plane'
    env_cfg.sim.dt = 0.001
    env_cfg.control.decimation = 10
    env = make_env_from_cfg('go2', env_cfg, headless=True)

    sl = env.sire_sim_loops[0]
    m = env.sire_models[0]
    sim = env.sire_simulators[0]

    # ---- Load trained policy ----
    log_dir = r'd:\code\sire\SireRLGym\logs\rough_go2\exp11'
    ckpt = torch.load(os.path.join(log_dir, 'model_3000.pt'), map_location='cpu')

    policy = ActorCritic(
        env_cfg.env.num_observations,
        env_cfg.env.num_privileged_obs,
        env_cfg.env.num_actions,
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
    )
    policy.load_state_dict(ckpt['model_state_dict'])
    policy.eval()
    print(f"Model loaded.  Params: {sum(v.numel() for v in policy.parameters()):,}")

    # ---- Run policy + record ----
    obs = env.get_observations()
    sim_duration = 5.0  # seconds
    target_sim_time = sim_duration

    t_start = time.time()
    step_count = 0
    last_t = -1.0
    max_ctrl_steps = int(sim_duration / (float(env_cfg.sim.dt) * env_cfg.control.decimation)) + 100
    print(f"Running policy for {sim_duration}s (max {max_ctrl_steps} steps)...")
    print(f"{'step':>5s} {'t':>7s} {'z':>7s} {'vz':>8s} {'contacts':>8s} {'rew':>8s}")
    print("-" * 55)
    while sl.simTime() < target_sim_time and step_count < max_ctrl_steps:
        with torch.no_grad():
            actions = policy.act(obs).detach()
        obs, _, rew, _, _ = env.step(actions)
        step_count += 1

        base = m.partPool()[1]
        cr = sl.lastContactPairResults()
        n_contacts = len(cr) if cr else 0

        if step_count % 30 == 0:
            print(f"{step_count:5d} {sl.simTime():7.3f} {base.pq[2]:7.3f} "
                  f"{base.vs[2]:8.3f} {n_contacts:8d} {rew[0].item():8.4f}")

        if torch.isnan(obs).any():
            print(f"  NaN at step {step_count}, simTime={sl.simTime():.3f}")
            break
        base = m.partPool()[1]
        if abs(base.pq[2]) > 10 or any(abs(v) > 500 for v in base.vs):
            print(f"  Blowup at step {step_count}, z={base.pq[2]:.2f} vz={base.vs[2]:.2f}")
            break
        # Detect frozen simTime (event queue stuck)
        if step_count > 10 and sl.simTime() == last_t:
            print(f"  simTime FROZEN at {sl.simTime():.3f}s — event queue stuck!")
            break
        last_t = sl.simTime()

    elapsed = time.time() - t_start
    print(f"  Done: {step_count} ctrl steps, {sl.simTime():.2f}s sim time, "
          f"{elapsed:.1f}s wall time ({step_count/elapsed:.0f} steps/s)")

    # ---- Export records for MeshCat ----
    result = sl.recordsToJson()
    display_init = m.displayInitJson()
    print(f"  Records: {len(result.get('timeIndex', []))} frames")

    # ---- MeshCat visualization ----
    try:
        import meshcat
        vis = meshcat.Visualizer()
        vis.open()

        resource_path = os.path.join(_DEMO_PY, 'dogRL')
        sire.robotInit(m.nbody, resource_path, display_init, vis)

        print("\n  Animating in MeshCat — open http://localhost:7000/static/ "
              "in your browser")
        print("  Press Ctrl+C to stop.\n")


        sire.animateRobotByRecords(m.nbody, result, 1000, vis)
    except ImportError:
        print("\n  meshcat not installed.  Install with: pip install meshcat")
        print(f"  Records saved in sl.recordsToJson() — "
              f"{len(result.get('timeIndex', []))} frames ready.")
    except Exception as e:
        print(f"\n  MeshCat error: {e}")


if __name__ == '__main__':
    demo()
