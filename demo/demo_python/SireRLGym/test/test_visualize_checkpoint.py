"""
Demo: load a trained checkpoint from a training log and visualize in meshcat.

Usage:
    python test/test_visualize_checkpoint.py
    python test/test_visualize_checkpoint.py --checkpoint D:/code/sire/SireRLGym/logs/rough_go2/exp10/model_0.pt
    python test/test_visualize_checkpoint.py --checkpoint logs/rough_go2/exp10/model_0.pt --sim_time 3.0
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np
import torch

_TEST_DIR = Path(__file__).resolve().parent
_SIREGYM_DIR = _TEST_DIR.parent
_DEMO_PY = _SIREGYM_DIR.parent
sys.path.insert(0, str(_DEMO_PY))

import sire
from SireRLGym.utils.task_registry import make_env_cfg, make_env_from_cfg
from rsl_rl.modules import ActorCritic


def _resolve_checkpoint(path: str) -> str:
    p = Path(path)
    if p.exists():
        return str(p.resolve())
    # try relative to _SIREGYM_DIR
    p2 = _SIREGYM_DIR / p
    if p2.exists():
        return str(p2.resolve())
    # try relative to repo root
    p3 = Path(r"D:\code\sire") / p
    if p3.exists():
        return str(p3.resolve())
    raise FileNotFoundError(f"Checkpoint not found: {path}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", type=str,
                   default=r"D:\code\sire\demo\demo_python\SireRLGym\scripts\SireRLGym\logs\rough_go2\exp12\model_50.pt")
    p.add_argument("--sim_time", type=float, default=3.0,
                   help="Simulation duration in seconds")
    p.add_argument("--resource_path", type=str, default=None,
                   help="meshcat resource path (default: auto-detect dogRL/)")
    args = p.parse_args()

    ckpt_path = _resolve_checkpoint(args.checkpoint)
    print(f"[test] Loading checkpoint: {ckpt_path}")

    # ---- Build env (single env) ----
    env_cfg = make_env_cfg("go2")
    env_cfg.env.num_envs = 1
    env_cfg.terrain.mesh_type = "plane"
    env_cfg.sim.dt = 0.001
    env_cfg.control.decimation = 10
    env = make_env_from_cfg("go2", env_cfg, headless=True)

    sl = env.sire_sim_loops[0]
    m = env.sire_models[0]
    sim = env.sire_simulators[0]
    # print(sire.toXmlString(sim))

    # ---- Load trained policy ----
    ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=True)
    policy = ActorCritic(
        env_cfg.env.num_observations,
        env_cfg.env.num_privileged_obs,
        env_cfg.env.num_actions,
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
    )
    policy.load_state_dict(ckpt["model_state_dict"])
    policy.eval()
    print(f"[test] Policy loaded.  Parameters: {sum(v.numel() for v in policy.parameters()):,}")

    # ---- Run policy + record ----
    obs = env.get_observations()
    dt = float(env_cfg.sim.dt)
    decimation = env_cfg.control.decimation
    max_steps = int(args.sim_time / (dt * decimation)) + 100

    print(f"[test] Running policy for {args.sim_time}s ({max_steps} ctrl steps)...")
    t_start = time.time()
    step_count = 0

    for _ in range(max_steps):
        with torch.no_grad():
            actions = policy.act(obs).detach()
        obs, _, _, _, _ = env.step(actions)
        step_count += 1

        # Blowup / NaN detection on env 0
        base = m.partPool()[1]
        if abs(base.pq[2]) > 10 or any(abs(v) > 500 for v in base.vs):
            print(f"[test] Blowup at step {step_count}, z={base.pq[2]:.2f}")
            break
        if torch.isnan(obs[0]).any():
            print(f"[test] NaN at step {step_count}")
            break

    elapsed = time.time() - t_start
    print(f"[test] Done: {step_count} ctrl steps, {sl.simTime():.2f}s sim, "
          f"{elapsed:.1f}s wall ({step_count/elapsed:.0f} steps/s)")
    print(sire.toXmlString(sim))
    # ---- Export records for meshcat ----
    result = sl.recordsToJson()
    display_init = sim.displayInitJson()
    print(f"[test] Records: {len(result.get('timeIndex', []))} frames")

    # ---- Auto-detect resource path ----
    resource_path = args.resource_path
    if resource_path is None:
        candidate = _DEMO_PY / "dogRL"
        if candidate.is_dir():
            resource_path = str(candidate)
    if resource_path is None or not Path(resource_path).is_dir():
        print(f"[test] Resource path not found.  Provide --resource_path")
        sys.exit(1)

    # ---- Meshcat visualization ----
    try:
        import meshcat

        vis = meshcat.Visualizer()
        vis.open()

        sire.robotInit(m.nbody, resource_path, display_init, vis)

        print(f"\n  Open http://localhost:7000/static/ in your browser")
        print("  Press Ctrl+C to stop.\n")

        sire.animateRobotByRecords(m.nbody, result, 1000, vis)
        input("[test] Press Enter to exit...")
    except ImportError:
        print("\n  meshcat not installed.  Install with: pip install meshcat")
    except KeyboardInterrupt:
        print("\n[test] Stopped.")
    except Exception as e:
        print(f"\n[test] meshcat error: {e}")


if __name__ == "__main__":
    main()
