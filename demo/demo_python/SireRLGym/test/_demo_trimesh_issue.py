"""
Demo: EXACT training config (trimesh terrain) — visualize why feet never touch ground.

Usage:
    python test/_demo_trimesh_issue.py
"""
import sys, os
import numpy as np
import torch

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_TEST_DIR))
sys.path.insert(0, os.path.dirname(os.path.dirname(_TEST_DIR)))

import sire
from SireRLGym.utils.task_registry import make_env_cfg, make_env_from_cfg


def demo():
    env_cfg = make_env_cfg('go2')
    env_cfg.env.num_envs = 1
    # ---- SAME AS TRAINING: trimesh terrain ----
    env_cfg.terrain.mesh_type = 'trimesh'
    env_cfg.sim.dt = 0.001
    env_cfg.control.decimation = 10

    env = make_env_from_cfg('go2', env_cfg, headless=True)
    sl = env.sire_sim_loops[0]
    m = env.sire_models[0]
    sim = env.sire_simulators[0]
    pe = sim.physicsEngine()

    print("=" * 70)
    print("TRAINING CONFIG (trimesh terrain) — INITIAL STATE after settling")
    print("=" * 70)
    print(f"  base_z = {m.partPool()[1].pq[2]:.4f}")
    print(f"  base_vs = {m.partPool()[1].vs}")
    print(f"  terrain mesh_type = {env_cfg.terrain.mesh_type}")

    # Foot positions
    print("\n  Foot sphere positions (world frame):")
    sphere_off = np.array([-0.002, 0.0, -0.213, 1.0])
    for pid, name in [(4, 'FL'), (7, 'FR'), (10, 'RL'), (13, 'RR')]:
        pm = np.array(m.partPool()[pid].getPm()).reshape(4, 4)
        sphere_w = pm @ sphere_off
        bottom = sphere_w[2] - 0.022
        print(f"    {name}: center_z={sphere_w[2]:.4f}  bottom_z={bottom:.4f}  "
              f"{'BELOW GROUND!' if bottom < -0.001 else 'TOUCHING' if abs(bottom) < 0.01 else 'ABOVE'}")

    # All part positions
    print("\n  All part z-positions:")
    for idx in range(min(20, m.numLinks())):
        pz = m.partPool()[idx].pq[2]
        print(f"    part[{idx}]: z={pz:.4f}")

    # Contact check
    try:
        cr = sl.lastContactPairResults()
        print(f"\n  Pre-step contacts: {len(cr)}")
        for c in cr:
            print(f"    {c}")
    except Exception as e:
        print(f"\n  Pre-step contacts: {e}")

    # Collision geometries
    print("\n  Collision geometries:")
    gp = pe.geometryPool
    for idx in range(pe.numGeometries()):
        g = gp[idx]
        if g is not None:
            pm = np.array(g.getPm()).reshape(4, 4) if g.getPm() is not None else None
            pz = pm[2, 3] if pm is not None else 'N/A'
            print(f"    id={g.id} part_id={g.prtId} type={type(g).__name__:>30s} z={pz}")

    # ---- Show that random action breaks everything in 1 step ----
    print("\n" + "=" * 70)
    print("AFTER 1 RANDOM STEP (same as training step 1)")
    print("=" * 70)
    torch.manual_seed(42)
    actions = torch.clip(torch.randn(1, 12), -1.0, 1.0)
    env.step(actions)

    print(f"  base_z = {m.partPool()[1].pq[2]:.4f}  (was {env_cfg.init_state.pos[2]:.2f})")
    print(f"  base_vs = {m.partPool()[1].vs}")
    cr = sl.lastContactPairResults()
    gc = sum(1 for c in cr if c[0] == 0 or c[1] == 0) if cr else 0
    print(f"  ground contacts: {gc}")

    # ---- 10 more steps ----
    print("\n  Following 10 steps (random actions):")
    for step in range(10):
        actions = torch.clip(torch.randn(1, 12), -1.0, 1.0)
        env.step(actions)
        base = m.partPool()[1]
        cr = sl.lastContactPairResults()
        gc = sum(1 for c in cr if c[0] == 0 or c[1] == 0) if cr else 0
        print(f"    step {step}: base_z={base.pq[2]:.4f}  vz={base.vs[2]:.3f}  ground_contacts={gc}")

    # ---- MeshCat visualization ----
    print("\n  Processing records for MeshCat...")
    sl.recordsContactCptInfo()
    display_init = m.displayInitJson()
    result = sl.recordsToJson()
    n_frames = len(result.get('timeIndex', []))
    print(f"  Records: {n_frames} frames")

    if n_frames == 0:
        print("  WARNING: No frames recorded!")
        return

    try:
        import meshcat
        vis = meshcat.Visualizer()
        vis.open()
        resource_path = os.path.join(os.path.dirname(os.path.dirname(_TEST_DIR)), 'dogRL')
        n_links = m.numLinks()
        print(f"  Robot has {n_links} links, {n_frames} frames")
        print(f"  Resource path: {resource_path}")
        sire.robotInit(n_links, resource_path, display_init, vis)
        print("  Open http://localhost:7000/static/ in browser")
        print("  Playing animation...")
        sire.animateRobotByRecords(n_links, result, 1000, vis)
        input("Press Enter to stop...")
    except ImportError:
        print("  meshcat not installed: pip install meshcat")
    except Exception as e:
        print(f"  MeshCat error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    demo()
