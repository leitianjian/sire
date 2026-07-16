"""
Demo: visualize first step fly-up — trace what happens when random actions hit.

Usage:
    python test/test_demo_first_step.py
"""
import sys, os, time
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
    env_cfg.terrain.mesh_type = 'plane'
    env_cfg.sim.dt = 0.001
    env_cfg.control.decimation = 10

    env = make_env_from_cfg('go2', env_cfg, headless=True)
    sl = env.sire_sim_loops[0]
    m = env.sire_models[0]
    sim = env.sire_simulators[0]
    pe = sim.physicsEngine()

    # ---- Show initial state ----
    print("=" * 70)
    print("INITIAL STATE (before any step)")
    print("=" * 70)
    print(f"  base_z = {m.partPool()[1].pq[2]:.4f}")
    print(f"  base_vs = {m.partPool()[1].vs}")

    # Joint angles
    print("\n  Joint angles (mp):")
    for j in range(12):
        mot = m.motionPool()[j]
        joint = m.jointPool()[j]
        default = env.default_dof_pos[0, j].item()
        print(f"    {joint.name}: mp={float(mot.mp):.4f} (default={default:.4f})")

    # Foot positions
    print("\n  Foot sphere positions (world frame):")
    sphere_off = np.array([-0.002, 0.0, -0.213, 1.0])
    for pid, name in [(4, 'FL'), (7, 'FR'), (10, 'RL'), (13, 'RR')]:
        pm = np.array(m.partPool()[pid].getPm()).reshape(4, 4)
        sphere_w = pm @ sphere_off
        bottom = sphere_w[2] - 0.022
        print(f"    {name}: center_z={sphere_w[2]:.4f}  bottom_z={bottom:.4f}  "
              f"{'PENETRATES!' if bottom < -0.001 else 'TOUCHING' if abs(bottom) < 0.005 else 'above'}")

    # Collision geometries
    print("\n  Collision geometries in physics engine:")
    gp = pe.geometryPool
    for idx in range(pe.numGeometries()):
        g = gp[idx]
        if g is not None:
            print(f"    id={g.id} part_id={g.prtId} type={type(g).__name__}")

    # Contact check
    try:
        cr = sl.lastContactPairResults()
        print(f"\n  Pre-step contacts: {len(cr)}")
    except:
        print(f"\n  Pre-step contacts: N/A")

    # ---- Generate random actions like training ----
    np.random.seed(42)
    actions = torch.clip(torch.randn(1, 12), -1.0, 1.0)
    print(f"\n  Random actions: {actions.numpy().round(3).flatten()}")

    # Compute what the PD torque would be
    kp = env.p_gains[0].item()
    kd = env.d_gains[0].item()
    as_scale = env_cfg.control.action_scale
    print(f"\n  PD params: kp={kp}, kd={kd}, action_scale={as_scale}")
    print("  Expected PD torques (before clamping):")
    for j in range(12):
        mot = m.motionPool()[j]
        mp = float(mot.mp)
        mv = float(mot.mv)
        default = env.default_dof_pos[0, j].item()
        action_scaled = actions[0, j].item() * as_scale
        target = action_scaled + default
        torque = kp * (target - mp) - kd * mv
        print(f"    joint_{j}: target={target:.4f} mp={mp:.4f} mv={mv:.4f} torque={torque:.2f}")

    # ---- Record pre-step state for MeshCat ----
    # NOTE: do NOT call recordsContactCptInfo() before stepping —
    # it may clear the record buffer.

    # ---- STEP 100 TIMES (1 second of sim time) ----
    print("\n" + "=" * 70)
    print("RUNNING 100 STEPS with random actions...")
    print("=" * 70)
    obs = env.get_observations()
    for step in range(100):
        actions = torch.clip(torch.randn(1, 12), -1.0, 1.0)
        obs, _, rew, _, _ = env.step(actions)

        if step % 10 == 0:
            base = m.partPool()[1]
            cr = sl.lastContactPairResults()
            gc = sum(1 for (pa, pb, *_) in cr if pa == 0 or pb == 0) if cr else 0
            print(f"  Step {step:3d}: base_z={base.pq[2]:.4f}  vz={base.vs[2]:.3f}  "
                  f"ground_contacts={gc}  {'TOUCHING!' if gc > 0 else 'floating...'}")

    print(f"\n  Final base_z = {m.partPool()[1].pq[2]:.4f} (started at 0.44)")

    # ---- MeshCat (exact dog.py pattern) ----
    print("\n  Processing records...")
    sl.recordsContactCptInfo()
    display_init = sim.displayInitJson()
    result = sl.recordsToJson()
    n_frames = len(result.get('timeIndex', []))
    print(f"  Records: {n_frames} frames")

    if n_frames == 0:
        print("  WARNING: No frames recorded! Simulation may not have produced records.")
        print("  Try running more steps or check if recorder is enabled.")
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
        input("Press Enter to stop animation...")
        print("  Animation complete.")
    except ImportError:
        print("  meshcat not installed. Install with: pip install meshcat")
    except Exception as e:
        print(f"  MeshCat error: {e}")


if __name__ == '__main__':
    demo()
