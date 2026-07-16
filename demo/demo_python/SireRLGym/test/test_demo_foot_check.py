"""
Diagnostic demo: verify initial state — are feet touching the ground?

Usage:
    python test/test_demo_foot_check.py
"""
import sys, os

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_SIREGYM_DIR = os.path.dirname(_TEST_DIR)
_DEMO_PY = os.path.dirname(_SIREGYM_DIR)
sys.path.insert(0, _DEMO_PY)

import numpy as np
import torch
import sire

from SireRLGym.utils.task_registry import make_env_cfg, make_env_from_cfg


def check_foot_heights():
    """Create 8 envs, report initial foot positions and ground contacts."""
    env_cfg = make_env_cfg('go2')
    env_cfg.env.num_envs = 8
    env_cfg.terrain.mesh_type = 'plane'
    env_cfg.sim.dt = 0.001
    env_cfg.control.decimation = 10

    print("Creating 8 Sire environments...")
    env = make_env_from_cfg('go2', env_cfg, headless=True)

    print("\n" + "=" * 80)
    print("INITIAL STATE (before any physics step)")
    print("=" * 80)

    for i in range(env.num_envs):
        m = env.sire_models[i]
        sl = env.sire_sim_loops[i]

        base = m.partPool()[1]
        base_z = base.pq[2]

        # Get foot positions (calf parts: 4=FL, 7=FR, 10=RL, 13=RR)
        foot_heights = []
        foot_names = ["FL", "FR", "RL", "RR"]
        for fi, pid in enumerate([4, 7, 10, 13]):
            pm = m.partPool()[pid].getPm()
            fz = pm[11]  # z position in world frame
            foot_heights.append(fz)

        # Get joint angles
        joint_angles = {}
        for j in range(m.numMotions()):
            mot = m.motionPool()[j]
            joint_angles[f"joint_{j}"] = (float(mot.mp), float(mot.mv))

        # Check contacts
        try:
            cr = sl.lastContactPairResults()
            n_contacts = len(cr) if cr else 0
        except Exception:
            n_contacts = -1  # not available

        foot_str = " ".join(f"{n}={h:.4f}" for n, h in zip(foot_names, foot_heights))
        min_foot = min(foot_heights)
        all_above = all(h > 0 for h in foot_heights)

        print(f"Env {i}: base_z={base_z:.4f}  feet=[{foot_str}]  "
              f"min_foot={min_foot:.4f}  "
              f"{'ALL ABOVE GROUND!' if all_above else 'SOME BELOW GROUND ✓'}  "
              f"contacts={n_contacts}")

    print("\n" + "=" * 80)
    print(f"FIRST 200 STEPS (zero policy action → default PD, ~2s sim time)")
    print("=" * 80)

    obs = env.get_observations()
    for step in range(200):
        actions = torch.zeros(env.num_envs, 12)
        obs, _, rew, _, _ = env.step(actions)

        # Only print every 20 steps to avoid spam
        if step % 20 == 0 or step < 5:
            i = 0  # env 0
            m = env.sire_models[i]
            sl = env.sire_sim_loops[i]
            base = m.partPool()[1]

            foot_heights = []
            for pid in [4, 7, 10, 13]:
                pm = m.partPool()[pid].getPm()
                foot_heights.append(pm[11])
            min_foot = min(foot_heights)

            try:
                cr = sl.lastContactPairResults()
                ground_contacts = sum(1 for (pa, pb, *_) in cr if pa == 0 or pb == 0) if cr else 0
            except Exception:
                ground_contacts = -1

            status = "TOUCHING!" if ground_contacts > 0 else "floating..."
            print(f"Step {step:3d}: base_z={base.pq[2]:.4f}  "
                  f"min_foot={min_foot:.4f}  "
                  f"vz={base.vs[2]:.3f}  "
                  f"ground_contacts={ground_contacts}  "
                  f"{status}")

    # Check all envs at the end
    print(f"\nFinal state (all {env.num_envs} envs):")
    touching = 0
    for i in range(env.num_envs):
        m = env.sire_models[i]
        sl = env.sire_sim_loops[i]
        base = m.partPool()[1]
        foot_heights = []
        for pid in [4, 7, 10, 13]:
            pm = m.partPool()[pid].getPm()
            foot_heights.append(pm[11])
        min_foot = min(foot_heights)
        try:
            cr = sl.lastContactPairResults()
            gc = sum(1 for (pa, pb, *_) in cr if pa == 0 or pb == 0) if cr else 0
        except Exception:
            gc = -1
        if gc > 0:
            touching += 1
        print(f"  Env {i}: base_z={base.pq[2]:.4f}  min_foot={min_foot:.4f}  "
              f"vz={base.vs[2]:.3f}  contacts={gc}  "
              f"{'TOUCHING' if gc > 0 else 'floating'}")
    print(f"\n{touching}/{env.num_envs} environments have ground contact")

    # ---- Test 2: No joint randomization, lower spawn height ----
    print("\n\n" + "=" * 80)
    print("TEST 2: No joint randomization + lower spawn height (z=0.25)")
    print("=" * 80)

    env_cfg2 = make_env_cfg('go2')
    env_cfg2.env.num_envs = 4
    env_cfg2.terrain.mesh_type = 'plane'
    env_cfg2.sim.dt = 0.001
    env_cfg2.control.decimation = 10
    env_cfg2.init_state.pos = [0.0, 0.0, 0.25]  # lower spawn

    env2 = make_env_from_cfg('go2', env_cfg2, headless=True)

    # Override randomization: set ALL dof_pos to exact default
    for i in range(env2.num_envs):
        m = env2.sire_models[i]
        for j, dof_name in enumerate(env2.dof_names):
            mot_idx = env2._sire_dof_to_motion[dof_name]
            m.motionPool()[mot_idx].mp = float(env2.default_dof_pos[0, j])
            m.motionPool()[mot_idx].mv = 0.0
        # Also set base height
        m.partPool()[1].pq = np.array([0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 1.0])
        m.forwardKinematics()
        m.forwardKinematicsVel()

    print("Initial state (no randomization, z=0.25):")
    for i in range(env2.num_envs):
        m = env2.sire_models[i]
        base = m.partPool()[1]
        foot_heights = []
        for pid in [4, 7, 10, 13]:
            pm = m.partPool()[pid].getPm()
            foot_heights.append(pm[11])
        min_foot = min(foot_heights)
        print(f"  Env {i}: base_z={base.pq[2]:.4f}  min_foot={min_foot:.4f}  "
              f"{'BELOW GROUND' if min_foot < 0 else 'above ground'}")

    # Step for 2 seconds
    obs2 = env2.get_observations()
    for step in range(200):
        actions = torch.zeros(env2.num_envs, 12)
        obs2, _, _, _, _ = env2.step(actions)
        if step % 40 == 0:
            i = 0
            m = env2.sire_models[i]
            sl = env2.sire_sim_loops[i]
            base = m.partPool()[1]
            foot_heights = []
            for pid in [4, 7, 10, 13]:
                pm = m.partPool()[pid].getPm()
                foot_heights.append(pm[11])
            min_foot = min(foot_heights)
            try:
                cr = sl.lastContactPairResults()
                gc = sum(1 for (pa, pb, *_) in cr if pa == 0 or pb == 0) if cr else 0
            except Exception:
                gc = -1
            status = "TOUCHING!" if gc > 0 else "floating..."
            print(f"  Step {step:3d}: base_z={base.pq[2]:.4f}  "
                  f"min_foot={min_foot:.4f}  vz={base.vs[2]:.3f}  "
                  f"contacts={gc}  {status}")

    print("\n" + "=" * 80)
    print("DONE. To visualize, use test_demo_visual.py")
    print("=" * 80)


if __name__ == '__main__':
    check_foot_heights()
