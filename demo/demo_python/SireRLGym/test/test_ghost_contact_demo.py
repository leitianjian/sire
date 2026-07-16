"""
Demo: Ghost contact bug in PsVsSolver2.

When the robot hits the ground, it bounces up, but the contact solver
keeps a "ghost" contact alive, applying continuous upward force.
The robot accelerates upward instead of falling back down under gravity.

Root cause: filterPairsAndPreprocessInfo() in ps_vs_solver2.cpp
re-adds contacts that are no longer detected by collision detection,
creating a spurious force that blows up the simulation.

Usage:
    python test/test_ghost_contact_demo.py
"""
import sys, os
import numpy as np

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_SIREGYM_DIR = os.path.dirname(_TEST_DIR)
_DEMO_PY = os.path.dirname(_SIREGYM_DIR)
sys.path.insert(0, _DEMO_PY)

import sire, torch
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
    actions = torch.zeros(1, env.num_actions)

    print("=" * 60)
    print("  Ghost Contact Demo — robot falls, bounces, then FLOATS")
    print("=" * 60)
    print(f"  Initial base height: {m.partPool()[1].pq[2]:.3f} m")
    print(f"  Ground plane at:     z = 0.0 m")
    print()

    header = f"{'step':>5s}  {'simTime':>8s}  {'base_z':>8s}  {'base_vz':>9s}  {'contacts':>9s}  note"
    print(header)
    print("-" * len(header))

    phase = "falling"
    for step_i in range(201):
        try:
            while not sl.headerIsCtrl():
                sl.handleContact()
            sl.handleContact()
        except RuntimeError as e:
            print(f"  >>> CRASH at step {step_i}: {e}")
            break

        base = m.partPool()[1]
        z = base.pq[2]
        vz = base.vs[2]
        cr = sl.lastContactPairResults()
        n_contacts = len(cr)

        # Determine phase
        if n_contacts > 0 and phase == "falling":
            phase = "contact"
        if vz > 0 and phase == "contact" and z > 0.3:
            phase = "bouncing"

        note = ""
        if phase == "bouncing" and vz > 0 and n_contacts > 0 and z > 0.3:
            note = "<<< GHOST! Robot is above ground but contact persists"
        elif z < -1.0:
            note = "<<< Penetrating ground!"
        elif abs(vz) > 50:
            note = "<<< Velocity explosion!"

        if step_i <= 30 or step_i % 10 == 0 or note:
            print(f"{step_i:5d}  {sl.simTime():8.4f}  {z:8.3f}  {vz:9.3f}  {n_contacts:9d}  {note}")

        if abs(vz) > 100 or abs(z) > 100 or (phase == "bouncing" and z > 2.0):
            print("\n  >>> Simulation blown up — ghost contact caused energy explosion")
            break


if __name__ == '__main__':
    demo()
