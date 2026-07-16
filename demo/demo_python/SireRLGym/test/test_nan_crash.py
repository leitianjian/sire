"""
Diagnostic: reproduce the NaN crash and trace root cause.

Key findings:
- Robot starts at z=0.34, falls to ground at ~step 12 (t≈0.13s)
- After contact, robot BOUNCES UP with increasing velocity:
    step 20: z=0.16  vz=-1.92 (falling)
    step 40: z=0.19  vz=+7.28 (bouncing up!)
    step 60: z=1.57  vz=+14.46 (still accelerating upward!!)
    step 80: z=2.93  vz=+34.34
    step 90: CRASH — position overflows to 1e124
- Root cause: filterPairsAndPreprocessInfo() in ps_vs_solver2.cpp
  re-adds ghost contacts even when collision detection no longer finds them.
  This creates a continuous upward force → energy blow-up → NaN.

Usage:
    python test/test_nan_crash.py
"""
import sire, torch, sys, os, numpy as np

# test/ -> SireRLGym/ -> demo_python/
_SELF_DIR = os.path.dirname(os.path.abspath(__file__))
_SIREGYM_DIR = os.path.dirname(_SELF_DIR)
_DEMO_PY_DIR = os.path.dirname(_SIREGYM_DIR)
sys.path.insert(0, _DEMO_PY_DIR)

from SireRLGym.utils.task_registry import make_env_cfg, make_env_from_cfg


def run_diagnostic():
    env_cfg = make_env_cfg('go2')
    env_cfg.env.num_envs = 1
    env_cfg.terrain.mesh_type = 'plane'
    env_cfg.sim.dt = 0.001
    env_cfg.control.decimation = 10

    env = make_env_from_cfg('go2', env_cfg, headless=True)
    sl = env.sire_sim_loops[0]
    m = env.sire_models[0]
    actions = torch.zeros(1, env.num_actions)

    print("=== Reproducing NaN crash ===")
    print(f"Initial base z: {m.partPool()[1].pq[2]:.4f}")
    print(f"sim.dt={env_cfg.sim.dt}, decimation={env_cfg.control.decimation}")
    print()

    for step_i in range(300):
        try:
            while not sl.headerIsCtrl():
                sl.handleContact()
            sl.handleContact()
        except RuntimeError as e:
            print(f"*** CRASH at env step {step_i}, "
                  f"simTime={sl.simTime():.4f} ***")
            print(f"    Error: {e}")

            # Dump base state
            base = m.partPool()[1]
            pq = list(base.pq)
            vs = list(base.vs)
            print(f"    base pq: {[f'{x:.4e}' for x in pq[:3]]} "
                  f"q=({pq[3]:.4f},{pq[4]:.4f},{pq[5]:.4f},{pq[6]:.4f})")
            print(f"    base vs: {[f'{x:.4e}' for x in vs]}")

            # Dump all parts with NaN
            for pid in range(m.nbody):
                p = m.partPool()[pid]
                ppq = list(p.pq)
                pvs = list(p.vs)
                if any(not np.isfinite(x) for x in ppq) or \
                   any(not np.isfinite(x) for x in pvs):
                    print(f"    part {pid} ({p.name}): "
                          f"pq={ppq[:3]} vs={pvs[:3]}")

            # Dump last contacts
            cr = sl.lastContactPairResults()
            print(f"    lastContactPairResults: {len(cr)} records")
            for x in cr:
                print(f"      A={x[0]} B={x[1]} "
                      f"f=({x[2]:.1f},{x[3]:.1f},{x[4]:.1f}) "
                      f"pt=({x[5]:.3f},{x[6]:.3f},{x[7]:.3f})")

            # Dump contact force pool
            pe = env.sire_physics[0]
            fp = m.forcePool()
            cf_off = pe.contactForceIdx
            n_forces = len(fp)
            print(f"    contactForceIdx={cf_off}, forcePool.len={n_forces}")
            for i in range(cf_off, min(cf_off + m.nbody, n_forces)):
                f = fp[i]
                fce = list(f.fce)
                if any(abs(x) > 1e-3 for x in fce):
                    print(f"      force[{i}]: fce={[f'{x:.2f}' for x in fce[:3]]}")
            return

        # Periodic status
        if step_i % 20 == 0:
            base = m.partPool()[1]
            cr = sl.lastContactPairResults()
            c_str = f"  contacts={len(cr)}" if cr else ""
            print(f"  step {step_i:3d}: t={sl.simTime():.4f} "
                  f"z={base.pq[2]:.4f} vz={base.vs[2]:.4f}{c_str}")

    else:
        print("No crash in 300 steps (unexpected)")


if __name__ == '__main__':
    run_diagnostic()
