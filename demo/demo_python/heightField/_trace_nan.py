"""
Trace exactly where NaN first appears in the data pipeline.
"""
import sire, torch, sys, os, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
os.chdir(os.path.join(os.path.dirname(__file__), '..'))

from SireRLGym.utils.task_registry import make_env_cfg, make_env_from_cfg
env_cfg = make_env_cfg('go2')
env_cfg.env.num_envs = 1
env_cfg.terrain.mesh_type = 'plane'
env_cfg.sim.dt = 0.001
env_cfg.control.decimation = 10
env = make_env_from_cfg('go2', env_cfg, headless=True)
sl = env.sire_sim_loops[0]
m = env.sire_models[0]
actions = torch.zeros(1, env.num_actions)

def arr_has_nan(arr):
    """Check a list/array for NaN."""
    try:
        return bool(np.any(np.isnan(np.asarray(arr, dtype=np.float64))))
    except Exception:
        return False

print('=== Tracing NaN origin ===')
for step_i in range(500):
    # ---- Check PRE-STEP C++ state ----
    pq0 = list(m.partPool()[1].pq)
    vs0 = list(m.partPool()[1].vs)
    pre_nan = arr_has_nan(pq0) or arr_has_nan(vs0)
    
    # ---- Execute one ctrl step ----
    try:
        while not sl.headerIsCtrl():
            sl.handleContact()
        sl.handleContact()
    except RuntimeError as e:
        print(f'[step {step_i}] C++ EXCEPTION: {e}')
        # Check state right after crash
        for pid in range(min(m.nbody, 20)):
            p = m.partPool()[pid]
            ppq = list(p.pq)
            pvs = list(p.vs)
            if arr_has_nan(ppq) or arr_has_nan(pvs):
                print(f'  part {pid} ({p.name}): pq={[round(x,6) for x in ppq[:3]]}, vs={[round(x,6) for x in pvs[:3]]}')
        break
    
    # ---- Check POST-STEP C++ state ----
    pq1 = list(m.partPool()[1].pq)
    vs1 = list(m.partPool()[1].vs)
    if arr_has_nan(pq1) or arr_has_nan(vs1):
        print(f'[step {step_i}] NaN in C++ base state AFTER handleContact:')
        print(f'  pq={[round(x,6) for x in pq1]}')
        print(f'  vs={[round(x,6) for x in vs1]}')
        for pid in range(m.nbody):
            p = m.partPool()[pid]
            ppq = list(p.pq)
            pvs = list(p.vs)
            if arr_has_nan(ppq) or arr_has_nan(pvs):
                print(f'  part {pid} ({p.name}): pq={[round(x,6) for x in ppq[:3]]}, vs={[round(x,6) for x in pvs[:3]]}')
        break
    
    # ---- Check Python tensors after refresh ----
    env._refresh_sim_tensors_sire()
    for tensor_name in ['root_states', 'dof_pos', 'dof_vel']:
        t = getattr(env, tensor_name)
        if torch.isnan(t).sum().item() > 0:
            print(f'[step {step_i}] NaN in Python tensor: {tensor_name}')
            break
    
    if step_i % 50 == 0:
        cr = sl.lastContactPairResults()
        c = f' contacts={len(cr)}' if cr else ''
        print(f'  step {step_i:3d}: t={sl.simTime():.4f} z={pq1[2]:.4f}{c}')
else:
    print('No NaN in 500 steps')
