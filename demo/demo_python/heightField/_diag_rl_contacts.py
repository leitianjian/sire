"""
Diagnose: RL env step() — does the robot fall? Are contacts detected?
"""
import sire, torch, sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
os.chdir(os.path.join(os.path.dirname(__file__), '..'))

from SireRLGym.utils.task_registry import make_env_cfg, make_env_from_cfg

# Use plane terrain (same as default training)
env_cfg = make_env_cfg('go2')
env_cfg.env.num_envs = 1
env_cfg.terrain.mesh_type = 'plane'
env_cfg.sim.dt = 0.001
env_cfg.control.decimation = 10

env = make_env_from_cfg('go2', env_cfg, headless=True)
print(f'num_envs: {env.num_envs}, num_actions: {env.num_actions}')
print(f'dt={env_cfg.sim.dt}, decimation={env_cfg.control.decimation}, '
      f'ctrl_dt={env_cfg.sim.dt * env_cfg.control.decimation}')

sl = env.sire_sim_loops[0]
m = env.sire_models[0]

# Check initial state
base = m.partPool()[1]
pq0 = list(base.pq)
print(f'Initial base: x={pq0[0]:.3f} y={pq0[1]:.3f} z={pq0[2]:.3f}')
print(f'Initial simTime: {sl.simTime():.6f}')

actions = torch.zeros(1, env.num_actions)
found_contact = False
for step_i in range(500):
    t0 = sl.simTime()
    env.step(actions)
    t1 = sl.simTime()

    base = m.partPool()[1]
    pq = base.pq
    cr = sl.lastContactPairResults()

    if step_i < 5 or (cr and not found_contact) or step_i % 50 == 0:
        contact_str = ''
        if cr:
            contact_str = (f'  CONTACTS={len(cr)} '
                           f'fz={[f"{x[4]:.0f}" for x in cr[:4]]}')
        print(f'  step {step_i:3d}: t={t1:.4f} dt={t1-t0:.4f} '
              f'z={pq[2]:.4f}{contact_str}')

    if cr and not found_contact:
        found_contact = True
        print(f'  >>> First contact at step {step_i}, '
              f'simTime={t1:.4f}, base_z={pq[2]:.4f}')

if not found_contact:
    print(f'  >>> NO contacts after 500 steps!')
    print(f'  final simTime={sl.simTime():.4f}, base_z={base.pq[2]:.4f}')

    # Check collision detection raw output
    pe = env.sire_physics[0]
    pe.updateGeometryLocationFromModel()
    try:
        pairs = pe.computePointPairPenetration()
        print(f'  raw penetration pairs: {len(pairs)}')
    except Exception as e:
        print(f'  computePointPairPenetration not available: {e}')
    print(f'  numGeometries: {pe.numGeometries()}')
