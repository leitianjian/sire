"""Debug: check base_z before and after settling."""
import sys, torch
import numpy as np
sys.path.insert(0, r'D:\code\sire\demo\demo_python')
from SireRLGym.utils.task_registry import make_env_cfg, make_env_from_cfg

env_cfg = make_env_cfg('go2')
env_cfg.env.num_envs = 4
env_cfg.terrain.mesh_type = 'plane'
env_cfg.sim.dt = 0.001
env_cfg.control.decimation = 10
env_cfg.domain_rand.randomize_friction = False
env_cfg.domain_rand.randomize_base_mass = False
env = make_env_from_cfg('go2', env_cfg, headless=True)

# After init (which includes settling), check base_z
for i in range(4):
    m = env.sire_models[i]
    print(f"env {i}: base_z AFTER settling = {m.partPool()[1].pq[2]:.4f}")

# Now do one random step
torch.manual_seed(42)
actions = torch.clip(torch.randn(4, 12), -1.0, 1.0)
obs, priv, rew, reset, extras = env.step(actions)
for i in range(4):
    m = env.sire_models[i]
    print(f"env {i}: base_z AFTER 1 random step = {m.partPool()[1].pq[2]:.4f}  rew={rew[i]:.4f}")
print("DONE")
