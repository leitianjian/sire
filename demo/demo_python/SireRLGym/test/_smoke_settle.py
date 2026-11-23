"""Quick smoke test: verify settling phase works without crash."""
import sys, torch
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

torch.manual_seed(42)
for step in range(6):
    actions = torch.clip(torch.randn(4, 12), -1.0, 1.0)
    obs, privileged_obs, rew, reset_buf, extras = env.step(actions)
    vz = env.base_lin_vel[:, 2]
    bz = env.root_states[:, 2]
    contact = env.foot_ground_contact.float().mean(dim=1)
    print(f"step {step}: rew={rew.tolist()} | "
          f"bz={[f'{z:.3f}' for z in bz]} | "
          f"vz={[f'{v:.3f}' for v in vz]} | "
          f"contact={[f'{c:.2f}' for c in contact]}")
print("SMOKE TEST PASSED")
