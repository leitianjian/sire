"""Quick test: verify contacts work in Sire RL env with plane terrain."""
import sys, torch
sys.path.insert(0, 'd:/code/sire/demo/demo_python')
from SireRLGym.envs.base.legged_robot_sire import LeggedRobotSire
from SireRLGym.envs.go2.go2_config import GO2RoughCfg

cfg = GO2RoughCfg()
cfg.env.num_envs = 2
cfg.terrain.mesh_type = 'plane'
cfg.terrain.measure_heights = False
cfg.control.control_type = 'P'
env = LeggedRobotSire(cfg, headless=True)

# Step with small random torques to trigger foot-ground contact
for _ in range(10):
    env.step(torch.randn(2, 12) * 0.3)

# Check contacts
for ei in range(2):
    bc = env.body_ground_contact[ei].nonzero().flatten().tolist()
    fc = env.foot_ground_contact[ei].tolist()
    print(f'env {ei}: body_contacts={bc} foot_contacts={fc}')

foot_ids = env.feet_indices.tolist()
print(f'feet at part ids: {foot_ids}')
ok = any(env.foot_ground_contact.flatten().tolist())
print(f'\n{"PASS: contacts working" if ok else "FAIL: no contacts"}')
