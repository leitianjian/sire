"""Demo: EXACT same config as training (trimesh terrain) to show the real issue."""
import sys, torch
sys.path.insert(0, r'D:\code\sire\demo\demo_python')
from SireRLGym.utils.task_registry import make_env_cfg, make_env_from_cfg

env_cfg = make_env_cfg('go2')
env_cfg.env.num_envs = 16  # Same as training
# DEFAULT terrain: mesh_type='trimesh' — same as training!
# DEFAULT domain_rand: same as training
env = make_env_from_cfg('go2', env_cfg, headless=True)

print("=== After init (settling) ===")
failures = getattr(env, '_sire_physics_failures', [])
print(f"Physics failures during init: {len(failures)} envs: {failures}")

for i in range(min(16, env.num_envs)):
    m = env.sire_models[i]
    bz = m.partPool()[1].pq[2]
    print(f"  env {i}: base_z={bz:.3f}  (spawn at ~0.44+terrain)")

# Check foot contact
contact = env.foot_ground_contact.float().mean()
print(f"\nAvg foot contact after settling: {contact:.3f}")

# One random step
torch.manual_seed(42)
actions = torch.clip(torch.randn(16, 12), -1.0, 1.0)
obs, priv, rew, reset, extras = env.step(actions)
print(f"\n=== After 1 random step ===")
print(f"Rewards: {rew.tolist()}")
print(f"Avg base_z: {env.root_states[:, 2].mean():.3f}")
print(f"Avg lin_vel_z: {env.base_lin_vel[:, 2].abs().mean():.3f} m/s")
contact = env.foot_ground_contact.float().mean()
print(f"Avg foot contact: {contact:.3f}")
print(f"Terminated envs: {env.reset_buf.sum().item()}/{env.num_envs}")
