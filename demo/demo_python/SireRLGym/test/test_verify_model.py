"""Verify trained model — does the robot stay stable?"""
import sys, os, torch

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_SIREGYM_DIR = os.path.dirname(_TEST_DIR)
_DEMO_PY = os.path.dirname(_SIREGYM_DIR)
sys.path.insert(0, _DEMO_PY)

from SireRLGym.utils.task_registry import make_env_cfg, make_env_from_cfg

env_cfg = make_env_cfg('go2')
env_cfg.env.num_envs = 1
env_cfg.terrain.mesh_type = 'plane'
env = make_env_from_cfg('go2', env_cfg, headless=True)

log_dir = r'd:\code\sire\SireRLGym\logs\rough_go2\exp8'
ckpt_path = os.path.join(log_dir, 'model_500.pt')
ckpt = torch.load(ckpt_path, map_location='cpu')

print(f"Iteration: {ckpt.get('iteration', '?')}")
sd = ckpt['model_state_dict']
has_nan = any(torch.isnan(v).any() for v in sd.values())
print(f"NaN in weights: {has_nan}")
print(f"Model params: {sum(v.numel() for v in sd.values()):,}")

sl = env.sire_sim_loops[0]
m = env.sire_models[0]

print("\nRunning 100 steps with zero action...")
for step_i in range(100):
    obs, _, _, _, _ = env.step(torch.zeros(1, 12))
    base = m.partPool()[1]
    z = base.pq[2]
    cr = sl.lastContactPairResults()
    c = f"  contacts={len(cr)}" if cr else ""
    if step_i % 20 == 0:
        print(f"  step {step_i:3d}: z={z:.4f}{c}")
    if torch.isnan(obs).any():
        print(f"  NaN at step {step_i}!")
        break
    if z < -2:
        print(f"  Fell through at step {step_i}!")
        break
else:
    print("PASS: 100 steps, no NaN, no fall-through")
