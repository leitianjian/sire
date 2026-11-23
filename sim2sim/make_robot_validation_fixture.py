"""Generate read-only deployment test vectors locally, not on the robot."""
from pathlib import Path
import numpy as np
import torch

HERE = Path(__file__).resolve().parent

if __name__ == "__main__":
    torch.set_num_threads(1)
    policy = torch.jit.load(str(HERE / "policies/exp7/model_550_jit.pt"), map_location="cpu").eval()
    rng = np.random.RandomState(1234)
    observations = np.concatenate((np.zeros((1, 45), dtype=np.float32), rng.normal(size=(64, 45)).astype(np.float32)))
    with torch.no_grad():
        actions = policy(torch.from_numpy(observations)).numpy()
    target = HERE / "deploy_sire/validation/expected_io.npz"
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("xb") as file:
        np.savez(file, observations=observations, actions=actions)
    print(target)
