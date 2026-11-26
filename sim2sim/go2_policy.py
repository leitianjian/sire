"""Load the shared Go2 TorchScript deployment profile."""

import torch
import yaml


def load_jit(path, deployment_config=None):
    extra = {"deployment.yaml": ""}
    try:
        actor = torch.jit.load(str(path), map_location="cpu", _extra_files=extra).eval()
    except RuntimeError as exc:
        raise ValueError("Expected an actor JIT, not a training checkpoint. Use: bash sim2sim/run.sh export <checkpoint>") from exc
    if deployment_config:
        profile = yaml.safe_load(deployment_config.read_text())
    elif extra["deployment.yaml"]:
        profile = yaml.safe_load(extra["deployment.yaml"])
    else:
        raise ValueError("JIT has no deployment metadata; pass --deployment-config or export using sim2sim/run.sh export.")
    if profile["num_observations"] != 45 or profile["num_actions"] != 12:
        raise ValueError("This deployment expects 45 observations and 12 actions.")
    if not all(torch.isfinite(p).all().item() for p in actor.parameters()):
        raise ValueError("JIT contains nonfinite actor parameters.")
    with torch.inference_mode():
        sample = actor(torch.zeros(1, 45))
    if not isinstance(sample, torch.Tensor) or sample.shape != (1, 12) or not torch.isfinite(sample).all():
        raise ValueError("JIT must map a [1, 45] observation to a finite [1, 12] action tensor.")
    return actor, profile
