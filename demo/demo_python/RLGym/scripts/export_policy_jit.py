"""Export a mujoco_rl checkpoint as a TorchScript policy for go2_rl_gym's deploy/.

Rebuilds the actor MLP from state_dict, scripts it, and saves a standalone .pt
that `torch.jit.load()` in deploy_go2.py can consume directly.

Usage:
  export_policy_jit.py <checkpoint.pt> [--out <out.pt>] [--activation elu]

If --out is omitted, writes <checkpoint_stem>_jit.pt next to the source.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import torch
from torch import nn

ROOT_DIR = Path(__file__).resolve().parents[2]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))


def _activation_from_name(name: str) -> nn.Module:
    name = name.strip().lower()
    if name == "elu":
        return nn.ELU()
    if name == "relu":
        return nn.ReLU()
    if name == "tanh":
        return nn.Tanh()
    raise ValueError(f"Unsupported activation: {name}")


class ActorPolicy(nn.Module):
    def __init__(self, actor_state_dict: dict[str, torch.Tensor], activation: str):
        super().__init__()
        layer_indices = sorted(
            int(key.split(".")[1])
            for key in actor_state_dict.keys()
            if key.startswith("actor.") and key.endswith(".weight")
        )
        layers: list[nn.Module] = []
        for pos, layer_idx in enumerate(layer_indices):
            weight = actor_state_dict[f"actor.{layer_idx}.weight"]
            out_dim, in_dim = weight.shape
            layers.append(nn.Linear(in_dim, out_dim))
            if pos != len(layer_indices) - 1:
                layers.append(_activation_from_name(activation))
        self.actor = nn.Sequential(*layers)
        self.load_state_dict(actor_state_dict, strict=True)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return self.actor(obs)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("checkpoint", type=Path)
    p.add_argument("--out", type=Path, default=None)
    p.add_argument("--activation", type=str, default="elu")
    return p.parse_args()


def main():
    args = parse_args()
    ckpt_path = args.checkpoint.resolve()
    if not ckpt_path.exists():
        raise FileNotFoundError(ckpt_path)

    ckpt = torch.load(str(ckpt_path), map_location="cpu", weights_only=False)
    model_state = ckpt.get("model_state_dict", ckpt)
    actor_state = {k: v for k, v in model_state.items() if k.startswith("actor.")}
    if not actor_state:
        raise RuntimeError(f"No 'actor.*' parameters found in {ckpt_path}")

    policy = ActorPolicy(actor_state, activation=args.activation).eval()
    num_obs = policy.actor[0].in_features
    num_actions = policy.actor[-1].out_features

    with torch.no_grad():
        sample = torch.zeros(1, num_obs, dtype=torch.float32)
        y_eager = policy(sample)
    scripted = torch.jit.script(policy)
    with torch.no_grad():
        y_jit = scripted(sample)
    max_err = (y_eager - y_jit).abs().max().item()
    if max_err > 1e-6:
        raise RuntimeError(f"Scripted output diverges (max_err={max_err:.3e})")

    out_path = args.out
    if out_path is None:
        out_path = ckpt_path.with_name(f"{ckpt_path.stem}_jit.pt")
    out_path = out_path.resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    scripted.save(str(out_path))

    iter_tag = ckpt.get("iter", None)
    print(
        "export_policy_jit ok "
        f"src={ckpt_path} "
        f"out={out_path} "
        f"num_obs={num_obs} "
        f"num_actions={num_actions} "
        f"activation={args.activation} "
        f"iter={iter_tag} "
        f"jit_vs_eager_max_err={max_err:.2e}",
        flush=True,
    )


if __name__ == "__main__":
    main()
