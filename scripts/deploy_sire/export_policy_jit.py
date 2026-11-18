"""Export a Sire RLGym checkpoint as a standalone TorchScript actor.

Rebuilds the actor MLP from state_dict, scripts it, and saves a standalone .pt
that `torch.jit.load()` in deploy_go2.py can consume directly.

Usage::

  python export_policy_jit.py <checkpoint.pt> [--out <out.pt>] [--activation elu]

If --out is omitted, writes <checkpoint_stem>_jit.pt next to the source.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import re
from collections.abc import Mapping

import torch
from torch import nn

def _activation_from_name(name: str) -> nn.Module:
    name = name.strip().lower()
    if name == "elu":
        return nn.ELU()
    if name == "relu":
        return nn.ReLU()
    if name == "selu":
        return nn.SELU()
    if name in {"lrelu", "leaky_relu"}:
        return nn.LeakyReLU()
    if name == "tanh":
        return nn.Tanh()
    if name == "sigmoid":
        return nn.Sigmoid()
    raise ValueError(f"Unsupported activation: {name}")


class ActorPolicy(nn.Module):
    def __init__(self, actor_state_dict: dict[str, torch.Tensor], activation: str):
        super().__init__()
        layer_pattern = re.compile(r"^actor\.(\d+)\.weight$")
        layer_indices = sorted(
            int(match.group(1))
            for key in actor_state_dict
            if (match := layer_pattern.fullmatch(key)) is not None
        )
        if not layer_indices:
            raise ValueError("Checkpoint does not contain actor linear layers")

        layers: list[nn.Module] = []
        for pos, layer_idx in enumerate(layer_indices):
            weight = actor_state_dict[f"actor.{layer_idx}.weight"]
            bias_key = f"actor.{layer_idx}.bias"
            if weight.ndim != 2 or bias_key not in actor_state_dict:
                raise ValueError(f"Actor layer {layer_idx} is not a complete linear layer")
            out_dim, in_dim = weight.shape
            bias = actor_state_dict[bias_key]
            if tuple(bias.shape) != (out_dim,):
                raise ValueError(f"Actor layer {layer_idx} has an incompatible bias")
            linear = nn.Linear(in_dim, out_dim)
            with torch.no_grad():
                linear.weight.copy_(weight)
                linear.bias.copy_(bias)
            layers.append(linear)
            if pos != len(layer_indices) - 1:
                layers.append(_activation_from_name(activation))
        self.actor = nn.Sequential(*layers)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return self.actor(obs)


def _extract_actor_state(checkpoint) -> tuple[dict[str, torch.Tensor], object]:
    if isinstance(checkpoint, nn.Module):
        model_state = checkpoint.state_dict()
        iteration = None
    elif isinstance(checkpoint, Mapping):
        model_state = checkpoint.get("model_state_dict", checkpoint.get("state_dict", checkpoint))
        iteration = checkpoint.get("iter")
    else:
        raise TypeError(f"Unsupported checkpoint object: {type(checkpoint).__name__}")
    if not isinstance(model_state, Mapping):
        raise TypeError("Checkpoint model state is not a parameter mapping")

    actor_state: dict[str, torch.Tensor] = {}
    for raw_key, value in model_state.items():
        if not isinstance(raw_key, str) or not isinstance(value, torch.Tensor):
            continue
        key = raw_key
        for prefix in ("module.", "actor_critic."):
            if key.startswith(prefix):
                key = key[len(prefix):]
        # Current Sire RLGym checkpoints use actor.<index>.*. Some compatible
        # rsl_rl variants wrap the same sequential MLP as actor.network.*.
        if key.startswith("actor.network."):
            key = "actor." + key[len("actor.network."):]
        if key.startswith("actor."):
            actor_state[key] = value.detach().to(device="cpu", dtype=torch.float32)
    if not actor_state:
        raise RuntimeError("No 'actor.*' parameters found in checkpoint")
    return actor_state, iteration


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("checkpoint", type=Path, help="training checkpoint containing model_state_dict")
    p.add_argument("--out", type=Path, default=None, help="output path (default: <checkpoint>_jit.pt)")
    p.add_argument("--activation", choices=("elu", "relu", "selu", "lrelu", "tanh", "sigmoid"), default="elu")
    p.add_argument("--expect-obs", type=int, default=None, help="fail unless actor input has this size")
    p.add_argument("--expect-actions", type=int, default=None, help="fail unless actor output has this size")
    return p.parse_args()


def main():
    args = parse_args()
    ckpt_path = args.checkpoint.resolve()
    if not ckpt_path.exists():
        raise FileNotFoundError(ckpt_path)

    ckpt = torch.load(str(ckpt_path), map_location="cpu", weights_only=False)
    actor_state, iteration = _extract_actor_state(ckpt)

    policy = ActorPolicy(actor_state, activation=args.activation).eval()
    num_obs = policy.actor[0].in_features
    num_actions = policy.actor[-1].out_features
    if args.expect_obs is not None and num_obs != args.expect_obs:
        raise ValueError(f"Actor expects {num_obs} observations, not {args.expect_obs}")
    if args.expect_actions is not None and num_actions != args.expect_actions:
        raise ValueError(f"Actor produces {num_actions} actions, not {args.expect_actions}")

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

    reloaded = torch.jit.load(str(out_path), map_location="cpu").eval()
    with torch.no_grad():
        y_reloaded = reloaded(sample)
    reload_err = (y_eager - y_reloaded).abs().max().item()
    if reload_err > 1e-6:
        raise RuntimeError(f"Saved policy output diverges (max_err={reload_err:.3e})")

    print(
        "export_policy_jit ok "
        f"src={ckpt_path} "
        f"out={out_path} "
        f"num_obs={num_obs} "
        f"num_actions={num_actions} "
        f"activation={args.activation} "
        f"iter={iteration} "
        f"jit_vs_eager_max_err={max_err:.2e} "
        f"reload_max_err={reload_err:.2e}",
        flush=True,
    )


if __name__ == "__main__":
    main()
