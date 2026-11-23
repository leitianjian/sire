"""Export the actor, using the repository's existing ActorPolicy + jit.script.

The JIT embeds its deployment configuration: the simulation does not need the
original checkpoint, optimizer, critic, rsl_rl, or original training config.
"""
from __future__ import annotations

import argparse
import hashlib
import sys
from pathlib import Path

import torch
import yaml

ROOT = Path(__file__).resolve().parents[1]
# Reuse the exact existing exporter model; do not modify any frozen training code.
sys.path.insert(0, str(ROOT / "demo/demo_python/SireRLGym/scripts"))
from export_policy_jit import ActorPolicy

LEGS = ("FL", "FR", "RL", "RR")
JOINT_ORDERS = {
    "sire": [f"{leg}_{joint}_joint" for joint in ("hip", "thigh", "calf") for leg in LEGS],
    "go2_rl_gym": [f"{leg}_{joint}_joint" for leg in LEGS for joint in ("hip", "thigh", "calf")],
}


def export(checkpoint_path, output, training_config=None, joint_order="auto"):
    checkpoint_path = checkpoint_path.resolve()
    cfg_path = training_config or checkpoint_path.parent / "config.yaml"
    cfg = yaml.safe_load(cfg_path.read_text())
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
    state = checkpoint["model_state_dict"]
    actor_state = {key: value for key, value in state.items() if key.startswith("actor.")}
    actor = ActorPolicy(actor_state, cfg["train_cfg"]["policy"]["activation"]).cpu().eval()
    if actor.actor[0].in_features != 45 or actor.actor[-1].out_features != 12:
        raise ValueError("Only current 45-observation / 12-action Go2 actors are supported.")
    if not all(torch.isfinite(p).all().item() for p in actor.parameters()):
        raise ValueError("Nonfinite actor parameters.")
    priv_dim = int(state["critic.0.weight"].shape[1])
    if joint_order == "auto":
        if priv_dim not in (235, 263):
            raise ValueError(f"Unknown policy layout (critic inputs={priv_dim}); pass --joint-order explicitly.")
        # Matches RLGym/utils/joint_order.py's established compatibility rule.
        joint_order = "go2_rl_gym" if priv_dim == 263 else "sire"
    env_cfg = cfg["env_cfg"]
    profile = {
        "schema_version": 1,
        "source_checkpoint": str(checkpoint_path),
        "source_sha256": hashlib.sha256(checkpoint_path.read_bytes()).hexdigest(),
        "iteration": checkpoint.get("iter"),
        "num_observations": 45,
        "num_actions": 12,
        "policy_joint_names": JOINT_ORDERS[joint_order],
        "env_cfg": {key: env_cfg[key] for key in ("control", "init_state", "normalization")},
    }
    profile["env_cfg"]["sim"] = {"dt": env_cfg["sim"]["dt"]}
    metadata = yaml.safe_dump(profile, sort_keys=False)
    scripted = torch.jit.script(actor)
    generator = torch.Generator().manual_seed(1234)
    samples = torch.cat((torch.zeros(1, 45), torch.randn(32, 45, generator=generator)), dim=0)
    with torch.inference_mode():
        reference = actor(samples)
        torch.testing.assert_close(scripted(samples), reference, rtol=1e-6, atol=1e-6)
    output = output.resolve()
    sidecar = output.with_suffix(".yaml")
    if output.exists() or sidecar.exists():
        raise FileExistsError(f"Will not overwrite an existing export: {output}; choose another --out.")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("xb") as file:
        torch.jit.save(scripted, file, _extra_files={"deployment.yaml": metadata})
    with sidecar.open("x") as file:
        file.write(metadata)
    with torch.inference_mode():
        reloaded = torch.jit.load(str(output), map_location="cpu").eval()
        error = float((reloaded(samples) - reference).abs().max())
        torch.testing.assert_close(reloaded(samples), reference, rtol=1e-6, atol=1e-6)
    print(f"[export] {checkpoint_path}\n[jit] {output}\n[profile] {sidecar}\n"
          f"[verified] actor only, 45 -> 12, iteration={profile['iteration']}, order={joint_order}, "
          f"reloaded_jit_vs_actor_max_error={error:.3g}", flush=True)
    return output


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("checkpoint", type=Path)
    p.add_argument("--out", type=Path)
    p.add_argument("--training-config", type=Path)
    p.add_argument("--joint-order", choices=("auto", *JOINT_ORDERS), default="auto")
    args = p.parse_args()
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    output = args.out or ROOT / "sim2sim/policies" / args.checkpoint.parent.name / f"{args.checkpoint.stem}_jit.pt"
    export(args.checkpoint, output, args.training_config, args.joint_order)


if __name__ == "__main__":
    main()
