"""Read-only policy provenance check against checkpoints and logged actions."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import torch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"hardware"))
from export_policy_jit import ActorPolicy, _extract_actor_state


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    results=[]
    for seed in (2,3,4):
        checkpoint=ROOT/f"data/training/seed_{seed}/flat_go2/exp1/model_1200.pt"
        jitpath=ROOT/f"hardware/policies/sire_seeds_1_4_20260924/sire_seed_{seed}_model_1200_jit.pt"
        ckpt=torch.load(checkpoint,map_location="cpu",weights_only=False)
        weights,_=_extract_actor_state(ckpt)
        actor=ActorPolicy(weights,"elu").eval()
        jit=torch.jit.load(jitpath,map_location="cpu").eval()
        record_path=next((ROOT/f"data/hardware_raw/seed_{seed}").glob("*.jsonl"))
        metadata=json.loads(next((ROOT/f"data/hardware_raw/seed_{seed}").glob("*.metadata.json")).read_text())
        raw=[json.loads(line) for line in record_path.read_text().splitlines() if line.strip()]
        observations=torch.tensor([r["observation"] for r in raw],dtype=torch.float32)
        actions=torch.tensor([r["action"] for r in raw],dtype=torch.float32)
        with torch.inference_mode():
            eager=actor(observations)
            scripted=jit(observations)
        results.append({"seed":seed,"logged_cycles":len(raw),
            "jit_file_sha256":hashlib.sha256(jitpath.read_bytes()).hexdigest(),
            "deployed_sha256_from_metadata":metadata["policy_sha256"],
            "file_hash_matches_deployment":hashlib.sha256(jitpath.read_bytes()).hexdigest()==metadata["policy_sha256"],
            "max_jit_vs_checkpoint_action_difference":float((eager-scripted).abs().max()),
            "max_jit_vs_logged_action_difference":float((scripted-actions).abs().max()),
            "finite_outputs":bool(torch.isfinite(scripted).all())})
    args.output.write_text(json.dumps(results,indent=2),encoding="utf-8")
    print(json.dumps(results,indent=2))


if __name__=="__main__":
    main()
