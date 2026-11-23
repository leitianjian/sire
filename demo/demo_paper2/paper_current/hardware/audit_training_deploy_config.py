"""Compare saved Sire training configs with a Go2 deployment YAML."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import yaml


def differences(left: Any, right: Any, path: str = "") -> list[dict[str, Any]]:
    if isinstance(left, dict) and isinstance(right, dict):
        result = []
        for key in sorted(set(left) | set(right)):
            child = f"{path}.{key}" if path else key
            if key not in left or key not in right:
                result.append(
                    {"path": child, "left": left.get(key), "right": right.get(key)}
                )
            else:
                result.extend(differences(left[key], right[key], child))
        return result
    return [] if left == right else [{"path": path, "left": left, "right": right}]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("training_root", type=Path)
    parser.add_argument("deployment_config", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    training = []
    for seed in range(1, 5):
        path = args.training_root / f"seed_{seed}" / "flat_go2" / "exp1" / "config.yaml"
        training.append(yaml.safe_load(path.read_text(encoding="utf-8")))
    deploy = yaml.safe_load(args.deployment_config.read_text(encoding="utf-8"))
    task = training[0]["env_cfg"]
    expected = {
        "control_dt": task["sim"]["dt"] * task["control"]["decimation"],
        "kps": [task["control"]["stiffness"]["joint"]] * 12,
        "kds": [task["control"]["damping"]["joint"]] * 12,
        "action_scale": task["control"]["action_scale"],
        "clip_actions": task["normalization"]["clip_actions"],
        "clip_observations": task["normalization"]["clip_observations"],
        "obs_scales_ang_vel": task["normalization"]["obs_scales"]["ang_vel"],
        "obs_scales_dof_pos": task["normalization"]["obs_scales"]["dof_pos"],
        "obs_scales_dof_vel": task["normalization"]["obs_scales"]["dof_vel"],
        "command_scale": [
            task["normalization"]["obs_scales"]["lin_vel"],
            task["normalization"]["obs_scales"]["lin_vel"],
            task["normalization"]["obs_scales"]["ang_vel"],
        ],
        "max_cmd": [
            max(map(abs, task["commands"]["ranges"]["lin_vel_x"])),
            max(map(abs, task["commands"]["ranges"]["lin_vel_y"])),
            max(map(abs, task["commands"]["ranges"]["ang_vel_yaw"])),
        ],
        "num_actions": task["env"]["num_actions"],
        "num_obs": task["env"]["num_observations"],
        "default_angles": [
            0.1,
            -0.1,
            0.1,
            -0.1,
            0.8,
            0.8,
            1.0,
            1.0,
            -1.5,
            -1.5,
            -1.5,
            -1.5,
        ],
        "joint2motor_idx": [3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8],
    }
    checks = {
        key: {"training": value, "deployment": deploy.get(key), "match": deploy.get(key) == value}
        for key, value in expected.items()
    }
    report = {
        "training_seed_differences_from_seed_1": {
            str(seed): differences(training[0], training[seed - 1])
            for seed in range(2, 5)
        },
        "deployment_checks": checks,
        "all_deployment_checks_match": all(item["match"] for item in checks.values()),
    }
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    if not report["all_deployment_checks_match"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
