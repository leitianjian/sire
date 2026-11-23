#!/usr/bin/env python3
"""Extract four archived PPO runs into matched training curves and summaries."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np
import yaml
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator


TAGS = {
    "mean_reward": "Train/mean_reward",
    "mean_episode_length": "Train/mean_episode_length",
    "total_fps": "Perf/total_fps",
    "collection_time": "Perf/collection time",
    "learning_time": "Perf/learning_time",
    "mean_noise_std": "Policy/mean_noise_std",
    "physics_failure_count": "Episode/physics_failure_count",
    "episode_count": "Episode/episode_count",
}
REQUIRED = tuple(TAGS)[:5]
EXPECTED_STEPS = list(range(1, 1201))
FINAL_STEPS = EXPECTED_STEPS[-100:]
REWARD_COMPONENT_PREFIX = "Episode/rew_"
REWARD_COMPONENT_UNITS = "signed weighted reward contribution per simulated second"
LOG_ROOT = "/train_cfg/runner/log_root"
SOLVER = "/env_cfg/sim/sire_contact_solver"


def fingerprint(path: Path) -> dict:
    with path.open("rb") as stream:
        digest = hashlib.file_digest(stream, "sha256").hexdigest()
    return {"path": str(path.resolve()), "bytes": path.stat().st_size, "sha256": digest}


def config_diff(baseline, actual, prefix="") -> list[dict]:
    if isinstance(baseline, dict) and isinstance(actual, dict):
        return [item for key in sorted(baseline.keys() | actual.keys())
                for item in config_diff(baseline.get(key), actual.get(key), prefix + "/" + key)]
    return [] if baseline == actual else [{"field": prefix, "baseline": baseline, "actual": actual}]


def read_run(root: Path, method: str, seed: int):
    experiment = root / "flat_go2/exp1"
    event_files = sorted(experiment.glob("events.out.tfevents.*"))
    assert len(event_files) == 1, f"Expected one TensorBoard event file: {experiment}"
    config_path, log_path = experiment / "config.yaml", root / "stdout.log"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    assert config["train_cfg"]["seed"] == seed, f"Wrong seed in {config_path}"
    environments = int(config["env_cfg"]["env"]["num_envs"])
    rollout = int(config["train_cfg"]["runner"]["num_steps_per_env"])
    assert (environments, rollout) == (1024, 120), f"Unexpected rollout dimensions: {root}"
    assert config["train_cfg"]["runner"]["max_iterations"] == 1200
    accumulator = EventAccumulator(str(event_files[0]), size_guidance={"scalars": 0})
    accumulator.Reload()
    available = set(accumulator.Tags().get("scalars", []))
    assert all(TAGS[column] in available for column in REQUIRED), f"Missing core scalars: {root}"
    events = {column: accumulator.Scalars(tag) if tag in available else [] for column, tag in TAGS.items()}
    for column in REQUIRED:
        assert [int(event.step) for event in events[column]] == EXPECTED_STEPS, f"Unexpected raw steps: {root}, {column}"
    scalar_maps = {column: {int(event.step): float(event.value) for event in values} for column, values in events.items()}
    wall_times = {int(event.step): event.wall_time for event in events["mean_reward"]}
    rows = [{
        "method": method, "seed": seed, "raw_step": step, "iteration_zero_based": step - 1,
        "iteration": step, "wall_time": wall_times[step],
        "environment_transitions": step * environments * rollout,
        **{column: values.get(step, math.nan) for column, values in scalar_maps.items()},
    } for step in EXPECTED_STEPS]
    for column in REQUIRED:
        assert np.isfinite([row[column] for row in rows]).all(), f"Non-finite scalar: {root}, {column}"

    # Preserve every logged component. Selection for a manuscript table belongs
    # downstream; this extractor must also retain small and negative terms.
    component_tags = sorted(tag for tag in available if tag.startswith(REWARD_COMPONENT_PREFIX))
    expected_components = {
        REWARD_COMPONENT_PREFIX + name
        for name, scale in config["env_cfg"]["rewards"]["scales"].items()
        if float(scale) != 0.0
    }
    assert component_tags and set(component_tags) == expected_components, f"Missing or unexpected reward components: {root}"
    component_maps = {}
    component_summaries = []
    for tag in component_tags:
        component = tag.removeprefix(REWARD_COMPONENT_PREFIX)
        component_events = accumulator.Scalars(tag)
        assert [int(event.step) for event in component_events] == EXPECTED_STEPS, f"Unexpected component raw steps: {root}, {tag}"
        values = {int(event.step): float(event.value) for event in component_events}
        assert np.isfinite(list(values.values())).all(), f"Non-finite reward component: {root}, {tag}"
        assert all(step in values for step in FINAL_STEPS), f"Incomplete component final window: {root}, {tag}"
        component_maps[component] = values
        component_summaries.append({
            "method": method, "seed": seed, "component": component, "tag": tag,
            "mean_final100": float(np.mean([values[step] for step in FINAL_STEPS])),
            "n_window": len(FINAL_STEPS), "raw_step_start": FINAL_STEPS[0], "raw_step_end": FINAL_STEPS[-1],
            "units": REWARD_COMPONENT_UNITS,
        })
    component_curves = [{
        "method": method, "seed": seed, "raw_step": step, "iteration": step,
        **{component: values[step] for component, values in component_maps.items()},
    } for step in EXPECTED_STEPS]
    log = log_path.read_text(encoding="utf-8", errors="replace")
    blocks = re.findall(r"Learning iteration\s+(\d+)/(\d+)", log)
    assert len(blocks) == 1200 and blocks[-1] == ("1200", "1200"), f"Incomplete stdout: {root}"
    transitions = re.findall(r"Total timesteps:\s*(\d+)", log)
    training_times = re.findall(r"Total time:\s*([0-9.]+)s", log)
    assert transitions and int(transitions[-1]) == 147456000, f"Unexpected transition count: {root}"
    assert training_times, f"Training time missing: {root}"
    summary = {
        "method": method, "seed": seed, "iterations": len(rows),
        "reward_last100_mean": float(np.mean([row["mean_reward"] for row in rows[-100:]])),
        "episode_length_last100_mean": float(np.mean([row["mean_episode_length"] for row in rows[-100:]])),
        "total_control_transitions": int(transitions[-1]),
        "reported_training_time_s": float(training_times[-1]),
        "recovered_environment_count": log.count("[Sire recovered environment]"),
    }
    provenance = {
        "method": method, "seed": seed,
        "sources": {"tensorboard": fingerprint(event_files[0]), "config": fingerprint(config_path), "stdout": fingerprint(log_path)},
        "raw_tensorboard_step_range": [1, 1200],
        "missing_iterations_per_scalar": {column: sum(not math.isfinite(row[column]) for row in rows) for column in TAGS},
        "reward_component_tags": {tag.removeprefix(REWARD_COMPONENT_PREFIX): tag for tag in component_tags},
        "reward_component_raw_step_range": [EXPECTED_STEPS[0], EXPECTED_STEPS[-1]],
        "reward_component_final_window": {"raw_step_start": FINAL_STEPS[0], "raw_step_end": FINAL_STEPS[-1], "n_window": len(FINAL_STEPS)},
        "missing_iterations_per_reward_component": {component: 0 for component in component_maps},
    }
    return rows, summary, config, provenance, component_curves, component_summaries


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--proposed-root", type=Path, required=True, help="Directory containing seed_2, seed_3, seed_4")
    parser.add_argument("--velocity-root", type=Path, required=True, help="Run directory containing flat_go2/exp1 and stdout.log")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    curves, summaries, configs, provenance = [], [], [], []
    component_curves, component_summaries = [], []
    for method, seed, root in [("proposed", seed, args.proposed_root / f"seed_{seed}") for seed in (2, 3, 4)] + [("velocity_regularized", 2, args.velocity_root)]:
        rows, summary, config, source, reward_rows, reward_summaries = read_run(root, method, seed)
        curves.extend(rows)
        summaries.append(summary)
        configs.append(config)
        provenance.append(source)
        component_curves.extend(reward_rows)
        component_summaries.extend(reward_summaries)
    assert all(source["reward_component_tags"] == provenance[0]["reward_component_tags"] for source in provenance), "Reward component tags differ between runs"
    comparisons = []
    for index, allowed in ((1, {LOG_ROOT, "/train_cfg/seed"}), (2, {LOG_ROOT, "/train_cfg/seed"}), (3, {LOG_ROOT, SOLVER})):
        differences = config_diff(configs[0], configs[index])
        assert {item["field"] for item in differences} <= allowed, f"Unexpected configuration differences: {differences}"
        comparisons.append({"baseline": {"method": "proposed", "seed": 2}, "actual": {"method": summaries[index]["method"], "seed": summaries[index]["seed"]}, "differences": differences})
    assert configs[3]["env_cfg"]["sim"].get("sire_contact_solver") == "smooth_linear_shared_v5"
    assert configs[0]["env_cfg"]["sim"].get("sire_contact_solver") != "smooth_linear_shared_v5"
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "training_curves.csv", curves)
    write_csv(args.output_dir / "training_summary.csv", summaries)
    write_csv(args.output_dir / "reward_component_curves.csv", component_curves)
    write_csv(args.output_dir / "reward_component_summary.csv", component_summaries)
    report = {
        "schema_version": 2, "extractor": fingerprint(Path(__file__)), "runs": provenance,
        "scalar_tags": TAGS, "configuration_comparisons": comparisons,
        "indexing": "Raw TensorBoard steps are 1..1200; iteration equals raw_step. iteration_zero_based is derived as raw_step-1.",
        "terminal_statistics": {"iteration_range": [1101, 1200], "samples_per_run": 100, "statistic": "arithmetic mean of raw logged scalars"},
        "reward_components": {
            "tags": provenance[0]["reward_component_tags"],
            "selection": "Every logged Episode/rew_* tag; no manuscript-table selection or zero imputation.",
            "units": REWARD_COMPONENT_UNITS,
            "per_control_step_definition": "The reward function value is multiplied by its configured signed scale and env.dt before accumulation in episode_sums.",
            "actual_episode_duration_normalization": "At reset, each environment's episode_sums component is divided by max(max(episode_length_buf, 1) * env.dt, env.dt); those values are averaged over the resetting environments.",
            "iteration_aggregation": "The runner takes the arithmetic mean of the available reset-batch component means within each training iteration and logs Episode/rew_*; batches are weighted equally, not by their episode counts.",
            "terminal_window": {"field": "raw TensorBoard event.step", "start_inclusive": FINAL_STEPS[0], "end_inclusive": FINAL_STEPS[-1], "n_window": len(FINAL_STEPS), "statistic": "arithmetic mean of the 100 raw logged scalar values"},
            "transformations": "None. Logged terms already include reward weights and duration normalization; the extractor applies no further scale or fixed episode duration.",
            "relation_to_total_reward": "Components are normalized by each actual episode duration and averaged over reset batches. Train/mean_reward instead averages the latest 100 completed episode returns. Their sum is not assumed to equal Train/mean_reward or its value divided by a fixed 20 s.",
            "implementation_references": [
                "sources/demo/demo_python/SireRLGym/envs/base/legged_robot_sire.py: _prepare_reward_function, compute_reward, reset_idx",
                "sources/demo/demo_python/SireRLGym/runners/on_policy_runner.py: log",
            ],
        },
        "limitations": ["Velocity-regularized has one training seed.", "Training times describe archived runs from different execution dates."],
        "outputs": {name: fingerprint(args.output_dir / name) for name in ("training_curves.csv", "training_summary.csv", "reward_component_curves.csv", "reward_component_summary.csv")},
    }
    (args.output_dir / "provenance.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"rows": len(curves), "runs": len(summaries), "reward_component_curve_rows": len(component_curves), "reward_component_summary_rows": len(component_summaries), "output_dir": str(args.output_dir.resolve())}, indent=2))


if __name__ == "__main__":
    main()
