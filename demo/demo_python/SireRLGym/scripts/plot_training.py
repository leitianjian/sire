from __future__ import annotations

import argparse
import csv
import json
import re
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator


def _load_scalars(log_dir: Path) -> dict[str, list[tuple[int, float]]]:
    accumulator = EventAccumulator(str(log_dir), size_guidance={"scalars": 0})
    accumulator.Reload()
    result = {}
    for tag in accumulator.Tags().get("scalars", []):
        result[tag] = [(int(v.step), float(v.value)) for v in accumulator.Scalars(tag)]
    if not result:
        raise RuntimeError(f"No TensorBoard scalars found in {log_dir}")
    return result


def _ema(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1 or len(values) < 2:
        return values.copy()
    alpha = 2.0 / (window + 1.0)
    smoothed = np.empty_like(values, dtype=np.float64)
    smoothed[0] = values[0]
    for i in range(1, len(values)):
        smoothed[i] = alpha * values[i] + (1.0 - alpha) * smoothed[i - 1]
    return smoothed


def _series(scalars, tag: str) -> tuple[np.ndarray, np.ndarray]:
    values = scalars.get(tag, [])
    return (
        np.asarray([v[0] for v in values], dtype=np.int64),
        np.asarray([v[1] for v in values], dtype=np.float64),
    )


def _load_memory(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    records = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def _load_recovery_iterations(path: Path) -> list[int]:
    if not path.is_file():
        return []
    iteration_pattern = re.compile(r"Learning iteration\s+(\d+)/")
    recoveries = []
    pending = 0
    last_iteration = 0
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            if "[Sire recovered environment]" in line:
                pending += 1
            match = iteration_pattern.search(line)
            if match:
                last_iteration = int(match.group(1))
                if pending:
                    recoveries.extend([last_iteration] * pending)
                    pending = 0
    if pending:
        recoveries.extend([last_iteration] * pending)
    return recoveries


def _decorate_iterations(
    ax, checkpoint_steps: list[int], recovery_iterations: list[int]
):
    for step in checkpoint_steps:
        ax.axvline(step, color="#777777", linewidth=0.8, linestyle=":", alpha=0.55)
    for step in sorted(set(recovery_iterations)):
        ax.axvline(step, color="#d62728", linewidth=1.0, linestyle="--", alpha=0.65)


def _write_scalar_csv(path: Path, scalars: dict[str, list[tuple[int, float]]]):
    by_step: dict[int, dict[str, float]] = defaultdict(dict)
    for tag, values in scalars.items():
        for step, value in values:
            by_step[step][tag] = value
    tags = sorted(scalars)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["iteration", *tags])
        writer.writeheader()
        for step in sorted(by_step):
            writer.writerow({"iteration": step, **by_step[step]})


def _plot_overview(
    output_path: Path,
    scalars: dict[str, list[tuple[int, float]]],
    memory: list[dict],
    checkpoint_steps: list[int],
    recovery_iterations: list[int],
    smooth_window: int,
    title: str,
):
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)

    ax = axes[0, 0]
    # Raw values remain visible behind the EMA to expose regressions.
    steps, rewards = _series(scalars, "Train/mean_reward")
    ax.plot(steps, rewards, color="#1f77b4", alpha=0.25, linewidth=1.0)
    ax.plot(steps, _ema(rewards, smooth_window), color="#1f77b4", linewidth=2.4, label="Mean reward")
    ax.set_title("Learning outcome")
    ax.set_xlabel("PPO iteration")
    ax.set_ylabel("Mean reward", color="#1f77b4")
    ax.tick_params(axis="y", labelcolor="#1f77b4")
    ax2 = ax.twinx()
    ep_steps, ep_len = _series(scalars, "Train/mean_episode_length")
    ax2.plot(ep_steps, ep_len, color="#ff7f0e", alpha=0.25, linewidth=1.0)
    ax2.plot(ep_steps, _ema(ep_len, smooth_window), color="#ff7f0e", linewidth=2.0, label="Episode length")
    ax2.set_ylabel("Mean episode length", color="#ff7f0e")
    ax2.tick_params(axis="y", labelcolor="#ff7f0e")
    _decorate_iterations(ax, checkpoint_steps, recovery_iterations)
    if len(steps):
        ax.annotate(f"{rewards[-1]:.2f}", (steps[-1], rewards[-1]), xytext=(-35, 8), textcoords="offset points")

    ax = axes[0, 1]
    _plot_scalar(ax, scalars, "Episode/rew_tracking_lin_vel", "Linear velocity tracking", "#2ca02c", smooth_window)
    _plot_scalar(ax, scalars, "Episode/rew_tracking_ang_vel", "Angular velocity tracking", "#17becf", smooth_window)
    _plot_scalar(ax, scalars, "Episode/rew_orientation", "Orientation penalty", "#d62728", smooth_window)
    _plot_scalar(ax, scalars, "Episode/rew_dof_pos_limits", "Joint-limit penalty", "#9467bd", smooth_window)
    ax.set_title("Task tracking and stability")
    ax.set_xlabel("PPO iteration")
    ax.set_ylabel("Reward contribution per second")
    ax.legend(frameon=False, fontsize=9)
    _decorate_iterations(ax, checkpoint_steps, recovery_iterations)

    ax = axes[1, 0]
    _plot_scalar(ax, scalars, "Policy/mean_noise_std", "Action noise std", "#9467bd", smooth_window)
    ax.set_title("Policy convergence")
    ax.set_xlabel("PPO iteration")
    ax.set_ylabel("Action noise std", color="#9467bd")
    ax.tick_params(axis="y", labelcolor="#9467bd")
    ax2 = ax.twinx()
    _plot_scalar(ax2, scalars, "Loss/value_function", "Value loss", "#d62728", smooth_window)
    _plot_scalar(ax2, scalars, "Loss/surrogate", "Surrogate loss", "#8c564b", smooth_window)
    ax2.set_ylabel("PPO loss")
    lines = ax.get_lines()[-1:] + ax2.get_lines()[-4:]
    labels = [line.get_label() for line in lines if not line.get_label().startswith("_")]
    lines = [line for line in lines if not line.get_label().startswith("_")]
    ax.legend(lines, labels, frameon=False, fontsize=9, loc="best")
    _decorate_iterations(ax, checkpoint_steps, recovery_iterations)

    ax = axes[1, 1]
    _plot_scalar(ax, scalars, "Perf/total_fps", "Physics + PPO throughput", "#1f77b4", smooth_window)
    ax.set_title("Throughput and process memory")
    ax.set_xlabel("PPO iteration")
    ax.set_ylabel("Steps / second", color="#1f77b4")
    ax.tick_params(axis="y", labelcolor="#1f77b4")
    if memory:
        ax2 = ax.twinx()
        stage_colors = {
            "before_rollout": "#7f7f7f",
            "after_rollout": "#ff7f0e",
            "after_update": "#2ca02c",
        }
        for stage, color in stage_colors.items():
            records = [r for r in memory if r.get("stage") == stage]
            if records:
                x = [int(r["iteration"]) for r in records]
                y = [float(r["rss_bytes"]) / (1024.0**3) for r in records]
                ax2.plot(x, y, marker=".", markersize=3, linewidth=1.2, color=color, label=stage.replace("_", " "))
        ax2.set_ylabel("Resident memory (GiB)")
        ax2.legend(frameon=False, fontsize=8, loc="lower right")
    _decorate_iterations(ax, checkpoint_steps, recovery_iterations)

    fig.suptitle(
        f"{title}\nDotted: saved checkpoints; red dashed: {len(recovery_iterations)} recovered physics failures",
        fontsize=15,
    )
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def _plot_scalar(ax, scalars, tag, label, color, smooth_window):
    _plot_series(ax, scalars, tag, label, color, smooth_window)


def _plot_series(ax, scalars, tag, label, color, smooth_window):
    steps, values = _series(scalars, tag)
    if not len(steps):
        return
    ax.plot(steps, values, color=color, alpha=0.20, linewidth=0.9)
    ax.plot(steps, _ema(values, smooth_window), color=color, linewidth=2.0, label=label)


def _plot_reward_breakdown(
    output_path,
    scalars,
    checkpoint_steps,
    recovery_iterations,
    smooth_window,
    title,
):
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    groups = [
        (
            axes[0, 0],
            "Positive task rewards",
            [
                ("Episode/rew_tracking_lin_vel", "Linear tracking"),
                ("Episode/rew_tracking_ang_vel", "Angular tracking"),
                ("Episode/rew_feet_air_time", "Feet air time"),
            ],
        ),
        (
            axes[0, 1],
            "Body stability penalties",
            [
                ("Episode/rew_orientation", "Orientation"),
                ("Episode/rew_base_height", "Base height"),
                ("Episode/rew_lin_vel_z", "Vertical velocity"),
                ("Episode/rew_ang_vel_xy", "Roll/pitch velocity"),
                ("Episode/rew_collision", "Collision"),
            ],
        ),
        (
            axes[1, 0],
            "Actuation penalties",
            [
                ("Episode/rew_action_rate", "Action rate"),
                ("Episode/rew_action_magnitude", "Action magnitude"),
                ("Episode/rew_torques", "Torque"),
                ("Episode/rew_dof_vel", "Joint velocity"),
                ("Episode/rew_dof_acc", "Joint acceleration"),
                ("Episode/rew_dof_pos_limits", "Joint limits"),
            ],
        ),
        (
            axes[1, 1],
            "Foot contact ratios",
            [
                ("Episode/contact_ratio_fl_foot", "FL"),
                ("Episode/contact_ratio_fr_foot", "FR"),
                ("Episode/contact_ratio_rl_foot", "RL"),
                ("Episode/contact_ratio_rr_foot", "RR"),
            ],
        ),
    ]
    colors = plt.get_cmap("tab10").colors
    for ax, panel_title, tags in groups:
        for index, (tag, label) in enumerate(tags):
            _plot_scalar(ax, scalars, tag, label, colors[index % len(colors)], smooth_window)
        ax.set_title(panel_title)
        ax.set_xlabel("PPO iteration")
        ax.set_ylabel("Ratio" if "ratios" in panel_title else "Reward contribution per second")
        ax.legend(frameon=False, fontsize=8, ncol=2)
        _decorate_iterations(ax, checkpoint_steps, recovery_iterations)
    fig.suptitle(title, fontsize=15)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def _write_summary(path, scalars, memory, recovery_iterations):
    reward_steps, rewards = _series(scalars, "Train/mean_reward")
    length_steps, lengths = _series(scalars, "Train/mean_episode_length")
    fps_steps, fps = _series(scalars, "Perf/total_fps")
    rss = [float(r["rss_bytes"]) for r in memory]
    summary = {
        "iteration_start": int(reward_steps[0]),
        "iteration_end": int(reward_steps[-1]),
        "mean_reward_start": float(rewards[0]),
        "mean_reward_final": float(rewards[-1]),
        "mean_reward_best": float(np.max(rewards)),
        "mean_reward_best_iteration": int(reward_steps[int(np.argmax(rewards))]),
        "mean_episode_length_final": float(lengths[-1]) if len(length_steps) else None,
        "throughput_mean_steps_per_second": float(np.mean(fps)) if len(fps_steps) else None,
        "rss_min_gib": min(rss) / (1024.0**3) if rss else None,
        "rss_max_gib": max(rss) / (1024.0**3) if rss else None,
        "physics_recovery_count": len(recovery_iterations),
        "physics_recovery_logged_iterations": recovery_iterations,
    }
    path.write_text(json.dumps(summary, indent=2), encoding="utf-8")


def parse_args():
    parser = argparse.ArgumentParser(description="Plot SireRLGym TensorBoard training data.")
    parser.add_argument("log_dir", type=Path, help="Training run directory containing TensorBoard events.")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument(
        "--training-log",
        type=Path,
        default=None,
        help="Console log used to locate recovered physics failures.",
    )
    parser.add_argument("--smooth-window", type=int, default=3, help="EMA window in logged samples.")
    parser.add_argument("--title", default="GO2 shifted-NCP ADMM domain-randomized training")
    return parser.parse_args()


def main():
    args = parse_args()
    log_dir = args.log_dir.expanduser().resolve()
    output_dir = (args.output_dir or (log_dir / "plots")).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    if args.smooth_window < 1:
        raise ValueError("--smooth-window must be at least 1")

    scalars = _load_scalars(log_dir)
    memory = _load_memory(log_dir / "memory.jsonl")
    checkpoint_steps = []
    for path in log_dir.glob("model_*.pt"):
        match = re.fullmatch(r"model_(\d+)\.pt", path.name)
        if match:
            checkpoint_steps.append(int(match.group(1)))
    checkpoint_steps.sort()
    training_log = args.training_log
    if training_log is None:
        log_candidates = sorted(log_dir.glob("*.log"))
        training_log = log_candidates[-1] if log_candidates else None
    recovery_iterations = (
        _load_recovery_iterations(training_log.expanduser().resolve())
        if training_log is not None
        else []
    )

    _write_scalar_csv(output_dir / "training_scalars.csv", scalars)
    _write_summary(output_dir / "training_summary.json", scalars, memory, recovery_iterations)
    _plot_overview(
        output_dir / "training_overview.png",
        scalars,
        memory,
        checkpoint_steps,
        recovery_iterations,
        args.smooth_window,
        args.title,
    )
    _plot_reward_breakdown(
        output_dir / "reward_breakdown.png",
        scalars,
        checkpoint_steps,
        recovery_iterations,
        args.smooth_window,
        args.title,
    )
    print(f"plots_written={output_dir}")
    for path in sorted(output_dir.iterdir()):
        print(f"  {path.name}")


if __name__ == "__main__":
    main()
