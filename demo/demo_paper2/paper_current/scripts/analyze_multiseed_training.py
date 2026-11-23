#!/usr/bin/env python3
"""Analyze independent TensorBoard training runs across random seeds."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path
from typing import Any

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator


try:
    from audit_panel_alignment import require_matplotlib_panel_alignment
except ImportError:
    require_matplotlib_panel_alignment = None


CORE_TAGS = {
    "mean_reward": "Train/mean_reward",
    "mean_episode_length": "Train/mean_episode_length",
    "total_fps": "Perf/total_fps",
    "collection_time": "Perf/collection time",
    "learning_time": "Perf/learning_time",
    "mean_noise_std": "Policy/mean_noise_std",
    "physics_failure_count": "Episode/physics_failure_count",
    "episode_count": "Episode/episode_count",
}

COMPONENT_PREFIXES = (
    "Episode/rew_",
    "Episode/contact_ratio_",
    "Episode/hip_offset_",
)

COLORS = ("#0072B2", "#D55E00", "#009E73", "#CC79A7", "#7A7A7A")

mpl.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans"],
        "mathtext.fontset": "stixsans",
        "font.size": 8.0,
        "axes.labelsize": 8.0,
        "xtick.labelsize": 7.5,
        "ytick.labelsize": 7.5,
        "legend.fontsize": 7.0,
        "axes.spines.right": False,
        "axes.spines.top": False,
        "axes.linewidth": 0.8,
        "legend.frameon": False,
        "pdf.fonttype": 42,
        "svg.fonttype": "none",
    }
)


def seed_number(path: Path) -> int:
    match = re.fullmatch(r"seed_(\d+)", path.name)
    if not match:
        raise ValueError(f"invalid seed directory: {path}")
    return int(match.group(1))


def scalar_map(accumulator: EventAccumulator, tag: str) -> dict[int, float]:
    return {int(event.step): float(event.value) for event in accumulator.Scalars(tag)}


def trailing_mean(values: np.ndarray, window: int) -> np.ndarray:
    if values.size == 0:
        return values.copy()
    cumulative = np.cumsum(np.insert(values, 0, 0.0))
    result = np.empty_like(values, dtype=float)
    for index in range(values.size):
        begin = max(0, index + 1 - window)
        result[index] = (cumulative[index + 1] - cumulative[begin]) / (index + 1 - begin)
    return result


def finite_tail(values: np.ndarray, count: int) -> np.ndarray:
    finite = values[np.isfinite(values)]
    return finite[-count:]


def sample_sd(values: np.ndarray) -> float:
    finite = values[np.isfinite(values)]
    return float(np.std(finite, ddof=1)) if finite.size > 1 else math.nan


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = sorted({field for row in rows for field in row})
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def load_seed(seed_dir: Path) -> tuple[list[dict[str, Any]], dict[str, Any], list[dict[str, Any]]]:
    seed = seed_number(seed_dir)
    event_paths = sorted((seed_dir / "flat_go2" / "exp1").glob("events.out.tfevents.*"))
    if len(event_paths) != 1:
        raise RuntimeError(f"expected one event file for seed {seed}, found {len(event_paths)}")
    accumulator = EventAccumulator(str(event_paths[0]), size_guidance={"scalars": 0})
    accumulator.Reload()
    available = set(accumulator.Tags().get("scalars", []))
    missing = [tag for tag in CORE_TAGS.values() if tag not in available]
    if missing:
        raise RuntimeError(f"seed {seed} is missing scalar tags: {missing}")

    selected_tags = dict(CORE_TAGS)
    for tag in sorted(available):
        if tag.startswith(COMPONENT_PREFIXES):
            selected_tags[tag.replace("Episode/", "episode_").replace("/", "_")] = tag

    maps = {column: scalar_map(accumulator, tag) for column, tag in selected_tags.items()}
    reward_steps = sorted(maps["mean_reward"])
    rows: list[dict[str, Any]] = []
    for step in reward_steps:
        row: dict[str, Any] = {"seed": seed, "iteration": step + 1}
        for column, values in maps.items():
            row[column] = values.get(step, math.nan)
        rows.append(row)

    def column(name: str) -> np.ndarray:
        return np.asarray([float(row.get(name, math.nan)) for row in rows], dtype=float)

    reward = column("mean_reward")
    reward_25 = trailing_mean(reward, 25)
    reward_100 = finite_tail(reward, 100)
    episode_length_100 = finite_tail(column("mean_episode_length"), 100)
    fps = column("total_fps")
    fps_100 = finite_tail(fps, 100)
    failures = column("physics_failure_count")
    peak_index = int(np.nanargmax(reward_25))
    iterations = np.asarray([int(row["iteration"]) for row in rows], dtype=int)
    failure_indices = np.flatnonzero(np.isfinite(failures) & (failures > 0.0))
    stdout_text = (seed_dir / "stdout.log").read_text(encoding="utf-8", errors="replace")
    recovery_count = stdout_text.count("[Sire recovered environment]")
    total_times = [
        float(value)
        for value in re.findall(r"Total time:\s*([0-9.]+)s", stdout_text)
    ]
    total_timesteps = [
        int(value)
        for value in re.findall(r"Total timesteps:\s*(\d+)", stdout_text)
    ]

    def first_crossing(threshold: float) -> int | None:
        candidates = np.flatnonzero(reward_25 >= threshold)
        return int(iterations[candidates[0]]) if candidates.size else None

    def smoothed_at(iteration: int) -> float:
        matches = np.flatnonzero(iterations == iteration)
        return float(reward_25[matches[0]]) if matches.size else math.nan

    summary = {
        "seed": seed,
        "iterations": len(rows),
        "final_reward": float(reward[-1]),
        "reward_last25_mean": float(np.mean(finite_tail(reward, 25))),
        "reward_last100_mean": float(np.mean(reward_100)),
        "reward_last100_within_seed_sd": float(np.std(reward_100, ddof=1)),
        "reward_25ma_at_400": smoothed_at(400),
        "reward_25ma_at_800": smoothed_at(800),
        "reward_25ma_at_1000": smoothed_at(1000),
        "reward_25ma_at_1200": smoothed_at(1200),
        "reward_25ma_gain_1000_to_1200": smoothed_at(1200) - smoothed_at(1000),
        "first_iteration_reward_25ma_ge_15": first_crossing(15.0),
        "first_iteration_reward_25ma_ge_18": first_crossing(18.0),
        "peak_reward_25ma": float(reward_25[peak_index]),
        "peak_reward_25ma_iteration": int(rows[peak_index]["iteration"]),
        "episode_length_last100_mean": float(np.mean(episode_length_100)),
        "episode_length_last100_within_seed_sd": float(
            np.std(episode_length_100, ddof=1)
        ),
        "fps_all_mean": float(np.mean(fps[np.isfinite(fps)])),
        "fps_last100_mean": float(np.mean(fps_100)),
        "physics_failure_tagged_iterations": int(failure_indices.size),
        "last_physics_failure_iteration": (
            int(iterations[failure_indices[-1]]) if failure_indices.size else None
        ),
        "physics_failure_tagged_iterations_last100": int(
            np.count_nonzero(
                np.isfinite(failures[-100:]) & (failures[-100:] > 0.0)
            )
        ),
        "recovered_environment_count": recovery_count,
        "total_control_transitions": total_timesteps[-1],
        "recovery_fraction_of_transitions": recovery_count / total_timesteps[-1],
        "reported_training_time_s": total_times[-1],
        "event_file_bytes": event_paths[0].stat().st_size,
        "checkpoint_1200_bytes": (
            seed_dir / "flat_go2" / "exp1" / "model_1200.pt"
        ).stat().st_size,
    }

    component_rows: list[dict[str, Any]] = []
    for column_name in selected_tags:
        if not column_name.startswith("episode_") or column_name in {
            "episode_count",
        }:
            continue
        values = finite_tail(column(column_name), 100)
        component_rows.append(
            {
                "seed": seed,
                "metric": column_name.removeprefix("episode_"),
                "last100_mean": float(np.mean(values)) if values.size else math.nan,
                "last100_within_seed_sd": (
                    float(np.std(values, ddof=1)) if values.size > 1 else math.nan
                ),
            }
        )
    return rows, summary, component_rows


def aggregate_metric(rows: list[dict[str, Any]], field: str) -> dict[str, float]:
    values = np.asarray([float(row[field]) for row in rows], dtype=float)
    return {
        "mean": float(np.mean(values)),
        "sample_sd": sample_sd(values),
        "minimum": float(np.min(values)),
        "maximum": float(np.max(values)),
    }


def plot_reward(
    output_dir: Path,
    rows_by_seed: dict[int, list[dict[str, Any]]],
    summaries: list[dict[str, Any]],
) -> Path:
    common_iterations = sorted(
        set.intersection(
            *(set(int(row["iteration"]) for row in rows) for rows in rows_by_seed.values())
        )
    )
    seed_smoothed: dict[int, np.ndarray] = {}
    for seed, rows in rows_by_seed.items():
        by_iteration = {int(row["iteration"]): float(row["mean_reward"]) for row in rows}
        raw = np.asarray([by_iteration[iteration] for iteration in common_iterations])
        seed_smoothed[seed] = trailing_mean(raw, 25)
    matrix = np.vstack([seed_smoothed[seed] for seed in sorted(seed_smoothed)])
    cross_seed_mean = np.mean(matrix, axis=0)
    cross_seed_sd = np.std(matrix, axis=0, ddof=1)

    figure, axes = plt.subplots(1, 2, figsize=(7.15, 2.75), constrained_layout=True)
    axis_curve, axis_terminal = axes
    for index, seed in enumerate(sorted(seed_smoothed)):
        axis_curve.plot(
            common_iterations,
            seed_smoothed[seed],
            color=COLORS[index % len(COLORS)],
            linewidth=0.9,
            alpha=0.85,
            label=f"Seed {seed}",
        )
    axis_curve.fill_between(
        common_iterations,
        cross_seed_mean - cross_seed_sd,
        cross_seed_mean + cross_seed_sd,
        color="#777777",
        alpha=0.18,
        linewidth=0.0,
        label="Across-seed mean ± SD",
    )
    axis_curve.plot(common_iterations, cross_seed_mean, color="#202020", linewidth=1.5)
    axis_curve.set_xlabel("PPO iteration")
    axis_curve.set_ylabel("Mean episode reward")
    handles, labels = axis_curve.get_legend_handles_labels()
    figure.legend(
        handles,
        labels,
        loc="upper center",
        ncol=5,
        bbox_to_anchor=(0.5, 1.04),
    )

    values = np.asarray([float(row["reward_last100_mean"]) for row in summaries])
    seeds = np.asarray([int(row["seed"]) for row in summaries])
    for index, (seed, value) in enumerate(zip(seeds, values)):
        axis_terminal.scatter(
            [1.0 + (index - 1.5) * 0.07],
            [value],
            s=24,
            color=COLORS[index % len(COLORS)],
            label=f"Seed {seed}",
            zorder=3,
        )
    mean_value = float(np.mean(values))
    sd_value = sample_sd(values)
    axis_terminal.errorbar(
        [1.0],
        [mean_value],
        yerr=[sd_value],
        color="#202020",
        marker="_",
        markersize=18,
        capsize=5,
        linewidth=1.2,
        zorder=2,
    )
    axis_terminal.set_xlim(0.75, 1.25)
    axis_terminal.set_xticks([1.0], ["Iterations 1101–1200"])
    axis_terminal.set_ylabel("Per-seed reward mean")

    for label, axis in zip("ab", axes):
        axis.grid(True, alpha=0.18)
        axis.annotate(
            label,
            xy=(0.0, 1.0),
            xycoords="axes fraction",
            xytext=(-18, 8),
            textcoords="offset points",
            fontsize=9.0,
            fontweight="bold",
            ha="left",
            va="bottom",
            annotation_clip=False,
        )

    base = output_dir / "multiseed_training_analysis"
    figure.canvas.draw()
    if require_matplotlib_panel_alignment is not None:
        require_matplotlib_panel_alignment(
            figure,
            json_out=str(base.with_suffix(".alignment-layout.json")),
            overlay_svg=str(base.with_suffix(".alignment-overlay.svg")),
            tolerance_pt=1.5,
            gutter_tolerance_pt=1.5,
            require_panel_labels=True,
            strict=True,
        )
    figure.savefig(base.with_suffix(".pdf"), bbox_inches="tight")
    figure.savefig(base.with_suffix(".svg"), bbox_inches="tight")
    figure.savefig(base.with_suffix(".png"), dpi=300, bbox_inches="tight")
    figure.savefig(base.with_suffix(".tiff"), dpi=600, bbox_inches="tight")
    plt.close(figure)
    return base


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log_root", type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    log_root = args.log_root.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    seed_dirs = sorted(log_root.glob("seed_*"), key=seed_number)
    if not seed_dirs:
        raise RuntimeError(f"no seed directories found under {log_root}")

    all_rows: list[dict[str, Any]] = []
    summaries: list[dict[str, Any]] = []
    components: list[dict[str, Any]] = []
    rows_by_seed: dict[int, list[dict[str, Any]]] = {}
    for seed_dir in seed_dirs:
        rows, summary, component_rows = load_seed(seed_dir)
        seed = int(summary["seed"])
        rows_by_seed[seed] = rows
        all_rows.extend(rows)
        summaries.append(summary)
        components.extend(component_rows)

    write_csv(output_dir / "multiseed_training_curves.csv", all_rows)
    write_csv(output_dir / "multiseed_seed_summary.csv", summaries)
    write_csv(output_dir / "multiseed_terminal_components.csv", components)

    aggregate_fields = (
        "final_reward",
        "reward_last25_mean",
        "reward_last100_mean",
        "episode_length_last100_mean",
        "fps_all_mean",
        "fps_last100_mean",
        "reported_training_time_s",
        "reward_25ma_gain_1000_to_1200",
    )
    component_summary: list[dict[str, Any]] = []
    for metric in sorted({str(row["metric"]) for row in components}):
        selected = [row for row in components if row["metric"] == metric]
        values = np.asarray([float(row["last100_mean"]) for row in selected])
        component_summary.append(
            {
                "metric": metric,
                "n_seeds": len(selected),
                "across_seed_mean": float(np.mean(values)),
                "across_seed_sample_sd": sample_sd(values),
                "minimum": float(np.min(values)),
                "maximum": float(np.max(values)),
                "minimum_seed": int(selected[int(np.argmin(values))]["seed"]),
                "maximum_seed": int(selected[int(np.argmax(values))]["seed"]),
            }
        )
    write_csv(output_dir / "multiseed_component_summary.csv", component_summary)

    total_transitions = sum(int(row["total_control_transitions"]) for row in summaries)
    total_recoveries = sum(int(row["recovered_environment_count"]) for row in summaries)
    aggregate = {
        "independent_unit": "training seed",
        "n_seeds": len(summaries),
        "iterations_per_seed": sorted({int(row["iterations"]) for row in summaries}),
        "total_control_transitions": total_transitions,
        "total_recovered_environments": total_recoveries,
        "recovery_fraction_of_transitions": total_recoveries / total_transitions,
        "total_training_time_s": sum(
            float(row["reported_training_time_s"]) for row in summaries
        ),
        "metrics": {field: aggregate_metric(summaries, field) for field in aggregate_fields},
    }
    with (output_dir / "multiseed_summary.json").open("w", encoding="utf-8") as stream:
        json.dump(aggregate, stream, indent=2)

    figure = plot_reward(output_dir, rows_by_seed, summaries)
    print(json.dumps(aggregate, indent=2))
    for row in summaries:
        print(
            f"seed={row['seed']} last100={row['reward_last100_mean']:.4f} "
            f"within_sd={row['reward_last100_within_seed_sd']:.4f} "
            f"episode_length={row['episode_length_last100_mean']:.2f} "
            f"fps={row['fps_all_mean']:.1f} recoveries={row['recovered_environment_count']} "
            f"last_recovery_it={row['last_physics_failure_iteration']}"
        )
    print(f"figure: {figure}.pdf")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
