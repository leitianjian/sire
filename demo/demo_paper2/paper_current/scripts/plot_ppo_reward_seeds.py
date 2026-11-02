"""Plot three proposed runs and the archived velocity-regularized seed-2 run."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import scienceplots  # noqa: F401

from plot_high_stiffness_regression import FIGURE_STYLE, read_rows


SEEDS = (2, 3, 4)
RUNS = (("proposed", 2), ("proposed", 3), ("proposed", 4), ("velocity_regularized", 2))
METHOD_LABELS = {"proposed": "Proposed", "velocity_regularized": "Velocity-regularized"}
COLORS = {"proposed": "#2E86AB", "velocity_regularized": "#F18F01"}
LINESTYLES = ("-", ":", "-.", "--")


def write_rows(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def trailing_mean(values: np.ndarray, window: int = 25) -> np.ndarray:
    cumulative = np.cumsum(np.insert(values, 0, 0.0))
    end = np.arange(1, values.size + 1)
    begin = np.maximum(0, end - window)
    return (cumulative[end] - cumulative[begin]) / (end - begin)


COMPONENT_LABELS = {
    "tracking_lin_vel": "Planar velocity tracking",
    "tracking_ang_vel": "Yaw-rate tracking",
    "feet_air_time": "Foot air time",
    "feet_slip": "Stance-foot slip",
    "action_rate": "Action change",
    "dof_acc": "Joint acceleration",
    "ang_vel_xy": "Roll--pitch rate",
    "lin_vel_z": "Vertical velocity",
    "orientation": "Orientation error",
    "hip_pos": "Hip deviation",
    "action_magnitude": "Action magnitude",
    "default_pos": "Default-pose deviation",
    "base_height": "Base-height error",
    "torques": "Torque",
    "stand_still": "Stand-still pose error",
    "collision": "Thigh/calf collision",
    "dof_pos_limits": "Joint-limit violation",
}


def format_row_minima(values: list[float], decimals: int) -> list[str]:
    minimum = min(values)
    cells = []
    for value in values:
        cell = f"{value:.{decimals}f}"
        if np.isclose(value, minimum, rtol=0., atol=1e-12):
            cell = r"\textbf{" + cell + "}"
        cells.append(cell)
    return cells


def write_training_table(path: Path, rows: list[dict], summaries: list[dict]) -> list[str]:
    values = {(r["method"], int(r["seed"]), r["component"]): float(r["mean_final100"]) for r in rows}
    components = {r["component"] for r in rows}
    ranked = sorted(components, key=lambda component: abs(
        values["proposed", 2, component] - values["velocity_regularized", 2, component]
    ), reverse=True)
    # Show the largest signed-component differences for the matched seed;
    # retain all three proposed seeds to expose variation across runs.
    selected = ranked[:7]
    positives = [c for c in selected if values["proposed", 2, c] >= 0]
    penalties = [c for c in selected if values["proposed", 2, c] < 0]
    primary = [c for c in ("feet_slip", "action_rate", "dof_acc") if c in selected]
    ordered_components = primary + [c for c in positives + penalties if c not in primary]
    episode_rewards = [float(next(
        row for row in summaries if row["method"] == method and int(row["seed"]) == seed
    )["reward_last100_mean"]) for method, seed in RUNS]
    lines = [
        r"% Generated from all 17 archived weighted reward-component logs.",
        r"\begin{table}[htbp]",
        r"\caption{Training reward comparison between contact solvers.}",
        r"\label{tab:training-comparison}",
        r"\centering\footnotesize",
        r"\setlength{\tabcolsep}{2.5pt}",
        r"\renewcommand{\arraystretch}{1.08}",
        r"\begin{tabular*}{\columnwidth}{@{\extracolsep{\fill}}lrrrr@{}}",
        r"\toprule",
        r"\multirow{2}{*}{Metric} & \multicolumn{3}{c}{Proposed} & \shortstack{Velocity-\\regularized} \\",
        r"\cmidrule(lr){2-4}\cmidrule(l){5-5}",
        r" & Seed 2 & Seed 3 & Seed 4 & Seed 2 \\",
        r"\midrule",
        "Total episode reward & " + " & ".join(format_row_minima(episode_rewards, 2)) + " " + r"\\",
        r"\midrule",
        r"\multicolumn{5}{l}{\itshape Weighted components (reward/s)} \\",
    ]
    for index, component in enumerate(ordered_components):
        if index == len(primary):
            lines.append(r"\midrule")
        cells = format_row_minima([values[method, seed, component] for method, seed in RUNS], 4)
        lines.append(COMPONENT_LABELS[component] + " & " + " & ".join(cells) + " " + r"\\")
    lines.extend([
        r"\bottomrule", r"\end{tabular*}", r"\end{table}",
    ])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return ordered_components


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--comparison-dir", type=Path, required=True)
    parser.add_argument("--cohort-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    selected = {}
    for name in ("multiseed_training_curves.csv", "multiseed_seed_summary.csv",
                 "hardware_seed_summary.csv", "hardware_control_cycles.csv"):
        rows = [r for r in read_rows(args.source_dir / name) if int(r["seed"]) in SEEDS]
        assert {int(r["seed"]) for r in rows} == set(SEEDS)
        selected[name] = rows
        write_rows(args.cohort_dir / name, rows)
    training = selected["multiseed_training_curves.csv"]
    summaries = selected["multiseed_seed_summary.csv"]
    hardware = selected["hardware_seed_summary.csv"]
    cycles = selected["hardware_control_cycles.csv"]
    curve_path = args.comparison_dir / "training_curves.csv"
    summary_path = args.comparison_dir / "training_summary.csv"
    comparison_curves = read_rows(curve_path)
    comparison_summary = read_rows(summary_path)
    component_path = args.comparison_dir / "reward_component_summary.csv"
    component_summary = read_rows(component_path)
    assert {(r["method"], int(r["seed"])) for r in comparison_summary} == set(RUNS)
    display_rows = []

    with plt.style.context(["science", "ieee", "grid"]):
        mpl.rcParams.update(FIGURE_STYLE)
        figure, axis = plt.subplots(figsize=(3.0, 2.35), constrained_layout=True)
        for (method, seed), linestyle in zip(RUNS, LINESTYLES):
            rows = sorted((r for r in comparison_curves if r["method"] == method and int(r["seed"]) == seed),
                          key=lambda r: int(r["iteration"]))
            iterations = np.array([int(r["iteration"]) for r in rows])
            raw = np.array([float(r["mean_reward"]) for r in rows])
            assert len(rows) == 1200 and np.all(np.isfinite(raw))
            assert np.array_equal(iterations, np.arange(1, 1201))
            assert all(int(r["raw_step"]) == int(r["iteration"]) for r in rows)
            smoothed = trailing_mean(raw)
            source_summary = next(r for r in comparison_summary if r["method"] == method and int(r["seed"]) == seed)
            assert np.isclose(raw[-100:].mean(), float(source_summary["reward_last100_mean"]))
            axis.plot(iterations, smoothed, color=COLORS[method], linestyle=linestyle,
                      linewidth=1.0, label=f"{METHOD_LABELS[method]}, seed {seed}")
            display_rows.extend({"method": method, "seed": seed, "iteration": int(i),
                                 "logged_reward": float(r), "display_reward": float(s)}
                                for i, r, s in zip(iterations, raw, smoothed))
        axis.set_xlabel("PPO iteration")
        axis.set_ylabel("Episode reward")
        axis.grid(True, linestyle="--", color="k", alpha=0.3, linewidth=0.5)
        axis.legend(loc="lower right", frameon=True, framealpha=0.9, fontsize=6.2)
        axis.spines[["top", "right"]].set_visible(False)
        axis.tick_params(which="both", top=False, right=False, direction="out")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        for suffix in (".pdf", ".svg", ".png", ".tiff"):
            figure.savefig(args.output.with_suffix(suffix), bbox_inches="tight",
                           dpi=600 if suffix == ".tiff" else 300)
        plt.close(figure)

    write_rows(args.cohort_dir / "reward_display.csv", display_rows)
    write_rows(args.cohort_dir / "training_method_summary.csv", comparison_summary)
    selected_components = write_training_table(args.output.parent / "training_comparison_table.tex", component_summary, comparison_summary)
    transitions = sum(int(r["total_control_transitions"]) for r in summaries)
    recoveries = sum(int(r["recovered_environment_count"]) for r in summaries)
    seconds = sum(float(r["reported_training_time_s"]) for r in summaries)
    compute_ms = np.array([float(r["compute_to_send_ms"]) for r in cycles])
    metadata = {
        "included_seeds": SEEDS, "excluded_seed": 1,
        "exclusion_reason": "Incomplete hardware acquisition after supply interruptions; onboard battery not installed, as reported by the experimenter.",
        "training_logs_are_retained_and_not_classified_as_training_failure": True,
        "source_dir": str(args.source_dir.resolve()),
        "plot": {"curves": 4, "smoothing": "25-iteration trailing mean, unchanged",
                 "across_seed_mean_curve": False, "sd_band": False,
                 "source_iteration_labels_preserved": True, "points_per_seed": 1200,
                 "scienceplots": ["science", "ieee", "grid"], "colors": COLORS},
        "comparison": {
            "runs": [{"method": method, "seed": seed} for method, seed in RUNS],
            "sources": [{"path": str(p.resolve()), "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
                        for p in (curve_path, summary_path)],
            "iteration_definition": "Unmodified TensorBoard event steps 1..1200; historical CSV +1 label offset corrected",
            "terminal_reward_definition": "Arithmetic mean of the final 100 logged reward values in each run",
            "training_time_definition": "Final Total time from each training stdout log, divided by 3600; retained in archived training_summary.csv",
            "component_source": {"path": str(component_path.resolve()), "sha256": hashlib.sha256(component_path.read_bytes()).hexdigest()},
            "table_component_definition": "Signed weighted reward contribution per simulated second; mean of 100 logged iterations 1101..1200",
            "table_component_selection": "Seven largest absolute component differences between the two seed-2 runs; all 17 components retained in source data",
            "selected_components": selected_components,
            "table_total_episode_reward": "Independent Train/mean_reward statistic from training_summary.csv; not a sum of the displayed duration-normalized components",
            "table_bold_rule": "Smallest signed raw value in each row; all exact ties (absolute tolerance 1e-12) bold; negative penalties farther below zero are smaller",
            "results": comparison_summary,
        },
        "training": {
            "total_transitions": transitions, "total_training_hours": seconds / 3600,
            "aggregate_transitions_per_second": transitions / seconds,
            "total_recoveries": recoveries, "recovery_fraction": recoveries / transitions,
            "final100_reward_per_seed": {r["seed"]: float(r["reward_last100_mean"]) for r in summaries},
            "hours_per_seed": {r["seed"]: float(r["reported_training_time_s"]) / 3600 for r in summaries},
        },
        "hardware": {
            "duration_s": sum(float(r["duration_s"]) for r in hardware),
            "complete_cycles": len(cycles),
            "nonzero_command_cycles": sum(any(abs(float(r[c])) > 1e-8
                for c in ("command_forward", "command_lateral", "command_yaw")) for r in cycles),
            "deadline_misses": sum(int(r["deadline_miss"]) for r in cycles),
            "compute_to_send_p95_ms": float(np.percentile(compute_ms, 95)),
            "compute_to_send_max_ms": float(compute_ms.max()),
            "tilt_p95_min_deg": min(float(r["tilt_deg_p95"]) for r in hardware),
            "tilt_p95_max_deg": max(float(r["tilt_deg_p95"]) for r in hardware),
            "tilt_max_deg": max(float(r["tilt_deg_max"]) for r in hardware),
            "completion": {r["seed"]: r["completion"] for r in hardware},
        },
    }
    (args.cohort_dir / "selection_and_summary.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
