"""Experiment 2: frictional high-stiffness regression for the shifted solver.

The experiment extends the undamped bouncing-sphere test with nonzero
tangential speed and Coulomb friction. It reports two separate questions:

1. Does the shifted NCP solve remain accurate on each accepted contact system?
2. Does the full collision/contact/integration pipeline preserve incident
   normal energy as stiffness increases?

Keeping these checks separate prevents a small NCP residual from hiding energy
injected by a nonzero penetration retained at contact onset.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
import traceback
from dataclasses import asdict, replace
from pathlib import Path
from typing import Any

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
REPO_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(HERE))

from run_contact_law_experiment import (  # noqa: E402
    COLORS,
    Case,
    first_contact_episode,
    run_case,
    write_csv,
)

# Historical optional auditor is not used by this paper workflow.
require_matplotlib_panel_alignment = None

COMPARE_SOLVERS = Path(__file__).resolve().parents[1] / "tools/compare_solvers.py"
DEFAULT_STIFFNESS = (
    1.0e7,
    1.0e9,
    1.0e12,
    1.0e15,
    1.0e18,
    1.0e21,
    1.0e24,
    1.0e30,
)

mpl.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans"],
        "mathtext.fontset": "stixsans",
        "font.size": 8.0,
        "axes.labelsize": 8.0,
        "xtick.labelsize": 7.5,
        "ytick.labelsize": 7.5,
        "legend.fontsize": 7.4,
        "axes.spines.right": False,
        "axes.spines.top": False,
        "legend.frameon": False,
        "pdf.fonttype": 42,
        "svg.fonttype": "none",
    }
)


def parse_number_list(value: str) -> tuple[float, ...]:
    numbers = tuple(float(item.strip()) for item in value.split(",") if item.strip())
    if not numbers or any(not math.isfinite(item) or item < 0.0 for item in numbers):
        raise argparse.ArgumentTypeError("expected comma-separated nonnegative numbers")
    return numbers


def replay_shifted_trace(
    trace_dir: Path, output_dir: Path, timing_runs: int
) -> dict[str, float]:
    command = [
        sys.executable,
        str(COMPARE_SOLVERS),
        "--load",
        str(trace_dir),
        "--solvers",
        "Shifted-Spectral-ADMM",
        "Exact-Coulomb",
        "--reference",
        "Exact-Coulomb",
        "--runs",
        str(timing_runs),
        "--max-iters",
        "200",
        "--max-err",
        "1e-8",
        "--output-dir",
        str(output_dir),
    ]
    subprocess.run(command, cwd=REPO_ROOT, check=True)
    with (output_dir / "metrics.csv").open(newline="", encoding="utf-8") as stream:
        metrics = [
            row
            for row in csv.DictReader(stream)
            if row["solver"] == "Shifted-Spectral-ADMM" and row["status"] == "ok"
        ]
    if not metrics:
        return {
            "replayed_frames": 0,
            "converged_fraction": math.nan,
            "p95_exact_residual": math.nan,
            "p95_relative_force_error": math.nan,
            "max_velocity_error": math.nan,
            "median_time_ms": math.nan,
        }

    def finite(field: str) -> np.ndarray:
        values = np.asarray([float(row[field]) for row in metrics], dtype=float)
        return values[np.isfinite(values)]

    def percentile(field: str, q: float) -> float:
        values = finite(field)
        return float(np.percentile(values, q)) if values.size else math.nan

    return {
        "replayed_frames": len(metrics),
        "converged_fraction": float(
            np.mean([int(row["converged"]) for row in metrics])
        ),
        "p95_exact_residual": percentile("exact_residual", 95.0),
        "p95_relative_force_error": percentile("force_diff_rel", 95.0),
        "max_velocity_error": percentile("velocity_diff_inf", 100.0),
        "median_time_ms": percentile("time_ms", 50.0),
    }


def contact_onset_metrics(
    rows: list[dict[str, Any]], mass: float, stiffness: float
) -> dict[str, float]:
    episode = first_contact_episode(rows)
    if episode is None:
        return {
            "incident_normal_speed": math.nan,
            "onset_modified_depth": math.nan,
            "parasitic_energy_ratio": math.nan,
        }
    start, _ = episode
    row = rows[start]
    incident_speed = abs(float(row.get("trace_initial_vn", math.nan)))
    if not math.isfinite(incident_speed):
        incident_speed = abs(float(row["vz"]))
    depth = float(row.get("modified_depth", math.nan))
    if not math.isfinite(depth):
        depth = float(row.get("depth", math.nan))
    kinetic = 0.5 * mass * incident_speed**2
    elastic = 0.5 * stiffness * max(depth, 0.0) ** 2
    return {
        "incident_normal_speed": incident_speed,
        "onset_modified_depth": depth,
        "parasitic_energy_ratio": elastic / kinetic if kinetic > 0.0 else math.nan,
    }


def first_bounce_metrics(rows: list[dict[str, Any]], case: Case) -> dict[str, float]:
    """Return the first apex after separation, before any second contact."""
    episode = first_contact_episode(rows)
    if episode is None:
        return {
            "first_bounce_height": math.nan,
            "reference_bounce_height": math.nan,
            "first_bounce_height_ratio": math.nan,
        }
    start, end = episode
    if end + 1 >= len(rows) or rows[end + 1]["contact"]:
        return {
            "first_bounce_height": math.nan,
            "reference_bounce_height": math.nan,
            "first_bounce_height_ratio": math.nan,
        }
    next_contact = next(
        (index for index in range(end + 1, len(rows)) if rows[index]["contact"]),
        len(rows),
    )
    peak_z = max(float(rows[index]["z"]) for index in range(end + 1, next_contact))
    incident_speed = abs(float(rows[start].get("trace_initial_vn", math.nan)))
    if not math.isfinite(incident_speed):
        incident_speed = abs(float(rows[start]["vz"]))
    onset_height = float(rows[start]["z"]) - case.radius
    reference_height = onset_height + incident_speed**2 / (2.0 * 9.81)
    bounce_height = peak_z - case.radius
    return {
        "first_bounce_height": bounce_height,
        "reference_bounce_height": reference_height,
        "first_bounce_height_ratio": (
            bounce_height / reference_height if reference_height > 0.0 else math.nan
        ),
    }


def run_sweep(args: argparse.Namespace) -> list[dict[str, Any]]:
    rows_out: list[dict[str, Any]] = []
    base = Case(
        "unused",
        normal_speed=args.normal_speed,
        tangential_speed=args.tangential_speed,
        damping_ratio=0.0,
        nominal_dt=args.dt,
        initial_gap=args.initial_gap,
        duration=args.duration,
    )
    total = len(args.stiffness) * len(args.friction)
    sequence = 0
    for friction in args.friction:
        for stiffness in args.stiffness:
            sequence += 1
            name = f"mu_{friction:g}_k_{stiffness:.0e}".replace("+", "")
            case = replace(base, name=name, stiffness=stiffness, friction=friction)
            row: dict[str, Any] = {
                **asdict(case),
                "damping": case.damping,
                "status": "error",
                "error": "",
            }
            print(f"[{sequence}/{total}] mu={friction:g}, k={stiffness:.3e}")
            try:
                trajectory, summary, trace_dir = run_case(case, "shifted", args.output_dir)
                row.update(summary)
                row.update(contact_onset_metrics(trajectory, case.mass, stiffness))
                row.update(first_bounce_metrics(trajectory, case))
                terminal_speed = float(summary.get("terminal_normal_velocity", math.nan))
                incident_speed = float(row["incident_normal_speed"])
                row["normal_rebound_speed_ratio"] = (
                    terminal_speed / incident_speed
                    if incident_speed > 0.0 and math.isfinite(terminal_speed)
                    else math.nan
                )
                row["normal_energy_ratio"] = (
                    row["normal_rebound_speed_ratio"] ** 2
                    if math.isfinite(row["normal_rebound_speed_ratio"])
                    else math.nan
                )
                row["status"] = "ok"
                if not args.skip_replay and trace_dir is not None:
                    replay_dir = args.output_dir / "solver_replay" / name
                    row.update(replay_shifted_trace(trace_dir, replay_dir, args.timing_runs))
            except Exception as error:  # Keep the failure boundary as experiment data.
                row["error"] = f"{type(error).__name__}: {error}"
                (args.output_dir / "failures").mkdir(parents=True, exist_ok=True)
                with (args.output_dir / "failures" / f"{name}.txt").open(
                    "w", encoding="utf-8"
                ) as stream:
                    traceback.print_exc(file=stream)
                print(f"  failed: {row['error']}", file=sys.stderr)
            rows_out.append(row)
            write_csv(args.output_dir / "high_stiffness_regression.csv", rows_out)
    return rows_out


def plot_results(output_dir: Path, rows: list[dict[str, Any]]) -> Path:
    figure, axis = plt.subplots(figsize=(3.5, 2.55), constrained_layout=True)
    friction_values = sorted({float(row["friction"]) for row in rows})
    markers = ("o", "s", "^")
    colors = (COLORS["reference"], COLORS["shifted"], COLORS["stress"])
    for index, friction in enumerate(friction_values):
        selected = sorted(
            (row for row in rows if math.isclose(float(row["friction"]), friction)),
            key=lambda row: float(row["stiffness"]),
        )
        valid = [
            row
            for row in selected
            if row.get("status") == "ok"
            and math.isfinite(float(row.get("first_bounce_height_ratio", math.nan)))
        ]
        axis.semilogx(
            [float(row["stiffness"]) for row in valid],
            [float(row["first_bounce_height_ratio"]) for row in valid],
            marker=markers[index % len(markers)],
            color=colors[index % len(colors)],
            linewidth=1.2,
            markersize=4.0,
            label=rf"$\mu={friction:g}$",
        )
    axis.axhline(1.0, color="#777777", linestyle=":", linewidth=1.0)
    axis.set_xlabel(r"Contact stiffness $k$ [N/m]")
    axis.set_ylabel(r"First-bounce height ratio $h_1/h_{\rm ref}$")
    height_ratios = [
        float(row["first_bounce_height_ratio"])
        for row in rows
        if row.get("status") == "ok"
        and math.isfinite(float(row.get("first_bounce_height_ratio", math.nan)))
    ]
    if height_ratios:
        observed_min = min(min(height_ratios), 1.0)
        observed_max = max(max(height_ratios), 1.0)
        span = max(observed_max - observed_min, 0.02)
        padding = 0.15 * span
        axis.set_ylim(max(0.0, observed_min - padding), observed_max + padding)
    axis.grid(True, which="both", alpha=0.18)
    axis.legend(
        loc="lower center",
        bbox_to_anchor=(0.5, 1.01),
        ncol=len(friction_values),
        columnspacing=1.4,
    )

    base = output_dir / "high_stiffness_regression"
    figure.canvas.draw()
    if require_matplotlib_panel_alignment is not None:
        require_matplotlib_panel_alignment(
            figure,
            json_out=str(base.with_suffix(".alignment-layout.json")),
            overlay_svg=str(base.with_suffix(".alignment-overlay.svg")),
            tolerance_pt=1.5,
            gutter_tolerance_pt=1.5,
            require_panel_labels=False,
            strict=True,
        )
    else:
        print(
            "warning: audit_panel_alignment is unavailable; "
            "figure exported without a formal alignment manifest",
            file=sys.stderr,
        )
    figure.savefig(base.with_suffix(".pdf"), bbox_inches="tight")
    figure.savefig(base.with_suffix(".svg"), bbox_inches="tight")
    figure.savefig(base.with_suffix(".png"), dpi=300, bbox_inches="tight")
    figure.savefig(base.with_suffix(".tiff"), dpi=600, bbox_inches="tight")
    plt.close(figure)
    return base


def summarize_results(output_dir: Path, rows: list[dict[str, Any]]) -> None:
    summary: dict[str, Any] = {"conditions": len(rows)}
    summary["completed"] = sum(row.get("status") == "ok" for row in rows)
    summary["failed"] = sum(row.get("status") != "ok" for row in rows)
    for friction in sorted({float(row["friction"]) for row in rows}):
        selected = [row for row in rows if math.isclose(float(row["friction"]), friction)]
        physical = [
            row
            for row in selected
            if row.get("status") == "ok"
            and math.isfinite(float(row.get("first_bounce_height_ratio", math.nan)))
            and abs(float(row["first_bounce_height_ratio"]) - 1.0) <= 0.01
        ]
        summary[f"mu_{friction:g}"] = {
            "largest_stiffness_within_1_percent_first_bounce_height": (
                max(float(row["stiffness"]) for row in physical) if physical else None
            ),
            "failed_stiffness": [
                float(row["stiffness"])
                for row in selected
                if row.get("status") != "ok"
            ],
        }
    with (output_dir / "summary.json").open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=HERE / "results" / "experiment2_high_stiffness",
    )
    parser.add_argument(
        "--stiffness",
        type=parse_number_list,
        default=DEFAULT_STIFFNESS,
        help="comma-separated stiffness values in N/m",
    )
    parser.add_argument(
        "--friction",
        type=parse_number_list,
        default=(0.0, 0.3),
        help="comma-separated friction coefficients",
    )
    parser.add_argument("--normal-speed", type=float, default=0.5)
    parser.add_argument("--tangential-speed", type=float, default=0.5)
    parser.add_argument("--initial-gap", type=float, default=0.01)
    parser.add_argument("--dt", type=float, default=1.0e-3)
    parser.add_argument("--duration", type=float, default=0.25)
    parser.add_argument("--timing-runs", type=int, default=5)
    parser.add_argument("--skip-replay", action="store_true")
    args = parser.parse_args()
    args.output_dir = args.output_dir.resolve()
    return args


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with (args.output_dir / "configuration.json").open("w", encoding="utf-8") as stream:
        json.dump(
            {
                "stiffness": args.stiffness,
                "friction": args.friction,
                "normal_speed": args.normal_speed,
                "tangential_speed": args.tangential_speed,
                "initial_gap": args.initial_gap,
                "dt": args.dt,
                "duration": args.duration,
                "timing_runs": args.timing_runs,
                "skip_replay": args.skip_replay,
            },
            stream,
            indent=2,
        )
    rows = run_sweep(args)
    summarize_results(args.output_dir, rows)
    figure = plot_results(args.output_dir, rows)
    print(f"figure: {figure}.pdf")
    print(f"results: {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
