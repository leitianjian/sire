"""Ablate FBF, damped mixed SSN, and their safeguarded hybrid.

The three modes call the same C++ problem construction and exact-Coulomb
residual.  Timing calls disable histories; one extra untimed call per problem
collects iteration diagnostics for convergence plots.

Example (PowerShell):
    python scripts/tools/compare_exact_coulomb_modes.py `
      --load scripts/tools/solver_traces/dog/trajectory_20260825_134820_716 `
      --budgets 5 10 20 30 50 100 200 --runs 100 --plot
"""

import argparse
import csv
import json
import math
import time
from pathlib import Path
from typing import Dict, Iterable, List

import numpy as np

from compare_solvers import (
    discover_frames,
    exact_coulomb_residual,
    load_frame,
    physical_metrics,
)


MODES = ("fbf", "newton", "hybrid")


def solver_args(data: dict) -> tuple:
    return (
        int(data["n"]),
        [float(x) for x in data["fri_coef"]],
        [float(x) for x in data["invM"]],
        [float(x) for x in data["v0"]],
        [float(x) for x in data["v_target"]],
        [float(x) for x in data["b"]],
        float(data["h"]),
    )


def percentile(values: Iterable[float], q: float) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return float(np.percentile(finite, q)) if finite else math.inf


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare exact-Coulomb FBF, SSN, and hybrid updates"
    )
    parser.add_argument("--load", required=True)
    parser.add_argument("--modes", nargs="+", choices=MODES,
                        default=list(MODES))
    parser.add_argument("--budgets", nargs="+", type=int,
                        default=[5, 10, 20, 30, 50, 100, 200])
    parser.add_argument("--runs", type=int, default=20,
                        help="timed calls per frame/mode/budget")
    parser.add_argument("--warmup-runs", type=int, default=2)
    parser.add_argument("--max-err", type=float, default=1e-8)
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--newton-start", type=int, default=2)
    parser.add_argument(
        "--newton-switch-residual", type=float, default=math.inf,
        help="hybrid Newton gate; inf reproduces opportunistic Newton",
    )
    parser.add_argument("--output-dir")
    parser.add_argument("--plot", action="store_true")
    return parser.parse_args()


def write_csv(path: Path, rows: List[dict]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def aggregate(rows: List[dict]) -> List[dict]:
    output = []
    for mode in sorted({row["mode"] for row in rows}):
        for budget in sorted({int(row["budget"]) for row in rows}):
            group = [row for row in rows
                     if row["mode"] == mode and int(row["budget"]) == budget]
            if not group:
                continue
            output.append({
                "mode": mode,
                "budget": budget,
                "frames": len(group),
                "converged_fraction": float(np.mean(
                    [int(row["converged"]) for row in group]
                )),
                "newton_rejection_fraction": float(np.mean([
                    int(row["terminated_on_newton_rejection"])
                    for row in group
                ])),
                "residual_median": percentile(
                    [float(row["exact_residual"]) for row in group], 50
                ),
                "residual_p95": percentile(
                    [float(row["exact_residual"]) for row in group], 95
                ),
                "time_median_ms": percentile(
                    [float(row["time_median_ms"]) for row in group], 50
                ),
                "time_p95_ms": percentile(
                    [float(row["time_p95_ms"]) for row in group], 95
                ),
                "iterations_median": percentile([
                    float(row["nonlinear_iterations"]) for row in group
                ], 50),
                "newton_accepted_mean": float(np.mean([
                    float(row["newton_accepted"]) for row in group
                ])),
                "fbf_steps_mean": float(np.mean([
                    float(row["fbf_steps"]) for row in group
                ])),
            })
    return output


def make_plot(output_dir: Path, summary: List[dict]) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib unavailable; skipping plot")
        return
    figure, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    for mode in MODES:
        group = sorted((row for row in summary if row["mode"] == mode),
                       key=lambda row: int(row["budget"]))
        if not group:
            continue
        budgets = [int(row["budget"]) for row in group]
        axes[0].semilogy(budgets,
                        [max(float(row["residual_p95"]), 1e-16)
                         for row in group], marker="o", label=mode)
        axes[1].plot(budgets,
                     [float(row["time_median_ms"]) for row in group],
                     marker="o", label=mode)
        axes[2].plot(budgets,
                     [float(row["converged_fraction"]) for row in group],
                     marker="o", label=mode)
    axes[0].set_ylabel("p95 exact-Coulomb residual")
    axes[1].set_ylabel("median time [ms]")
    axes[2].set_ylabel("converged frame fraction")
    for axis in axes:
        axis.set_xlabel("nonlinear update budget")
        axis.grid(True, which="both", alpha=0.3)
    axes[0].legend()
    figure.tight_layout()
    figure.savefig(output_dir / "exact_mode_ablation.png", dpi=180)
    plt.close(figure)


def main() -> int:
    args = parse_args()
    if args.runs < 1 or args.warmup_runs < 0:
        raise ValueError("runs must be positive and warmup-runs nonnegative")
    if any(budget < 1 for budget in args.budgets):
        raise ValueError("all budgets must be positive")
    if args.max_err <= 0.0:
        raise ValueError("max-err must be positive")

    frames = discover_frames(Path(args.load))
    if args.max_frames is not None:
        frames = frames[:args.max_frames]
    if not frames:
        raise ValueError("no frames selected")

    import sire
    if not hasattr(sire, "cptContactForceExactCoulombDetailed"):
        raise RuntimeError(
            "sire module lacks cptContactForceExactCoulombDetailed; rebuild it"
        )
    solve = sire.cptContactForceExactCoulombDetailed

    input_path = Path(args.load)
    output_dir = (Path(args.output_dir) if args.output_dir else
                  (input_path if input_path.is_dir() else input_path.parent)
                  / "exact_mode_ablation")
    output_dir.mkdir(parents=True, exist_ok=True)
    rows: List[dict] = []

    print(f"sire module: {sire.__file__}")
    print(f"frames={len(frames)} modes={args.modes} budgets={args.budgets}")
    for sequence, frame_path in enumerate(frames):
        data = load_frame(frame_path)
        base_args = solver_args(data)
        for budget in args.budgets:
            for mode in args.modes:
                call_args = (
                    *base_args, mode, budget, args.max_err,
                    args.newton_start, args.newton_switch_residual,
                )
                for _ in range(args.warmup_runs):
                    solve(*call_args, False)

                samples_ms = []
                for _ in range(args.runs):
                    start = time.perf_counter_ns()
                    solve(*call_args, False)
                    samples_ms.append(
                        (time.perf_counter_ns() - start) * 1e-6
                    )

                force, solver_error, stats = solve(*call_args, True)
                force = np.asarray(force, dtype=np.float64)
                metrics = physical_metrics(data, force)
                # Keep the independent Python residual as the reported value;
                # a mismatch against the C++ value catches instrumentation bugs.
                independent_residual = exact_coulomb_residual(data, force)
                rows.append({
                    "frame": int(data.get("frame", sequence)),
                    "sim_time": data.get("sim_time", ""),
                    "file": str(frame_path.resolve()),
                    "n": int(data["n"]),
                    "mode": mode,
                    "budget": budget,
                    "max_err": args.max_err,
                    "time_median_ms": percentile(samples_ms, 50),
                    "time_p95_ms": percentile(samples_ms, 95),
                    "solver_error": float(solver_error),
                    "exact_residual": independent_residual,
                    "cpp_residual": float(stats["final_residual"]),
                    "residual_disagreement": abs(
                        independent_residual - float(stats["final_residual"])
                    ),
                    "cone_violation": metrics["cone_violation"],
                    "velocity_violation": metrics["normal_velocity_violation"],
                    "converged": int(bool(stats["converged"])),
                    "nonlinear_iterations": int(stats["nonlinear_iterations"]),
                    "newton_attempts": int(stats["newton_attempts"]),
                    "newton_accepted": int(stats["newton_accepted"]),
                    "newton_rejected": int(stats["newton_rejected"]),
                    "fbf_steps": int(stats["fbf_steps"]),
                    "fbf_backtracks": int(stats["fbf_backtracks"]),
                    "terminated_on_newton_rejection": int(bool(
                        stats["terminated_on_newton_rejection"]
                    )),
                    "residual_history": json.dumps(
                        list(stats["residual_history"]), separators=(",", ":")
                    ),
                    "step_history": json.dumps(
                        list(stats["step_history"]), separators=(",", ":")
                    ),
                })
        if (sequence + 1) % 25 == 0 or sequence + 1 == len(frames):
            print(f"processed {sequence + 1}/{len(frames)} frames")

    summary = aggregate(rows)
    write_csv(output_dir / "per_frame.csv", rows)
    write_csv(output_dir / "summary.csv", summary)
    with (output_dir / "settings.json").open("w", encoding="utf-8") as stream:
        json.dump(vars(args), stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    if args.plot:
        make_plot(output_dir, summary)

    print("\nmode budget  conv%   p95 residual   median ms  p95 ms")
    for row in summary:
        print(
            f"{row['mode']:<7} {int(row['budget']):>6} "
            f"{100.0 * float(row['converged_fraction']):>6.1f} "
            f"{float(row['residual_p95']):>14.3e} "
            f"{float(row['time_median_ms']):>10.4f} "
            f"{float(row['time_p95_ms']):>8.4f}"
        )
    print(f"results: {output_dir.resolve()}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, ValueError, RuntimeError) as error:
        print(f"error: {error}")
        raise SystemExit(2)
