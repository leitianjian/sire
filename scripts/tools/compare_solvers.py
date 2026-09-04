"""Compare contact solvers on one frame or a recorded simulation trajectory.

Examples:
    python compare_solvers.py --load result.json
    python compare_solvers.py --load solver_traces/trajectory_20260825_120000_000
    python compare_solvers.py --load solver_traces --plot --top 20

The C++ trace recorder writes ``frame_XXXXXXXX.json`` files. When ``--load``
is a directory this script discovers those files recursively, runs every
selected solver on every frame, prints the important outliers, and writes a
CSV plus JSON summaries.
"""

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np


REQUIRED_FIELDS = ("n", "fri_coef", "invM", "v0", "v_target", "b", "h")
SOLVER_FUNCTIONS = {
    "v2": "cptContactForceWithTargetState2",
    "v3": "cptContactForceWithTargetState3",
    "v4": "cptContactForceWithTargetState4",
    "v5": "cptContactForceWithTargetState5",
    "v6": "cptContactForceWithTargetState6",
    "Spectral-ADMM": "cptContactForceSpectralAdmm",
    "Shifted-Spectral-ADMM": "cptContactForceShiftedSpectralAdmm",
    "v7": "cptContactForceWithTargetState7",
    "v8": "cptContactForceWithTargetState8",
    "Newton-PIPG": "cptContactForceNewtonPipg",
    "Exact-FBF": "cptContactForceExactCoulombFbf",
    "Exact-SSN": "cptContactForceExactCoulombSemismoothNewton",
    "Single-loop-FBF-SSN": "cptContactForceSingleLoopFbfSsn",
    "Explicit-FBF-Gated-SSN": "cptContactForceExplicitFbfGatedSsn",
    "Mode-Predictor-SSN": "cptContactForceModePredictorSsn",
    "Exact-Coulomb": "cptContactForceExactCoulomb",
}
DEFAULT_COMPARISON_MAX_ITERS = 200
SOLVER_METHODS = {
    "v2": "v2 | Clarabel tangential SOCP + regularized COD normal solve",
    "v3": "v3 | COD normal solve + max-dissipation direction fixed point",
    "v4": "v4 | per-contact SOR + stick/slip bisection",
    "v5": "v5 | DAE-NCP ADMM",
    "v6": "v6 | nested ADMM NCP with frozen De Saxce inner solves",
    "Spectral-ADMM": (
        "Spectral ADMM [Carpentier et al.]"
    ),
    "Shifted-Spectral-ADMM": "Spectral ADMM | NCP normal shift by v_target",
    "v7": "v7 | DAE Davis-Yin three-operator splitting",
    "v8": "v8 | DAE PDDY / primal-dual DRS with exact F prox",
    "Newton-PIPG": "Newton-PIPG | equality QP + SOC-face Newton/rank compression",
    "Exact-FBF": "Exact-FBF | target-shifted de Saxce FBF only",
    "Exact-SSN": "Exact-SSN | target-shifted damped mixed SSN only",
    "Single-loop-FBF-SSN": (
        "Single-loop FBF-SSN | one-sweep prox tracker + mixed SSN"
    ),
    "Explicit-FBF-Gated-SSN": (
        "Explicit FBF + gated SSN | A=N_K analytic cone projection"
    ),
    "Mode-Predictor-SSN": (
        "Block mode predictor +  SSN + rare FBF rescue"
    ),
    "Exact-Coulomb": "Exact-Coulomb | target-shifted de Saxce FBF + mixed SSN",
    "Recorded": "Recorded | simulation output",
}
DEFAULT_SOLVERS = list(SOLVER_FUNCTIONS)
METRIC_FIELDS = (
    "exact_residual",
    "complementarity",
    "cone_violation",
    "normal_force_violation",
    "normal_velocity_violation",
    "target_residual",
)


def discover_frames(load_path: Path) -> List[Path]:
    if load_path.is_file():
        return [load_path]
    if not load_path.is_dir():
        raise FileNotFoundError(f"input does not exist: {load_path}")
    frames = sorted(load_path.rglob("frame_*.json"))
    if not frames:
        frames = sorted(p for p in load_path.glob("*.json")
                        if p.name != "manifest.json")
    if not frames:
        raise FileNotFoundError(f"no frame_*.json files under: {load_path}")
    return frames


def load_frame(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    missing = [field for field in REQUIRED_FIELDS if field not in data]
    if missing:
        raise ValueError(f"missing fields {missing}")

    n = int(data["n"])
    expected_lengths = {
        "fri_coef": n,
        "invM": 9 * n * n,
        "v0": 3 * n,
        "v_target": n,
        "b": 3 * n,
    }
    for field, expected in expected_lengths.items():
        if len(data[field]) != expected:
            raise ValueError(
                f"{field} has length {len(data[field])}, expected {expected}"
            )
    return data


def solver_arguments(data: dict, max_iters: int, max_err: float) -> tuple:
    return (
        int(data["n"]),
        [float(x) for x in data["fri_coef"]],
        [float(x) for x in data["invM"]],
        [float(x) for x in data["v0"]],
        [float(x) for x in data["v_target"]],
        [float(x) for x in data["b"]],
        float(data["h"]),
        max_iters,
        max_err,
    )


def run_solver(sire_module, name: str, data: dict, runs: int,
               max_err: float, max_iters: int) -> Tuple[
                   np.ndarray, float, float, int
               ]:
    function_name = SOLVER_FUNCTIONS[name]
    function = getattr(sire_module, function_name)
    arguments = solver_arguments(data, max_iters, max_err)
    start = time.perf_counter()
    result = None
    error = math.nan
    for _ in range(runs):
        raw_result, error = function(*arguments)
        result = np.asarray(raw_result, dtype=np.float64)
    elapsed_ms = (time.perf_counter() - start) * 1000.0 / runs
    return result, float(error), elapsed_ms, max_iters


def problem_matrices(data: dict) -> Tuple[np.ndarray, np.ndarray]:
    n = int(data["n"])
    h = float(data["h"])
    inverse_mass = np.asarray(data["invM"], dtype=np.float64).reshape(
        3 * n, 3 * n
    )
    hessian = -h * inverse_mass
    hessian = 0.5 * (hessian + hessian.T)
    free_velocity = (
        np.asarray(data["v0"], dtype=np.float64)
        - h * np.asarray(data["b"], dtype=np.float64)
    )
    return hessian, free_velocity


def exact_coulomb_residual(data: dict, force: np.ndarray) -> float:
    """Dimensionless mixed NCP/maximum-dissipation residual used by C++."""
    n = int(data["n"])
    mu = np.asarray(data["fri_coef"], dtype=np.float64)
    hessian, free_velocity = problem_matrices(data)
    shifted_free = free_velocity.copy()
    shifted_free[2::3] += np.asarray(data["v_target"], dtype=np.float64)
    velocity = hessian @ force + shifted_free
    hessian_scale = max(float(np.linalg.norm(hessian)), 1e-12)
    base_rho = 1.0 / hessian_scale
    maximum = 0.0

    for i in range(n):
        tangent = 3 * i
        normal = tangent + 2
        normal_curvature = max(
            abs(hessian[normal, normal]), 1e-6 * hessian_scale
        )
        tangent_curvature = max(
            0.5
            * (abs(hessian[tangent, tangent])
               + abs(hessian[tangent + 1, tangent + 1])),
            1e-6 * hessian_scale,
        )
        rho_n = np.clip(1.0 / normal_curvature,
                        1e-3 * base_rho, 1e3 * base_rho)
        rho_t = np.clip(1.0 / tangent_curvature,
                        1e-3 * base_rho, 1e3 * base_rho)
        normal_force = force[normal]
        tangent_force = force[tangent:tangent + 2]
        normal_velocity = velocity[normal]
        tangent_velocity = velocity[tangent:tangent + 2]
        normal_residual = normal_force - max(
            0.0, normal_force - rho_n * normal_velocity
        )
        disk_argument = tangent_force - rho_t * tangent_velocity
        radius = max(0.0, mu[i] * max(0.0, normal_force))
        disk_norm = float(np.linalg.norm(disk_argument))
        disk_projection = (
            disk_argument
            if disk_norm <= radius
            else radius * disk_argument / max(disk_norm, 1e-300)
        )
        residual = np.r_[tangent_force - disk_projection, normal_residual]
        scale = 1.0 + max(
            abs(normal_force),
            float(np.linalg.norm(tangent_force)),
            rho_n * abs(normal_velocity),
            rho_t * float(np.linalg.norm(tangent_velocity)),
        )
        maximum = max(maximum, float(np.linalg.norm(residual)) / scale)
    return maximum


def physical_metrics(data: dict, force: np.ndarray) -> Dict[str, float]:
    if not np.all(np.isfinite(force)):
        return {name: math.inf for name in METRIC_FIELDS}

    n = int(data["n"])
    mu = np.asarray(data["fri_coef"], dtype=np.float64)
    hessian, free_velocity = problem_matrices(data)
    post_velocity = hessian @ force + free_velocity
    target_residual = (
        post_velocity[2::3]
        + np.asarray(data["v_target"], dtype=np.float64)
    )
    normal_force = force[2::3]
    cone_violation = 0.0
    for i in range(n):
        tangent_norm = float(np.linalg.norm(force[3 * i:3 * i + 2]))
        cone_violation = max(
            cone_violation,
            max(0.0, -normal_force[i]),
            max(0.0, tangent_norm - mu[i] * max(0.0, normal_force[i])),
        )

    return {
        "exact_residual": exact_coulomb_residual(data, force),
        "complementarity": float(np.max(np.abs(
            normal_force * target_residual
        ))),
        "cone_violation": cone_violation,
        "normal_force_violation": max(0.0, -float(np.min(normal_force))),
        "normal_velocity_violation": max(
            0.0, -float(np.min(target_residual))
        ),
        "target_residual": float(np.max(np.abs(target_residual))),
    }


def recorded_force(data: dict) -> Optional[np.ndarray]:
    for field in ("cResult", "cADMMResult"):
        if field in data:
            value = np.asarray(data[field], dtype=np.float64)
            if value.size == 3 * int(data["n"]):
                return value
    return None


def percentile(values: Iterable[float], quantile: float) -> Optional[float]:
    finite = [value for value in values if math.isfinite(value)]
    return float(np.percentile(finite, quantile)) if finite else None


def json_number(value):
    if isinstance(value, (float, np.floating)) and not math.isfinite(value):
        return None
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        return float(value)
    return value


def write_outputs(output_dir: Path, rows: List[dict], interesting: List[dict],
                  input_path: Path, frame_count: int, reference: str,
                  args: argparse.Namespace) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0]) if rows else []
    with (output_dir / "metrics.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    aggregates = {}
    solver_names = sorted({row["solver"] for row in rows})
    for solver in solver_names:
        solver_rows = [row for row in rows if row["solver"] == solver]
        aggregates[solver] = {
            "method": SOLVER_METHODS[solver],
            "max_iters": sorted({int(row["max_iters"]) for row in solver_rows}),
            "frames": len(solver_rows),
            "failures": sum(row["status"] != "ok" for row in solver_rows),
            "converged_frames": sum(bool(row["converged"])
                                    for row in solver_rows),
            "convergence_rate": (
                sum(bool(row["converged"]) for row in solver_rows)
                / len(solver_rows) if solver_rows else None
            ),
            "interesting_frames": sum(bool(row["interesting"])
                                      for row in solver_rows),
        }
        for field in ("time_ms", "solver_error", "exact_residual", "force_diff_rel",
                      "velocity_diff_inf"):
            values = [float(row[field]) for row in solver_rows
                      if row[field] not in ("", None)]
            aggregates[solver][field] = {
                "mean": json_number(float(np.mean(values))) if values else None,
                "p95": json_number(percentile(values, 95.0)),
                "max": json_number(max(values)) if values else None,
            }

    summary = {
        "input": str(input_path.resolve()),
        "frames": frame_count,
        "reference": reference,
        "requested_solver_tolerance": args.max_err,
        "comparison_definitions": {
            "force_diff_rel": (
                "||f-reference||_2 / max(||f||_2, ||reference||_2, 1e-12)"
            ),
            "interesting": (
                "solver failure, physical residual threshold, induced velocity "
                "threshold, or both absolute and relative force thresholds"
            ),
            "force_abs_threshold": args.force_abs_threshold,
            "force_rel_threshold": args.force_rel_threshold,
            "velocity_threshold": args.velocity_threshold,
            "residual_threshold": args.residual_threshold,
        },
        "aggregates": aggregates,
    }
    with (output_dir / "summary.json").open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    with (output_dir / "interesting_frames.json").open(
        "w", encoding="utf-8"
    ) as stream:
        json.dump(
            [{key: json_number(value) for key, value in item.items()}
             for item in interesting],
            stream,
            ensure_ascii=False,
            indent=2,
        )
        stream.write("\n")


def make_plot(output_dir: Path, rows: List[dict]) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is unavailable; skipping plot", file=sys.stderr)
        return

    comparable = [row for row in rows if row["force_diff_rel"] != ""]
    if not comparable:
        return
    figure, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for solver in sorted({row["solver"] for row in comparable}):
        solver_rows = [row for row in comparable if row["solver"] == solver]
        x = [row["frame"] for row in solver_rows]
        axes[0].semilogy(x, [max(float(row["force_diff_rel"]), 1e-16)
                            for row in solver_rows],
                        label=SOLVER_METHODS[solver])
        axes[1].semilogy(x, [max(float(row["velocity_diff_inf"]), 1e-16)
                            for row in solver_rows], label=solver)
    axes[0].set_ylabel("relative force difference")
    axes[1].set_ylabel("max induced velocity difference [m/s]")
    axes[1].set_xlabel("recorded contact frame")
    axes[0].legend(ncol=3, fontsize=8)
    axes[0].grid(True, which="both", alpha=0.25)
    axes[1].grid(True, which="both", alpha=0.25)
    figure.tight_layout()
    figure.savefig(output_dir / "trajectory_comparison.png", dpi=160)
    plt.close(figure)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare contact solvers on one JSON frame or a trajectory"
    )
    parser.add_argument(
        "--load", required=True,
        help="frame JSON, trajectory directory, or root containing trajectories",
    )
    parser.add_argument(
        "--solvers", nargs="+", choices=DEFAULT_SOLVERS,
        default=DEFAULT_SOLVERS, help="solvers to run (default: all)",
    )
    parser.add_argument(
        "--reference", choices=DEFAULT_SOLVERS + ["Recorded"],
        default="Exact-Coulomb", help="baseline for force/velocity differences",
    )
    parser.add_argument("--runs", type=int, default=1,
                        help="timing repetitions per solver/frame (default: 1)")
    parser.add_argument(
        "--max-iters", type=int,
        help=("override the shared iteration budget; by default it is read "
              "from each trace frame's solver_max_iters, falling back to 200"),
    )
    parser.add_argument("--max-err", type=float, default=1e-8)
    parser.add_argument("--max-frames", type=int,
                        help="only process the first N discovered frames")
    parser.add_argument("--output-dir",
                        help="result directory (default: beside input)")
    parser.add_argument("--top", type=int, default=10,
                        help="number of worst comparisons to display")
    parser.add_argument("--force-rel-threshold", type=float, default=5e-2)
    parser.add_argument(
        "--force-abs-threshold", type=float, default=1e-6,
        help="ignore relative force differences smaller than this many newtons",
    )
    parser.add_argument("--velocity-threshold", type=float, default=1e-4)
    parser.add_argument("--residual-threshold", type=float, default=1e-4)
    parser.add_argument("--plot", action="store_true",
                        help="save trajectory_comparison.png")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.runs < 1:
        raise ValueError("--runs must be at least 1")
    if args.max_iters is not None and args.max_iters < 1:
        raise ValueError("--max-iters must be at least 1")
    thresholds = {
        "--force-rel-threshold": args.force_rel_threshold,
        "--force-abs-threshold": args.force_abs_threshold,
        "--velocity-threshold": args.velocity_threshold,
        "--residual-threshold": args.residual_threshold,
    }
    for option, value in thresholds.items():
        if value <= 0.0:
            raise ValueError(f"{option} must be positive")

    input_path = Path(args.load)
    frames = discover_frames(input_path)
    if args.max_frames is not None:
        frames = frames[:args.max_frames]
    if not frames:
        raise ValueError("no frames selected")

    import sire

    budget_from_trace = False
    try:
        first_frame_data = load_frame(frames[0])
        budget_from_trace = "solver_max_iters" in first_frame_data
        trace_max_iters = int(first_frame_data.get(
            "solver_max_iters", DEFAULT_COMPARISON_MAX_ITERS
        ))
    except (OSError, ValueError, json.JSONDecodeError, TypeError):
        trace_max_iters = DEFAULT_COMPARISON_MAX_ITERS
    if trace_max_iters < 1:
        trace_max_iters = DEFAULT_COMPARISON_MAX_ITERS
    displayed_max_iters = (args.max_iters if args.max_iters is not None
                           else trace_max_iters)
    if args.max_iters is not None:
        budget_source = "--max-iters"
    elif budget_from_trace:
        budget_source = "trace solver_max_iters"
    else:
        budget_source = "fallback default"

    solver_names = list(dict.fromkeys(args.solvers))
    if args.reference != "Recorded" and args.reference not in solver_names:
        solver_names.append(args.reference)
    missing_bindings = [
        name for name in solver_names
        if not hasattr(sire, SOLVER_FUNCTIONS[name])
    ]
    if missing_bindings:
        raise RuntimeError(
            "sire module is missing bindings for: " + ", ".join(missing_bindings)
        )

    if args.output_dir:
        output_dir = Path(args.output_dir)
    elif input_path.is_dir():
        output_dir = input_path / "compare_results"
    else:
        output_dir = input_path.parent / f"{input_path.stem}_compare"

    print(f"sire module: {sire.__file__}")
    print(f"frames: {len(frames)}  reference: {args.reference}")
    print(f"shared max_iters: {displayed_max_iters} ({budget_source})")
    print("solvers:")
    for solver in solver_names:
        print(f"  {SOLVER_METHODS[solver]}  [max_iters={displayed_max_iters}]")

    rows: List[dict] = []
    interesting: List[dict] = []
    invalid_frames = 0
    for sequence, frame_path in enumerate(frames):
        try:
            data = load_frame(frame_path)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            invalid_frames += 1
            print(f"skip {frame_path}: {error}", file=sys.stderr)
            continue

        frame_number = int(data.get("frame", sequence))
        sim_time = data.get("sim_time", "")
        try:
            frame_max_iters = (
                args.max_iters if args.max_iters is not None else int(
                    data.get("solver_max_iters", DEFAULT_COMPARISON_MAX_ITERS)
                )
            )
        except (TypeError, ValueError):
            frame_max_iters = DEFAULT_COMPARISON_MAX_ITERS
        if frame_max_iters < 1:
            frame_max_iters = DEFAULT_COMPARISON_MAX_ITERS
        hessian, _ = problem_matrices(data)
        results: Dict[str, np.ndarray] = {}
        execution: Dict[str, Tuple[float, float, int, str]] = {}
        for solver in solver_names:
            try:
                force, solver_error, elapsed_ms, max_iters = run_solver(
                    sire, solver, data, args.runs, args.max_err,
                    frame_max_iters,
                )
                expected = 3 * int(data["n"])
                if force.size != expected:
                    raise ValueError(
                        f"returned {force.size} force values, expected {expected}"
                    )
                results[solver] = force
                execution[solver] = (
                    solver_error, elapsed_ms, max_iters, "ok"
                )
            except Exception as error:  # keep the rest of a long trace usable
                execution[solver] = (math.nan, math.nan, frame_max_iters,
                                     f"{type(error).__name__}: {error}")

        recorded = recorded_force(data)
        if recorded is not None:
            results["Recorded"] = recorded
            execution["Recorded"] = (
                float(data.get("solver_error", math.nan)),
                0.0,
                int(data.get("solver_max_iters", 200)),
                "ok",
            )

        reference = results.get(args.reference)
        for solver, values in execution.items():
            solver_error, elapsed_ms, max_iters, status = values
            solver_tolerance = (
                float(data.get("solver_max_error", args.max_err))
                if solver == "Recorded" else args.max_err
            )
            converged = (
                status == "ok"
                and math.isfinite(float(solver_error))
                and float(solver_error) <= solver_tolerance
            )
            force = results.get(solver)
            if force is None:
                metrics = {field: math.inf for field in METRIC_FIELDS}
            else:
                metrics = physical_metrics(data, force)

            force_diff_l2 = force_diff_rel = ""
            velocity_diff_l2 = velocity_diff_inf = ""
            if reference is not None and force is not None:
                force_difference = force - reference
                velocity_difference = hessian @ force_difference
                force_diff_l2 = float(np.linalg.norm(force_difference))
                force_diff_rel = force_diff_l2 / max(
                    float(np.linalg.norm(reference)),
                    float(np.linalg.norm(force)),
                    1e-12,
                )
                velocity_diff_l2 = float(np.linalg.norm(velocity_difference))
                velocity_diff_inf = float(np.max(np.abs(velocity_difference)))

            is_interesting = status != "ok" or not converged or (
                force_diff_rel != "" and (
                    not math.isfinite(force_diff_rel)
                    or (
                        force_diff_l2 >= args.force_abs_threshold
                        and force_diff_rel >= args.force_rel_threshold
                    )
                    or velocity_diff_inf >= args.velocity_threshold
                )
            ) or metrics["exact_residual"] >= args.residual_threshold

            row = {
                "frame": frame_number,
                "sim_time": sim_time,
                "file": str(frame_path.resolve()),
                "n": int(data["n"]),
                "solver": solver,
                "method": SOLVER_METHODS[solver],
                "max_iters": max_iters,
                "solver_tolerance": solver_tolerance,
                "solver_error": solver_error,
                "converged": int(converged),
                "time_ms": elapsed_ms,
                **metrics,
                "reference": args.reference,
                "force_diff_l2": force_diff_l2,
                "force_diff_rel": force_diff_rel,
                "velocity_diff_l2": velocity_diff_l2,
                "velocity_diff_inf": velocity_diff_inf,
                "interesting": int(is_interesting),
                "status": status,
            }
            rows.append(row)
            if is_interesting:
                interesting.append({
                    "frame": frame_number,
                    "sim_time": sim_time,
                    "file": str(frame_path.resolve()),
                    "solver": solver,
                    "method": SOLVER_METHODS[solver],
                    "status": status,
                    "solver_error": solver_error,
                    "solver_tolerance": solver_tolerance,
                    "converged": int(converged),
                    "force_diff_rel": force_diff_rel,
                    "force_diff_l2": force_diff_l2,
                    "velocity_diff_inf": velocity_diff_inf,
                    "exact_residual": metrics["exact_residual"],
                    "cone_violation": metrics["cone_violation"],
                })

        if len(frames) == 1:
            print(f"\nframe={frame_number} t={sim_time} n={data['n']}")
            for solver, force in results.items():
                metric = physical_metrics(data, force)
                print(
                    f"  {SOLVER_METHODS[solver]}\n"
                    f"    error={execution[solver][0]:.3e} "
                    f"R={metric['exact_residual']:.3e} "
                    f"cone={metric['cone_violation']:.3e} "
                    f"force={np.array2string(force, precision=6)}"
                )
        elif (sequence + 1) % 25 == 0 or sequence + 1 == len(frames):
            print(f"processed {sequence + 1}/{len(frames)} frames")

    if not rows:
        print("no valid frames were processed", file=sys.stderr)
        return 1

    write_outputs(output_dir, rows, interesting, input_path,
                  len(frames) - invalid_frames, args.reference, args)
    if args.plot:
        make_plot(output_dir, rows)

    def severity(item: dict) -> float:
        candidates = [item["exact_residual"] / args.residual_threshold]
        velocity = item["velocity_diff_inf"]
        if velocity != "" and velocity is not None:
            candidates.append(float(velocity) / args.velocity_threshold)
        relative = item["force_diff_rel"]
        absolute = item["force_diff_l2"]
        if (relative != "" and relative is not None
                and absolute != "" and absolute is not None
                and float(absolute) >= args.force_abs_threshold):
            candidates.append(float(relative) / args.force_rel_threshold)
        return max(value if math.isfinite(value) else math.inf
                   for value in candidates)

    worst = sorted(interesting, key=severity, reverse=True)[:args.top]
    print(f"\ninteresting comparisons: {len(interesting)} / {len(rows)}")
    print("aggregate (finite values):")
    print(f"  {'method':<58} {'conv%':>7} {'mean ms':>10} {'p95 R':>11} "
          f"{'p95 rel_df':>12} {'max dv_inf':>12} {'flagged':>9}")
    for solver in sorted({row["solver"] for row in rows}):
        solver_rows = [row for row in rows if row["solver"] == solver]
        times = [float(row["time_ms"]) for row in solver_rows
                 if math.isfinite(float(row["time_ms"]))]
        residuals = [float(row["exact_residual"]) for row in solver_rows]
        relative = [float(row["force_diff_rel"]) for row in solver_rows
                    if row["force_diff_rel"] != ""]
        velocities = [float(row["velocity_diff_inf"]) for row in solver_rows
                      if row["velocity_diff_inf"] != ""]

        def display(value: Optional[float]) -> str:
            return "-" if value is None else f"{value:.3e}"

        print(
            f"  {SOLVER_METHODS[solver]:<58} "
            f"{100.0 * sum(bool(row['converged']) for row in solver_rows) / len(solver_rows):>6.1f}% "
            f"{display(float(np.mean(times)) if times else None):>10} "
            f"{display(percentile(residuals, 95.0)):>11} "
            f"{display(percentile(relative, 95.0)):>12} "
            f"{display(max((v for v in velocities if math.isfinite(v)), default=None)):>12} "
            f"{sum(bool(row['interesting']) for row in solver_rows):>9}"
        )
    if worst:
        print("worst frames:")
        for item in worst:
            print(
                f"  frame={item['frame']} t={item['sim_time']} "
                f"method={item['method']} rel_df={item['force_diff_rel']} "
                f"abs_df={item['force_diff_l2']} "
                f"dv_inf={item['velocity_diff_inf']} "
                f"R={item['exact_residual']:.3e} status={item['status']}"
            )
    print(f"results: {output_dir.resolve()}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (FileNotFoundError, ValueError, RuntimeError) as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(2)
