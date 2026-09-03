"""Run the contact-solver comparison at engineering and strict tolerances.

The default profile follows the two accuracy regimes used in the paper:
1e-4 for simulation throughput and 1e-8 for high-accuracy robustness.
Invoke this script with the same Python interpreter that imports the built
Sire module; each child comparison reuses ``sys.executable``.
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path


def tolerance_directory(value: float) -> str:
    return f"tol_{value:.0e}".replace("+", "")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare contact solvers at multiple stopping tolerances"
    )
    parser.add_argument("--load", required=True)
    parser.add_argument(
        "--solvers", nargs="+",
        default=["Spectral-ADMM", "Exact-Coulomb"],
    )
    parser.add_argument("--reference", default="Exact-Coulomb")
    parser.add_argument("--precisions", nargs="+", type=float,
                        default=[1e-4, 1e-8])
    parser.add_argument("--max-iters", type=int, default=1000)
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--top", type=int, default=10)
    parser.add_argument("--output-dir")
    parser.add_argument("--plot", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if any(value <= 0.0 for value in args.precisions):
        raise ValueError("all --precisions values must be positive")
    if args.max_iters < 1 or args.runs < 1:
        raise ValueError("--max-iters and --runs must be positive")

    input_path = Path(args.load)
    if args.output_dir:
        output_root = Path(args.output_dir)
    elif input_path.is_dir():
        output_root = input_path / "precision_profile"
    else:
        output_root = input_path.parent / f"{input_path.stem}_precision_profile"

    compare_script = Path(__file__).with_name("compare_solvers.py")
    summaries = []
    for tolerance in args.precisions:
        result_dir = output_root / tolerance_directory(tolerance)
        command = [
            sys.executable,
            str(compare_script),
            "--load", str(input_path),
            "--solvers", *args.solvers,
            "--reference", args.reference,
            "--max-iters", str(args.max_iters),
            "--max-err", f"{tolerance:.17g}",
            "--runs", str(args.runs),
            "--top", str(args.top),
            "--output-dir", str(result_dir),
        ]
        if args.max_frames is not None:
            command.extend(["--max-frames", str(args.max_frames)])
        if args.plot:
            command.append("--plot")
        print(f"\n=== tolerance={tolerance:.1e} ===", flush=True)
        subprocess.run(command, check=True)
        with (result_dir / "summary.json").open(encoding="utf-8") as stream:
            summary = json.load(stream)
        summaries.append((tolerance, summary))

    print("\nprecision profile:")
    print(f"  {'tolerance':>10} {'method':<58} {'conv%':>7} "
          f"{'mean ms':>10} {'p95 ms':>10} {'p95 error':>12}")
    for tolerance, summary in summaries:
        for _, aggregate in summary["aggregates"].items():
            rate = aggregate["convergence_rate"]
            convergence = 100.0 * float(rate) if rate is not None else None
            mean_ms = aggregate["time_ms"]["mean"]
            p95_ms = aggregate["time_ms"]["p95"]
            p95_error = aggregate["solver_error"]["p95"]
            convergence_text = ("-" if convergence is None
                                else f"{convergence:.1f}%")
            mean_text = "-" if mean_ms is None else f"{mean_ms:.4f}"
            p95_text = "-" if p95_ms is None else f"{p95_ms:.4f}"
            error_text = "-" if p95_error is None else f"{p95_error:.3e}"
            print(
                f"  {tolerance:>10.1e} {aggregate['method']:<58} "
                f"{convergence_text:>7} {mean_text:>10} {p95_text:>10} "
                f"{error_text:>12}"
            )
    print(f"results: {output_root.resolve()}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, ValueError, subprocess.CalledProcessError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
