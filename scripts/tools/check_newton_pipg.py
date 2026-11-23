"""Check Newton-PIPG on contact snapshots produced by the C++ solver."""

import argparse
import json
from pathlib import Path

import numpy as np
import sire


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("files", nargs="+", type=Path)
    parser.add_argument("--max-iters", type=int, default=500)
    parser.add_argument("--tol", type=float, default=1e-9)
    args = parser.parse_args()

    failed = False
    for path in args.files:
        data = json.loads(path.read_text(encoding="utf-8"))
        n = int(data["n"])
        force, residual = sire.cptContactForceNewtonPipg(
            n, data["fri_coef"], data["invM"], data["v0"],
            data["v_target"], data["b"], data["h"], args.max_iters,
            args.tol,
        )
        force = np.asarray(force, dtype=float)
        W = np.asarray(data["invM"], dtype=float).reshape(3 * n, 3 * n)
        post_velocity = (np.asarray(data["v0"], dtype=float)
                         - data["h"] * np.asarray(data["b"], dtype=float)
                         - data["h"] * W @ force)
        cone_violation = max(
            (np.linalg.norm(force[3*i:3*i+2])
             - data["fri_coef"][i] * force[3*i+2] for i in range(n)),
            default=0.0,
        )
        dae_error = np.max(np.abs(
            post_velocity[2::3] + np.asarray(data["v_target"], dtype=float)
        ))
        finite = bool(np.isfinite(force).all() and np.isfinite(residual))
        print(f"{path}: residual={residual:.3e}, cone={cone_violation:.3e}, "
              f"dae={dae_error:.3e}, finite={finite}")
        failed |= not finite or cone_violation > 1e-8 or dae_error > 1e-6
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
