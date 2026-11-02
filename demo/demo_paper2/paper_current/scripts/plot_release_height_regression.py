"""Compare the new 4 m release regression with the previous paper's archived data."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import scienceplots  # noqa: F401

from plot_high_stiffness_regression import COLORS, FIGURE_STYLE, read_rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--previous-data", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    current = read_rows(args.input)
    archived = read_rows(args.previous_data)
    old = sorted((r for r in archived if float(r["stiffness"]) <= 1e38),
                 key=lambda r: float(r["stiffness"]))
    old_k = np.array([float(r["stiffness"]) for r in old])
    # The archived column stores height minus the 4 m release height.
    old_h = 4 + np.array([float(r["height"]) for r in old])
    groups = {
        mu: sorted((r for r in current if float(r["friction"]) == mu),
                   key=lambda r: float(r["stiffness"]))
        for mu in (0.0, 0.3)
    }
    assert len(groups[0.0]) == len(groups[0.3])
    assert all(float(r["reference_bounce_height"]) == 4.0 for r in current)
    pair_diff = max(abs(float(a["first_bounce_height"]) - float(b["first_bounce_height"]))
                    for a, b in zip(groups[0.0], groups[0.3]))
    high = [r for r in current if float(r["stiffness"]) >= 1e7]
    failed = [r for r in groups[0.0] if abs(float(r["first_bounce_height_ratio"]) - 1) > 0.01]
    validation = {
        "cases": len(current), "height_reference_m": 4.0,
        "max_mu_pair_height_difference_m": pair_diff,
        "max_tangential_impulse_Ns": max(float(r["tangent_impulse_magnitude"]) for r in current),
        "max_sampled_high_stiffness_height_error_percent": max(
            100 * abs(float(r["first_bounce_height_ratio"]) - 1) for r in high),
        "stiffness_values_outside_1_percent": [float(r["stiffness"]) for r in failed],
        "all_stiffness_regression_within_1_percent": not failed,
        "historical_data_are_archived_not_a_fresh_old_solver_run": True,
    }
    with plt.style.context(["science", "ieee", "grid"]):
        mpl.rcParams.update(FIGURE_STYLE)
        figure, axes = plt.subplots(1, 2, figsize=(7.15, 2.8), constrained_layout=True)
        for index, axis in enumerate(axes):
            selection = old_k >= 1e7 if index else np.ones(old_k.size, dtype=bool)
            axis.semilogx(old_k[selection], old_h[selection], color="#777777",
                         linestyle=":", linewidth=0.9, label="Previous paper (archived)")
            for mu, color, marker, style in (
                (0.0, COLORS["mu0"], "o", "-"),
                (0.3, COLORS["mu03"], "s", "--"),
            ):
                selected = [r for r in groups[mu] if not index or float(r["stiffness"]) >= 1e7]
                axis.semilogx(
                    [float(r["stiffness"]) for r in selected],
                    [float(r["first_bounce_height"]) for r in selected],
                    color=color, marker=marker, linestyle=style, linewidth=1.0,
                    markersize=3, markerfacecolor="white", markeredgewidth=0.7,
                    label=rf"Current solver, $\mu={mu:g}$",
                )
            axis.axhline(4, color="#555555", linestyle="-.", linewidth=0.6)
            axis.set_xlabel(r"Contact stiffness $k$ [N/m]")
            axis.set_ylabel("First-bounce height [m]")
            axis.set_title("(a) All sampled stiffness values" if not index
                           else "(b) High-stiffness subset", fontsize=7.3)
            axis.grid(True, linestyle="--", color="k", alpha=0.3, linewidth=0.5)
            axis.spines[["top", "right"]].set_visible(False)
            axis.tick_params(which="both", top=False, right=False, direction="out")
        axes[0].legend(loc="lower right", fontsize=5.4)
        figure.suptitle(r"4 m release from rest; $r=0.5$ m, $m=1$ kg, $d=0$, $h=1$ ms",
                         fontsize=8)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(args.output.with_suffix(".pdf"), bbox_inches="tight")
        figure.savefig(args.output.with_suffix(".png"), dpi=240, bbox_inches="tight")
        figure.savefig(args.output.with_suffix(".svg"), bbox_inches="tight")
        plt.close(figure)
    args.output.with_suffix(".validation.json").write_text(json.dumps(validation, indent=2), encoding="utf-8")
    print(json.dumps(validation, indent=2))


if __name__ == "__main__":
    main()
