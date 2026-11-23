"""Plot current Fig. 2: normal target, impact impulse, load support, and slip."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import scienceplots  # noqa: F401

from plot_high_stiffness_regression import FIGURE_STYLE, read_rows
from plot_box_sliding_combined_paper_figure import COLORS, panel_label

BLUE, ORANGE, REFERENCE = COLORS["production"], COLORS["smooth"], "#333333"
METHOD_ALPHA, REFERENCE_ALPHA = 0.72, 0.50


def values(rows, key):
    return np.array([float(row[key]) for row in rows])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--height", type=float, default=1.75, help="Figure height in inches")
    args = parser.parse_args()
    names = ["panel_a_trajectory_v5.csv", "panel_c_impulse_refined_v6.csv",
             "panel_c_impulse_v5.csv", "panel_b_velocity_regularized_v7.csv",
             "panel_d_static_friction_v5.csv"]
    trajectory, impulse, original, regularized, static = [read_rows(args.source_dir / name) for name in names]
    with plt.style.context(["science", "ieee", "grid"]):
        mpl.rcParams.update(FIGURE_STYLE)
        mpl.rcParams.update({"axes.labelsize": 6.8, "xtick.labelsize": 6.6,
                             "ytick.labelsize": 6.6})
        figure, axes = plt.subplots(1, 4, figsize=(7.15, args.height))
        figure.subplots_adjust(left=0.055, right=0.99, bottom=0.27, top=0.90,
                               wspace=0.35)
        for method, label, color, style in (
            ("Shifted", "Proposed", BLUE, "-"),
            ("Zero target", "Unshifted NCP", ORANGE, "--"),
        ):
            rows = [row for row in trajectory if row["method"] == method]
            axes[0].plot(values(rows, "time_from_contact_ms"), values(rows, "normal_velocity_m_per_s"),
                         label=label, color=color, ls=style, lw=1.0, alpha=METHOD_ALPHA,
                         marker="o" if method == "Shifted" else None,
                         markevery=max(1, len(rows) // 11), markersize=2.7,
                         markerfacecolor="white", markeredgewidth=0.75)
        axes[0].axhline(0, color="0.7", lw=0.5)
        axes[0].set_xlabel("Time from first contact [ms]")
        axes[0].set_ylabel(r"Normal velocity $v_N$ [m/s]")

        x = values(impulse, "normalized_incident_demand")
        regularized_x = values(regularized, "normalized_incident_demand")
        reference_x = np.unique(np.r_[np.linspace(0, max(max(x), max(regularized_x)) * 1.03, 300), 1.0])
        axes[1].plot(reference_x, np.minimum(reference_x, 1), color=COLORS["analytical"],
                     lw=1.4, alpha=REFERENCE_ALPHA, label="Maximum dissipation", zorder=2)
        marker_targets = np.r_[values(original, "normalized_incident_demand"), 0.96, 0.99, 1.02]
        marker_indices = sorted({int(np.argmin(np.abs(x - target))) for target in marker_targets})
        axes[1].plot(x, values(impulse, "normalized_tangential_impulse"), color=BLUE,
                     marker="o", ls="-", lw=1.0, alpha=METHOD_ALPHA, zorder=3,
                     markersize=2.7, markerfacecolor="white", markevery=marker_indices,
                     markeredgewidth=0.75, label="Proposed")
        axes[1].plot(regularized_x, values(regularized, "normalized_tangential_impulse"),
                     color=ORANGE, ls="--", lw=1.0, alpha=METHOD_ALPHA,
                     label="Velocity-regularized")
        axes[1].set_xlabel(r"Incident demand $m_T|v_T^-|/(\mu J_N)$")
        axes[1].set_ylabel(r"Tangential impulse $|J_T|/(\mu J_N)$")

        load = values(static, "load_ratio")
        limit = values(static, "friction_limit_N")
        axes[2].plot(load, values(static, "reference_tangent_force_N") / limit,
                     color=REFERENCE, lw=1.4, alpha=REFERENCE_ALPHA, label="Numerical reference")
        axes[2].plot(load, values(static, "shifted_tangent_force_N") / limit,
                     color=BLUE, marker="o", markersize=2.7, markevery=3,
                     markerfacecolor="white", markeredgewidth=0.75,
                     ls="-", lw=1.0, alpha=METHOD_ALPHA, label="Proposed", zorder=3)
        axes[2].plot(load, values(static, "smooth_tangent_force_N") / limit,
                     color=ORANGE, ls="--", lw=1.0, alpha=METHOD_ALPHA,
                     label="Velocity-regularized")
        axes[2].set_xlabel(r"Applied load $F_{\rm ext}/(\mu mg)$")
        axes[2].set_ylabel(r"Opposing force $|F_T|/(\mu mg)$")

        axes[3].plot(load, values(static, "reference_tangent_speed_m_per_s"),
                     color=REFERENCE, lw=1.4, alpha=REFERENCE_ALPHA,
                     label="Numerical reference", zorder=2)
        axes[3].plot(load, values(static, "shifted_tangent_speed_m_per_s"),
                     color=BLUE, ls="-", lw=1.0, alpha=METHOD_ALPHA,
                     marker="o", markersize=2.7, markevery=3,
                     markerfacecolor="white", markeredgewidth=0.75, label="Proposed", zorder=3)
        axes[3].plot(load, values(static, "smooth_tangent_speed_m_per_s"),
                     color=ORANGE, ls="--", lw=1.0, alpha=METHOD_ALPHA,
                     label="Velocity-regularized")
        axes[3].set_xlabel(r"Applied load $F_{\rm ext}/(\mu mg)$")
        axes[3].set_ylabel(r"Slip speed $|v_T|$ [m/s]")
        axes[3].ticklabel_format(axis="y", style="sci", scilimits=(0, 0), useMathText=True)
        for index, (letter, axis) in enumerate(zip("abcd", axes)):
            panel_label(axis, letter)
            axis.legend(loc="lower right" if index < 3 else "upper left",
                        frameon=True, framealpha=0.75, fontsize=5.0, handlelength=1.5,
                        handletextpad=0.35, borderpad=0.25, labelspacing=0.20)
            axis.grid(True, linestyle="--", color="k", alpha=0.3, lw=0.5)
            axis.spines[["top", "right"]].set_visible(False)
            axis.tick_params(which="both", top=False, right=False, direction="out")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        for extension in ("pdf", "png", "svg"):
            figure.savefig(args.output.with_suffix("." + extension), dpi=300, bbox_inches="tight")
        plt.close(figure)
    args.output.with_suffix(".plot-config.json").write_text(json.dumps({
        "script": str(Path(__file__).resolve()), "layout": "1 by 4; independent axes",
        "figure_inches": [7.15, args.height],
        "subplot_wspace": 0.35,
        "inputs": [{"path": str((args.source_dir / name).resolve()),
                    "sha256": hashlib.sha256((args.source_dir / name).read_bytes()).hexdigest()} for name in names],
        "method_alpha": METHOD_ALPHA, "reference_alpha": REFERENCE_ALPHA,
        "proposed_impulse_samples": len(impulse), "regularized_impulse_samples": len(regularized),
        "original_force_and_slip_data_unchanged": True,
        "slip_reference": "reference_tangent_speed_m_per_s",
        "slip_speed_unit": "m/s; unnormalized",
        "normalization": "Each impact solver uses its own measured normal impulse",
        "impulse_samples_interpolated_or_smoothed": False,
    }, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
