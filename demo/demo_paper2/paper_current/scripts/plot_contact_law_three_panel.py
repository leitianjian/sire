"""Rearrange original Fig. 1 a/c/d as a single row, without rerunning data."""
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
METHOD_ALPHA = 0.72
REFERENCE_ALPHA = 0.50
SLIP_ALPHA = 0.65


def values(rows, key):
    return np.array([float(row[key]) for row in rows])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--height", type=float, default=2.05, help="Figure height in inches")
    args = parser.parse_args()
    trajectory = read_rows(args.source_dir / "panel_a_trajectory_v5.csv")
    impulse = read_rows(args.source_dir / "panel_c_impulse_refined_v6.csv")
    original_impulse = read_rows(args.source_dir / "panel_c_impulse_v5.csv")
    static = read_rows(args.source_dir / "panel_d_static_friction_v5.csv")

    with plt.style.context(["science", "ieee", "grid"]):
        mpl.rcParams.update(FIGURE_STYLE)
        figure, axes = plt.subplots(1, 3, figsize=(7.15, args.height))
        figure.subplots_adjust(left=0.065, right=0.94, bottom=0.23, top=0.90, wspace=0.50)
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
        reference_x = np.unique(np.r_[np.linspace(0, max(x) * 1.03, 300), 1.0])
        # Keep every simulation sample in the solid line; display fewer markers
        # near the densely sampled transition so the circles remain legible.
        marker_targets = np.r_[values(original_impulse, "normalized_incident_demand"),
                               0.96, 0.99, 1.02]
        impulse_markers = sorted({int(np.argmin(np.abs(x - target))) for target in marker_targets})
        axes[1].plot(reference_x, np.minimum(reference_x, 1), color=COLORS["analytical"],
                     lw=1.4, alpha=REFERENCE_ALPHA, zorder=2,
                     label="Maximum dissipation")
        axes[1].plot(x, values(impulse, "normalized_tangential_impulse"), color=BLUE,
                     marker="o", ls="-", lw=1.0, alpha=METHOD_ALPHA, zorder=3,
                     markersize=2.7, markerfacecolor="white", markevery=impulse_markers,
                     markeredgewidth=0.75, label="Proposed")
        axes[1].set_xlabel(r"Incident demand $m_T|v_T^-|/(\mu J_N)$")
        axes[1].set_ylabel(r"Tangential impulse $|J_T|/(\mu J_N)$")

        load = values(static, "load_ratio")
        limit = values(static, "friction_limit_N")
        axes[2].plot(load, values(static, "reference_tangent_force_N") / limit,
                     color=REFERENCE, lw=1.4, alpha=REFERENCE_ALPHA, zorder=2,
                     label="Numerical reference")
        axes[2].plot(load, values(static, "shifted_tangent_force_N") / limit,
                     color=BLUE, marker="o", markersize=2.7, markevery=3,
                     markerfacecolor="white", markeredgewidth=0.75,
                     ls="-", lw=1.0, alpha=METHOD_ALPHA, zorder=3, label="Proposed")
        axes[2].plot(load, values(static, "smooth_tangent_force_N") / limit,
                     color=ORANGE, ls="--", lw=1.0, alpha=METHOD_ALPHA,
                     label="Velocity-regularized")
        slip_axis = axes[2].twinx()
        scale = values(static, "smooth_velocity_m_per_s")
        slip_axis.plot(load, values(static, "shifted_tangent_speed_m_per_s") / scale,
                       color=BLUE, ls=":", lw=0.8, alpha=SLIP_ALPHA, label="Proposed")
        slip_axis.plot(load, values(static, "smooth_tangent_speed_m_per_s") / scale,
                       color=ORANGE, ls=":", lw=0.8, alpha=SLIP_ALPHA, label="Velocity-regularized")
        slip_axis.set_ylabel(r"Slip speed $|v_T|/v_s$")
        axes[2].set_xlabel(r"Applied load $F_{\rm ext}/(\mu mg)$")
        axes[2].set_ylabel(r"Opposing force $|F_T|/(\mu mg)$")
        for letter, axis in zip("abc", axes):
            panel_label(axis, letter)
            legend = axis.legend(loc={"a": "lower right", "b": "lower right", "c": "upper left"}[letter],
                        frameon=True, framealpha=0.75, fontsize=5.4, handlelength=1.4,
                        handletextpad=0.35, borderpad=0.25, labelspacing=0.20)
            if letter == "c":
                legend.set_title("Force (left axis)", prop={"size": 5.4})
            axis.grid(True, linestyle="--", color="k", alpha=0.3, lw=0.5)
            axis.spines[["top", "right"]].set_visible(False)
            axis.tick_params(which="both", top=False, right=False, direction="out")
        slip_axis.spines["top"].set_visible(False)
        slip_axis.tick_params(which="both", top=False, left=False, direction="out")
        slip_axis.grid(False)
        slip_axis.legend(loc="lower right", title="Slip speed (right axis)",
                         title_fontsize=5.4, fontsize=5.4, frameon=True, framealpha=0.75,
                         handlelength=1.8, handletextpad=0.35, borderpad=0.25,
                         labelspacing=0.20)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(args.output.with_suffix(".pdf"), bbox_inches="tight")
        figure.savefig(args.output.with_suffix(".png"), dpi=300, bbox_inches="tight")
        figure.savefig(args.output.with_suffix(".svg"), bbox_inches="tight")
        plt.close(figure)
    inputs = [args.source_dir / name for name in (
        "panel_a_trajectory_v5.csv", "panel_c_impulse_refined_v6.csv",
        "panel_c_impulse_v5.csv", "panel_d_static_friction_v5.csv")]
    args.output.with_suffix(".plot-config.json").write_text(json.dumps({
        "script": str(Path(__file__).resolve()),
        "style_reference": "plot_box_sliding_combined_paper_figure.py (Fig. 3)",
        "inputs": [{"path": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                   for path in inputs],
        "figure_inches": [7.15, args.height],
        "data_changed": True,
        "data_change": "29 new simulations near the transition; all 17 archived samples retained",
        "right_axis_line_style": "dotted",
        "curve_alpha": {"methods": METHOD_ALPHA, "references": REFERENCE_ALPHA,
                        "slip_speed": SLIP_ALPHA},
        "proposed_line_style": "solid with hollow circles in all three panels",
        "impulse_simulation_samples": len(impulse),
        "impulse_displayed_markers": len(impulse_markers),
        "impulse_samples_interpolated_or_smoothed": False,
        "impulse_line_connects_actual_samples": True,
        "panel_c_legends": {"left_axis": "Force", "right_axis": "Slip speed"},
    }, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
