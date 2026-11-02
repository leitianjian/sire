"""Full-platform sphere validation, with standalone panels for LaTeX subfloats."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
from pathlib import Path
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.ticker import LogLocator
from matplotlib.transforms import Bbox
import numpy as np
import scienceplots  # noqa: F401


def read(path):
    return list(csv.DictReader(path.open(newline="", encoding="utf-8")))


def val(rows, key):
    return np.asarray([float(r[key]) for r in rows])


def draw_normal_reference(ax, trajectory):
    for method, label, color, style in (
        ("Shifted", "Proposed", "#2E86AB", "-"),
        ("Zero target", "Unshifted NCP", "#F18F01", "--"),
    ):
        rows = [r for r in trajectory if r["method"] == method]
        ax.plot(
            val(rows, "time_from_contact_ms"), val(rows, "normal_velocity_m_per_s"),
            color=color, ls=style, lw=.9, alpha=.72, label=label,
            marker="o" if method == "Shifted" else "s", markevery=1,
            markersize=2.1, markerfacecolor="white", markeredgewidth=.6,
        )
    ax.axhline(0, color=".7", lw=.4)
    ax.set_xlabel("Time from first contact [ms]")
    ax.set_ylabel(r"Normal velocity $v_N$ [m/s]")


def draw_dissipation(ax, residuals):
    for method, label, color, style in (
        ("shifted", "Proposed", "#2E86AB", "-"),
        ("velocity_regularized", "Velocity-regularized", "#F18F01", "--"),
    ):
        rows = sorted(
            (r for r in residuals if r["method"] == method),
            key=lambda r: float(r["initial_tangential_speed_m_per_s"]),
        )
        ax.semilogy(
            val(rows, "initial_tangential_speed_m_per_s"),
            np.maximum(val(rows, "max_projection_residual"), 1e-12),
            color=color, ls=style, lw=.9, alpha=.72, label=label,
            marker="o" if method == "shifted" else "s", markevery=1,
            markersize=1.8, markerfacecolor="none", markeredgewidth=.5,
        )
    ax.set_xlabel(r"Initial tangential speed $v_{T,0}$ [m/s]")
    ax.set_ylabel(r"Max. tangential residual $\varepsilon_T$")
    ax.set_xticks([0., .2, .4, .6, .8])
    ax.yaxis.set_major_locator(LogLocator(base=10, numticks=6))


def style_axes(ax, panel):
    ax.legend(
        loc="lower right" if panel == "a" else "center right",
        fontsize=4.6, frameon=True, framealpha=.75, handlelength=1.35,
        handletextpad=.3, borderpad=.2, labelspacing=.2,
    )
    ax.grid(True, ls="--", color="k", alpha=.3, lw=.4)
    ax.spines[["top", "right"]].set_visible(False)
    ax.tick_params(which="both", top=False, right=False, direction="out")


def save_figure(figure, stem, bbox_inches="tight"):
    stem.parent.mkdir(parents=True, exist_ok=True)
    for extension in ("pdf", "png", "svg"):
        figure.savefig(stem.with_suffix("." + extension), dpi=400, bbox_inches=bbox_inches)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    paths = [args.source_dir / name for name in (
        "panel_a_center_velocity_audited.csv", "dissipation_summary.csv",
    )]
    trajectory, residuals = map(read, paths)
    panel_stems = {
        letter: args.output.with_name(args.output.name + "_panel_" + letter)
        for letter in "ab"
    }
    with plt.style.context(["science", "ieee", "grid"]):
        mpl.rcParams.update({
            "font.family": "serif", "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
            "font.size": 6.5, "axes.labelsize": 6.5, "xtick.labelsize": 6., "ytick.labelsize": 6.,
            "axes.linewidth": .6, "pdf.fonttype": 42, "svg.fonttype": "none",
        })
        # Keep the combined export available for earlier manuscript versions.
        figure, axes = plt.subplots(1, 2, figsize=(3.4, 1.65))
        figure.subplots_adjust(left=.11, right=.99, bottom=.27, top=.88, wspace=.48)
        draw_normal_reference(axes[0], trajectory)
        draw_dissipation(axes[1], residuals)
        for letter, ax in zip("ab", axes):
            style_axes(ax, letter)
            ax.annotate(
                letter, (0, 1), xycoords="axes fraction", xytext=(-10, 4),
                textcoords="offset points", fontsize=7, fontweight="bold",
            )
        save_figure(figure, args.output)
        plt.close(figure)

        # Independent canvases; LaTeX supplies the visible (a)/(b) below them.
        panels = {}
        for letter, drawer, rows in (
            ("a", draw_normal_reference, trajectory), ("b", draw_dissipation, residuals),
        ):
            figure, ax = plt.subplots(figsize=(1.72, 1.55))
            figure.subplots_adjust(left=.30, right=.98, bottom=.29, top=.98)
            drawer(ax, rows)
            style_axes(ax, letter)
            figure.canvas.draw()
            panels[letter] = figure
        # Use identical page bounds so widths, axes, fonts and baselines align.
        bounds = Bbox.union([
            figure.get_tightbbox(figure.canvas.get_renderer())
            for figure in panels.values()
        ]).padded(.03)
        for letter, figure in panels.items():
            save_figure(figure, panel_stems[letter], bbox_inches=bounds)
            plt.close(figure)

    config = {
        "script": str(Path(__file__).resolve()), "layout": "single column; 1 by 2",
        "inputs": [{"path": str(p.resolve()), "sha256": hashlib.sha256(p.read_bytes()).hexdigest()} for p in paths],
        "raw_data_simulated": True,
        "panel_b": "maximum per-contact-interval tangential disk-projection residual per initial speed",
        "post_velocity_definition": "same frozen rigid-sphere contact operator for both methods",
        "residual_display_floor": 1e-12,
        "center_velocity": "u + omega cross p", "no_locked_rotation": True,
        "panel_a_markers": "every recorded endpoint; Proposed hollow circles, Unshifted NCP hollow squares",
        "panel_b_markers": "every sampled initial speed; Proposed hollow circles, Velocity-regularized hollow squares; no marker subsampling",
        "no_smoothing": True, "frozen_algebraic_panels_removed": True,
        "paper_layout": "two standalone panels composed with LaTeX subfloat; labels below",
        "standalone_panels": {letter: stem.name + ".pdf" for letter, stem in panel_stems.items()},
        "standalone_canvas_inches": [1.72, 1.55],
        "standalone_pdf_bounds_inches": list(bounds.bounds),
    }
    args.output.with_suffix(".plot-config.json").write_text(json.dumps(config, indent=2), encoding="utf-8")
    for letter, stem in panel_stems.items():
        panel_config = dict(config, layout="standalone panel", panel=letter, internal_panel_label=False)
        stem.with_suffix(".plot-config.json").write_text(json.dumps(panel_config, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
