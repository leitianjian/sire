"""Show the horizontal-speed regression and the first-flight height difference."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import scienceplots  # noqa: F401

from plot_high_stiffness_regression import COLORS, FIGURE_STYLE, read_rows


def first_flight(rows: list[dict[str, str]]) -> tuple[int, int]:
    start = next(i for i, row in enumerate(rows) if int(row["contact"]))
    end = next(i for i in range(start + 1, len(rows)) if not int(rows[i]["contact"]))
    next_contact = next(
        (i for i in range(end, len(rows)) if int(rows[i]["contact"])), len(rows)
    )
    return start, next_contact


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rerun-dir", type=Path, required=True)
    parser.add_argument("--sweep", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    sweep = read_rows(args.sweep)
    rerun = read_rows(args.rerun_dir / "oblique/high_stiffness_regression.csv")
    for row in rerun:
        old = next(r for r in sweep if float(r["friction"]) == float(row["friction"])
                   and float(r["stiffness"]) == float(row["stiffness"]))
        assert np.isclose(float(old["first_bounce_height"]),
                          float(row["first_bounce_height"]), rtol=1e-12, atol=1e-14)

    series = []
    for mu in (0.0, 0.3):
        path = args.rerun_dir / f"oblique/runs/mu_{mu:g}_k_1e30/shifted/trajectory.csv"
        rows = read_rows(path)
        start, stop = first_flight(rows)
        selected = rows[:stop]
        time = np.array([float(r["time"]) for r in selected])
        height = np.array([float(r["z"]) - 0.1 for r in selected])
        peak = max(range(start + 1, stop), key=lambda i: float(rows[i]["z"]))
        series.append((time, height, float(rows[start]["time"]), peak))
    t0, z0, contact_time, _ = series[0]
    t1, z1, _, _ = series[1]
    common = t0[(t0 >= contact_time) & (t0 <= min(t0[-1], t1[-1]))]
    delta_um = 1e6 * (np.interp(common, t1, z1) - np.interp(common, t0, z0))
    peak_gap_um = (max(z1) - max(z0)) * 1e6

    with plt.style.context(["science", "ieee", "grid"]):
        mpl.rcParams.update(FIGURE_STYLE)
        figure, axes = plt.subplots(1, 3, figsize=(7.8, 2.65), constrained_layout=True)
        for mu, color, marker, style in (
            (0.0, COLORS["mu0"], "o", "-"),
            (0.3, COLORS["mu03"], "s", "--"),
        ):
            selected = sorted((r for r in sweep if float(r["friction"]) == mu),
                              key=lambda r: float(r["stiffness"]))
            assert all(float(r["tangential_speed"]) == 0.5 for r in selected)
            axes[0].semilogx(
                [float(r["stiffness"]) for r in selected],
                [float(r["first_bounce_height_ratio"]) for r in selected],
                color=color, marker=marker, linestyle=style, linewidth=1.0,
                markersize=3, markerfacecolor="white", label=rf"$\mu={mu:g}$",
            )
            time, height, _, peak = series[0 if mu == 0 else 1]
            keep = time <= min(t0[-1], t1[-1])
            axes[1].plot(time[keep], 1e3 * height[keep], color=color, linestyle=style,
                         linewidth=1.0, label=rf"$\mu={mu:g}$")
            axes[1].plot(time[peak], 1e3 * height[peak], marker=marker,
                         color=color, markersize=3, markerfacecolor="white")
        axes[0].axhline(1, color=COLORS["reference"], linestyle=":", linewidth=0.7)
        axes[0].set_xlabel(r"Stiffness $k$ [N/m]")
        axes[0].set_ylabel(r"Height ratio $h_1/h_{\rm ref}$")
        axes[0].set_title("(a) Horizontal-speed regression", fontsize=7.2)
        axes[0].legend(loc="upper right")
        axes[1].set_xlabel("Time [s]")
        axes[1].set_ylabel("Bottom clearance [mm]")
        axes[1].set_title(r"(b) First bounce, $k=10^{30}$ N/m", fontsize=7.2)
        axes[1].legend(loc="upper right")
        axes[1].text(0.50, 0.07, rf"Peak-height difference: {peak_gap_um:.2f} $\mu$m",
                     transform=axes[1].transAxes, ha="center", fontsize=6.0)
        axes[2].plot(common, delta_um, color=COLORS["mu03"], linewidth=1.0)
        axes[2].axhline(0, color=COLORS["reference"], linestyle=":", linewidth=0.7)
        axes[2].set_xlabel("Time [s]")
        axes[2].set_ylabel(r"$z_{\mu=0.3}-z_{\mu=0}$ [$\mu$m]")
        axes[2].set_title("(c) Height difference during first flight", fontsize=7.2)
        figure.suptitle(
            r"$v_{x,0}=0.5$ m/s, $v_{z,0}=-0.5$ m/s, $h_{\rm nom}=1$ ms",
            fontsize=8,
        )
        for axis in axes:
            axis.grid(True, linestyle="--", color="k", alpha=0.3, linewidth=0.5)
            axis.spines[["top", "right"]].set_visible(False)
            axis.tick_params(which="both", top=False, right=False, direction="out")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(args.output.with_suffix(".pdf"), bbox_inches="tight")
        figure.savefig(args.output.with_suffix(".png"), dpi=240, bbox_inches="tight")
        figure.savefig(args.output.with_suffix(".svg"), bbox_inches="tight")
        plt.close(figure)
    with args.output.with_suffix(".csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_s", "height_mu0_m", "height_mu03_m", "difference_um"])
        writer.writerows(zip(common, np.interp(common, t0, z0),
                             np.interp(common, t1, z1), delta_um))
    print(f"Peak-height difference: {peak_gap_um:.8f} micrometers")


if __name__ == "__main__":
    main()
