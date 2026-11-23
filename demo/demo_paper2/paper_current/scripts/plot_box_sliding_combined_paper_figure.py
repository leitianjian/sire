"""Build the two-row incline comparison used as the manuscript figure."""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
import scienceplots

ANGLE_RAD = math.radians(15.0)
GRAVITY = 9.81
INITIAL_SPEED = 2.0
COLORS = {
    "analytical": "#C73E1D",
    "production": "#2E86AB",
    "reference": "#A23B72",
    "smooth": "#F18F01",
}


mpl.rcParams.update(
    {
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "STIXGeneral", "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "font.size": 7.0,
        "axes.labelsize": 7.3,
        "xtick.labelsize": 7.2,
        "ytick.labelsize": 7.2,
        "legend.fontsize": 5.4,
        "axes.linewidth": 0.7,
        "xtick.major.width": 0.6,
        "ytick.major.width": 0.6,
        "legend.frameon": True,
        "legend.framealpha": 0.9,
        "legend.fancybox": True,
        "pdf.fonttype": 42,
        "svg.fonttype": "none",
        "savefig.dpi": 600,
    }
)


@dataclass(frozen=True)
class RowSpec:
    mu: float
    duration: float
    data_dir: Path
    reference_file: str
    reference_step_label: str


def read_trajectory(path: Path) -> dict[str, np.ndarray]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    return {
        "time": np.asarray([float(row["time_s"]) for row in rows]),
        "displacement": np.asarray([float(row["displacement_m"]) for row in rows]),
        "velocity": np.asarray([float(row["velocity_m_per_s"]) for row in rows]),
    }


def analytical_state(mu: float, time: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    deceleration = GRAVITY * (math.sin(ANGLE_RAD) + mu * math.cos(ANGLE_RAD))
    stop_time = INITIAL_SPEED / deceleration
    stop_distance = INITIAL_SPEED**2 / (2.0 * deceleration)
    displacement = np.empty_like(time)
    velocity = np.empty_like(time)
    uphill = time <= stop_time
    displacement[uphill] = (
        INITIAL_SPEED * time[uphill] - 0.5 * deceleration * time[uphill] ** 2
    )
    velocity[uphill] = INITIAL_SPEED - deceleration * time[uphill]
    if mu >= math.tan(ANGLE_RAD):
        displacement[~uphill] = stop_distance
        velocity[~uphill] = 0.0
    else:
        downhill_acceleration = GRAVITY * (
            math.sin(ANGLE_RAD) - mu * math.cos(ANGLE_RAD)
        )
        elapsed = time[~uphill] - stop_time
        displacement[~uphill] = stop_distance - 0.5 * downhill_acceleration * elapsed**2
        velocity[~uphill] = -downhill_acceleration * elapsed
    return displacement, velocity


def panel_label(axis: mpl.axes.Axes, label: str) -> None:
    axis.annotate(
        label,
        xy=(0.0, 1.0),
        xycoords="axes fraction",
        xytext=(-14, 5),
        textcoords="offset points",
        fontsize=7.2,
        fontweight="bold",
        ha="left",
        va="bottom",
        annotation_clip=False,
    )


def plot_row(axes: np.ndarray, spec: RowSpec, labels: str) -> None:
    production = read_trajectory(
        spec.data_dir / f"shifted_mu_{spec.mu:g}_trajectory.csv"
    )
    smooth = read_trajectory(spec.data_dir / f"smooth_mu_{spec.mu:g}_trajectory.csv")
    analytical_time = np.linspace(0.0, spec.duration, 5000)
    analytical_displacement, analytical_velocity = analytical_state(
        spec.mu, analytical_time
    )

    axes[0].plot(
        analytical_time,
        analytical_displacement,
        color=COLORS["analytical"],
        lw=1.25,
        alpha=0.72,
        label="Analytical",
        zorder=2,
    )
    axes[1].plot(
        analytical_time,
        analytical_velocity,
        color=COLORS["analytical"],
        lw=1.25,
        alpha=0.72,
        label="Analytical",
        zorder=2,
    )

    series = (
        (
            production,
            COLORS["production"],
            "-",
            "o",
            "Proposed",
        ),
        (
            smooth,
            COLORS["smooth"],
            "--",
            None,
            "Velocity-regularized",
        ),
    )

    with plt.style.context(["science", "ieee", "grid"]):
        for values, color, style, marker, legend_label in series:
            time = values["time"]
            displacement = values["displacement"]
            velocity = values["velocity"]
            valid = time <= spec.duration
            time = time[valid]
            displacement = displacement[valid]
            velocity = velocity[valid]
            ref_displacement, ref_velocity = analytical_state(spec.mu, time)
            markevery = max(1, time.size // 11) if marker else None
            line_options = {
                "color": color,
                "ls": style,
                "lw": 1.0,
                "marker": marker,
                "markevery": markevery,
                "markersize": 2.7,
                "markerfacecolor": "white",
                "markeredgewidth": 0.75,
                "label": legend_label,
                "zorder": 4 if marker else 3,
            }
            axes[0].plot(time, displacement, alpha=0.72, **line_options)
            axes[1].plot(time, velocity, alpha=0.72, **line_options)
            axes[2].plot(time, np.abs(displacement - ref_displacement), **line_options)
            axes[3].semilogy(
                time,
                np.maximum(np.abs(velocity - ref_velocity), 1.0e-12),
                **line_options,
            )
            # axes[3].semilogy(
            #     time,
            #     np.maximum(np.abs(velocity - ref_velocity), 1.0e-12),
            #     color=color,
            #     ls=style,
            #     lw=1.0,
            #     marker=marker,
            #     markevery=markevery,
            #     markersize=2.7,
            #     markerfacecolor="white",
            #     markeredgewidth=0.75,
            # )
            # axes[2].semilogy(
            #     time,
            #     np.maximum(np.abs(displacement - ref_displacement), 1.0e-12),
            #     color=color,
            #     ls=style,
            #     lw=1.0,
            #     marker=marker,
            #     markevery=markevery,
            #     markersize=2.7,
            #     markerfacecolor="white",
            #     markeredgewidth=0.75,
            # )
            # axes[3].semilogy(
            #     time,
            #     np.maximum(np.abs(velocity - ref_velocity), 1.0e-12),
            #     color=color,
            #     ls=style,
            #     lw=1.0,
            #     marker=marker,
            #     markevery=markevery,
            #     markersize=2.7,
            #     markerfacecolor="white",
            #     markeredgewidth=0.75,
            # )

        stop_time = INITIAL_SPEED / (
            GRAVITY * (math.sin(ANGLE_RAD) + spec.mu * math.cos(ANGLE_RAD))
        )
        for label, axis in zip(labels, axes):
            axis.axvline(stop_time, color="#555555", ls=":", lw=0.65, zorder=1)
            axis.grid(True, linestyle="--", color="k", alpha=0.3, lw=0.5)
            axis.spines["top"].set_visible(False)
            axis.spines["right"].set_visible(False)
            panel_label(axis, label)
            axis.text(
                0.04, 1.1, rf"$\mu={spec.mu:g}$",
                transform=axis.transAxes,
                fontsize=6.4,
                ha="left", va="top",
            )
            # axis.set_xlabel("Time [s]")

        axes[0].set_ylabel("Uphill displacement [m]")
        axes[1].set_ylabel("Uphill velocity [m/s]")
        axes[2].set_ylabel("Position error [m]")
        axes[3].set_ylabel("Velocity error [m/s]")
        legend_locs = {"b": "lower right", "g": "lower right", "h": "center right"}
        for label, axis in zip(labels, axes):
            axis.legend(
                loc=legend_locs.get(
                    label, "upper left" if axis is axes[2] or axis is axes[3] else "upper right"
                ),
                fontsize=4.5,
                handlelength=1.4,
                handletextpad=0.35,
                borderpad=0.25,
                labelspacing=0.2,
                framealpha=0.75,
                bbox_to_anchor=(0.0, 0.86) if label in "ef" else None,
            )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sliding-dir", type=Path, required=True)
    parser.add_argument("--static-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    specs = (
        RowSpec(
            mu=0.2,
            duration=2.0,
            data_dir=args.sliding_dir,
            reference_file="shifted_mu_0.2_dt_0.0001_trajectory.csv",
            reference_step_label="h=100 μs",
        ),
        RowSpec(
            mu=0.3,
            duration=2.0,
            data_dir=args.static_dir,
            reference_file="shifted_mu_0.3_dt_0.0001_trajectory.csv",
            reference_step_label="h=100 μs",
        ),
    )

    width_mm = 183.0
    figure, axes = plt.subplots(
        2, 4, figsize=(width_mm / 25.4, 2.5),
        sharex="col", constrained_layout=False
    )
    figure.subplots_adjust(
        left=0.058, right=0.994, bottom=0.13, top=0.965, wspace=0.42, hspace=0.25
    )
    # Group panels by function, alternating the two friction coefficients:
    # displacement (.2, .3), velocity (.2, .3), position error (.2, .3),
    # and velocity error (.2, .3).
    mu02_axes = np.concatenate((axes[0, 0:1], axes[0, 2:3], axes[1, 0:1], axes[1, 2:3]))
    mu03_axes = np.concatenate((axes[0, 1:2], axes[0, 3:4], axes[1, 1:2], axes[1, 3:4]))
    plot_row(mu02_axes, specs[0], "aceg")
    plot_row(mu03_axes, specs[1], "bdfh")
    # One y-axis label per function pair; retain the right-panel tick values.
    for axis in (axes[0, 1], axes[0, 3], axes[1, 1], axes[1, 3]):
        axis.set_ylabel("")
    for axis in axes[0]:
        axis.tick_params(axis="x", labelbottom=False)

    # Label each bottom panel, leaving space below its tick labels.
    for axis in axes[1]:
        axis.set_xlabel("Time [s]", fontsize=7.3, labelpad=5)
    # legend_handles = [
    #     Line2D([0], [0], color=COLORS["analytical"], lw=1.25),
    #     Line2D(
    #         [0], [0], color=COLORS["production"], lw=1.0, marker="o",
    #         markersize=2.7, markerfacecolor="white", markeredgewidth=0.75,
    #     ),
    #     Line2D([0], [0], color=COLORS["reference"], lw=1.0, ls=":"),
    #     Line2D([0], [0], color=COLORS["smooth"], lw=1.0, ls="--"),
    # ]
    # figure.legend(
    #     legend_handles,
    #     [
    #         "Piecewise analytical",
    #         "Shifted Coulomb, h=1 ms",
    #         "Fine-step reference",
    #         "Previous smooth law, vₛ=10⁻⁴ m/s",
    #     ],
    #     loc="upper right",
    #     bbox_to_anchor=(0.995, 0.56),
    #     ncol=4,
    #     handlelength=1.8,
    #     columnspacing=0.9,
    #     fontsize=6.2,
    #     frameon=True,
    #     framealpha=0.9,
    #     fancybox=True,
    # )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.canvas.draw()
    figure.savefig(args.output.with_suffix(".pdf"), bbox_inches="tight")
    figure.savefig(args.output.with_suffix(".svg"), bbox_inches="tight")
    figure.savefig(args.output.with_suffix(".png"), dpi=300, bbox_inches="tight")
    figure.savefig(args.output.with_suffix(".tiff"), dpi=600, bbox_inches="tight")
    plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
