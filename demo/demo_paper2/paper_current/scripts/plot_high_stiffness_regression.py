"""Plot the high-stiffness first-bounce regression figure."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import scienceplots  # noqa: F401


COLORS = {
    "mu0": "#2E86AB",
    "mu03": "#A23B72",
    "reference": "#555555",
}

FIGURE_STYLE = {
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "STIXGeneral", "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "font.size": 7.0,
        "axes.labelsize": 7.3,
        "xtick.labelsize": 7.2,
        "ytick.labelsize": 7.2,
        "legend.fontsize": 6.2,
        "axes.linewidth": 0.7,
        "xtick.major.width": 0.6,
        "ytick.major.width": 0.6,
        "pdf.fonttype": 42,
        "svg.fonttype": "none",
        "savefig.dpi": 600,
}


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--width", type=float, default=3.5, help="Figure width in inches")
    parser.add_argument("--height", type=float, default=2.55, help="Figure height in inches")
    parser.add_argument("--stiffness", help="Optional comma-separated stiffness values to plot")
    parser.add_argument("--reference", choices=("stored", "incident", "release"), default="stored",
                        help="Normalize by the stored reference, incident-speed equivalent height, or initial release clearance")
    args = parser.parse_args()

    rows = read_rows(args.input)
    if args.stiffness:
        requested = {float(value) for value in args.stiffness.split(",")}
        rows = [row for row in rows if float(row["stiffness"]) in requested]
        for mu in (0.0, 0.3):
            if {float(row["stiffness"]) for row in rows if float(row["friction"]) == mu} != requested:
                raise ValueError(f"missing requested stiffness data for mu={mu}")
    if args.reference == "incident":
        for row in rows:
            row["reference_bounce_height"] = row["incident_reference_bounce_height"]
            row["first_bounce_height_ratio"] = row["first_bounce_height_ratio_incident"]
    elif args.reference == "release":
        for row in rows:
            height = float(row["initial_gap"])
            if height <= 0:
                raise ValueError("Initial release clearance must be positive")
            row["reference_bounce_height"] = row["initial_gap"]
            row["first_bounce_height_ratio"] = str(float(row["first_bounce_height"]) / height)
    groups = {0.0: [], 0.3: []}
    for row in rows:
        groups[float(row["friction"])].append(row)

    with plt.style.context(["science", "ieee", "grid"]):
        mpl.rcParams.update(FIGURE_STYLE)
        figure, axis = plt.subplots(
            1, 1, figsize=(args.width, args.height), constrained_layout=True
        )
        for friction, color, marker, linestyle, label in (
            (0.0, COLORS["mu0"], "o", "-", r"$\mu=0$"),
            (0.3, COLORS["mu03"], "s", "--", r"$\mu=0.3$"),
        ):
            selected = sorted(groups[friction], key=lambda row: float(row["stiffness"]))
            stiffness = np.asarray([float(row["stiffness"]) for row in selected])
            ratio = np.asarray(
                [float(row["first_bounce_height_ratio"]) for row in selected]
            )
            axis.semilogx(
                stiffness,
                ratio,
                color=color,
                linestyle=linestyle,
                linewidth=1.0,
                marker=marker,
                markersize=3.0,
                markerfacecolor="white",
                markeredgewidth=0.75,
                label=label,
                zorder=3,
            )

        axis.axhline(
            1.0,
            color=COLORS["reference"],
            linestyle=":",
            linewidth=0.8,
            zorder=1,
        )
        axis.set_xlabel(r"Contact stiffness $k$ [N/m]")
        axis.set_ylabel(r"First-bounce height ratio $z_1/z_0$" if args.reference == "release"
                        else r"First-bounce height ratio $z_1/z_{\rm ref}$")
        axis.legend(loc="upper right", ncol=2, framealpha=0.85)
        axis.grid(True, linestyle="--", color="k", alpha=0.3, linewidth=0.5)
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)
        axis.tick_params(which="both", top=False, right=False, direction="out")

        args.output.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(args.output.with_suffix(".pdf"), bbox_inches="tight")
        figure.savefig(args.output.with_suffix(".svg"), bbox_inches="tight")
        figure.savefig(args.output.with_suffix(".png"), dpi=300, bbox_inches="tight")
        figure.savefig(args.output.with_suffix(".tiff"), dpi=600, bbox_inches="tight")
        plt.close(figure)
    with args.output.with_suffix(".source-data.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    args.output.with_suffix(".plot-config.json").write_text(json.dumps({
        "input": str(args.input.resolve()), "reference": args.reference,
        "stiffness": sorted({float(row["stiffness"]) for row in rows}),
        "cases": len(rows),
        "figure_inches": [args.width, args.height],
        "normalization": "first-bounce bottom clearance / initial bottom clearance" if args.reference == "release" else args.reference,
        "reference_heights_m": sorted({float(row["reference_bounce_height"]) for row in rows}),
        "max_height_error_percent": max(abs(float(row["first_bounce_height_ratio"]) - 1) * 100 for row in rows),
    }, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
