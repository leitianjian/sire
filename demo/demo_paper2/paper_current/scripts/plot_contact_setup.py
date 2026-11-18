"""Vector schematic of the archived nominal oblique sphere--plane test."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch
import numpy as np
import scienceplots  # noqa: F401

ROOT = Path(__file__).resolve().parents[1]


def arrow(axis, start, end, color="black", size=10, lw=1.1, style="-|>"):
    axis.add_patch(FancyArrowPatch(start, end, arrowstyle=style,
                                  mutation_scale=size, lw=lw, color=color,
                                  shrinkA=0, shrinkB=0, zorder=5))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--parameters", type=Path,
                        default=ROOT / "data/contact_law/runs/nominal/shifted/summary.json")
    args = parser.parse_args()
    parameters = json.loads(args.parameters.read_text(encoding="utf-8"))
    for key, expected in {"mass": 1, "radius": .1, "initial_gap": .01,
                          "normal_speed": .5, "tangential_speed": .5,
                          "friction": .1}.items():
        if not math.isclose(parameters[key], expected):
            raise ValueError(f"Review schematic for changed {key}: {parameters[key]}")
    # Normalize coordinates by radius; gap/r=.1 preserves the physical geometry.
    center = np.array([0., 1 + parameters["initial_gap"] / parameters["radius"]])
    green, red, blue = "#227000", "#E51C23", "#0088CC"
    with plt.style.context(["science", "ieee"]):
        # Labels remain readable when the scene is used at half-column width.
        mpl.rcParams.update({"font.size": 16, "pdf.fonttype": 42, "svg.fonttype": "none"})
        fig, ax = plt.subplots(figsize=(3.8, 2.25))
        fig.subplots_adjust(left=.02, right=.98, bottom=.03, top=.97)
        ax.set_aspect("equal")
        ax.set_xlim(-2.65, 3.0)
        ax.set_ylim(-.43, 2.4)
        ax.axis("off")
        # Horizontal fixed plane and short hatching, as in the existing schematics.
        ax.plot([-2.5, 2.8], [0, 0], color="black", ls="-", lw=1.15, zorder=4)
        for x in np.arange(-2.5, 2.81, .19):
            ax.plot([x, x-.13], [0, -.14], color="black", ls="-", lw=.6)
        ax.add_patch(Circle(center, 1, facecolor="#D9DDE0", edgecolor="black", lw=1.15))
        ax.plot(*center, marker="o", ms=2.2, color="black", zorder=6)
        ax.text(-.4, .82, r"$m$", fontsize=16)
        radial_end = center + np.array([-.52, .85])
        arrow(ax, center, radial_end, green, size=9)
        ax.text(-.53, 1.62, r"$r$", color=green, ha="right")
        # Both arrows represent initial velocity, not contact forces.
        arrow(ax, center, (2.12, center[1]))
        ax.text(1.37, center[1]+.16, r"$v_{T,0}$", ha="center")
        arrow(ax, center, (center[0], .35))
        ax.text(center[0]+.15, .66, r"$v_{N,0}$", va="center")
        # Show the actual initial bottom clearance with extension lines and ticks.
        gap = center[1]-1
        ax.plot([-1.53, .0], [gap, gap], color=green, lw=.75, ls=":")
        ax.plot([-1.47, -1.47], [0, gap], color=green, lw=.9)
        for y in (0, gap):
            ax.plot([-1.56, -1.38], [y, y], color=green, lw=.9)
        ax.annotate(r"$z_0$", xy=(-1.47, gap/2), xytext=(-1.67, .43),
                    color=green, ha="right", va="center",
                    arrowprops={"arrowstyle": "-", "color": green, "lw": .85})
        ax.text(2.35, .12, r"$\mu$", color="black", ha="center")
        # Global coordinates: x horizontal, z upward; these are separate from velocities.
        origin = (-2.25, 0)
        arrow(ax, origin, (-1.72, 0), red, size=8, lw=.9,
              style="-|>,head_length=0.75,head_width=0.20")
        arrow(ax, origin, (-2.25, .62), blue, size=8, lw=.9,
              style="-|>,head_length=0.75,head_width=0.20")
        ax.text(-1.64, -.04, r"$x$", color=red, va="top")
        ax.text(-2.35, .65, r"$z$", color=blue, ha="right")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        for suffix in (".pdf", ".svg", ".png"):
            fig.savefig(args.output.with_suffix(suffix), bbox_inches="tight",
                        pad_inches=.015, dpi=300)
        plt.close(fig)
    args.output.with_suffix(".config.json").write_text(json.dumps({
        "source_parameters": str(args.parameters.resolve()),
        "source_sha256": hashlib.sha256(args.parameters.read_bytes()).hexdigest(),
        "physical_parameters": {key: parameters[key] for key in
                                ("mass", "radius", "initial_gap", "normal_speed",
                                 "tangential_speed", "friction", "stiffness", "damping")},
        "shown_state": "Initial state before contact, at t=0",
        "initial_center_world_m": [0.0, 0.0, parameters["radius"] + parameters["initial_gap"]],
        "coordinate_axes": "Direction indicator displaced to the left for readability; not the numerical world origin",
        "velocity_labels": {"v_T,0": "Initial +x component, +0.5 m/s",
                            "v_N,0": "Initial +z component, -0.5 m/s"},
        "velocity_arrow_origin": "Both initial translational-velocity components start at the sphere center",
        "geometry_to_scale": True,
        "gap_symbol": "z_0 is geometric bottom clearance, not effective penetration",
        "force_arrows": "None; no contact force exists in the depicted initial state",
        "figure_inches": [3.8, 2.25],
        "label_font_pt": 16,
    }, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
