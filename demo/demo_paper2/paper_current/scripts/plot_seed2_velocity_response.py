"""Plot forward and turning onboard responses without regenerating training figures."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import scienceplots  # noqa: F401


ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--height", type=float, default=2.5, help="Figure height in inches")
    parser.add_argument("--width", type=float, default=3.3, help="Figure width in inches")
    parser.add_argument("--input", type=Path, default=ROOT / "analysis/seed2_velocity_comparison/seed2_velocity_comparison_full.csv")
    parser.add_argument("--output", type=Path, default=ROOT / "res/seed2_forward_turn_response")
    args = parser.parse_args()
    with args.input.open(encoding="utf-8", newline="") as stream:
        rows = [r for r in csv.DictReader(stream) if 10 <= float(r["time_s"]) <= 40]
    assert len(rows) == 1500
    time = np.array([float(r["time_s"]) for r in rows])
    assert np.all(np.diff(time) > 0)
    specs = [
        ("a", "Forward velocity\n" + r"$v_x$ (m/s)", "command_forward_m_s",
         "onboard_base_velocity_body_forward_m_s_display_5sample_mean", "Kalman estimate", "#2E86AB"),
        ("b", r"Body-$z$ angular velocity" + "\n" + r"$\omega_z$ (rad/s)", "command_yaw_rate_rad_s",
         "onboard_imu_yaw_rate_rad_s_display_5sample_mean", "IMU body-z rate", "#F18F01"),
    ]
    with plt.style.context(["science", "ieee"]):
        mpl.rcParams.update({"pdf.fonttype": 42, "svg.fonttype": "none",
                             "font.size": 7.0, "axes.labelsize": 7.0,
                             "xtick.labelsize": 6.5, "ytick.labelsize": 6.5,
                             "legend.fontsize": 6.0})
        fig, axes = plt.subplots(2, 1, figsize=(args.width, args.height), sharex=True)
        fig.subplots_adjust(left=.22, right=.975, bottom=.12, top=.98, hspace=0.0)
        for ax, (letter, ylabel, cmd_col, resp_col, resp_label, color) in zip(axes, specs):
            target = np.array([float(r[cmd_col]) for r in rows])
            response = np.array([float(r[resp_col]) for r in rows])
            assert np.all(np.isfinite(target)) and np.all(np.isfinite(response))
            ax.plot(time, target, color="black", ls="--", lw=.75, label="Target", zorder=3)
            ax.plot(time, response, color=color, ls="-", lw=.8, label=resp_label, zorder=2)
            ax.axhline(0, color="0.65", lw=.5, zorder=0)
            for timestamp in np.arange(12.5, 40, 2.5):
                ax.axvline(timestamp, color="0.75", ls=":", lw=.45, zorder=0)
            ax.set_xlim(10, 40)
            ax.set_ylim(-1.65, 1.65)
            ax.set_yticks([-1, 0, 1])
            ax.set_xticks(np.arange(10, 41, 5))
            ax.set_ylabel(ylabel)
            ax.legend(loc="upper right", ncol=2 if letter == "a" else 1,
                      columnspacing=.8, frameon=True, framealpha=.9,
                      facecolor="white", edgecolor="0.7", fancybox=True,
                      fontsize=6.0, borderaxespad=.25, handlelength=1.4,
                      labelspacing=.20, handletextpad=.40, borderpad=.25)
            ax.annotate(letter, xy=(0, .5), xycoords="axes fraction", xytext=(-44, 0),
                        textcoords="offset points", fontsize=7.2, fontweight="bold",
                        ha="center", va="center", annotation_clip=False)
        for ax in axes[:-1]:
            ax.tick_params(axis="x", which="both", top=False, bottom=False, labelbottom=False)
        axes[-1].tick_params(axis="x", which="both", top=False)
        axes[-1].set_xlabel("Time since first policy cycle (s)")
        for extension, dpi in (("pdf", 600), ("svg", 600), ("png", 300), ("tiff", 600)):
            fig.savefig(args.output.with_suffix("." + extension), dpi=dpi,
                        facecolor="white", bbox_inches="tight")
        plt.close(fig)
    args.output.with_suffix(".plot-config.json").write_text(json.dumps({
        "script": str(Path(__file__).resolve()),
        "input": str(args.input.resolve()),
        "input_sha256": hashlib.sha256(args.input.read_bytes()).hexdigest(),
        "figure_inches": [args.width, args.height], "subplot_hspace": 0.0,
        "font_pt": {"axes_labels": 7.0, "ticks": 6.5, "legend": 6.0, "panel_letters": 7.2},
        "legend_columns": {"a": 2, "b": 1},
        "panels": {"a": "forward velocity", "b": "body-z angular velocity"},
        "omitted_panel": "lateral velocity; raw source columns retained",
        "selected_cycles": len(rows), "time_window_s": [10, 40],
        "response_columns": [spec[3] for spec in specs],
        "display_mean": "Existing centered five-sample display mean; unchanged",
        "data_changed": False, "units_preserved": True,
    }, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
