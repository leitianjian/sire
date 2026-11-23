"""Four-contact Coulomb validation on an inclined plane.

A one-kilogram box is supported by four massless corner spheres on a 15-degree
incline. After settling, the box is launched uphill at 2 m/s. Two friction
coefficients distinguish the Coulomb branches:

* mu = 0.3 > tan(15 deg): the box must stop and remain at rest;
* mu = 0.2 < tan(15 deg): the box must stop and reverse downhill.

The shifted spectral-ADMM trajectory is compared with the piecewise analytical
Coulomb solution. The script exports source data, summary metrics, and a
two-panel publication figure.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

import sire


HERE = Path(__file__).resolve().parent

# Historical optional auditor is not used by this paper workflow.
require_matplotlib_panel_alignment = None


COLORS = {0.2: "#D55E00", 0.3: "#0072B2"}

mpl.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans"],
        "mathtext.fontset": "stixsans",
        "font.size": 8.0,
        "axes.labelsize": 8.0,
        "xtick.labelsize": 7.5,
        "ytick.labelsize": 7.5,
        "legend.fontsize": 7.2,
        "axes.spines.right": False,
        "axes.spines.top": False,
        "axes.linewidth": 0.8,
        "legend.frameon": False,
        "pdf.fonttype": 42,
        "svg.fonttype": "none",
    }
)


@dataclass(frozen=True)
class Case:
    friction: float
    solver_kind: str = "shifted_coulomb"
    smooth_velocity: float = 1.0e-4
    angle_deg: float = 15.0
    mass: float = 1.0
    initial_speed: float = 2.0
    stiffness: float = 2.0e8
    damping: float = 1.5e5
    nominal_dt: float = 1.0e-3
    settle_time: float = 0.2
    measurement_time: float = 1.0
    ramp_length: float = 100.0
    gravity: float = 9.81

    @property
    def name(self) -> str:
        prefix = "smooth" if self.solver_kind == "smooth" else "shifted"
        dt_suffix = "" if math.isclose(self.nominal_dt, 1.0e-3) else f"_dt_{self.nominal_dt:g}"
        return f"{prefix}_mu_{self.friction:g}{dt_suffix}"


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = sorted({field for row in rows for field in row})
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def analytical_state(case: Case, time: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    angle = math.radians(case.angle_deg)
    deceleration = case.gravity * (
        math.sin(angle) + case.friction * math.cos(angle)
    )
    stop_time = case.initial_speed / deceleration
    stop_distance = case.initial_speed**2 / (2.0 * deceleration)

    displacement = np.empty_like(time, dtype=float)
    velocity = np.empty_like(time, dtype=float)
    uphill = time <= stop_time
    displacement[uphill] = (
        case.initial_speed * time[uphill]
        - 0.5 * deceleration * np.square(time[uphill])
    )
    velocity[uphill] = case.initial_speed - deceleration * time[uphill]

    after = ~uphill
    if case.friction >= math.tan(angle):
        displacement[after] = stop_distance
        velocity[after] = 0.0
    else:
        downhill_acceleration = case.gravity * (
            math.sin(angle) - case.friction * math.cos(angle)
        )
        elapsed = time[after] - stop_time
        displacement[after] = stop_distance - 0.5 * downhill_acceleration * np.square(elapsed)
        velocity[after] = -downhill_acceleration * elapsed
    return displacement, velocity


def analytical_metrics(case: Case) -> dict[str, float]:
    angle = math.radians(case.angle_deg)
    deceleration = case.gravity * (
        math.sin(angle) + case.friction * math.cos(angle)
    )
    return {
        "analytical_stop_time_s": case.initial_speed / deceleration,
        "analytical_stop_distance_m": case.initial_speed**2 / (2.0 * deceleration),
        "static_hold_expected": float(case.friction >= math.tan(angle)),
    }


def build_simulation(case: Case):
    os.environ.pop("SIRE_SOLVER_TRACE_DIR", None)
    sim = sire.Simulator()
    model = sim.model()
    model.addSolvers()
    loop = sim.simulationLoop()
    loop.simDuration = case.settle_time + case.measurement_time
    loop.deltaT = case.nominal_dt
    loop.ctrlT = 10
    loop.setEventHandlerMap({0: 12, 1: 13, 2: 14})

    engine = sim.physicsEngine()
    engine.collisionDetectionFlag = True
    engine.contactSolverFlag = True
    if case.solver_kind == "smooth":
        solver = engine.addAnalyticalTangentForceSolver()
    else:
        solver = engine.addShiftedSpectralADMMSolver()
    if case.solver_kind != "smooth":
        solver.setInitialPenetrationPolicy("eliminate")
        solver.setContactTimeMethod("exponential")
        solver.warmStartEnabled = True
    threshold = case.smooth_velocity if case.solver_kind == "smooth" else 1.0e-6
    material = (
        "{k:%0.17g,d:%0.17g,cr:0.0,cof:%0.17g,threshold_velocity:%0.17g}"
        % (case.stiffness, case.damping, case.friction, threshold)
    )
    solver.setDefaultProp(material)
    solver.addMaterialPair("ramp", "foot", material)

    model.setGravity([0.0, 0.0, -case.gravity, 0.0, 0.0, 0.0])
    model.ground().addMarker("ground_marker")

    angle = math.radians(case.angle_deg)
    ramp_pose = [0.0, 0.0, 0.5, 0.0, -angle, 0.0]
    box_pose = [0.0, 0.0, 0.5 + 1.0 / math.cos(angle), 0.0, -angle, 0.0]
    ramp_transform = sire.pe3132tfmatrix(ramp_pose).tolist()
    ramp = engine.addBoxGeometry(
        case.ramp_length,
        case.ramp_length,
        1.0,
        0,
        False,
        ramp_transform,
        True,
        "ramp",
    )

    box = model.addPartByPe(
        box_pose,
        "313",
        [case.mass, 0.0, 0.0, 0.0, 1.0 / 6.0, 1.0 / 6.0, 1.0 / 6.0, 0.0, 0.0, 0.0],
    )
    box.addMarker("box_center")
    model.init()

    box_geometry = engine.addBoxGeometry(1.0, 1.0, 1.0, box.id, True)
    box_geometry.cptInertial(box, case.mass)
    helper_transforms = (
        [1, 0, 0, 0.5, 0, 1, 0, 0.5, 0, 0, 1, -0.4, 0, 0, 0, 1],
        [1, 0, 0, 0.5, 0, 1, 0, -0.5, 0, 0, 1, -0.4, 0, 0, 0, 1],
        [1, 0, 0, -0.5, 0, 1, 0, 0.5, 0, 0, 1, -0.4, 0, 0, 0, 1],
        [1, 0, 0, -0.5, 0, 1, 0, -0.5, 0, 0, 1, -0.4, 0, 0, 0, 1],
    )
    helpers = [
        engine.addSphereGeometry(
            0.1,
            box.id,
            True,
            transform,
            False,
            "foot",
        )
        for transform in helper_transforms
    ]

    sim.init()
    for helper in helpers:
        engine.collisionFilter().enableCollisionPair(ramp.id, helper.id)
    engine.collisionFilter().saveMatConfig()
    return loop, box


def active_contact_count(records: dict[str, Any], index: int) -> float:
    found_field = False
    for field in ("contactPairResults", "penetrationPairs", "contactInfo"):
        frames = records.get(field, [])
        if field in records:
            found_field = True
        if index < len(frames):
            frame = frames[index]
            return len(frame) if frame is not None else 0
    return math.nan if not found_field else 0


def run_case(case: Case, output_dir: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    loop, box = build_simulation(case)
    launched = False
    launch_time = math.nan
    launch_position: np.ndarray | None = None
    angle = math.radians(case.angle_deg)
    uphill = np.array([0.0, -math.cos(angle), math.sin(angle)], dtype=float)

    while not loop.isTimeout() and not loop.isEventListEmpty():
        current_time = float(loop.simTime())
        if not launched and current_time >= case.settle_time - 1.0e-12:
            launch_time = current_time
            launch_position = np.asarray(box.pq[:3], dtype=float).copy()
            box.vs = sire.vp2vs(
                box.pq[:3], (case.initial_speed * uphill).tolist()
            )
            launched = True
        loop.integrate()
        loop.handleContact()

    if not launched or launch_position is None:
        raise RuntimeError("the uphill launch event was not reached")

    records = loop.recordsToJson()
    rows: list[dict[str, Any]] = []
    for index, absolute_time in enumerate(records["timeIndex"]):
        relative_time = float(absolute_time) - launch_time
        if relative_time < -0.5 * case.nominal_dt:
            continue
        if records["partPq"][index] is None or records["partVs"][index] is None:
            continue
        pose = records["partPq"][index][box.id]
        spatial_velocity = records["partVs"][index][box.id]
        position = np.asarray(pose[:3], dtype=float)
        point_velocity = np.asarray(
            sire.vs2vp(pose, spatial_velocity, [0.0, 0.0, 0.0]), dtype=float
        )
        rows.append(
            {
                "time_s": max(0.0, relative_time),
                "displacement_m": float(np.dot(position - launch_position, uphill)),
                "velocity_m_per_s": float(np.dot(point_velocity, uphill)),
                "angular_speed_rad_per_s": float(
                    np.linalg.norm(np.asarray(spatial_velocity[3:6], dtype=float))
                ),
                "active_contacts": active_contact_count(records, index),
            }
        )

    if len(rows) < 2:
        raise RuntimeError("insufficient post-launch samples")

    time = np.asarray([row["time_s"] for row in rows], dtype=float)
    displacement = np.asarray([row["displacement_m"] for row in rows], dtype=float)
    velocity = np.asarray([row["velocity_m_per_s"] for row in rows], dtype=float)
    analytical_displacement, analytical_velocity = analytical_state(case, time)
    for row, s_ref, v_ref in zip(rows, analytical_displacement, analytical_velocity):
        row["analytical_displacement_m"] = float(s_ref)
        row["analytical_velocity_m_per_s"] = float(v_ref)

    reference = analytical_metrics(case)
    stop_time = reference["analytical_stop_time_s"]
    stop_distance = reference["analytical_stop_distance_m"]
    turning_index = int(np.argmax(displacement))
    hold_mask = time >= min(case.measurement_time, stop_time + 0.10)
    post_stop_speed = np.abs(velocity[hold_mask])
    post_stop_mask = time >= stop_time
    post_stop_displacement = displacement[post_stop_mask] - stop_distance
    post_stop_velocity = velocity[post_stop_mask]
    post_stop_reference_velocity = analytical_velocity[post_stop_mask]
    summary = {
        **asdict(case),
        **reference,
        "samples": len(rows),
        "measured_turning_time_s": float(time[turning_index]),
        "measured_turning_distance_m": float(displacement[turning_index]),
        "stop_time_abs_error_s": abs(float(time[turning_index]) - stop_time),
        "stop_distance_abs_error_m": abs(float(displacement[turning_index]) - stop_distance),
        "displacement_rmse_m": float(
            np.sqrt(np.mean(np.square(displacement - analytical_displacement)))
        ),
        "velocity_rmse_m_per_s": float(
            np.sqrt(np.mean(np.square(velocity - analytical_velocity)))
        ),
        "post_stop_p95_abs_speed_m_per_s": (
            float(np.percentile(post_stop_speed, 95.0)) if post_stop_speed.size else math.nan
        ),
        "post_stop_displacement_abs_m": (
            float(abs(displacement[-1] - stop_distance)) if post_stop_mask.any() else math.nan
        ),
        "post_stop_mean_abs_speed_m_per_s": (
            float(np.mean(np.abs(post_stop_velocity))) if post_stop_velocity.size else math.nan
        ),
        "post_stop_velocity_rmse_m_per_s": (
            float(np.sqrt(np.mean(np.square(post_stop_velocity - post_stop_reference_velocity))))
            if post_stop_velocity.size else math.nan
        ),
        "maximum_angular_speed_rad_per_s": max(
            float(row["angular_speed_rad_per_s"]) for row in rows
        ),
        "minimum_active_contacts": (
            int(min(row["active_contacts"] for row in rows if math.isfinite(row["active_contacts"])))
            if case.solver_kind != "smooth" and any(math.isfinite(row["active_contacts"]) for row in rows) else math.nan
        ),
        "maximum_active_contacts": (
            int(max(row["active_contacts"] for row in rows if math.isfinite(row["active_contacts"])))
            if case.solver_kind != "smooth" and any(math.isfinite(row["active_contacts"]) for row in rows) else math.nan
        ),
    }
    write_csv(output_dir / f"{case.name}_trajectory.csv", rows)
    return rows, summary


def add_panel_label(axis: Any, label: str) -> None:
    axis.annotate(
        label,
        xy=(0.0, 1.0),
        xycoords="axes fraction",
        xytext=(-18, 8),
        textcoords="offset points",
        fontsize=9.0,
        fontweight="bold",
        ha="left",
        va="bottom",
        annotation_clip=False,
    )


def plot_results(
    output_dir: Path,
    cases: list[Case],
    trajectories: dict[float, list[dict[str, Any]]],
) -> Path:
    figure, axes = plt.subplots(1, 2, figsize=(7.15, 2.75), constrained_layout=True)
    for case in cases:
        rows = trajectories[case.name]
        time = np.asarray([row["time_s"] for row in rows], dtype=float)
        displacement = np.asarray([row["displacement_m"] for row in rows], dtype=float)
        velocity = np.asarray([row["velocity_m_per_s"] for row in rows], dtype=float)
        analytical_displacement, analytical_velocity = analytical_state(case, time)
        color = COLORS[case.friction]
        label = rf"$\mu={case.friction:g}$, {case.solver_kind.replace('_', ' ')}"
        axes[0].plot(
            time,
            analytical_displacement,
            color=color,
            linewidth=1.4,
            label=f"Analytical {label}",
        )
        axes[0].plot(
            time,
            displacement,
            color=color,
            linestyle="none",
            marker="o",
            markerfacecolor="white",
            markeredgewidth=0.8,
            markersize=2.6,
            markevery=25,
            label=label,
        )
        axes[1].plot(time, analytical_velocity, color=color, linewidth=1.4)
        axes[1].plot(
            time,
            velocity,
            color=color,
            linestyle="none",
            marker="o",
            markerfacecolor="white",
            markeredgewidth=0.8,
            markersize=2.6,
            markevery=25,
        )

    axes[0].set_xlabel("Time from launch [s]")
    axes[0].set_ylabel("Uphill displacement [m]")
    axes[1].set_xlabel("Time from launch [s]")
    axes[1].set_ylabel("Uphill velocity [m/s]")
    axes[1].axhline(0.0, color="#777777", linestyle=":", linewidth=0.9)
    for label, axis in zip("ab", axes):
        axis.grid(True, alpha=0.18)
        axis.set_xlim(0.0, max(case.measurement_time for case in cases))
        add_panel_label(axis, label)
    figure.legend(
        *axes[0].get_legend_handles_labels(),
        loc="upper center",
        ncol=4,
        bbox_to_anchor=(0.5, 1.04),
    )
    figure.suptitle("Four-contact box on a 15-degree incline", fontsize=7.8, y=1.11)

    base = output_dir / "box_sliding_validation"
    figure.canvas.draw()
    if require_matplotlib_panel_alignment is not None:
        require_matplotlib_panel_alignment(
            figure,
            json_out=str(base.with_suffix(".alignment-layout.json")),
            overlay_svg=str(base.with_suffix(".alignment-overlay.svg")),
            tolerance_pt=1.5,
            gutter_tolerance_pt=1.5,
            require_panel_labels=True,
            strict=True,
        )
    else:
        print(
            "warning: audit_panel_alignment is unavailable; no alignment manifest written",
            file=sys.stderr,
        )
    figure.savefig(base.with_suffix(".pdf"), bbox_inches="tight")
    figure.savefig(base.with_suffix(".svg"), bbox_inches="tight")
    figure.savefig(base.with_suffix(".png"), dpi=300, bbox_inches="tight")
    figure.savefig(base.with_suffix(".tiff"), dpi=600, bbox_inches="tight")
    plt.close(figure)
    return base


def plot_static_comparison(
    output_dir: Path,
    shifted_case: Case,
    smooth_case: Case,
    trajectories: dict[str, list[dict[str, Any]]],
) -> Path:
    """Plot the long post-stop replay only for the static branch."""
    figure, axes = plt.subplots(1, 2, figsize=(7.15, 2.75), constrained_layout=True)
    for case, color, label in (
        (shifted_case, COLORS[0.3], "Shifted Coulomb"),
        (smooth_case, "#7A7A7A", "Previous smooth law"),
    ):
        rows = trajectories[case.name]
        time = np.asarray([row["time_s"] for row in rows], dtype=float)
        displacement = np.asarray([row["displacement_m"] for row in rows], dtype=float)
        velocity = np.asarray([row["velocity_m_per_s"] for row in rows], dtype=float)
        axes[0].plot(time, displacement, color=color, lw=1.0, label=label)
        axes[1].plot(time, velocity, color=color, lw=1.0, label=label)
    axes[0].axvline(shifted_case.initial_speed / (
        shifted_case.gravity * (
            math.sin(math.radians(shifted_case.angle_deg))
            + shifted_case.friction * math.cos(math.radians(shifted_case.angle_deg))
        )
    ), color="#555555", ls=":", lw=0.8)
    axes[1].axhline(0.0, color="#777777", ls=":", lw=0.8)
    axes[0].set_xlabel("Time from launch [s]")
    axes[0].set_ylabel("Uphill displacement [m]")
    axes[1].set_xlabel("Time from launch [s]")
    axes[1].set_ylabel("Uphill velocity [m/s]")
    for label, axis in zip("ab", axes):
        axis.grid(True, alpha=0.18)
        axis.set_xlim(0.0, max(shifted_case.measurement_time, smooth_case.measurement_time))
        add_panel_label(axis, label)
    figure.legend(*axes[0].get_legend_handles_labels(), loc="upper center", ncol=2,
                  bbox_to_anchor=(0.5, 1.04))
    base = output_dir / "box_sliding_static_comparison"
    figure.savefig(base.with_suffix(".pdf"), bbox_inches="tight")
    figure.savefig(base.with_suffix(".svg"), bbox_inches="tight")
    figure.savefig(base.with_suffix(".png"), dpi=300, bbox_inches="tight")
    plt.close(figure)
    return base


def plot_static_diagnostic(
    output_dir: Path,
    cases: list[Case],
    trajectories: dict[str, list[dict[str, Any]]],
) -> Path:
    """Compare trajectories and their errors against the piecewise reference."""
    figure, axes = plt.subplots(2, 2, figsize=(7.2, 5.1), constrained_layout=True)
    colors = {"shifted_coulomb": COLORS[0.3], "smooth": "#7A7A7A"}
    stop_time = analytical_metrics(cases[0])["analytical_stop_time_s"]
    analytical_time = np.linspace(
        0.0, max(case.measurement_time for case in cases), 4000
    )
    analytical_displacement, analytical_velocity = analytical_state(
        cases[0], analytical_time
    )
    axes[0, 0].plot(
        analytical_time,
        analytical_displacement,
        color="#222222",
        lw=1.2,
        label="Piecewise analytical solution",
    )
    axes[0, 1].plot(
        analytical_time,
        analytical_velocity,
        color="#222222",
        lw=1.2,
        label="Piecewise analytical solution",
    )
    for case in cases:
        rows = trajectories[case.name]
        time = np.asarray([row["time_s"] for row in rows], dtype=float)
        displacement = np.asarray([row["displacement_m"] for row in rows], dtype=float)
        velocity = np.asarray([row["velocity_m_per_s"] for row in rows], dtype=float)
        reference_displacement, reference_velocity = analytical_state(case, time)
        if case.solver_kind != "smooth" and case.nominal_dt < 1.0e-3:
            label = rf"Fine-step shifted reference ($dt={case.nominal_dt * 1e6:.0f}\,\mu s$)"
            style = ":"
        elif case.solver_kind != "smooth":
            label = "Shifted Coulomb ($dt=1$ ms)"
            style = "-"
        else:
            label = rf"Smooth $v_s={case.smooth_velocity:.0e}$"
            style = "--"
        color = colors[case.solver_kind]
        axes[0, 0].plot(time, displacement, color=color, ls=style, lw=1.0, label=label)
        axes[0, 1].plot(time, velocity, color=color, ls=style, lw=1.0, label=label)
        axes[1, 0].semilogy(time, np.maximum(np.abs(displacement - reference_displacement), 1.0e-12), color=color, ls=style, lw=1.0, label=label)
        axes[1, 1].semilogy(time, np.maximum(np.abs(velocity - reference_velocity), 1.0e-12), color=color, ls=style, lw=1.0, label=label)
    for axis in axes.flat:
        axis.axvline(stop_time, color="#555555", ls=":", lw=0.8)
        axis.grid(True, alpha=0.18)
        axis.set_xlim(0.0, max(case.measurement_time for case in cases))
    axes[0, 0].set_ylabel("Uphill displacement [m]")
    axes[0, 1].set_ylabel("Uphill velocity [m/s]")
    axes[1, 0].set_ylabel(r"$|s-s_{\rm analytical}|$ [m]")
    axes[1, 1].set_ylabel(r"$|v-v_{\rm analytical}|$ [m/s]")
    axes[1, 0].set_xlabel("Time from launch [s]")
    axes[1, 1].set_xlabel("Time from launch [s]")
    axes[0, 0].legend(loc="upper right", fontsize=6.2)
    axes[0, 1].legend(loc="upper right", fontsize=6.2)
    for label, axis in zip("abcd", axes.flat):
        add_panel_label(axis, label)
    figure.suptitle(
        rf"Incline-friction diagnostic ($\mu={cases[0].friction:g}$); dotted line: analytical stop time",
        fontsize=8.0,
    )
    base = output_dir / "box_sliding_static_diagnostic"
    figure.savefig(base.with_suffix(".pdf"), bbox_inches="tight")
    figure.savefig(base.with_suffix(".png"), dpi=300, bbox_inches="tight")
    figure.savefig(base.with_suffix(".svg"), bbox_inches="tight")
    plt.close(figure)
    return base


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=HERE / "results" / "experiment_box_sliding",
    )
    parser.add_argument("--dt", type=float, default=1.0e-3)
    parser.add_argument("--settle-time", type=float, default=0.2)
    parser.add_argument("--measurement-time", type=float, default=1.0)
    parser.add_argument("--comparison-time", type=float, default=5.0)
    parser.add_argument("--smooth-velocity", type=float, default=1.0e-4)
    parser.add_argument("--reference-dt", type=float, default=1.0e-4)
    parser.add_argument("--friction", type=float, default=0.3)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    cases = [
        Case(args.friction, nominal_dt=args.dt, settle_time=args.settle_time,
             measurement_time=args.comparison_time),
        Case(args.friction, nominal_dt=args.reference_dt, settle_time=args.settle_time,
             measurement_time=args.comparison_time),
        Case(args.friction, solver_kind="smooth", smooth_velocity=args.smooth_velocity,
             nominal_dt=args.dt, settle_time=args.settle_time,
             measurement_time=args.comparison_time),
    ]
    trajectories: dict[str, list[dict[str, Any]]] = {}
    summaries: list[dict[str, Any]] = []
    for case in cases:
        print(f"running {case.name}")
        rows, summary = run_case(case, output_dir)
        trajectories[case.name] = rows
        summaries.append(summary)
    write_csv(output_dir / "box_sliding_summary.csv", summaries)
    with (output_dir / "configuration.json").open("w", encoding="utf-8") as stream:
        json.dump([asdict(case) for case in cases], stream, indent=2)
    diagnostic_base = plot_static_diagnostic(output_dir, cases, trajectories)
    print(json.dumps(summaries, indent=2, allow_nan=True))
    print(f"diagnostic figure: {diagnostic_base}.pdf")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
