"""Experiment 1: velocity-shifted Coulomb contact validation.

The experiment deliberately separates two questions:

1. Oblique-impact trajectories compare the shifted law with an otherwise
   identical unshifted De Saxce spectral-ADMM ablation.
2. Step and slip sweeps test explicit separation and maximum dissipation.
3. A one-contact load sweep tests static friction and smooth-law creep.
4. Frozen systems are replayed through the production and reference solvers.

Run this script with the Sire virtual environment; see README.md for examples.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import subprocess
import sys
from dataclasses import asdict, dataclass, replace
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

import sire


REPO_ROOT = Path(__file__).resolve().parents[4]
COMPARE_SOLVERS = Path(__file__).resolve().parents[1] / "tools/compare_solvers.py"
DEFAULT_OUTPUT_ROOT = Path(__file__).resolve().parent / "results"

# Historical optional auditor is not used by this paper workflow.
require_matplotlib_panel_alignment = None


COLORS = {
    "shifted": "#0072B2",
    "zero_target": "#D55E00",
    "reference": "#202020",
    "smooth": "#7A7A7A",
    "stress": "#CC79A7",
}

mpl.rcParams.update(
    {
        "font.family": "Times New Roman",
        "font.serif": ["Times New Roman", "Times", "STIXGeneral", "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "font.size": 8.0,
        "axes.labelsize": 8.0,
        "axes.titlesize": 8.2,
        "xtick.labelsize": 7.5,
        "ytick.labelsize": 7.5,
        "legend.fontsize": 7.4,
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
    name: str
    normal_speed: float = 0.5
    tangential_speed: float = 0.25
    stiffness: float = 2.0e5
    damping_ratio: float = 0.25
    friction: float = 0.3
    nominal_dt: float = 1.0e-3
    mass: float = 1.0
    radius: float = 0.10
    initial_gap: float = 0.01
    duration: float = 0.12

    @property
    def damping(self) -> float:
        return 2.0 * self.damping_ratio * math.sqrt(self.stiffness * self.mass)


def unique_cases(cases: Iterable[Case]) -> list[Case]:
    result: list[Case] = []
    seen: set[tuple[float, ...]] = set()
    for case in cases:
        key = (
            case.normal_speed,
            case.tangential_speed,
            case.stiffness,
            case.damping_ratio,
            case.friction,
            case.nominal_dt,
        )
        if key not in seen:
            result.append(case)
            seen.add(key)
    return result


def make_cases(preset: str) -> list[Case]:
    nominal = Case("nominal")
    if preset == "smoke":
        return [nominal]
    if preset == "convergence":
        return [
            replace(
                nominal,
                name=f"normal_dt_{value * 1.0e6:.0f}us",
                tangential_speed=0.0,
                friction=0.0,
                nominal_dt=value,
            )
            for value in (1.25e-4, 2.5e-4, 5.0e-4, 1.0e-3, 2.0e-3)
        ]
    if preset == "paper":
        nominal = replace(
            nominal,
            tangential_speed=0.5,
            friction=0.1,
        )
        cases = [nominal]
        cases.extend(
            replace(
                nominal,
                name=f"step_{value * 1.0e6:.0f}us",
                nominal_dt=value,
            )
            for value in (1.25e-4, 2.5e-4, 5.0e-4, 2.0e-3)
        )
        cases.extend(
            replace(
                nominal,
                name=f"slip_{value:.3f}",
                tangential_speed=float(value),
            )
            for value in np.linspace(0.0, 0.8, 17)
            if not math.isclose(float(value), nominal.tangential_speed)
        )
        return cases

    cases: list[Case] = [nominal]
    for value in (0.2, 0.5, 1.0):
        cases.append(replace(nominal, name=f"vn_{value:g}", normal_speed=value))
    for value in (0.0, 0.25, 0.5):
        cases.append(
            replace(nominal, name=f"vt_{value:g}", tangential_speed=value)
        )
    for value in (5.0e4, 2.0e5, 8.0e5):
        cases.append(replace(nominal, name=f"k_{value:.0e}", stiffness=value))
    for value in (0.1, 0.25, 0.6):
        cases.append(
            replace(nominal, name=f"zeta_{value:g}", damping_ratio=value)
        )
    for value in (0.0, 0.3, 0.7):
        cases.append(replace(nominal, name=f"mu_{value:g}", friction=value))
    for value in (5.0e-4, 1.0e-3, 2.0e-3):
        cases.append(replace(nominal, name=f"dt_{value:.0e}", nominal_dt=value))
    return unique_cases(cases)


def latest_trajectory(trace_root: Path) -> Path | None:
    trajectories = sorted(
        (path for path in trace_root.glob("trajectory_*") if path.is_dir()),
        key=lambda path: path.stat().st_mtime_ns,
    )
    return trajectories[-1] if trajectories else None


def build_simulation(
    case: Case,
    variant: str,
    trace_root: Path,
    initial_penetration_policy: str = "eliminate",
):
    trace_root.mkdir(parents=True, exist_ok=True)
    os.environ["SIRE_SOLVER_TRACE_DIR"] = str(trace_root)

    sim = sire.Simulator()
    model = sim.model()
    model.addSolvers()
    loop = sim.simulationLoop()
    loop.simDuration = case.duration
    loop.deltaT = case.nominal_dt
    loop.ctrlT = 10
    loop.setEventHandlerMap({0: 12, 1: 13, 2: 14})

    engine = sim.physicsEngine()
    engine.collisionDetectionFlag = True
    engine.contactSolverFlag = True
    if variant == "shifted":
        solver = engine.addShiftedSpectralADMMSolver()
    elif variant == "unshifted":
        solver = engine.addSpectralADMMSolver()
        # Make the contact-time/depth pipeline identical; only the normal
        # target shift is removed by the solver implementation.
        solver.setContactModelMode("single_point")
    elif variant == "velocity_regularized":
        solver = engine.addAnalyticalTangentForceSolver()
    else:
        raise ValueError(f"unknown variant: {variant}")

    # Use the same contact-onset state in both variants so the trajectory
    # ablation isolates only the normal velocity shift.
    if variant != "velocity_regularized":
        solver.setInitialPenetrationPolicy(initial_penetration_policy)
        solver.setContactTimeMethod("exponential")
        solver.warmStartEnabled = True

    threshold = 1.0e-4 if variant == "velocity_regularized" else 1.0e-6
    solver.setDefaultProp(
        "{k:%0.17g,d:%0.17g,cr:0.0,cof:%0.17g,threshold_velocity:%0.17g}"
        % (case.stiffness, case.damping, case.friction, threshold)
    )
    solver.addMaterialPair(
        "ground",
        "ball",
        "{k:%0.17g,d:%0.17g,cr:0.0,cof:%0.17g,threshold_velocity:%0.17g}"
        % (case.stiffness, case.damping, case.friction, threshold),
    )

    model.setGravity([0, 0, -9.81, 0, 0, 0])
    model.ground().addMarker("ground_marker")

    # A shallow fixed box acts as an infinite-enough horizontal plane.
    ground_geometry = engine.addBoxGeometry(
        20.0,
        20.0,
        0.10,
        0,
        False,
        [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -0.05, 0, 0, 0, 1],
        True,
        "ground",
    )

    pose = [0.0, 0.0, case.radius + case.initial_gap, 0.0, 0.0, 0.0]
    ball = model.addPartByPe(
        pose,
        "313",
        [case.mass, 0, 0, 0, 1.0e-3, 1.0e-3, 1.0e-3, 0, 0, 0],
    )
    ball.addMarker("ball_center")
    model.init()

    ball_geometry = engine.addSphereGeometry(
        case.radius,
        ball.id,
        True,
        [],
        True,
        "ball",
    )
    ball_geometry.cptInertial(ball, case.mass)
    ball.vs = sire.vp2vs(
        pose[:3], [case.tangential_speed, 0.0, -case.normal_speed]
    )

    sim.init()
    engine.collisionFilter().enableCollisionPair(
        ground_geometry.id, ball_geometry.id
    )
    engine.collisionFilter().saveMatConfig()
    return sim, loop, ball


def contact_values(contact: dict[str, Any] | None) -> tuple[float, ...]:
    if not contact:
        return (math.nan,) * 7
    force = np.asarray(contact.get("contactForce", [0.0, 0.0, 0.0]), dtype=float)
    pair = contact.get("point_pair", {})
    return (
        float(pair.get("depth", math.nan)),
        float(pair.get("modifiedDepth", math.nan)),
        float(np.linalg.norm(force[:2])),
        float(abs(force[2])),
        float(contact.get("separation_speed", math.nan)),
        float(contact.get("slip_speed", math.nan)),
        float(np.linalg.norm(force)),
    )


def extract_time_series(records: dict[str, Any], ball_id: int) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    count = len(records.get("timeIndex", []))
    for index in range(count):
        pq = records["partPq"][index][ball_id]
        vs = records["partVs"][index][ball_id]
        velocity = np.asarray(sire.vs2vp(pq, vs, [0, 0, 0]), dtype=float)
        contacts = records.get("contactInfo", [])[index]
        contact = contacts[0] if contacts else None
        depth, modified_depth, ft, fn, target_vn, slip, force_norm = (
            contact_values(contact)
        )
        penetration_frames = records.get("penetrationPairs", [])
        penetrations = penetration_frames[index] if index < len(penetration_frames) else []
        force_frames = records.get("contactPairResults", [])
        pair_forces = force_frames[index] if index < len(force_frames) else []
        if contact is None and penetrations:
            pair = penetrations[0]
            depth = float(pair.get("depth", math.nan))
            modified_depth = float(pair.get("modifiedDepth", math.nan))
        if contact is None and pair_forces:
            force = np.asarray(pair_forces[0].get("force_W", [0.0] * 3), dtype=float)
            ft = float(np.linalg.norm(force[:2]))
            fn = float(abs(force[2]))
            force_norm = float(np.linalg.norm(force))
        has_contact = contact is not None or bool(penetrations) or bool(pair_forces)
        rows.append(
            {
                "index": index,
                "time": float(records["timeIndex"][index]),
                "dt": float(records["dts"][index]),
                "x": float(pq[0]),
                "y": float(pq[1]),
                "z": float(pq[2]),
                "vx": float(velocity[0]),
                "vy": float(velocity[1]),
                "vz": float(velocity[2]),
                "contact": int(has_contact),
                "depth": depth,
                "modified_depth": modified_depth,
                "ft": ft,
                "fn": fn,
                "force_norm": force_norm,
                "solver_target_vn": target_vn,
                "trace_initial_vn": math.nan,
                "solver_error": math.nan,
                "slip_speed": slip,
            }
        )
    return rows


def read_solver_trace(trace_dir: Path | None) -> list[dict[str, Any]]:
    if trace_dir is None:
        return []
    result: list[dict[str, Any]] = []
    for path in sorted(trace_dir.glob("frame_*.json")):
        with path.open("r", encoding="utf-8") as stream:
            frame = json.load(stream)
        if int(frame.get("n", 0)) != 1:
            continue
        force = np.asarray(frame.get("cResult", [0.0, 0.0, 0.0]), dtype=float)
        velocity = np.asarray(frame.get("v0", [math.nan] * 3), dtype=float)
        target = np.asarray(frame.get("v_target", [math.nan]), dtype=float)
        result.append(
            {
                "time": float(frame["sim_time"]),
                "h": float(frame["h"]),
                "ft": float(np.linalg.norm(force[:2])),
                "fn": float(abs(force[2])),
                "force_norm": float(np.linalg.norm(force)),
                # The public v_target convention enters g_N with a plus sign,
                # so the requested physical normal velocity is -v_target.
                "solver_target_vn": float(-target[0]),
                "trace_initial_vn": float(velocity[2]),
                "solver_error": float(frame.get("solver_error", math.nan)),
            }
        )
    return result


def merge_trace_into_time_series(
    rows: list[dict[str, Any]], trace_rows: list[dict[str, Any]], radius: float
) -> None:
    by_time: dict[float, list[int]] = {}
    for index, row in enumerate(rows):
        by_time.setdefault(round(float(row["time"]), 12), []).append(index)
    for trace in trace_rows:
        candidates = by_time.get(round(trace["time"], 12), [])
        if not candidates:
            # Recorder and trace both use the simulation clock, but retain a
            # defensive nearest-time fallback for different JSON precision.
            index = min(
                range(len(rows)),
                key=lambda candidate: abs(rows[candidate]["time"] - trace["time"]),
            )
            if abs(rows[index]["time"] - trace["time"]) > 1.0e-9:
                continue
        else:
            index = candidates[0]
        row = rows[index]
        row["contact"] = 1
        if not math.isfinite(row["depth"]):
            row["depth"] = max(0.0, radius - row["z"])
        if not math.isfinite(row["modified_depth"]):
            row["modified_depth"] = min(row["depth"], 1.0e-4)
        for field in (
            "ft",
            "fn",
            "force_norm",
            "solver_target_vn",
            "trace_initial_vn",
            "solver_error",
        ):
            row[field] = trace[field]


def first_contact_episode(rows: list[dict[str, Any]]) -> tuple[int, int] | None:
    start = next((i for i, row in enumerate(rows) if row["contact"]), None)
    if start is None:
        return None
    end = start
    while end + 1 < len(rows) and rows[end + 1]["contact"]:
        end += 1
    return start, end


def affine_kelvin_voigt_state(
    t: float, mass: float, stiffness: float, damping: float,
    depth0: float, depth_rate0: float, normal_gravity: float = 9.81,
) -> np.ndarray:
    matrix = np.array(
        [[0.0, 1.0], [-stiffness / mass, -damping / mass]], dtype=float
    )
    affine = np.array([0.0, normal_gravity], dtype=float)
    equilibrium = -np.linalg.solve(matrix, affine)
    values, vectors = np.linalg.eig(matrix)
    exponential = vectors @ np.diag(np.exp(values * t)) @ np.linalg.inv(vectors)
    state = equilibrium + exponential @ (
        np.array([depth0, depth_rate0], dtype=float) - equilibrium
    )
    return np.real_if_close(state, tol=1000).astype(float)


def analytical_separation(
    case: Case, depth0: float, depth_rate0: float
) -> tuple[float, float, float, float, float]:
    """Return the unilateral Kelvin--Voigt geometric-separation state.

    The linear Kelvin--Voigt ODE is valid only while its unprojected normal
    force is positive.  On unloading, that force reaches zero at a positive
    penetration.  From that instant onward the unilateral force is clamped to
    zero, so the body follows free normal motion until penetration vanishes.
    """
    # First find the positive-to-nonpositive force crossing on unloading.
    natural_period = 2.0 * math.pi * math.sqrt(case.mass / case.stiffness)
    # The old fixed 0.1-s/5000 scan has a 20-us spacing and can miss the
    # first force/separation root when k is very large.  Use a stiffness-aware
    # bracket grid, while retaining a generous horizon for damped cases.
    horizon = max(10.0 * natural_period, 100.0 * math.sqrt(case.mass / case.stiffness))
    if case.damping <= 1.0e-14:
        omega = math.sqrt(case.stiffness / case.mass)
        # d(t) = d0 cos(omega*t) + (d_dot0/omega) sin(omega*t).
        # The first positive unloading zero is the first positive root after
        # the initial compression branch.
        phase = math.atan2(-depth0, depth_rate0 / max(omega, 1.0e-30))
        while phase <= 0.0:
            phase += math.pi
        horizon = max(2.0 * phase / omega, 4.0 * math.pi / omega)
    bracket: tuple[float, float] | None = None
    if case.damping <= 1.0e-14:
        root = phase / omega
        bracket = (max(1.0e-15, 0.5 * root), 1.5 * root)
    else:
        samples = np.linspace(1.0e-12, horizon, 20001)
        previous_t = float(samples[0])
        previous_state = affine_kelvin_voigt_state(
            previous_t,
            case.mass,
            case.stiffness,
            case.damping,
            depth0,
            depth_rate0,
        )
        previous_force = float(
            case.stiffness * previous_state[0] + case.damping * previous_state[1]
        )
        seen_positive = previous_force > 0.0
        for current_t in samples[1:]:
            current_state = affine_kelvin_voigt_state(
                float(current_t),
                case.mass,
                case.stiffness,
                case.damping,
                depth0,
                depth_rate0,
            )
            current_force = float(
                case.stiffness * current_state[0] + case.damping * current_state[1]
            )
            seen_positive = seen_positive or current_force > 0.0
            if (
                seen_positive
                and previous_force > 0.0
                and current_force <= 0.0
                and current_state[1] < 0.0
            ):
                bracket = (previous_t, float(current_t))
                break
            previous_t = float(current_t)
            previous_force = current_force
    if bracket is None:
        return (math.nan,) * 5

    lower, upper = bracket
    for _ in range(80):
        middle = 0.5 * (lower + upper)
        state = affine_kelvin_voigt_state(
            middle,
            case.mass,
            case.stiffness,
            case.damping,
            depth0,
            depth_rate0,
        )
        force = case.stiffness * state[0] + case.damping * state[1]
        if force > 0.0:
            lower = middle
        else:
            upper = middle
    force_release_time = 0.5 * (lower + upper)
    release_state = affine_kelvin_voigt_state(
        force_release_time,
        case.mass,
        case.stiffness,
        case.damping,
        depth0,
        depth_rate0,
    )
    release_depth = max(0.0, float(release_state[0]))
    release_depth_rate = float(release_state[1])

    # After force release, solve delta_f + delta_dot_f*t + 0.5*g*t^2 = 0.
    normal_gravity = 9.81
    if release_depth <= 1.0e-15:
        free_flight_time = 0.0
    elif abs(normal_gravity) <= 1.0e-15:
        if release_depth_rate >= 0.0:
            return (math.nan,) * 5
        free_flight_time = -release_depth / release_depth_rate
    else:
        discriminant = release_depth_rate**2 - 2.0 * normal_gravity * release_depth
        if discriminant < 0.0:
            return (math.nan,) * 5
        free_flight_time = (
            -release_depth_rate - math.sqrt(max(0.0, discriminant))
        ) / normal_gravity
        if free_flight_time < 0.0:
            return (math.nan,) * 5

    geometric_separation_time = force_release_time + free_flight_time
    terminal_depth_rate = release_depth_rate + normal_gravity * free_flight_time
    # World normal velocity is the negative of penetration rate.
    return (
        geometric_separation_time,
        -terminal_depth_rate,
        force_release_time,
        release_depth,
        -release_depth_rate,
    )


def summarize(
    case: Case, variant: str, rows: list[dict[str, Any]], trace_dir: Path | None
) -> dict[str, Any]:
    summary: dict[str, Any] = {"case": case.name, "variant": variant, **asdict(case)}
    summary["damping"] = case.damping
    summary["trace_dir"] = str(trace_dir) if trace_dir else ""
    episode = first_contact_episode(rows)
    if episode is None:
        summary.update({"contact_found": 0, "failure": "no contact detected"})
        return summary

    start, end = episode
    separation_found = end + 1 < len(rows) and not rows[end + 1]["contact"]
    prior = max(0, start - 1)
    depth0 = rows[start]["modified_depth"]
    if not math.isfinite(depth0):
        depth0 = max(0.0, rows[start]["depth"])
    depth_rate0 = max(0.0, -rows[start]["trace_initial_vn"])
    if not math.isfinite(depth_rate0):
        depth_rate0 = max(0.0, -rows[prior]["vz"])
    (
        analytical_tc,
        analytical_terminal_vn,
        analytical_force_release_time,
        analytical_force_release_depth,
        analytical_force_release_vn,
    ) = analytical_separation(case, depth0, depth_rate0)
    # Summing accepted physical intervals is less ambiguous than subtracting
    # recorder timestamps when an event splits a nominal integration step.
    measured_tc = sum(rows[i]["dt"] for i in range(start, end + 1))
    terminal_index = min(end + 1, len(rows) - 1)
    end_normal_velocity = rows[terminal_index]["vz"]
    terminal_vn = end_normal_velocity if separation_found else math.nan
    normal_impulse = sum(rows[i]["fn"] * rows[i]["dt"] for i in range(start, end + 1))
    tangent_impulse = sum(rows[i]["ft"] * rows[i]["dt"] for i in range(start, end + 1))
    tangential_displacement = rows[terminal_index]["x"] - rows[start]["x"]
    summary.update(
        {
            "contact_found": 1,
            "contact_start_time": rows[start]["time"],
            "contact_end_time": rows[end]["time"],
            "contact_intervals": end - start + 1,
            "separation_found": int(separation_found),
            "contact_time_is_lower_bound": int(not separation_found),
            "measured_contact_time": measured_tc,
            "analytical_contact_time": analytical_tc,
            "contact_time_abs_error": (
                abs(measured_tc - analytical_tc) if separation_found else math.nan
            ),
            "contact_time_relative_error": (
                abs(measured_tc - analytical_tc) / max(abs(analytical_tc), 1.0e-15)
                if separation_found
                else math.nan
            ),
            "terminal_normal_velocity": terminal_vn,
            "end_normal_velocity": end_normal_velocity,
            "analytical_terminal_normal_velocity": analytical_terminal_vn,
            "analytical_force_release_time": analytical_force_release_time,
            "analytical_force_release_depth": analytical_force_release_depth,
            "analytical_force_release_normal_velocity": analytical_force_release_vn,
            "terminal_velocity_abs_error": (
                abs(terminal_vn - analytical_terminal_vn)
                if separation_found
                else math.nan
            ),
            "terminal_velocity_relative_error": (
                abs(terminal_vn - analytical_terminal_vn)
                / max(abs(analytical_terminal_vn), 1.0e-15)
                if separation_found
                else math.nan
            ),
            "last_solver_target_vn": rows[end]["solver_target_vn"],
            "max_depth": max(rows[i]["depth"] for i in range(start, end + 1)),
            "normal_impulse": normal_impulse,
            "tangent_impulse_magnitude": tangent_impulse,
            "tangential_displacement": tangential_displacement,
            "terminal_tangential_speed": math.hypot(
                rows[terminal_index]["vx"], rows[terminal_index]["vy"]
            ),
        }
    )
    return summary


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = sorted({key for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def run_static_friction_sweep(
    output_dir: Path,
    *,
    mass: float = 1.0,
    friction: float = 0.1,
    step: float = 1.0e-3,
    gravity: float = 9.81,
    smooth_velocity: float = 1.0e-4,
) -> list[dict[str, Any]]:
    """Solve a locked-orientation, single-contact tangential load sweep.

    The contact-space operator is that of a point mass with horizontal and
    vertical translation.  It is equivalent to a body whose orientation and
    out-of-plane motion are constrained, without introducing a multipoint box
    contact or sphere rolling into the static-friction measurement.
    """
    inverse_mass = (-np.eye(3, dtype=float) / mass).ravel().tolist()
    normal_load = mass * gravity
    friction_limit = friction * normal_load
    rows: list[dict[str, Any]] = []
    for load_ratio in np.linspace(0.0, 1.5, 61):
        applied_force = float(load_ratio * friction_limit)
        free_velocity = [step * applied_force / mass, 0.0, -step * gravity]
        common = (
            1,
            [friction],
            inverse_mass,
            free_velocity,
            [0.0],
            [0.0, 0.0, 0.0],
            step,
            200,
        )
        shifted_raw, shifted_error = sire.cptContactForceShiftedSpectralAdmm(
            *common, 1.0e-8
        )
        reference_raw, reference_error = sire.cptContactForceExactCoulomb(
            *common, 1.0e-10
        )
        shifted_force = np.asarray(shifted_raw, dtype=float)
        reference_force = np.asarray(reference_raw, dtype=float)
        contact_operator = step * np.eye(3, dtype=float) / mass
        shifted_velocity = contact_operator @ shifted_force + np.asarray(
            free_velocity, dtype=float
        )
        reference_velocity = contact_operator @ reference_force + np.asarray(
            free_velocity, dtype=float
        )

        # Previous smooth law: |f_T|/limit = min(|v_T|/v_s, 1).
        compliance = step / mass
        regularized_gain = friction_limit / smooth_velocity
        smooth_speed_linear = (
            compliance * applied_force
            / (1.0 + compliance * regularized_gain)
        )
        if smooth_speed_linear <= smooth_velocity:
            smooth_speed = smooth_speed_linear
            smooth_force = regularized_gain * smooth_speed
        else:
            smooth_force = friction_limit
            smooth_speed = compliance * (applied_force - smooth_force)

        rows.append(
            {
                "load_ratio": float(load_ratio),
                "applied_force_N": applied_force,
                "friction_limit_N": friction_limit,
                "shifted_tangent_force_N": abs(float(shifted_force[0])),
                "reference_tangent_force_N": abs(float(reference_force[0])),
                "smooth_tangent_force_N": smooth_force,
                "shifted_tangent_speed_m_per_s": abs(float(shifted_velocity[0])),
                "reference_tangent_speed_m_per_s": abs(float(reference_velocity[0])),
                "smooth_tangent_speed_m_per_s": abs(float(smooth_speed)),
                "shifted_normal_force_N": float(shifted_force[2]),
                "reference_normal_force_N": float(reference_force[2]),
                "shifted_solver_error": float(shifted_error),
                "reference_solver_error": float(reference_error),
                "smooth_velocity_m_per_s": smooth_velocity,
            }
        )
    write_csv(output_dir / "static_friction_sweep.csv", rows)
    return rows


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


def save_publication_figure(
    figure: Any,
    base_path: Path,
    *,
    exclude_axes: list[Any] | None = None,
) -> None:
    base_path.parent.mkdir(parents=True, exist_ok=True)
    figure.canvas.draw()
    if require_matplotlib_panel_alignment is None:
        print(
            "warning: audit_panel_alignment is unavailable; "
            "figure exported without a formal alignment manifest",
            file=sys.stderr,
        )
    else:
        require_matplotlib_panel_alignment(
            figure,
            json_out=str(base_path.with_suffix(".alignment-layout.json")),
            overlay_svg=str(base_path.with_suffix(".alignment-overlay.svg")),
            tolerance_pt=1.5,
            gutter_tolerance_pt=1.5,
            require_panel_labels=True,
            exclude_axes=exclude_axes or [],
            strict=True,
        )
    figure.savefig(base_path.with_suffix(".pdf"), bbox_inches="tight")
    figure.savefig(base_path.with_suffix(".svg"), bbox_inches="tight")
    figure.savefig(base_path.with_suffix(".png"), dpi=300, bbox_inches="tight")
    figure.savefig(base_path.with_suffix(".tiff"), dpi=600, bbox_inches="tight")


def plot_paper_figure(
    output_dir: Path,
    trajectories: dict[tuple[str, str], list[dict[str, Any]]],
    summaries: list[dict[str, Any]],
    static_rows: list[dict[str, Any]],
) -> Path:
    """Create the four-panel contact-law evidence figure used by the paper."""
    figure, axes = plt.subplots(
        2,
        2,
        figsize=(7.15, 5.15),
        constrained_layout=True,
    )
    axis_a, axis_b, axis_c, axis_d = axes.ravel()

    shifted = trajectories[("nominal", "shifted")]
    unshifted = trajectories[("nominal", "unshifted")]
    episode = first_contact_episode(shifted)
    if episode is None:
        raise RuntimeError("nominal shifted trajectory contains no contact")
    start, end = episode
    contact_start = float(shifted[start]["time"])
    contact_end = float(shifted[end]["time"] + shifted[end]["dt"])
    upper = contact_start + 0.014
    panel_a_rows: list[dict[str, Any]] = []
    for label, rows, color, style in (
        ("Shifted", shifted, COLORS["shifted"], "-"),
        ("Unshifted De Saxce NCP", unshifted, COLORS["zero_target"], "--"),
    ):
        selected = [
            row
            for row in rows
            if contact_start - 2.0e-3 <= float(row["time"]) <= upper
        ]
        relative_time = np.asarray(
            [(float(row["time"]) - contact_start) * 1.0e3 for row in selected]
        )
        normal_velocity = np.asarray([float(row["vz"]) for row in selected])
        axis_a.plot(
            relative_time,
            normal_velocity,
            style,
            color=color,
            linewidth=1.35,
            label=label,
        )
        for time_value, velocity_value in zip(relative_time, normal_velocity):
            panel_a_rows.append(
                {
                    "method": label,
                    "time_from_contact_ms": float(time_value),
                    "normal_velocity_m_per_s": float(velocity_value),
                }
            )
    targets = [
        row
        for row in shifted
        if contact_start <= float(row["time"]) <= contact_end
        and math.isfinite(float(row["solver_target_vn"]))
    ]
    if targets:
        axis_a.plot(
            [(float(row["time"]) - contact_start) * 1.0e3 for row in targets],
            [float(row["solver_target_vn"]) for row in targets],
            ":",
            color=COLORS["reference"],
            linewidth=1.0,
            label=r"Trajectory target $\bar v_N$",
        )
    axis_a.axhline(0.0, color="#B0B0B0", linewidth=0.7)
    axis_a.set_xlabel("Time from first contact [ms]")
    axis_a.set_ylabel(r"Normal velocity $v_N$ [m/s]")
    axis_a.legend(
        loc="lower center",
        bbox_to_anchor=(0.5, 1.01),
        ncol=3,
        borderaxespad=0.0,
        handlelength=2.2,
        columnspacing=1.0,
    )
    write_csv(output_dir / "paper_figure" / "panel_a_trajectory.csv", panel_a_rows)

    convergence = sorted(
        (
            row
            for row in summaries
            if row.get("variant") == "shifted"
            and (row.get("case") == "nominal" or str(row.get("case", "")).startswith("step_"))
            and row.get("separation_found")
        ),
        key=lambda row: float(row["nominal_dt"]),
    )
    if len(convergence) < 3:
        raise RuntimeError("paper preset did not produce the step sweep")
    eta = np.asarray(
        [
            float(row["nominal_dt"])
            * math.sqrt(float(row["stiffness"]) / float(row["mass"]))
            for row in convergence
        ]
    )
    time_error = 100.0 * np.asarray(
        [float(row["contact_time_relative_error"]) for row in convergence]
    )
    velocity_error = 100.0 * np.asarray(
        [
            float(row["terminal_velocity_abs_error"])
            / max(float(row["normal_speed"]), 1.0e-15)
            for row in convergence
        ]
    )
    under_resolved = np.asarray(
        [int(row["contact_intervals"]) < 8 for row in convergence], dtype=bool
    )
    resolved = ~under_resolved
    # Keep the under-resolved stress point visible, but do not connect it to
    # the resolved branch. The nonmonotone endpoint error is a sampling effect,
    # not a convergence trend to be interpolated by the line.
    axis_b.loglog(
        eta[resolved],
        time_error[resolved],
        "o-",
        color=COLORS["shifted"],
        markersize=3.5,
        linewidth=1.2,
        label=r"Separation time $e_t$",
    )
    axis_b.loglog(
        eta[resolved],
        velocity_error[resolved],
        "s-",
        color=COLORS["zero_target"],
        markersize=3.2,
        linewidth=1.2,
        label=r"Terminal velocity $e_v$",
    )
    if np.any(under_resolved):
        axis_b.scatter(
            eta[under_resolved],
            time_error[under_resolved],
            facecolors="none",
            edgecolors=COLORS["stress"],
            s=36,
            linewidths=1.0,
            label="Under-resolved",
            zorder=4,
        )
        axis_b.scatter(
            eta[under_resolved],
            velocity_error[under_resolved],
            facecolors="none",
            edgecolors=COLORS["stress"],
            s=36,
            linewidths=1.0,
            zorder=4,
        )
    axis_b.set_xlabel(r"Normalized step $\Delta t\sqrt{k/m}$")
    axis_b.set_ylabel("Relative error [%]")
    axis_b.legend(
        loc="lower center",
        bbox_to_anchor=(0.5, 1.01),
        ncol=2,
        borderaxespad=0.0,
        handlelength=2.0,
        columnspacing=1.0,
    )
    write_csv(
        output_dir / "paper_figure" / "panel_b_convergence.csv",
        [
            {
                "case": row["case"],
                "normalized_step": float(x),
                "separation_time_error_percent": float(et),
                "terminal_velocity_error_percent": float(ev),
                "contact_intervals": int(row["contact_intervals"]),
            }
            for row, x, et, ev in zip(convergence, eta, time_error, velocity_error)
        ],
    )

    slip = sorted(
        (
            row
            for row in summaries
            if row.get("variant") == "shifted"
            and (row.get("case") == "nominal" or str(row.get("case", "")).startswith("slip_"))
            and row.get("separation_found")
            and float(row.get("normal_impulse", 0.0)) > 0.0
        ),
        key=lambda row: float(row["tangential_speed"]),
    )
    slip_x = np.asarray(
        [
            (2.0 * float(row["mass"]) / 7.0)
            * abs(float(row["tangential_speed"]))
            / (float(row["friction"]) * float(row["normal_impulse"]))
            for row in slip
        ]
    )
    slip_y = np.asarray(
        [
            float(row["tangent_impulse_magnitude"])
            / (float(row["friction"]) * float(row["normal_impulse"]))
            for row in slip
        ]
    )
    theory_x = np.linspace(0.0, max(1.6, float(np.max(slip_x)) * 1.03), 300)
    axis_c.plot(
        theory_x,
        np.minimum(theory_x, 1.0),
        color=COLORS["reference"],
        linewidth=1.2,
        label="Maximum dissipation",
    )
    axis_c.scatter(
        slip_x,
        slip_y,
        color=COLORS["shifted"],
        edgecolor="white",
        linewidth=0.35,
        s=18,
        label="Shifted ADMM",
        zorder=3,
    )
    axis_c.set_xlabel(r"Incident demand $m_T|v_T^-|/(\mu J_N)$")
    axis_c.set_ylabel(r"Tangential impulse $|J_T|/(\mu J_N)$")
    axis_c.legend(
        loc="lower center",
        bbox_to_anchor=(0.5, 1.01),
        ncol=2,
        borderaxespad=0.0,
        handlelength=2.2,
        columnspacing=1.0,
    )
    write_csv(
        output_dir / "paper_figure" / "panel_c_impulse.csv",
        [
            {
                "case": row["case"],
                "normalized_incident_demand": float(x),
                "normalized_tangential_impulse": float(y),
                "normal_impulse_Ns": float(row["normal_impulse"]),
                "tangential_impulse_Ns": float(row["tangent_impulse_magnitude"]),
            }
            for row, x, y in zip(slip, slip_x, slip_y)
        ],
    )

    load_ratio = np.asarray([float(row["load_ratio"]) for row in static_rows])
    friction_limit = np.asarray(
        [float(row["friction_limit_N"]) for row in static_rows]
    )
    shifted_force_ratio = np.asarray(
        [float(row["shifted_tangent_force_N"]) for row in static_rows]
    ) / friction_limit
    reference_force_ratio = np.asarray(
        [float(row["reference_tangent_force_N"]) for row in static_rows]
    ) / friction_limit
    smooth_force_ratio = np.asarray(
        [float(row["smooth_tangent_force_N"]) for row in static_rows]
    ) / friction_limit
    velocity_scale = float(static_rows[0]["smooth_velocity_m_per_s"])
    shifted_speed_ratio = np.asarray(
        [float(row["shifted_tangent_speed_m_per_s"]) for row in static_rows]
    ) / velocity_scale
    smooth_speed_ratio = np.asarray(
        [float(row["smooth_tangent_speed_m_per_s"]) for row in static_rows]
    ) / velocity_scale
    axis_d.plot(
        load_ratio,
        reference_force_ratio,
        color=COLORS["reference"],
        linewidth=1.2,
        label="Coulomb reference",
    )
    axis_d.scatter(
        load_ratio[::3],
        shifted_force_ratio[::3],
        color=COLORS["shifted"],
        edgecolor="white",
        linewidth=0.3,
        s=15,
        label="Shifted ADMM",
        zorder=3,
    )
    axis_d.plot(
        load_ratio,
        smooth_force_ratio,
        ":",
        color=COLORS["smooth"],
        linewidth=1.3,
        label="Smooth law",
    )
    speed_axis = axis_d.twinx()
    speed_axis.spines["right"].set_visible(True)
    speed_axis.plot(
        load_ratio,
        shifted_speed_ratio,
        "--",
        color=COLORS["shifted"],
        linewidth=0.9,
        alpha=0.75,
    )
    speed_axis.plot(
        load_ratio,
        smooth_speed_ratio,
        "--",
        color=COLORS["smooth"],
        linewidth=0.9,
        alpha=0.9,
    )
    axis_d.set_xlabel(r"Applied load $F_{\rm ext}/(\mu mg)$")
    axis_d.set_ylabel(r"Opposing force $|f_T|/(\mu f_N)$")
    speed_axis.set_ylabel(r"Slip speed $|v_T|/v_s$")
    axis_d.legend(
        loc="lower center",
        bbox_to_anchor=(0.5, 1.01),
        ncol=3,
        borderaxespad=0.0,
        handlelength=2.0,
        columnspacing=0.9,
    )

    for label, axis in zip("abcd", (axis_a, axis_b, axis_c, axis_d)):
        add_panel_label(axis, label)

    base_path = output_dir / "paper_figure" / "contact_law_validation"
    save_publication_figure(figure, base_path, exclude_axes=[speed_axis])
    plt.close(figure)
    return base_path


def plot_case(
    output_path: Path,
    case: Case,
    shifted: list[dict[str, Any]],
    unshifted: list[dict[str, Any]],
) -> None:
    figure, axes = plt.subplots(4, 1, figsize=(7.0, 8.0), sharex=True)
    for label, rows, style in (
        ("event-targeted", shifted, "-"),
        ("unshifted", unshifted, "--"),
    ):
        time = np.asarray([row["time"] for row in rows])
        depth = np.asarray([row["depth"] for row in rows])
        normal_velocity = np.asarray([row["vz"] for row in rows])
        normal_force = np.asarray([row["fn"] for row in rows])
        tangential_velocity = np.asarray(
            [math.hypot(row["vx"], row["vy"]) for row in rows]
        )
        axes[0].plot(time, 1.0e3 * depth, style, label=label)
        axes[1].plot(time, normal_velocity, style, label=label)
        axes[2].plot(time, normal_force, style, label=label)
        axes[3].plot(time, tangential_velocity, style, label=label)
    axes[0].set_ylabel("depth [mm]")
    axes[1].set_ylabel(r"$v_N$ [m/s]")
    axes[2].set_ylabel(r"$f_N$ [N]")
    axes[3].set_ylabel(r"$\|v_T\|$ [m/s]")
    axes[3].set_xlabel("time [s]")
    axes[0].set_title(
        f"{case.name}: k={case.stiffness:.2g}, zeta={case.damping_ratio:g}, "
        f"mu={case.friction:g}"
    )
    axes[0].legend()
    for axis in axes:
        axis.grid(True, alpha=0.25)
    figure.tight_layout()
    figure.savefig(output_path, dpi=300)
    plt.close(figure)


def plot_first_contact(
    output_path: Path,
    shifted: list[dict[str, Any]],
    unshifted: list[dict[str, Any]],
) -> None:
    shifted_episode = first_contact_episode(shifted)
    if shifted_episode is None:
        return
    start, end = shifted_episode
    lower = max(0.0, shifted[max(0, start - 2)]["time"])
    upper = shifted[min(len(shifted) - 1, end + 5)]["time"]
    figure, axes = plt.subplots(3, 1, figsize=(6.7, 6.2), sharex=True)
    for label, rows, style in (
        ("event-targeted", shifted, "-"),
        ("unshifted", unshifted, "--"),
    ):
        selected = [row for row in rows if lower <= row["time"] <= upper]
        time = np.asarray([row["time"] for row in selected])
        axes[0].plot(
            time,
            1.0e3 * np.asarray([row["depth"] for row in selected]),
            style,
            label=label,
        )
        axes[1].plot(time, [row["vz"] for row in selected], style, label=label)
        axes[2].plot(time, [row["fn"] for row in selected], style, label=label)
    axes[0].set_ylabel("depth [mm]")
    axes[1].set_ylabel(r"$v_N$ [m/s]")
    axes[2].set_ylabel(r"$f_N$ [N]")
    axes[2].set_xlabel("time [s]")
    axes[0].legend()
    for axis in axes:
        axis.grid(True, alpha=0.25)
        axis.set_xlim(lower, upper)
    figure.tight_layout()
    figure.savefig(output_path, dpi=300)
    plt.close(figure)


def run_case(case: Case, variant: str, output_dir: Path):
    run_dir = output_dir / "runs" / case.name / variant
    trace_root = run_dir / "traces"
    sim, loop, ball = build_simulation(case, variant, trace_root)
    while not loop.isTimeout() and not loop.isEventListEmpty():
        loop.integrate()
        loop.handleContact()
    trace_dir = latest_trajectory(trace_root)
    records = loop.recordsToJson()
    rows = extract_time_series(records, ball.id)
    merge_trace_into_time_series(rows, read_solver_trace(trace_dir), case.radius)
    write_csv(run_dir / "trajectory.csv", rows)
    summary = summarize(case, variant, rows, trace_dir)
    with (run_dir / "summary.json").open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, allow_nan=True)
    return rows, summary, trace_dir


def replay_contact_systems(
    trace_dir: Path, output_dir: Path, runs: int, max_frames: int | None
) -> None:
    if not COMPARE_SOLVERS.exists():
        raise FileNotFoundError(f"missing comparison tool: {COMPARE_SOLVERS}")
    command = [
        sys.executable,
        str(COMPARE_SOLVERS),
        "--load",
        str(trace_dir),
        "--solvers",
        "Spectral-ADMM",
        "Shifted-Spectral-ADMM",
        "Exact-Coulomb",
        "--reference",
        "Exact-Coulomb",
        "--runs",
        str(runs),
        "--max-iters",
        "200",
        "--max-err",
        "1e-8",
        "--output-dir",
        str(output_dir),
        "--plot",
    ]
    if max_frames is not None:
        command.extend(["--max-frames", str(max_frames)])
    subprocess.run(command, cwd=REPO_ROOT, check=True)


def plot_aggregate(output_dir: Path, summaries: list[dict[str, Any]]) -> None:
    valid = [
        row
        for row in summaries
        if row.get("contact_found") and row.get("separation_found")
    ]
    if not valid:
        return
    figure, axes = plt.subplots(1, 3, figsize=(11.2, 3.4))
    marker = {"shifted": "o", "unshifted": "x"}
    for variant in ("shifted", "unshifted"):
        selected = [row for row in valid if row["variant"] == variant]
        if not selected:
            continue
        axes[0].scatter(
            [row["analytical_contact_time"] * 1.0e3 for row in selected],
            [row["measured_contact_time"] * 1.0e3 for row in selected],
            marker=marker[variant],
            label=variant,
        )
        axes[1].scatter(
            [row["analytical_terminal_normal_velocity"] for row in selected],
            [row["terminal_normal_velocity"] for row in selected],
            marker=marker[variant],
            label=variant,
        )
    for axis in axes[:2]:
        limits = [
            min(axis.get_xlim()[0], axis.get_ylim()[0]),
            max(axis.get_xlim()[1], axis.get_ylim()[1]),
        ]
        axis.plot(limits, limits, ":", color="black", linewidth=1)
        axis.set_xlim(limits)
        axis.set_ylim(limits)
        axis.grid(True, alpha=0.25)
    axes[0].set_xlabel("analytical geometric separation time [ms]")
    axes[0].set_ylabel("simulated geometric separation time [ms]")
    axes[1].set_xlabel("analytical geometric separation velocity [m/s]")
    axes[1].set_ylabel("simulated geometric separation velocity [m/s]")
    axes[0].legend()
    variants = ("shifted", "unshifted")
    totals = [sum(row["variant"] == variant for row in summaries) for variant in variants]
    separated = [
        sum(row["variant"] == variant and row.get("separation_found") for row in summaries)
        for variant in variants
    ]
    fractions = [count / total if total else 0.0 for count, total in zip(separated, totals)]
    axes[2].bar(variants, fractions, color=("C0", "C1"))
    axes[2].set_ylim(0.0, 1.08)
    axes[2].set_ylabel("geometric separation rate")
    axes[2].grid(True, axis="y", alpha=0.25)
    for index, (count, total) in enumerate(zip(separated, totals)):
        note = f"{count}/{total}"
        if count == 0 and total:
            note += "\nall right-censored"
        axes[2].text(index, fractions[index] + 0.025, note, ha="center", va="bottom")
    figure.tight_layout()
    figure.savefig(output_dir / "aggregate_contact_law.png", dpi=300)
    plt.close(figure)


def plot_timestep_convergence(
    output_dir: Path, summaries: list[dict[str, Any]]
) -> None:
    selected = sorted(
        (
            row
            for row in summaries
            if row.get("variant") == "shifted"
            and row.get("separation_found")
            and math.isfinite(float(row.get("contact_time_abs_error", math.nan)))
            and math.isfinite(float(row.get("terminal_velocity_abs_error", math.nan)))
        ),
        key=lambda row: float(row["nominal_dt"]),
    )
    if len(selected) < 2:
        return

    dt = np.asarray([float(row["nominal_dt"]) for row in selected])
    time_error = np.asarray([float(row["contact_time_abs_error"]) for row in selected])
    velocity_error = np.asarray(
        [float(row["terminal_velocity_abs_error"]) for row in selected]
    )
    time_slope = float(np.polyfit(np.log(dt), np.log(time_error), 1)[0])
    velocity_slope = float(np.polyfit(np.log(dt), np.log(velocity_error), 1)[0])
    # Five or fewer intervals do not resolve the contact transient and fall
    # outside the asymptotic regime.  Report both fits, and use the explicitly
    # defined >=8-interval subset for the order shown in the paper figure.
    resolved = np.asarray(
        [int(row["contact_intervals"]) >= 8 for row in selected], dtype=bool
    )
    resolved_time_slope = float(
        np.polyfit(np.log(dt[resolved]), np.log(time_error[resolved]), 1)[0]
    )
    resolved_velocity_slope = float(
        np.polyfit(np.log(dt[resolved]), np.log(velocity_error[resolved]), 1)[0]
    )

    convergence_rows = []
    for row in selected:
        convergence_rows.append(
            {
                "case": row["case"],
                "dt_s": row["nominal_dt"],
                "contact_intervals": row["contact_intervals"],
                "contact_time_abs_error_s": row["contact_time_abs_error"],
                "contact_time_relative_error": row["contact_time_relative_error"],
                "terminal_velocity_abs_error_m_per_s": row[
                    "terminal_velocity_abs_error"
                ],
                "terminal_velocity_relative_error": row[
                    "terminal_velocity_relative_error"
                ],
                "fitted_contact_time_order": time_slope,
                "fitted_terminal_velocity_order": velocity_slope,
                "resolved_contact_time_order": resolved_time_slope,
                "resolved_terminal_velocity_order": resolved_velocity_slope,
                "resolved_fit_minimum_intervals": 8,
            }
        )
    write_csv(output_dir / "timestep_convergence.csv", convergence_rows)

    figure, axes = plt.subplots(1, 2, figsize=(7.2, 3.2))
    panels = (
        (
            axes[0],
            time_error * 1.0e3,
            r"$|e_{t_g}|$ [ms]",
            resolved_time_slope,
            "Geometric separation time",
        ),
        (
            axes[1],
            velocity_error,
            r"$|e_{v_g}|$ [m/s]",
            resolved_velocity_slope,
            "Geometric separation velocity",
        ),
    )
    for axis, error, ylabel, slope, title in panels:
        axis.loglog(dt * 1.0e3, error, "o-", label=f"resolved order {slope:.2f}")
        first_order = error[0] * dt / dt[0]
        axis.loglog(dt * 1.0e3, first_order, ":", color="black", label="first order")
        axis.set_xlabel(r"nominal step $\Delta t$ [ms]")
        axis.set_ylabel(ylabel)
        axis.set_title(title)
        axis.grid(True, which="both", alpha=0.25)
        axis.legend()
    figure.tight_layout()
    figure.savefig(output_dir / "timestep_convergence.png", dpi=300)
    plt.close(figure)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--preset",
        choices=("smoke", "full", "convergence", "paper"),
        default="smoke",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="default: demo_paper2/results/<timestamp>",
    )
    parser.add_argument(
        "--skip-replay",
        action="store_true",
        help="do not run the offline Exact/ADMM contact-system comparison",
    )
    parser.add_argument(
        "--timing-runs",
        type=int,
        default=5,
        help="repetitions per solver/contact system during offline timing",
    )
    parser.add_argument(
        "--replay-max-frames",
        type=int,
        help="optional cap for each case's offline trace replay",
    )
    parser.add_argument(
        "--replay-all",
        action="store_true",
        help="replay every case; the paper preset otherwise replays nominal only",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.timing_runs < 1:
        raise ValueError("--timing-runs must be at least one")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = (args.output_dir or DEFAULT_OUTPUT_ROOT / timestamp).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    cases = make_cases(args.preset)
    with (output_dir / "cases.json").open("w", encoding="utf-8") as stream:
        json.dump([{**asdict(case), "damping": case.damping} for case in cases], stream, indent=2)

    summaries: list[dict[str, Any]] = []
    all_trajectories: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for sequence, case in enumerate(cases, start=1):
        print(f"[{sequence}/{len(cases)}] {case.name}")
        trajectories: dict[str, list[dict[str, Any]]] = {}
        shifted_trace: Path | None = None
        variants = (
            ("shifted", "unshifted")
            if args.preset != "paper" or case.name == "nominal"
            else ("shifted",)
        )
        for variant in variants:
            print(f"  running {variant}")
            rows, summary, trace_dir = run_case(case, variant, output_dir)
            trajectories[variant] = rows
            all_trajectories[(case.name, variant)] = rows
            summaries.append(summary)
            if variant == "shifted":
                shifted_trace = trace_dir
        if "unshifted" in trajectories:
            plot_case(
                output_dir / "runs" / case.name / "trajectory_comparison.png",
                case,
                trajectories["shifted"],
                trajectories["unshifted"],
            )
            plot_first_contact(
                output_dir / "runs" / case.name / "first_contact_comparison.png",
                trajectories["shifted"],
                trajectories["unshifted"],
            )
        should_replay = (
            not args.skip_replay
            and (
                args.replay_all
                or args.preset != "paper"
                or case.name == "nominal"
            )
        )
        if should_replay:
            if shifted_trace is None:
                print("  warning: shifted run produced no solver trace", file=sys.stderr)
            else:
                print("  replaying recorded contact systems")
                replay_contact_systems(
                    shifted_trace,
                    output_dir / "solver_comparison" / case.name,
                    args.timing_runs,
                    args.replay_max_frames,
                )

    write_csv(output_dir / "summaries.csv", summaries)
    plot_aggregate(output_dir, summaries)
    if args.preset == "convergence":
        plot_timestep_convergence(output_dir, summaries)
    if args.preset == "paper":
        static_rows = run_static_friction_sweep(output_dir)
        figure_base = plot_paper_figure(
            output_dir,
            all_trajectories,
            summaries,
            static_rows,
        )
        print(f"paper figure: {figure_base}.pdf")
    print(f"results: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
