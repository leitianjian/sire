"""Rerun Fig. 3 with zero horizontal speed and audit oblique controls.

Run with D:/code/sire/.venv/Scripts/python.exe. This reuses the existing
scene builder without rebuilding Sire or invoking its legacy plotting code.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import sys

import numpy as np


STIFFNESS = (1e7, 1e9, 1e12, 1e15, 1e18, 1e21, 1e24, 1e30)
PREVIOUS_PAPER_STIFFNESS = tuple(sorted(set(STIFFNESS + (2e2, 2e3, 2e5, 3e6, 2e8, 1e38))))


def build_previous_paper_scene(contact, case, trace_root: Path):
    """Match the previous paper's physical scene using the current shifted solver."""
    sire = contact.sire
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
    solver = engine.addShiftedSpectralADMMSolver()
    solver.setInitialPenetrationPolicy("eliminate")
    solver.setContactTimeMethod("exponential")
    solver.warmStartEnabled = True
    prop = ("{k:%0.17g,d:0,cr:0.2,cof:%0.17g,threshold_velocity:1e-4}"
            % (case.stiffness, case.friction))
    solver.setDefaultProp(prop)
    solver.addMaterialPair("steel", "copper", prop)
    model.setGravity([0, 0, -9.81, 0, 0, 0])
    model.ground().addMarker("ground_marker")
    ground = engine.addBoxGeometry(
        5.0, 5.0, 1.0, 0, False,
        [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -0.5, 0, 0, 0, 1], True, "steel",
    )
    pose = [0.0, 0.0, case.radius + case.initial_gap, 0.0, 0.0, 0.0]
    inertia = 0.4 * case.mass * case.radius ** 2
    ball = model.addPartByPe(pose, "313", [case.mass, 0, 0, 0, inertia, inertia, inertia, 0, 0, 0])
    ball.addMarker("ball_center")
    model.init()
    geometry = engine.addSphereGeometry(case.radius, ball.id, True, [], True, "copper")
    geometry.cptInertial(ball, case.mass)
    ball.vs = sire.vp2vs(pose[:3], [0.0, 0.0, 0.0])
    sim.init()
    engine.collisionFilter().enableCollisionPair(ground.id, geometry.id)
    engine.collisionFilter().saveMatConfig()
    return sim, loop, ball


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sire-demo", type=Path, default=Path(__file__).resolve().parents[1] / "simulation")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--refinement-only", action="store_true")
    parser.add_argument("--previous-paper", action="store_true",
                        help="4 m bottom clearance, radius .5 m, release from rest, 4 s duration")
    args = parser.parse_args()
    sys.path.insert(0, str(args.sire_demo.resolve()))
    import run_contact_law_experiment as contact
    from run_high_stiffness_regression import first_bounce_metrics, contact_onset_metrics

    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    config = {
        "stiffness": STIFFNESS, "friction": [0.0, 0.3],
        "normal_speed": 0.5, "tangential_speed": 0.0,
        "initial_gap": 0.01, "dt": 0.001, "duration": 0.25,
        "oblique_control_stiffness": [1e7, 1e30],
        "oblique_control_tangential_speed": 0.5,
        "python": sys.executable,
    }
    (output / "configuration.json").write_text(json.dumps(config, indent=2), encoding="utf-8")
    groups = (
        [(f"oblique_dt_{dt:g}", 0.5, (1e30,), dt) for dt in (0.0005, 0.00025, 0.000125)]
        if args.refinement_only else
        [("normal", 0.0, STIFFNESS, 0.001), ("oblique", 0.5, (1e7, 1e30), 0.001)]
    )
    if args.previous_paper:
        if args.refinement_only:
            parser.error("--previous-paper and --refinement-only cannot be combined")
        groups = [("normal", 0.0, PREVIOUS_PAPER_STIFFNESS, 0.001)]
        config.update({
            "stiffness": PREVIOUS_PAPER_STIFFNESS, "normal_speed": 0.0,
            "tangential_speed": 0.0, "initial_gap": 4.0, "radius": 0.5,
            "mass": 1.0, "duration": 4.0, "reference_height": 4.0,
            "reference_definition": "first bottom-clearance apex divided by initial bottom clearance",
            "ground_dimensions": [5.0, 5.0, 1.0], "ground_center_z": -0.5,
            "threshold_velocity": 1e-4, "cr": 0.2,
            "contact_time_method": "exponential", "solver": "ShiftedSpectralADMM",
            "source_scene": "demo/demo_paper/sire/bouncing_ball/sire_ball_free_fall.xml",
        })
        config.pop("oblique_control_stiffness")
        config.pop("oblique_control_tangential_speed")
    config["groups"] = groups
    (output / "configuration.json").write_text(json.dumps(config, indent=2), encoding="utf-8")
    summaries = {group: [] for group, _, _, _ in groups}
    for group, horizontal, stiffness_values, dt in groups:
        for mu in (0.0, 0.3):
            for stiffness in stiffness_values:
                name = f"mu_{mu:g}_k_{stiffness:.0e}".replace("+", "")
                case = contact.Case(
                    name, normal_speed=config["normal_speed"], tangential_speed=horizontal,
                    stiffness=stiffness, damping_ratio=0.0, friction=mu,
                    nominal_dt=dt, initial_gap=config["initial_gap"], duration=config["duration"],
                    radius=config.get("radius", 0.1),
                )
                run_dir = output / group / "runs" / name / "shifted"
                if args.previous_paper:
                    sim, loop, ball = build_previous_paper_scene(contact, case, run_dir / "traces")
                else:
                    sim, loop, ball = contact.build_simulation(case, "shifted", run_dir / "traces")
                steps = 0
                while not loop.isTimeout() and not loop.isEventListEmpty():
                    loop.integrate()
                    loop.handleContact()
                    steps += 1
                    if steps > max(10000, 20 * case.duration / dt):
                        raise RuntimeError(f"integration did not advance for {name}")
                records = loop.recordsToJson()
                rows = contact.extract_time_series(records, ball.id)
                for index, row in enumerate(rows):
                    position = np.asarray(records["partPq"][index][ball.id][:3], dtype=float)
                    spatial = np.asarray(records["partVs"][index][ball.id], dtype=float)
                    # World spatial velocity [u, omega] gives point velocity u + omega x p.
                    center_velocity = spatial[:3] + np.cross(spatial[3:], position)
                    for axis, value in zip("xyz", center_velocity):
                        row[f"legacy_v{axis}"] = row[f"v{axis}"]
                        row[f"center_v{axis}_from_spatial"] = float(value)
                        row[f"v{axis}"] = float(value)
                    for axis, value in zip("xyz", spatial[3:]):
                        row[f"omega_{axis}"] = float(value)
                    row["spatial_u_z"] = float(spatial[2])
                trace_dir = contact.latest_trajectory(run_dir / "traces")
                contact.merge_trace_into_time_series(rows, contact.read_solver_trace(trace_dir), case.radius)
                summary = contact.summarize(case, "shifted", rows, trace_dir)
                summary.update(contact_onset_metrics(rows, case.mass, stiffness))
                summary.update(first_bounce_metrics(rows, case))
                if args.previous_paper:
                    summary["incident_reference_bounce_height"] = summary["reference_bounce_height"]
                    summary["first_bounce_height_ratio_incident"] = summary["first_bounce_height_ratio"]
                    summary["reference_bounce_height"] = case.initial_gap
                    summary["first_bounce_height_ratio"] = summary["first_bounce_height"] / case.initial_gap
                    summary["release_height_relative_error"] = (
                        summary["first_bounce_height"] - case.initial_gap
                    ) / case.initial_gap
                summary.update({"status": "ok", "group": group})
                summary["max_center_velocity_extraction_error"] = max(
                    abs(row[f"legacy_v{axis}"] - row[f"center_v{axis}_from_spatial"])
                    for row in rows for axis in "xyz"
                )
                episode = contact.first_contact_episode(rows)
                if episode is not None:
                    start, end = episode
                    summary["initial_center_vz"] = rows[start]["center_vz_from_spatial"]
                    summary["terminal_center_vz"] = rows[end + 1]["center_vz_from_spatial"]
                    summary["terminal_omega_y"] = rows[end + 1]["omega_y"]
                    summary["terminal_spatial_u_z"] = rows[end + 1]["spatial_u_z"]
                    summary["impulse_predicted_center_vz"] = (
                        summary["initial_center_vz"] + summary["normal_impulse"] / case.mass
                        - 9.81 * summary["measured_contact_time"]
                    )
                contact.write_csv(run_dir / "trajectory.csv", rows)
                (run_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
                (run_dir / "case.json").write_text(json.dumps(asdict(case), indent=2), encoding="utf-8")
                summaries[group].append(summary)
                contact.write_csv(output / group / "high_stiffness_regression.csv", summaries[group])
                print(group, name, "height ratio", summary["first_bounce_height_ratio"],
                      "center extraction error", summary["max_center_velocity_extraction_error"], flush=True)
    (output / "summary.json").write_text(json.dumps(summaries, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
