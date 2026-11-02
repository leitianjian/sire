"""Rerun full Sire sphere impacts with one signed-vector impulse definition.

No force or endpoint is prescribed by the audit. Store raw simulator records,
world force integrals, center/contact velocities, and frozen NCP diagnostics.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(ROOT / "tools"))
from run_contact_law_experiment import Case, build_simulation, write_csv
from compare_solvers import exact_coulomb_residual, problem_matrices


def point_velocity(spatial, point):
    return spatial[:3] + np.cross(spatial[3:], point)


def run(case, variant, output, records_only=False):
    run_dir = output / "runs" / case.name / variant
    if records_only:
        raw = json.loads((run_dir / "raw_records.json").read_text(encoding="utf-8"))
        ball_id = 1
    else:
        sim, loop, ball = build_simulation(case, variant, run_dir / "traces")
        ball_id = ball.id
        while not loop.isTimeout() and not loop.isEventListEmpty():
            loop.integrate()
            loop.handleContact()
        raw = loop.recordsToJson()
        (run_dir / "raw_records.json").write_text(json.dumps(raw, allow_nan=True), encoding="utf-8")
    states = [i for i, p in enumerate(raw["partPq"]) if p is not None]
    intervals, trajectory, contact_rows = [], [], []
    active, ended = False, False
    inertia = .4 * case.mass * case.radius ** 2
    for i in states:
        p = np.array(raw["partPq"][i][ball_id][:3], dtype=float)
        v = np.array(raw["partVs"][i][ball_id], dtype=float)
        center = point_velocity(v, p)
        trajectory.append({"time_s": raw["timeIndex"][i], "x_m": p[0], "z_m": p[2],
                           "center_vx_m_per_s": center[0], "center_vz_m_per_s": center[2],
                           "omega_y_rad_per_s": v[4], "legacy_spatial_u_z": v[2]})
    for i, j in zip(states, states[1:]):
        t0, t1 = float(raw["timeIndex"][i]), float(raw["timeIndex"][j])
        dt = t1 - t0
        if dt <= 0:
            continue
        forces = raw["generalForces"][i]
        if forces is None or len(forces) != 2:
            raise RuntimeError("Unexpected two-body force record")
        force = np.asarray(forces[1][:3], dtype=float)
        pair_list = raw["penetrationPairs"][i] or []
        p0 = np.asarray(raw["partPq"][i][ball_id][:3], dtype=float)
        p1 = np.asarray(raw["partPq"][j][ball_id][:3], dtype=float)
        v0 = np.asarray(raw["partVs"][i][ball_id], dtype=float)
        v1 = np.asarray(raw["partVs"][j][ball_id], dtype=float)
        # The collision-reported point is fixed over this discrete contact solve.
        cp = np.asarray(pair_list[0]["p_WC"], dtype=float) if pair_list else p0 - [0., 0., case.radius]
        c0, c1 = point_velocity(v0, cp), point_velocity(v1, cp)
        u0, u1 = point_velocity(v0, p0), point_velocity(v1, p1)
        energy0 = .5 * case.mass * (u0 @ u0) + .5 * inertia * (v0[3:] @ v0[3:]) + case.mass * 9.81 * p0[2]
        energy1 = .5 * case.mass * (u1 @ u1) + .5 * inertia * (v1[3:] @ v1[3:]) + case.mass * 9.81 * p1[2]
        row = {"time_before_s": t0, "time_after_s": t1, "dt_s": dt,
               "Fx_N": force[0], "Fy_N": force[1], "Fn_N": force[2],
               "Jx_Ns": dt * force[0], "Jy_Ns": dt * force[1], "Jn_Ns": dt * force[2],
               "slip_vx_before": c0[0], "slip_vy_before": c0[1],
               "slip_vx_after_frozen_point": c1[0], "slip_vy_after_frozen_point": c1[1],
               "friction_power_before_W": force[:2] @ c0[:2],
               "friction_power_after_W": force[:2] @ c1[:2],
               "friction_work_midpoint_J": dt * force[:2] @ (.5 * (c0[:2] + c1[:2])),
               "mechanical_energy_change_J": energy1 - energy0,
               "normal_momentum_discrepancy_Ns": case.mass * (u1[2] - u0[2] + 9.81 * dt) - dt * force[2],
               "normal_tangential_force_cone_excess_N": np.linalg.norm(force[:2]) - case.friction * force[2]}
        intervals.append(row)
        if force[2] > 1e-8 and not ended:
            active = True
            contact_rows.append(row)
        elif active:
            ended = True
    if not contact_rows:
        raise RuntimeError("No positive normal contact impulse")
    net = np.sum([[r["Jx_Ns"], r["Jy_Ns"]] for r in contact_rows], axis=0)
    jn = sum(r["Jn_Ns"] for r in contact_rows)
    norm_sum = sum(np.hypot(r["Jx_Ns"], r["Jy_Ns"]) for r in contact_rows)
    vt = abs(case.tangential_speed)
    summary = {"case": case.name, "variant": variant, **asdict(case),
               "normal_impulse_Ns": jn, "net_tangential_impulse_Ns": float(np.linalg.norm(net)),
               "net_tangential_impulse_x_Ns": net[0], "sum_tangential_impulse_magnitudes_Ns": norm_sum,
               "sum_magnitudes_minus_net_Ns": norm_sum - np.linalg.norm(net),
               "normalized_incident_demand": (2 * case.mass / 7) * vt / (case.friction * jn),
               "normalized_tangential_impulse": np.linalg.norm(net) / (case.friction * jn),
               "friction_force_x_reversal_intervals": sum(r["Fx_N"] > 1e-6 for r in contact_rows) if vt else 0,
               "positive_pre_power_intervals": sum(r["friction_power_before_W"] > 1e-8 for r in contact_rows),
               "positive_post_power_intervals": sum(r["friction_power_after_W"] > 1e-8 for r in contact_rows),
               "positive_midpoint_work_intervals": sum(r["friction_work_midpoint_J"] > 1e-10 for r in contact_rows),
               "max_positive_post_power_W": max(0., max(r["friction_power_after_W"] for r in contact_rows)),
               "contact_start_s": contact_rows[0]["time_before_s"],
               "contact_last_positive_force_end_s": contact_rows[-1]["time_after_s"],
               "max_momentum_force_mismatch_Ns": max(abs(r["normal_momentum_discrepancy_Ns"]) for r in contact_rows)}
    traces = []
    for path in sorted((run_dir / "traces").rglob("frame_*.json")):
        frame = json.loads(path.read_text())
        if variant == "unshifted":
            frame["v_target"] = [0.] * frame["n"]
        f = np.asarray(frame["cResult"], dtype=float)
        matrix, free = problem_matrices(frame)
        v = matrix @ f + free
        tangential_power = sum(f[k:k+2] @ v[k:k+2] for k in range(0, len(f), 3))
        traces.append({"time_s": frame["sim_time"], "h_s": frame["h"],
                       "first_force_episode": int(summary["contact_start_s"] - 1e-10 <= frame["sim_time"] < summary["contact_last_positive_force_end_s"] - 1e-10),
                       "ncp_residual": exact_coulomb_residual(frame, f),
                       "reported_solver_error": frame["solver_error"],
                       "solver_tolerance": frame["solver_max_error"],
                       "discrete_friction_power_W": tangential_power,
                       "normal_force_N": f[2], "tangential_force_N": np.linalg.norm(f[:2]),
                       "post_slip_speed_m_per_s": np.linalg.norm(v[:2]),
                       "normal_target_error_m_per_s": v[2] + frame["v_target"][0]})
    if traces:
        write_csv(run_dir / "ncp_diagnostics.csv", traces)
        first = [t for t in traces if t["first_force_episode"]]
        summary.update({"max_recomputed_ncp_residual": max(t["ncp_residual"] for t in first),
                        "max_reported_solver_error": max(t["reported_solver_error"] for t in first),
                        "frames_above_tolerance": sum(t["ncp_residual"] > t["solver_tolerance"] for t in first),
                        "positive_ncp_power_frames": sum(t["discrete_friction_power_W"] > 1e-8 for t in first),
                        "max_positive_ncp_power_W": max(0., max(t["discrete_friction_power_W"] for t in first)),
                        "all_episodes_max_recomputed_residual": max(t["ncp_residual"] for t in traces)})
    write_csv(run_dir / "force_intervals.csv", intervals)
    write_csv(run_dir / "center_trajectory.csv", trajectory)
    summary = json.loads(json.dumps(summary, default=lambda x: x.item()))
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary, trajectory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--probe-only", action="store_true")
    parser.add_argument("--records-only", action="store_true", help="Reanalyze the saved raw simulator records without rerunning")
    args = parser.parse_args()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    inp = ROOT / "res/source_data/experiment1/panel_c_impulse_refined_v6.csv"
    rows = list(csv.DictReader(inp.open(newline="", encoding="utf-8")))
    nominal = json.loads((ROOT / "data/contact_law/runs/nominal/shifted/summary.json").read_text())
    base = Case("nominal", **{key: nominal[key] for key in Case.__dataclass_fields__ if key != "name"})
    cases = [replace(base, name=r["case"], tangential_speed=base.tangential_speed if r["case"] == "nominal"
                     else float(r["case"].split("_", 1)[1])) for r in rows]
    if args.probe_only:
        cases = [replace(base, name="slip_0.300", tangential_speed=.3), base]
    summaries, normal_rows = [], []
    for i, case in enumerate(cases, 1):
        for variant in ("shifted", "velocity_regularized"):
            print(f"[{i}/{len(cases)}] {case.name} {variant}", flush=True)
            summary, trajectory = run(case, variant, out, args.records_only)
            summaries.append(summary)
            if case.name == "nominal" and variant == "shifted":
                start = summary["contact_start_s"]
                normal_rows.extend({"method": "Shifted", "normal_velocity_m_per_s": r["center_vz_m_per_s"],
                                    "time_from_contact_ms": 1000 * (r["time_s"] - start)}
                                   for r in trajectory if start - .0010001 <= r["time_s"] <= start + .0140001)
        write_csv(out / "impact_audit.csv", summaries)
    summary, trajectory = run(base, "unshifted", out, args.records_only)
    start = summary["contact_start_s"]
    normal_rows.extend({"method": "Zero target", "normal_velocity_m_per_s": r["center_vz_m_per_s"],
                        "time_from_contact_ms": 1000 * (r["time_s"] - start)}
                       for r in trajectory if start - .0010001 <= r["time_s"] <= start + .0140001)
    write_csv(out / "panel_a_center_velocity_audited.csv", normal_rows)
    write_csv(out / "impact_audit.csv", summaries)
    report = {"python": sys.executable, "input": str(inp),
              "input_sha256": hashlib.sha256(inp.read_bytes()).hexdigest(),
              "parameters": asdict(base), "sample_count_per_method": len(cases),
              "rotation_locked": False, "common_impulse_definition": "norm of world vector integral; first contiguous force episode",
              "unshifted_summary": summary, "probe_only": args.probe_only}
    for variant in ("shifted", "velocity_regularized"):
        selected = [s for s in summaries if s["variant"] == variant]
        report[variant] = {
            "max_normalized_error": max(abs(s["normalized_tangential_impulse"] - min(s["normalized_incident_demand"], 1)) for s in selected),
            "max_norm_sum_minus_net_Ns": max(s["sum_magnitudes_minus_net_Ns"] for s in selected),
            "force_reversal_intervals": sum(s["friction_force_x_reversal_intervals"] for s in selected),
            "positive_pre_power_intervals": sum(s["positive_pre_power_intervals"] for s in selected),
            "positive_post_power_intervals": sum(s["positive_post_power_intervals"] for s in selected),
            "positive_midpoint_work_intervals": sum(s["positive_midpoint_work_intervals"] for s in selected),
            "max_positive_post_power_W": max(s["max_positive_post_power_W"] for s in selected),
            "max_momentum_force_mismatch_Ns": max(s["max_momentum_force_mismatch_Ns"] for s in selected)}
    proposed = [s for s in summaries if s["variant"] == "shifted"]
    report["shifted"].update({"max_recomputed_ncp_residual": max(s["max_recomputed_ncp_residual"] for s in proposed),
                              "frames_above_tolerance": sum(s["frames_above_tolerance"] for s in proposed),
                              "positive_ncp_power_frames": sum(s["positive_ncp_power_frames"] for s in proposed)})
    (out / "impact_audit_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
