"""Run the previous velocity-regularized solver at the archived impact inputs."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from dataclasses import replace
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
from run_contact_law_experiment import Case, build_simulation, write_csv


def force_impulses(case, variant, out):
    run_dir = out / "runs" / case.name / variant
    sim, loop, ball = build_simulation(case, variant, run_dir / "traces")
    while not loop.isTimeout() and not loop.isEventListEmpty():
        loop.integrate()
        loop.handleContact()
    raw = loop.recordsToJson()
    states = [index for index, pose in enumerate(raw["partPq"]) if pose is not None]
    rows, selected = [], []
    started = False
    for index, following in zip(states, states[1:]):
        before_time = float(raw["timeIndex"][index])
        after_time = float(raw["timeIndex"][following])
        dt = after_time - before_time
        if dt <= 0:
            continue
        forces = raw["generalForces"][index]
        if forces is None or len(forces) != 2:
            raise RuntimeError("Expected ground and sphere contact wrenches in this two-body scene")
        # The scene contains only the two equal/opposite contact wrenches;
        # the second is the sphere wrench expressed in the world frame.
        force = np.asarray(forces[1][:3], dtype=float)
        impulse = dt * force
        row = {"time_before_s": before_time, "time_after_s": after_time, "dt_s": dt,
               "Fx_N": force[0], "Fy_N": force[1], "Fn_N": force[2],
               "Jx_Ns": impulse[0], "Jy_Ns": impulse[1], "Jn_Ns": impulse[2]}
        rows.append(row)
        if impulse[2] > 1e-9:
            started = True
            selected.append(row)
        elif started:
            break
    if not selected:
        raise RuntimeError(f"No first-contact impulse: {case.name}, {variant}")
    write_csv(run_dir / "force_intervals.csv", rows)
    with (run_dir / "raw_records.json").open("w", encoding="utf-8") as stream:
        json.dump(raw, stream, allow_nan=True)
    summary = {"case": case.name, "variant": variant,
               "normal_impulse": sum(row["Jn_Ns"] for row in selected),
               "tangent_impulse_magnitude": math.hypot(sum(row["Jx_Ns"] for row in selected),
                                                       sum(row["Jy_Ns"] for row in selected)),
               "contact_start_s": selected[0]["time_before_s"],
               "contact_end_s": selected[-1]["time_after_s"],
               "contact_intervals": len(selected),
               "normal_impulse_min_Ns": min(row["Jn_Ns"] for row in selected)}
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--probe-only", action="store_true")
    args = parser.parse_args()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    source = ROOT / "res/source_data/experiment1/panel_c_impulse_refined_v6.csv"
    proposed = list(csv.DictReader(source.open(newline="", encoding="utf-8")))
    nominal_path = ROOT / "data/contact_law/runs/nominal/shifted/summary.json"
    nominal = json.loads(nominal_path.read_text(encoding="utf-8"))
    parameters = {key: nominal[key] for key in Case.__dataclass_fields__ if key != "name"}
    base = Case(name="nominal", **parameters)
    if args.probe_only:
        proposed = [row for row in proposed if row["case"] == "slip_0.300"]
    records, summaries = [], []
    anchor = next(row for row in proposed if row["case"] == "slip_0.300")
    anchor_case = replace(base, name="slip_0.300", tangential_speed=0.3)
    measured = force_impulses(anchor_case, "shifted", out)
    anchor_errors = {"normal_impulse_Ns": abs(measured["normal_impulse"] - float(anchor["normal_impulse_Ns"])),
                     "tangential_impulse_Ns": abs(measured["tangent_impulse_magnitude"] - float(anchor["tangential_impulse_Ns"]))}
    print("Force-integral extraction check:", anchor_errors, flush=True)
    if max(anchor_errors.values()) > 1e-8:
        raise RuntimeError("Force integration does not reproduce the archived Proposed impulses")
    for index, previous in enumerate(proposed, 1):
        name = previous["case"]
        speed = base.tangential_speed if name == "nominal" else float(name.split("_", 1)[1])
        case = replace(base, name=name, tangential_speed=speed)
        print(f"[{index}/{len(proposed)}] Velocity-regularized: {name}", flush=True)
        summary = force_impulses(case, "velocity_regularized", out)
        summaries.append(summary)
        if summary["normal_impulse"] <= 0:
            raise RuntimeError(f"Incomplete first-contact episode: {name}")
        normal = float(summary["normal_impulse"])
        tangent = float(summary["tangent_impulse_magnitude"])
        records.append({
            "case": name, "initial_tangential_speed_m_per_s": speed,
            "normal_impulse_Ns": normal,
            "normalized_incident_demand": (2 * case.mass / 7) * abs(speed) / (case.friction * normal),
            "normalized_tangential_impulse": tangent / (case.friction * normal),
            "tangential_impulse_Ns": tangent,
            "proposed_normal_impulse_Ns": float(previous["normal_impulse_Ns"]),
            "normal_impulse_relative_difference": normal / float(previous["normal_impulse_Ns"]) - 1,
        })
    write_csv(out / "panel_b_velocity_regularized_v7.csv", records)
    write_csv(out / "summaries.csv", summaries)
    errors = [abs(row["normalized_tangential_impulse"] - min(row["normalized_incident_demand"], 1))
              for row in records]
    report = {
        "proposed_source": str(source), "proposed_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "nominal_parameters": parameters, "velocity_scale_m_per_s": 1e-4,
        "solver": "AnalyticalTangentForceSolver", "runtime_python": sys.executable,
        "sample_count": len(records), "same_initial_speeds_as_proposed": True,
        "force_integral_anchor_absolute_errors": anchor_errors,
        "impulse_measurement": "World-frame contact force integrated over accepted physical intervals; tangential impulse is the norm of the net vector",
        "normalization": "Each method uses its own measured first-contact normal impulse",
        "max_absolute_normalized_impulse_error": max(errors),
        "max_normal_impulse_relative_difference": max(abs(row["normal_impulse_relative_difference"]) for row in records),
        "probe_only": args.probe_only,
    }
    (out / "velocity_regularized_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
