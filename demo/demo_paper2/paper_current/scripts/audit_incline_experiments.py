"""Rerun the actual four-support incline with center-velocity diagnostics."""
from __future__ import annotations
import argparse
from dataclasses import asdict
import gzip
import json
import math
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
from run_box_sliding_experiment import Case, build_simulation, analytical_state, analytical_metrics, write_csv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    summaries = []
    for mu in (.2, .3):
        for kind in ("shifted_coulomb", "smooth"):
            case = Case(mu, solver_kind=kind, measurement_time=2.)
            print("Running", case.name, flush=True)
            loop, box = build_simulation(case)
            launch_time = None
            angle = math.radians(case.angle_deg)
            uphill = np.array([0., -math.cos(angle), math.sin(angle)])
            while not loop.isTimeout() and not loop.isEventListEmpty():
                now = float(loop.simTime())
                if launch_time is None and now >= case.settle_time - 1e-12:
                    launch_time = now
                    launch_position = np.asarray(box.pq[:3], dtype=float).copy()
                    import sire
                    box.vs = sire.vp2vs(box.pq[:3], (case.initial_speed * uphill).tolist())
                loop.integrate()
                loop.handleContact()
            raw = loop.recordsToJson()
            target = out / f"mu_{mu}"
            target.mkdir(exist_ok=True)
            with gzip.open(target / f"{case.name}_raw_records.json.gz", "wt", encoding="utf-8") as stream:
                json.dump(raw, stream, allow_nan=True)
            rows = []
            for i, time in enumerate(raw["timeIndex"]):
                if raw["partPq"][i] is None or raw["partVs"][i] is None:
                    continue
                t = float(time) - launch_time
                if t < -.5 * case.nominal_dt or t > 2. + 1e-9:
                    continue
                p = np.asarray(raw["partPq"][i][box.id][:3], dtype=float)
                v = np.asarray(raw["partVs"][i][box.id], dtype=float)
                center = v[:3] + np.cross(v[3:], p)
                pair = raw["penetrationPairs"][i] or []
                contacts = raw["contactPairResults"][i] or []
                rows.append({"time_s": max(0., t), "displacement_m": (p-launch_position) @ uphill,
                             "velocity_m_per_s": center @ uphill,
                             "legacy_spatial_velocity_m_per_s": v[:3] @ uphill,
                             "angular_speed_rad_per_s": np.linalg.norm(v[3:]),
                             "active_contacts": len(contacts) if contacts else len(pair)})
            time = np.array([r["time_s"] for r in rows])
            displacement = np.array([r["displacement_m"] for r in rows])
            velocity = np.array([r["velocity_m_per_s"] for r in rows])
            ref_s, ref_v = analytical_state(case, time)
            for row, s, v in zip(rows, ref_s, ref_v):
                row.update(analytical_displacement_m=float(s), analytical_velocity_m_per_s=float(v))
            write_csv(target / f"{case.name}_trajectory.csv", rows)
            reference = analytical_metrics(case)
            turning_time = reference["analytical_stop_time_s"]
            hold = time >= turning_time + .1
            summary = {**asdict(case), **reference, "sample_count": len(rows),
                       "velocity_definition": "center velocity u + omega cross p, projected uphill",
                       "displacement_rmse_m": float(np.sqrt(np.mean((displacement-ref_s)**2))),
                       "velocity_rmse_m_per_s": float(np.sqrt(np.mean((velocity-ref_v)**2))),
                       "max_velocity_extraction_difference_m_per_s": max(abs(r["velocity_m_per_s"]-r["legacy_spatial_velocity_m_per_s"]) for r in rows),
                       "maximum_angular_speed_rad_per_s": max(r["angular_speed_rad_per_s"] for r in rows),
                       "minimum_detected_contacts": min(r["active_contacts"] for r in rows),
                       "maximum_detected_contacts": max(r["active_contacts"] for r in rows),
                       "p95_speed_after_turn_plus_100ms_m_per_s": float(np.percentile(abs(velocity[hold]),95)),
                       "max_uphill_distance_error_m": float(abs(max(displacement)-reference["analytical_stop_distance_m"])),
                       "net_post_stop_drift_m": float(displacement[-1] - np.interp(turning_time,time,displacement)),
                       "last_position_error_m": float(displacement[-1]-ref_s[-1])}
            summaries.append(summary)
            write_csv(out / "incline_audit.csv", summaries)
            print(json.dumps(summary), flush=True)
    (out / "incline_audit.json").write_text(json.dumps(summaries, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
