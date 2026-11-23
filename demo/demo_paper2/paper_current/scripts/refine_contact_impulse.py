"""Add real sphere-impact samples near the friction-impulse transition.

Replay two archived velocities first and refuse to merge incompatible runs.
All original rows are retained; no point is interpolated or imposed at (1, 1).
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from dataclasses import replace
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
from run_contact_law_experiment import Case, run_case, write_csv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--anchors-only", action="store_true")
    args = parser.parse_args()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    baseline_path = ROOT / "res/source_data/experiment1/panel_c_impulse_v5.csv"
    with baseline_path.open(newline="", encoding="utf-8") as stream:
        original = list(csv.DictReader(stream))
    archived = {row["case"]: row for row in original}
    nominal_path = ROOT / "data/contact_law/runs/nominal/shifted/summary.json"
    nominal = json.loads(nominal_path.read_text(encoding="utf-8"))
    case_fields = {key: nominal[key] for key in Case.__dataclass_fields__ if key != "name"}
    base = Case(name="nominal", **case_fields)
    baseline_sha = hashlib.sha256(baseline_path.read_bytes()).hexdigest()
    records = []
    summaries = []

    def simulate(speed):
        case = replace(base, name=f"corner_{speed:.5f}", tangential_speed=speed)
        print(f"Running {case.name}", flush=True)
        _, summary, _ = run_case(case, "shifted", out)
        summaries.append(summary)
        if not summary.get("separation_found") or summary["normal_impulse"] <= 0:
            raise RuntimeError(f"Invalid contact episode: {case.name}")
        normal = float(summary["normal_impulse"])
        tangent = float(summary["tangent_impulse_magnitude"])
        row = {
            "case": case.name,
            "normal_impulse_Ns": normal,
            "normalized_incident_demand": (2 * case.mass / 7) * abs(speed) / (case.friction * normal),
            "normalized_tangential_impulse": tangent / (case.friction * normal),
            "tangential_impulse_Ns": tangent,
        }
        return row

    anchor_checks = []
    for speed in (0.3, 0.35):
        row = simulate(speed)
        previous = archived[f"slip_{speed:.3f}"]
        errors = {key: abs(float(row[key]) - float(previous[key]))
                  for key in row if key != "case"}
        passed = all(math.isclose(float(row[key]), float(previous[key]), rel_tol=1e-7, abs_tol=1e-9)
                     for key in errors)
        anchor_checks.append({"speed_m_per_s": speed, "absolute_errors": errors, "passed": passed})
        (out / "anchor_checks.json").write_text(json.dumps(anchor_checks, indent=2), encoding="utf-8")
        if not passed:
            raise RuntimeError("Current runtime does not reproduce archived impulses; no merged data written.")
    if args.anchors_only:
        return

    # 0.0025 m/s spacing across the original bracket, refined to 0.0005 m/s
    # around the transition. The original endpoints are validation runs only.
    speeds = sorted({round(0.3 + index * 0.0025, 8) for index in range(1, 20)} |
                    {round(0.327 + index * 0.0005, 8) for index in range(13)})
    for speed in speeds:
        records.append(simulate(speed))
    merged = sorted([*original, *records], key=lambda row: float(row["normalized_incident_demand"]))
    write_csv(out / "panel_c_impulse_refined_v6.csv", merged)
    write_csv(out / "new_samples.csv", records)
    write_csv(out / "summaries.csv", summaries)
    max_error = max(abs(float(row["normalized_tangential_impulse"]) -
                        min(float(row["normalized_incident_demand"]), 1)) for row in merged)
    report = {
        "baseline": str(baseline_path), "baseline_sha256": baseline_sha,
        "nominal_summary_sha256": hashlib.sha256(nominal_path.read_bytes()).hexdigest(),
        "original_count": len(original), "new_sample_count": len(records),
        "merged_count": len(merged), "anchor_checks": anchor_checks,
        "added_tangential_speeds_m_per_s": speeds,
        "max_absolute_normalized_impulse_error": max_error,
        "parameters": case_fields, "python_executable": sys.executable,
        "no_interpolated_or_imposed_points": True,
    }
    (out / "refinement_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
