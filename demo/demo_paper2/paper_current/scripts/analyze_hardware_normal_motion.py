"""Compare recorded seed-2 locomotion outside identically defined tilt events."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from compare_go2_contact_models import load_session, metrics
from reanalyze_seed2_prefall import attitude_metrics, balance_commands, interval_weights


DEFINITIONS = {
    "tilt_p95_deg": "Time-weighted P95 of sqrt(roll_deg**2 + pitch_deg**2)",
    "command_demeaned_combined_tilt_rms_deg": "RMS of roll/pitch deviations from their separate weighted means within each matched command bin",
    "combined_angular_rate_rms_rad_s": "sqrt(weighted_mean(gyro_x**2 + gyro_y**2))",
    "target_q_rate_rms_rad_s": "RMS of original adjacent-sample joint-target backward differences across all 12 joints",
    "joint_tau_est_rms_Nm": "RMS estimated joint torque across matched time and all 12 joints",
}


def read_recording(directory):
    session = load_session(directory)
    raw_path = directory / Path(session["meta"]["log_path"]).name
    rows = [json.loads(line) for line in raw_path.read_text(encoding="utf-8").splitlines()]
    session["rpy_deg"] = np.rad2deg(np.array([row["imu"]["rpy"][:2] for row in rows]))
    session["values"]["combined_tilt_rms_deg"] = session["values"].pop("roll_pitch_rms_deg") * 2
    session["values"]["combined_angular_rate_rms_rad_s"] = session["values"].pop("roll_pitch_rate_rms_rad_s") * 2
    session["source_path"] = raw_path
    return session


def tilt_events(session, threshold):
    time = session["time_s"]
    above = session["tilt_deg"] >= threshold
    starts = np.flatnonzero(above & ~np.r_[False, above[:-1]])
    ends = np.flatnonzero(above & ~np.r_[above[1:], False]) + 1
    end_of_record = float(time[-1] + session["meta"]["config"]["control_dt"])
    return [{"start_s": float(time[a]), "end_s": float(time[b]) if b < len(time) else end_of_record,
             "source_start_row_1based": int(a + 1), "source_end_row_1based": int(b),
             "max_tilt_deg": float(session["tilt_deg"][a:b].max())}
            for a, b in zip(starts, ends)]


def merged_windows(events, guard):
    windows = []
    for event in events:
        start, end = event["start_s"] - guard, event["end_s"] + guard
        if windows and start <= windows[-1][1]:
            windows[-1][1] = max(windows[-1][1], end)
        else:
            windows.append([start, end])
    return windows


def retained_duration(start, end, windows):
    length = max(0.0, end - start)
    for a, b in windows:
        length -= max(0.0, min(end, b) - max(start, a))
    return max(0.0, length)


def select_intervals(session, cutoff, windows):
    candidate = interval_weights(session, cutoff)
    time = session["time_s"]
    guarded = np.array([retained_duration(float(t), float(t + w), windows) for t, w in zip(time, candidate)])
    # Derivatives were computed on the original consecutive samples. Never
    # difference across a removed gap; retain only valid backward supports.
    derivative_valid = np.zeros(len(time), dtype=bool)
    for i in range(1, len(time)):
        start, end = float(time[i - 1]), float(time[i])
        derivative_valid[i] = retained_duration(start, end, windows) >= end - start - 1e-10
    usable = guarded * derivative_valid
    assert np.all(usable >= 0) and np.all(usable <= candidate + 1e-10)
    return candidate, guarded, derivative_valid, usable


def analyze_pair(sessions, events, cutoff, guard, use_event_mask=True):
    windows = [merged_windows(e, guard) if use_event_mask else [] for e in events]
    selections = [select_intervals(s, cutoff, w) for s, w in zip(sessions, windows)]
    weights, cells = balance_commands(sessions, [selection[3] for selection in selections])
    if not cells:
        raise ValueError("No command bins have at least one second of retained data for both policies")
    values = []
    for session, weight in zip(sessions, weights):
        value = metrics(session, weight)
        value.update(attitude_metrics(session, weight))
        values.append(value)
    exposure = [float(w.sum()) for w in weights]
    assert np.isclose(exposure[0], exposure[1], rtol=0, atol=1e-8)
    result = {
        "end_time_exclusive_s": cutoff, "mode": "matched_commands", "event_mask_applied": use_event_mask,
        "guard_seconds": guard, "excluded_windows_s": windows,
        "sample_counts_before_selection": [len(s["time_s"]) for s in sessions],
        "selected_sample_counts": [int(np.count_nonzero(w)) for w in weights],
        "excluded_sample_counts": [int(len(w) - np.count_nonzero(w)) for w in weights],
        "effective_exposure_s": exposure,
        "candidate_duration_s": [float(x[0].sum()) for x in selections],
        "retained_duration_before_matching_s": [float(x[3].sum()) for x in selections],
        "guard_excluded_duration_s": [float((x[0] - x[1]).sum()) for x in selections],
        "derivative_boundary_excluded_duration_s": [float((x[1] - x[3]).sum()) for x in selections],
        "static": values[0], "smooth": values[1], "command_bins": cells,
        "smooth_relative_change_pct": {k: 100 * (values[1][k] / values[0][k] - 1) if values[0][k] else None for k in DEFINITIONS},
        "proposed_reduction_vs_velocity_regularized_pct": {k: 100 * (1 - values[0][k] / values[1][k]) for k in DEFINITIONS},
    }
    return result, selections, weights


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--proposed", type=Path, required=True)
    parser.add_argument("--velocity-regularized", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--cutoff", type=float, default=40.0)
    parser.add_argument("--tilt-threshold", type=float, default=20.0)
    parser.add_argument("--guard-seconds", type=float, default=2.0)
    args = parser.parse_args()
    if args.guard_seconds < 0 or args.cutoff <= 0 or args.tilt_threshold <= 0:
        raise ValueError("Invalid event-selection parameters")
    sessions = [read_recording(p) for p in (args.proposed, args.velocity_regularized)]
    checks = {k: sessions[0]["meta"].get(k) == sessions[1]["meta"].get(k)
              for k in ("config_sha256", "coverage_sha256", "policy_seed", "checkpoint_iteration")}
    if not all(checks.values()):
        raise ValueError(f"Deployment settings differ: {checks}")
    events = [tilt_events(s, args.tilt_threshold) for s in sessions]
    primary, selections, matched = analyze_pair(sessions, events, args.cutoff, args.guard_seconds)
    sensitivity = [analyze_pair(sessions, events, args.cutoff, g)[0] for g in (1.0, 3.0)]
    unexcluded = analyze_pair(sessions, events, args.cutoff, 0.0, use_event_mask=False)[0]
    report = {
        "schema": "hardware-normal-motion-v1", "sessions": [s["meta"]["session_id"] for s in sessions],
        "configuration_checks": checks, "metric_definitions": DEFINITIONS,
        "source_files": [{"path": str(s["source_path"]), "sha256": hashlib.sha256(s["source_path"].read_bytes()).hexdigest(),
                          "policy_sha256": s["meta"]["policy_sha256"]} for s in sessions],
        "selection": {"candidate_window_s": [0, args.cutoff], "tilt_threshold_deg": args.tilt_threshold,
                      "guard_seconds": args.guard_seconds, "raw_tilt_events": events,
                      "event_end": "First below-threshold sample, or final control-interval endpoint",
                      "rule": "Detect raw unsmoothed tilt events from complete logs; exclude events with equal fixed margins for both methods before command matching.",
                      "derivative_rule": "Keep original adjacent-sample differences; drop samples with a backward difference crossing an exclusion boundary, consistently for all metrics.",
                      "design_status": "Retrospective event-free locomotion analysis; rule specified after observing the records, not preregistered."},
        "matching_rule": "Signed 0.25-wide command bins; deadband 0.05; >=1 s retained coverage in each recording; equalize bin exposure to its smaller duration",
        "primary": primary, "results": [primary], "guard_sensitivity": sensitivity,
        "unexcluded_comparison": unexcluded,
        "quality_full_sessions": [s["quality"] for s in sessions],
        "limitations": ["One manual session per method; no independent repeated hardware trials or significance test.",
                        "Conditional comparison outside tilt events; it does not measure whole-run stability, recovery ability or overturning risk.",
                        "Instantaneous command matching does not equalize command history, achieved speed, external disturbance or surface state.",
                        "Complete records and the recovery/overturning video evidence are preserved separately."],
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "normal_motion_reanalysis.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    with (args.output_dir / "selection_intervals.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["method", "session_id", "source_row_1based", "time_s", "tilt_deg", "candidate_weight_s", "guarded_weight_s", "backward_difference_valid", "prematch_weight_s", "matched_weight_s", "command_bin_x", "command_bin_y", "command_bin_yaw"])
        for method, s, selection, weight in zip(("proposed", "velocity_regularized"), sessions, selections, matched):
            for i, t in enumerate(s["time_s"]):
                writer.writerow([method, s["meta"]["session_id"], i + 1, float(t), float(s["tilt_deg"][i]), float(selection[0][i]), float(selection[1][i]), bool(selection[2][i]), float(selection[3][i]), float(weight[i]), *[int(v) for v in s["cells"][i]]])
    with (args.output_dir / "normal_motion_metrics.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["analysis", "guard_seconds", "metric", "proposed", "velocity_regularized", "proposed_reduction_pct", "matched_s", "bins"])
        for label, result in [("primary", primary), *[("guard_sensitivity", r) for r in sensitivity], ("events_retained", unexcluded)]:
            for k in DEFINITIONS:
                writer.writerow([label, result["guard_seconds"], k, result["static"][k], result["smooth"][k], result["proposed_reduction_vs_velocity_regularized_pct"][k], result["effective_exposure_s"][0], len(result["command_bins"])])
    print(json.dumps({"sessions": report["sessions"], "events": events, "excluded_windows_s": primary["excluded_windows_s"], "matched_s": primary["effective_exposure_s"], "bins": len(primary["command_bins"]), "reductions_pct": primary["proposed_reduction_vs_velocity_regularized_pct"], "sensitivity_reductions": {str(r["guard_seconds"]): r["proposed_reduction_vs_velocity_regularized_pct"] for r in sensitivity}}, indent=2))


if __name__ == "__main__":
    main()
