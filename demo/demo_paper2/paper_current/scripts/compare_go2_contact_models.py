"""Offline, contact-free comparison of two Go2 operator sessions."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


METRICS = {
    "roll_pitch_rms_deg": "RMS roll/pitch (deg)",
    "tilt_p95_deg": "P95 combined roll/pitch magnitude (deg)",
    "roll_pitch_rate_rms_rad_s": "RMS body roll/pitch rates (rad/s)",
    "target_q_rate_rms_rad_s": "RMS joint target change rate (rad/s)",
    "joint_tau_est_rms_Nm": "RMS estimated joint torque (Nm)",
    "absolute_mechanical_power_mean_W": "Mean sum(abs(tau_est*dq)) (W), mechanical proxy",
    "system_electrical_power_mean_W": "Mean measured whole-robot voltage*current (W)",
}


def load_session(directory):
    directory = Path(directory).resolve()
    paths = list(directory.glob("*.metadata.json"))
    if len(paths) != 1:
        raise ValueError(f"Expected one metadata file in {directory}")
    meta = json.loads(paths[0].read_text(encoding="utf-8"))
    log = directory / Path(meta["log_path"]).name
    with log.open(encoding="utf-8") as stream:
        rows = [json.loads(line) for line in stream]
    if len(rows) < 2:
        raise ValueError("Insufficient policy samples")
    t = np.array([r["monotonic_ns"] for r in rows], dtype=float) * 1e-9
    t -= t[0]
    nominal_dt = float(meta["config"]["control_dt"])
    dt = np.diff(t)
    if not np.all(dt > 0):
        raise ValueError("Non-monotonic timestamps")
    # Attribute each logged state to the following control interval. Long gaps
    # are retained in quality reports but not treated as repeated measurements.
    weights = np.minimum(np.r_[dt, nominal_dt], 2 * nominal_dt)
    rpy = np.array([r["imu"]["rpy"][:2] for r in rows])
    gyro = np.array([r["imu"]["gyroscope"][:2] for r in rows])
    target = np.array([r["target_q"] for r in rows])
    tau = np.array([r["joint_tau_est"] for r in rows])
    dq = np.array([r["joint_dq"] for r in rows])
    target_rate = np.full(len(rows), np.nan)
    target_rate[1:] = np.mean((np.diff(target, axis=0) / dt[:, None]) ** 2, axis=1)
    command = np.array([r["command"] for r in rows])
    # Match sign and 0.25-wide magnitude bins independently for x, y and yaw.
    bins = np.where(abs(command) <= .05, 0,
                    np.sign(command) * np.ceil(np.minimum(abs(command), 1) * 4)).astype(int)
    cells = [tuple(v) for v in bins]
    values = {
        "roll_pitch_rms_deg": np.mean(np.rad2deg(rpy) ** 2, axis=1),
        "tilt_p95_deg": np.linalg.norm(np.rad2deg(rpy), axis=1),
        "roll_pitch_rate_rms_rad_s": np.mean(gyro ** 2, axis=1),
        "target_q_rate_rms_rad_s": target_rate,
        "joint_tau_est_rms_Nm": np.mean(tau ** 2, axis=1),
        "absolute_mechanical_power_mean_W": np.sum(abs(tau * dq), axis=1),
        "system_electrical_power_mean_W": np.array([r["power_v"] * r["power_a"] for r in rows]),
    }
    if not all(np.isfinite(v).all() for k, v in values.items() if k != "target_q_rate_rms_rad_s"):
        raise ValueError("Non-finite measurements")
    quality = {
        "samples": len(rows), "metadata_count_matches": len(rows) == meta.get("samples_written"),
        "duration_s": meta.get("duration_s"), "status": meta.get("status"),
        "exit_reason": meta.get("exit_reason"),
        "recording_integrity_ok": meta.get("recording_integrity_ok"),
        "policy_samples_dropped": meta.get("samples_dropped"),
        "raw_samples_dropped": meta.get("raw_lowstate_samples_dropped"),
        "deadline_miss_fraction": float(np.mean([r["deadline_miss"] for r in rows])),
        "skipped_deadlines": meta.get("session_total_skipped_deadlines"),
        "voltage_range_V": [min(r["power_v"] for r in rows), max(r["power_v"] for r in rows)],
        "motor_temperature_range_C": [min(min(r["motor_temperature"]) for r in rows),
                                      max(max(r["motor_temperature"]) for r in rows)],
    }
    return {"meta": meta, "weights": weights, "cells": cells, "values": values,
            "time_s": t, "tilt_deg": np.linalg.norm(np.rad2deg(rpy), axis=1),
            "quality": quality}


def metrics(session, weights):
    result = {}
    for name, values in session["values"].items():
        valid = np.isfinite(values) & (weights > 0)
        if not valid.any():
            result[name] = None
            continue
        v, w = values[valid], weights[valid]
        if name == "tilt_p95_deg":
            order = np.argsort(v)
            index = min(int(np.searchsorted(np.cumsum(w[order]), .95 * w.sum())), len(v) - 1)
            value = v[order[index]]
        else:
            value = np.average(v, weights=w)
            if "_rms_" in name:
                value = np.sqrt(value)
        result[name] = float(value)
    return result


def summary(session):
    meta = session["meta"]
    return {"session_id": meta["session_id"], "policy_sha256": meta["policy_sha256"],
            "config_sha256": meta["config_sha256"], "quality": session["quality"],
            "full_session_metrics": metrics(session, session["weights"])}


def orientation_event(session):
    tilt = session["tilt_deg"]
    time_s = session["time_s"]
    result = {"max_combined_roll_pitch_deg": float(np.max(tilt)),
              "fall_threshold_deg": 20.0}
    for threshold in (10.0, 20.0, 45.0, 90.0, 135.0):
        indices = np.flatnonzero(tilt >= threshold)
        result[f"first_tilt_ge_{int(threshold)}_deg_s"] = (
            float(time_s[indices[0]]) if len(indices) else None
        )
    indices = np.flatnonzero(tilt >= 20.0)
    result["fall_detected"] = bool(len(indices))
    result["first_fall_like_tilt_s"] = float(time_s[indices[0]]) if len(indices) else None
    return result


def compare(baseline_dir, smooth_dir, output):
    baseline, smooth = load_session(baseline_dir), load_session(smooth_dir)
    meta0, meta1 = baseline["meta"], smooth["meta"]
    comparable = {name: meta0.get(name) == meta1.get(name) for name in (
        "config_sha256", "coverage_sha256", "policy_seed", "checkpoint_iteration",
        "command_source", "motor_lost_abort_increment")}
    full = {"baseline_static": summary(baseline), "smooth": summary(smooth)}
    full["baseline_static"]["orientation_event"] = orientation_event(baseline)
    full["smooth"]["orientation_event"] = orientation_event(smooth)
    matched_weights = [np.zeros_like(s["weights"]) for s in (baseline, smooth)]
    cells = []
    common = sorted(set(baseline["cells"]) & set(smooth["cells"]))
    for cell in common:
        masks = [np.array([c == cell for c in s["cells"]]) for s in (baseline, smooth)]
        durations = [float(s["weights"][mask].sum()) for s, mask in zip((baseline, smooth), masks)]
        # Require >=1 second in each command cell; keep all data in full-session results.
        if min(durations) < 1.0:
            continue
        shared = min(durations)
        for i, (s, mask) in enumerate(zip((baseline, smooth), masks)):
            matched_weights[i][mask] = s["weights"][mask] * shared / durations[i]
        cells.append({"signed_command_bins_xyz_yaw": [int(value) for value in cell],
                      "baseline_duration_s": durations[0], "smooth_duration_s": durations[1],
                      "common_weight_s": shared})
    matched = {
        "baseline_static": metrics(baseline, matched_weights[0]),
        "smooth": metrics(smooth, matched_weights[1]),
        "common_weight_s_per_session": float(matched_weights[0].sum()),
        "coverage_fraction": [float(w.sum() / s["weights"].sum())
                              for w, s in zip(matched_weights, (baseline, smooth))],
        "cells": cells,
    }
    notes = [
        "Exploratory comparison of one seed and manual sessions; no significance test or general superiority claim.",
        "Full-session metrics reflect different command mixtures. Matched metrics balance instantaneous command bins only, not trajectory, transition history or actual distance.",
        "Lower effort or smoother actions alone may indicate weaker motion. Actual translational velocity/distance is unavailable, so no tracking-error, efficiency/COT, or equal-task-performance claim.",
        "No trusted contact labels: do not infer air time, stance slip, contact duty factor or static-friction realization.",
        "All recorded samples remain in full-session results, including startup/transients; motor target change rate has no value for the first sample.",
        "Mechanical power is an estimate from joint tau_est*dq. Electrical power includes the whole robot and depends on battery and peripherals.",
    ]
    report = {"metric_definitions": METRICS, "configuration_matches": comparable,
              "full_sessions": full, "command_matched": matched, "limitations": notes}
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    lines = ["# Original static-friction seed2 vs smooth seed2", "",
             "Exploratory manual-run comparison. See JSON for coverage, quality and limitations.", "",
             f"Deployment configuration checks: {comparable}", "",
             f"Matched command exposure per session: {matched['common_weight_s_per_session']:.2f} s", "",
             "| Metric | Static, full | Smooth, full | Static, matched | Smooth, matched |",
             "|---|---:|---:|---:|---:|"]
    def fmt(value):
        return "unavailable" if value is None else f"{value:.4f}"
    for name, description in METRICS.items():
        vals = [full["baseline_static"]["full_session_metrics"][name], full["smooth"]["full_session_metrics"][name],
                matched["baseline_static"][name], matched["smooth"][name]]
        lines.append("| " + " | ".join([description] + [fmt(v) for v in vals]) + " |")
    lines += ["", "All comparison limitations:", ""] + ["- " + n for n in notes]
    output.with_suffix(".md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines[:len(METRICS) + 11]), flush=True)
    print(f"Comparison saved: {output}", flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument("--smooth", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.smooth is None:
        report = summary(load_session(args.baseline))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
        print(json.dumps(report, indent=2), flush=True)
    else:
        compare(args.baseline, args.smooth, args.output)


if __name__ == "__main__":
    main()
