#!/usr/bin/env python3
"""Audit whether a Go2 JSONL trial supports timing, tracking, and foot metrics."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
from typing import Any


UNITREE_CHANNELS = ("index_0", "index_1", "index_2", "index_3")


def quantile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    position = probability * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def channel_summary(rows: list[dict[str, Any]], field: str) -> dict[str, Any]:
    channels = [[float(row[field][index]) for row in rows] for index in range(4)]
    summaries = []
    for name, values in zip(UNITREE_CHANNELS, channels):
        nonzero_fraction = sum(abs(value) > 1.0e-9 for value in values) / len(values)
        usable = max(abs(value) for value in values) > 1.0 and nonzero_fraction > 0.05
        summaries.append(
            {
                "channel": name,
                "minimum": min(values),
                "maximum": max(values),
                "mean": statistics.fmean(values),
                "nonzero_fraction": nonzero_fraction,
                "unique_values": len(set(values)),
                "usable": usable,
            }
        )
    return {
        "channels": summaries,
        "all_four_usable": all(item["usable"] for item in summaries),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rows = [
        json.loads(line)
        for line in args.log.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if not rows:
        raise RuntimeError("empty JSONL log")

    required = (
        "command",
        "action",
        "target_q",
        "joint_q",
        "joint_dq",
        "joint_tau_est",
        "imu",
        "compute_to_send_ms",
        "deadline_miss",
        "motor_lost",
        "foot_force_unitree_order",
        "foot_force_est_unitree_order",
    )
    missing = [field for field in required if any(field not in row for row in rows)]
    raw = channel_summary(rows, "foot_force_unitree_order")
    estimated = channel_summary(rows, "foot_force_est_unitree_order")
    sport_valid = [bool(row.get("sport_state", {}).get("valid", False)) for row in rows]
    contact_valid = [bool(row.get("foot_contact_valid", False)) for row in rows]
    compute = [float(row["compute_to_send_ms"]) for row in rows]
    tracking = [
        abs(float(q) - float(target))
        for row in rows
        for q, target in zip(row["joint_q"], row["target_q"])
    ]
    report = {
        "log": str(args.log.resolve()),
        "samples": len(rows),
        "schema_versions": sorted({int(row.get("schema_version", 0)) for row in rows}),
        "missing_required_fields": missing,
        "raw_foot_force": raw,
        "estimated_foot_force": estimated,
        "derived_contact_valid_fraction": statistics.fmean(contact_valid),
        "sport_state_valid_fraction": statistics.fmean(sport_valid),
        "deadline_misses": sum(bool(row["deadline_miss"]) for row in rows),
        "maximum_total_skipped_deadlines": max(
            int(row.get("total_skipped_deadlines", 0)) for row in rows
        ),
        "motor_loss_samples": sum(
            any(int(value) != 0 for value in row["motor_lost"]) for row in rows
        ),
        "compute_to_send_ms": {
            "median": statistics.median(compute),
            "p95": quantile(compute, 0.95),
            "maximum": max(compute),
        },
        "joint_tracking_abs_error_rad": {
            "median": statistics.median(tracking),
            "p95": quantile(tracking, 0.95),
            "maximum": max(tracking),
        },
    }
    report["metric_availability"] = {
        "execution_feasibility": not missing,
        "control_timing": not missing,
        "joint_tracking": not missing,
        "imu_attitude_and_angular_rate": not missing,
        "action_target_and_estimated_torque": not missing,
        "base_velocity_tracking": report["sport_state_valid_fraction"] > 0.95,
        "onboard_displacement_guard": report["sport_state_valid_fraction"] > 0.95,
        "four_foot_contact_timing": (
            raw["all_four_usable"] or estimated["all_four_usable"]
        ),
        "foot_air_time": raw["all_four_usable"] or estimated["all_four_usable"],
        "foot_slip": (
            report["sport_state_valid_fraction"] > 0.95
            and (raw["all_four_usable"] or estimated["all_four_usable"])
        ),
    }
    rendered = json.dumps(report, indent=2, ensure_ascii=False)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
