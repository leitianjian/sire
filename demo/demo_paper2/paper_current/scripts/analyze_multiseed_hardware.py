#!/usr/bin/env python3
"""Build auditable source data from continuous hardware runs for four policies."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run",
        action="append",
        required=True,
        metavar="SEED=RUN_DIR",
        help="seed number and directory containing policy JSONL and metadata",
    )
    parser.add_argument("--output-dir", required=True, type=Path)
    return parser.parse_args()


def percentile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_jsonl(path: Path) -> tuple[list[dict], int]:
    rows = []
    invalid = 0
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if not line.strip():
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                invalid += 1
    return rows, invalid


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = parse_args()
    runs: dict[int, Path] = {}
    for specification in args.run:
        seed_text, separator, directory_text = specification.partition("=")
        if not separator:
            raise ValueError(f"invalid --run value: {specification}")
        seed = int(seed_text)
        if seed in runs:
            raise ValueError(f"duplicate seed: {seed}")
        runs[seed] = Path(directory_text)
    if sorted(runs) != [1, 2, 3, 4]:
        raise ValueError("exactly seeds 1, 2, 3, and 4 are required")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    cycle_rows: list[dict] = []
    seed_rows: list[dict] = []
    manifest_runs = []
    for seed, run_dir in sorted(runs.items()):
        policy_path = next(
            path for path in run_dir.glob("*.jsonl")
            if not path.name.endswith(".lowstate.jsonl")
        )
        metadata_path = next(run_dir.glob("*.metadata.json"))
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        rows, invalid_lines = load_jsonl(policy_path)
        if len(rows) < 2:
            raise RuntimeError(f"seed {seed} contains fewer than two policy samples")
        start_ns = int(rows[0]["monotonic_ns"])
        duration_s = (int(rows[-1]["monotonic_ns"]) - start_ns) * 1.0e-9
        tilt_deg = []
        compute_ms = []
        inference_ms = []
        for row in rows:
            roll, pitch, yaw = row["imu"]["rpy"]
            gyro_values = [float(value) for value in row["imu"]["gyroscope"]]
            tilt = math.degrees(math.hypot(float(roll), float(pitch)))
            gyro = math.sqrt(sum(value ** 2 for value in gyro_values))
            elapsed_s = (int(row["monotonic_ns"]) - start_ns) * 1.0e-9
            tilt_deg.append(tilt)
            compute_ms.append(float(row["compute_to_send_ms"]))
            inference_ms.append(float(row["inference_ms"]))
            cycle_rows.append(
                {
                    "seed": seed,
                    "monotonic_ns": int(row["monotonic_ns"]),
                    "time_s": elapsed_s,
                    "command_forward": float(row["command"][0]),
                    "command_lateral": float(row["command"][1]),
                    "command_yaw": float(row["command"][2]),
                    "roll_rad": float(roll),
                    "pitch_rad": float(pitch),
                    "yaw_rad": float(yaw),
                    "tilt_deg": tilt,
                    "gyro_x_rad_s": gyro_values[0],
                    "gyro_y_rad_s": gyro_values[1],
                    "gyro_z_rad_s": gyro_values[2],
                    "body_rate_norm_rad_s": gyro,
                    "inference_ms": float(row["inference_ms"]),
                    "compute_to_send_ms": float(row["compute_to_send_ms"]),
                    "schedule_lateness_ms": float(row["schedule_lateness_ms"]),
                    "deadline_miss": int(bool(row["deadline_miss"])),
                    "power_v": float(row["power_v"]),
                    "power_a": float(row["power_a"]),
                    "motor_temperature_c_max": max(row["motor_temperature"]),
                    "motor_lost_max": max(row["motor_lost"]),
                }
            )
        completion = (
            "operator_finished"
            if metadata.get("completed_normally") is True
            else "power_interrupted"
        )
        seed_rows.append(
            {
                "seed": seed,
                "completion": completion,
                "duration_s": duration_s,
                "policy_samples": len(rows),
                "invalid_json_lines": invalid_lines,
                "deadline_misses": sum(bool(row["deadline_miss"]) for row in rows),
                "inference_ms_p95": percentile(inference_ms, 0.95),
                "inference_ms_max": max(inference_ms),
                "compute_to_send_ms_p95": percentile(compute_ms, 0.95),
                "compute_to_send_ms_max": max(compute_ms),
                "tilt_deg_p95": percentile(tilt_deg, 0.95),
                "tilt_deg_max": max(tilt_deg),
                "power_v_min": min(float(row["power_v"]) for row in rows),
                "power_a_max": max(float(row["power_a"]) for row in rows),
                "motor_temperature_c_max": max(
                    max(row["motor_temperature"]) for row in rows
                ),
                "motor_lost_max": max(max(row["motor_lost"]) for row in rows),
            }
        )
        manifest_runs.append(
            {
                "seed": seed,
                "completion": completion,
                "policy_log": str(policy_path.resolve()),
                "policy_log_sha256": sha256(policy_path),
                "metadata": str(metadata_path.resolve()),
                "metadata_sha256": sha256(metadata_path),
                "policy_checkpoint_sha256": metadata.get("policy_sha256"),
                "samples_included": len(rows),
                "invalid_json_lines_excluded": invalid_lines,
                "exclusion_reason": (
                    "one incomplete final JSON line after abrupt power interruption"
                    if invalid_lines else None
                ),
            }
        )

    write_csv(args.output_dir / "hardware_control_cycles.csv", cycle_rows)
    write_csv(args.output_dir / "hardware_seed_summary.csv", seed_rows)
    summary = {
        "schema_version": 2,
        "selection_policy": (
            "all complete policy-cycle JSON records from one continuous v3 run per seed; "
            "no performance-based sample exclusion"
        ),
        "control_deadline_ms": 20.0,
        "total_policy_samples": len(cycle_rows),
        "total_deadline_misses": sum(row["deadline_miss"] for row in cycle_rows),
        "runs": manifest_runs,
        "interpretation": (
            "operator commands are policy inputs; attitude and timing are onboard measurements; "
            "the data do not provide external base-velocity ground truth"
        ),
    }
    (args.output_dir / "hardware_run_manifest.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps({"seed_summary": seed_rows, **summary}, indent=2))


if __name__ == "__main__":
    main()
