"""Summarize phase-level motion and control variation in Go2 policy JSONL logs."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


def rms(values: np.ndarray) -> float:
    return float(math.sqrt(float(np.mean(np.square(values)))))


def summarize(path: Path) -> dict[str, Any]:
    rows = [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    phase_names = list(dict.fromkeys(str(row.get("phase", "")) for row in rows))
    phases = {}
    for phase in phase_names:
        selected = [row for row in rows if str(row.get("phase", "")) == phase]
        command = np.asarray([row["command"] for row in selected], dtype=float)
        action = np.asarray([row["action"] for row in selected], dtype=float)
        joint_dq = np.asarray([row["joint_dq"] for row in selected], dtype=float)
        target_q = np.asarray([row["target_q"] for row in selected], dtype=float)
        joint_q = np.asarray([row["joint_q"] for row in selected], dtype=float)
        gyro = np.asarray([row["imu"]["gyroscope"] for row in selected], dtype=float)
        rpy = np.asarray([row["imu"]["rpy"] for row in selected], dtype=float)
        motor_lost = np.asarray([row["motor_lost"] for row in selected], dtype=int)
        action_delta = np.diff(action, axis=0)
        action_delta_norm = (
            np.linalg.norm(action_delta, axis=1)
            if len(action_delta)
            else np.asarray([0.0])
        )
        roll_pitch_rate = np.linalg.norm(gyro[:, :2], axis=1)
        phases[phase] = {
            "samples": len(selected),
            "command_mean": np.mean(command, axis=0).tolist(),
            "roll_pitch_std_rad": np.std(rpy[:, :2], axis=0).tolist(),
            "roll_pitch_rate_rms_rad_s": rms(roll_pitch_rate),
            "roll_pitch_rate_p95_rad_s": float(
                np.quantile(roll_pitch_rate, 0.95)
            ),
            "yaw_rate_mean_rad_s": float(np.mean(gyro[:, 2])),
            "yaw_rate_std_rad_s": float(np.std(gyro[:, 2])),
            "joint_velocity_rms_rad_s": rms(joint_dq),
            "joint_tracking_rms_rad": rms(target_q - joint_q),
            "action_delta_norm_rms": rms(action_delta_norm),
            "action_delta_norm_p95": float(np.quantile(action_delta_norm, 0.95)),
            "motor_lost_start": motor_lost[0].tolist(),
            "motor_lost_end": motor_lost[-1].tolist(),
            "motor_lost_max_increment": (
                np.max(motor_lost - motor_lost[0], axis=0).tolist()
            ),
        }
    return {
        "path": str(path),
        "segment_id": rows[0].get("segment_id"),
        "samples": len(rows),
        "phases": phases,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    report = {"trials": [summarize(path) for path in args.logs]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
