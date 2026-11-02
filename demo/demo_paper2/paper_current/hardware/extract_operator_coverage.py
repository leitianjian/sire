"""Extract preregistered coverage windows from an operator Go2 session."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, separators=(",", ":"), allow_nan=False))
            stream.write("\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("metadata", type=Path)
    parser.add_argument("policy_log", type=Path)
    parser.add_argument("--policy-output", required=True, type=Path)
    parser.add_argument("--raw-log", type=Path, default=None)
    parser.add_argument("--raw-output", type=Path, default=None)
    args = parser.parse_args()

    metadata = json.loads(args.metadata.read_text(encoding="utf-8"))
    if metadata.get("eligible_for_analysis") is not True:
        raise RuntimeError("session metadata is not eligible for analysis")
    windows = metadata.get("coverage_windows", [])
    if not windows or metadata.get("missing_classes"):
        raise RuntimeError("coverage windows are incomplete")

    policy_rows = read_jsonl(args.policy_log)
    selected_policy = []
    rows_by_class: dict[str, list[dict[str, Any]]] = {}
    windows_by_class: dict[str, list[dict[str, Any]]] = {}
    for window in windows:
        start = int(window["accepted_start_sequence"])
        end = int(window["accepted_end_sequence"])
        name = str(window["class"])
        occurrence = int(window.get("occurrence", 1))
        rows = [
            dict(
                row,
                coverage_selected_class=name,
                coverage_selected_occurrence=occurrence,
            )
            for row in policy_rows
            if start <= int(row["sequence"]) <= end
        ]
        if not rows:
            raise RuntimeError(f"no policy rows found for coverage class {name}")
        selected_policy.extend(rows)
        rows_by_class.setdefault(name, []).extend(rows)
        windows_by_class.setdefault(name, []).append(window)

    summary_classes = {}
    for name, rows in rows_by_class.items():
        commands = np.asarray([row["command"] for row in rows], dtype=float)
        rpy = np.asarray([row["imu"]["rpy"] for row in rows], dtype=float)
        summary_classes[name] = {
            "window_count": len(windows_by_class[name]),
            "samples": len(rows),
            "duration_s": float(
                sum(
                    window["accepted_duration_s"]
                    for window in windows_by_class[name]
                )
            ),
            "windows": windows_by_class[name],
            "command_mean": np.mean(commands, axis=0).tolist(),
            "command_std": np.std(commands, axis=0).tolist(),
            "roll_pitch_std_rad": np.std(rpy[:, :2], axis=0).tolist(),
        }
    selected_policy.sort(key=lambda row: int(row["sequence"]))
    write_jsonl(args.policy_output, selected_policy)

    raw_selected_count = None
    if args.raw_log is not None or args.raw_output is not None:
        if args.raw_log is None or args.raw_output is None:
            raise ValueError("--raw-log and --raw-output must be supplied together")
        raw_rows = read_jsonl(args.raw_log)
        selected_raw = []
        for window in windows:
            start = int(window["accepted_start_monotonic_ns"])
            end = int(window["accepted_end_monotonic_ns"])
            name = str(window["class"])
            occurrence = int(window.get("occurrence", 1))
            selected_raw.extend(
                dict(
                    row,
                    coverage_selected_class=name,
                    coverage_selected_occurrence=occurrence,
                )
                for row in raw_rows
                if start <= int(row["monotonic_ns"]) <= end
            )
        selected_raw.sort(key=lambda row: int(row["monotonic_ns"]))
        write_jsonl(args.raw_output, selected_raw)
        raw_selected_count = len(selected_raw)

    summary = {
        "metadata": str(args.metadata.resolve()),
        "policy_log": str(args.policy_log.resolve()),
        "policy_output": str(args.policy_output.resolve()),
        "policy_selected_samples": len(selected_policy),
        "raw_selected_samples": raw_selected_count,
        "classes": summary_classes,
    }
    summary_path = args.policy_output.with_suffix(".summary.json")
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
