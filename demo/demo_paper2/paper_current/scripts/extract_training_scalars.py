#!/usr/bin/env python3
"""Export selected TensorBoard scalars from one continued training run."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

from tensorboard.backend.event_processing.event_accumulator import EventAccumulator


TAGS = {
    "mean_reward": "Train/mean_reward",
    "mean_episode_length": "Train/mean_episode_length",
    "total_fps": "Perf/total_fps",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("events", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    by_step: dict[int, dict[str, float | int | str]] = {}
    for event_path in args.events:
        accumulator = EventAccumulator(str(event_path), size_guidance={"scalars": 0})
        accumulator.Reload()
        available = set(accumulator.Tags().get("scalars", []))
        for column, tag in TAGS.items():
            if tag not in available:
                continue
            for event in accumulator.Scalars(tag):
                row = by_step.setdefault(
                    int(event.step),
                    {"iteration": int(event.step), "wall_time": float(event.wall_time)},
                )
                if float(event.wall_time) >= float(row.get("wall_time", 0.0)):
                    row[column] = float(event.value)
                    row["wall_time"] = max(float(row.get("wall_time", 0.0)), float(event.wall_time))
                    row["source"] = event_path.parent.name

    rows = [by_step[step] for step in sorted(by_step)]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = ["iteration", "mean_reward", "mean_episode_length", "total_fps", "wall_time", "source"]
    with args.output.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fieldnames})

    print(f"rows={len(rows)} first={rows[0]['iteration']} last={rows[-1]['iteration']}")
    for column in TAGS:
        values = [row[column] for row in rows if column in row]
        if values:
            print(f"{column}: n={len(values)} first={values[0]:.6g} last={values[-1]:.6g}")


if __name__ == "__main__":
    main()
