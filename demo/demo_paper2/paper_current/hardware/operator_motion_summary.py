"""Command-direction summaries; all samples and all segments remain usable."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def direction(command, deadband=0.05):
    if len(command) != 3 or not all(math.isfinite(float(v)) for v in command):
        return "invalid_command"
    labels = []
    for value, positive, negative in zip(
        command, ("forward", "left", "ccw"), ("backward", "right", "cw")
    ):
        if abs(value) > deadband:
            labels.append(positive if value > 0 else negative)
    return "+".join(labels) if labels else "stand"


class MotionSummary:
    def __init__(self, config):
        self.config = config
        self.totals = {}
        self.segments = []
        self.current = None
        self.previous_ns = None
        self.gap_s = 0.0

    def observe(self, row):
        now = int(row["monotonic_ns"])
        label = direction(row["command"], self.config["command_deadband"])
        if self.previous_ns is not None:
            dt = (now - self.previous_ns) * 1e-9
            if dt <= 0:
                raise ValueError("timestamps must increase")
            if dt <= self.config["maximum_counted_gap_s"]:
                self.current["duration_s"] += dt
                self.totals[self.current["class"]] += dt
            else:
                self.gap_s += dt
                self.current = None
        if self.current is None or self.current["class"] != label:
            self.current = {
                "class": label, "start_sequence": int(row["sequence"]),
                "end_sequence": int(row["sequence"]),
                "start_monotonic_ns": now, "end_monotonic_ns": now,
                "duration_s": 0.0, "samples": 0,
            }
            self.segments.append(self.current)
            self.totals.setdefault(label, 0.0)
        self.current["end_sequence"] = int(row["sequence"])
        self.current["end_monotonic_ns"] = now
        self.current["samples"] += 1
        self.previous_ns = now
        return {"motion_label": label,
                "motion_segment_elapsed_s": self.current["duration_s"],
                "motion_label_total_s": self.totals[label]}

    def report(self):
        return {
            "command_duration_s": self.totals,
            "underrepresented": [name for name in self.config["suggested_order"]
                                  if self.totals.get(name, 0) < self.config["reminder_total_s"]],
            "reminder_total_s": self.config["reminder_total_s"],
            "uncounted_timestamp_gap_s": self.gap_s,
            "segments": [dict(s, long_command_segment=(
                s["duration_s"] >= self.config["long_segment_s"])) for s in self.segments],
            "interpretation": "command labels only; duration does not establish stable physical motion",
        }


def main():
    import yaml
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--seed", type=int, choices=(1, 2, 3, 4), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    config["suggested_order"] = config["orders"][args.seed]
    tracker = MotionSummary(config)
    count = 0
    invalid_json_lines = 0
    with args.log.open(encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                try:
                    tracker.observe(json.loads(line))
                    count += 1
                except json.JSONDecodeError:
                    invalid_json_lines += 1
    report = dict(
        tracker.report(), samples=count, invalid_json_lines=invalid_json_lines,
        rules=config, source=str(args.log)
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k not in ("segments", "rules")}, indent=2))


if __name__ == "__main__":
    main()
