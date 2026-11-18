#!/usr/bin/env python3
"""Read-only probe for Go2 LowState and SportModeState telemetry.

The probe publishes no command and does not switch robot modes. Run it once
while sport mode is active and once while the low-level deployment is active.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import threading
import time
from typing import Any

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_ as LowStateGo
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_


FOOT_CHANNELS = ("index_0", "index_1", "index_2", "index_3")


def flatten_numeric(value: Any) -> list[float]:
    if isinstance(value, (list, tuple)):
        result: list[float] = []
        for item in value:
            result.extend(flatten_numeric(item))
        return result
    if all(hasattr(value, field) for field in ("x", "y", "z")):
        return [float(value.x), float(value.y), float(value.z)]
    try:
        iterator = iter(value)
    except TypeError:
        return [float(value)]
    result = []
    for item in iterator:
        result.extend(flatten_numeric(item))
    return result


class Probe:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.low_count = 0
        self.sport_count = 0
        self.low_first_ns: int | None = None
        self.low_last_ns: int | None = None
        self.sport_first_ns: int | None = None
        self.sport_last_ns: int | None = None
        self.raw: list[list[float]] = []
        self.estimated: list[list[float]] = []
        self.low_ticks: list[int] = []
        self.sport_samples: list[dict[str, Any]] = []

    def low_handler(self, msg: LowStateGo) -> None:
        now = time.monotonic_ns()
        raw = [float(value) for value in msg.foot_force]
        estimated = [float(value) for value in msg.foot_force_est]
        with self.lock:
            self.low_count += 1
            self.low_first_ns = now if self.low_first_ns is None else self.low_first_ns
            self.low_last_ns = now
            self.raw.append(raw)
            self.estimated.append(estimated)
            self.low_ticks.append(int(msg.tick))

    def sport_handler(self, msg: SportModeState_) -> None:
        now = time.monotonic_ns()
        sample = {
            "position": [float(value) for value in msg.position],
            "velocity": [float(value) for value in msg.velocity],
            "yaw_speed": float(msg.yaw_speed),
            "body_height": float(msg.body_height),
            "foot_force": [int(value) for value in msg.foot_force],
            "foot_position_body": flatten_numeric(msg.foot_position_body),
            "foot_speed_body": flatten_numeric(msg.foot_speed_body),
            "mode": int(msg.mode),
            "gait_type": int(msg.gait_type),
        }
        with self.lock:
            self.sport_count += 1
            self.sport_first_ns = (
                now if self.sport_first_ns is None else self.sport_first_ns
            )
            self.sport_last_ns = now
            self.sport_samples.append(sample)


def summarize_channels(rows: list[list[float]]) -> dict[str, Any]:
    if not rows:
        return {"available": False, "channels": []}
    if any(len(row) != 4 for row in rows):
        return {
            "available": False,
            "error": "foot-force array length is not four",
            "observed_lengths": sorted({len(row) for row in rows}),
        }
    result = []
    for index, name in enumerate(FOOT_CHANNELS):
        values = [row[index] for row in rows]
        nonzero = sum(abs(value) > 1.0e-9 for value in values) / len(values)
        result.append(
            {
                "channel": name,
                "minimum": min(values),
                "maximum": max(values),
                "mean": sum(values) / len(values),
                "nonzero_fraction": nonzero,
                "unique_values": len(set(values)),
                "candidate_usable": max(abs(value) for value in values) > 1.0
                and nonzero > 0.05,
            }
        )
    return {
        "available": True,
        "channels": result,
        "all_four_candidate_usable": all(
            item["candidate_usable"] for item in result
        ),
    }


def summarize_sport_field(
    samples: list[dict[str, Any]],
    field: str,
) -> dict[str, Any]:
    if not samples:
        return {"available": False}
    rows = []
    for sample in samples:
        value = sample[field]
        rows.append(value if isinstance(value, list) else [float(value)])
    lengths = sorted({len(row) for row in rows})
    if len(lengths) != 1:
        return {"available": False, "observed_lengths": lengths}
    components = []
    for index in range(lengths[0]):
        values = [float(row[index]) for row in rows]
        components.append(
            {
                "index": index,
                "minimum": min(values),
                "maximum": max(values),
                "mean": sum(values) / len(values),
                "nonzero_fraction": (
                    sum(abs(value) > 1.0e-9 for value in values) / len(values)
                ),
                "unique_values": len(set(values)),
            }
        )
    return {
        "available": True,
        "component_count": lengths[0],
        "all_zero": all(
            max(abs(item["minimum"]), abs(item["maximum"])) <= 1.0e-9
            for item in components
        ),
        "components": components,
    }


def rate(count: int, first_ns: int | None, last_ns: int | None) -> float:
    if count < 2 or first_ns is None or last_ns is None or last_ns <= first_ns:
        return math.nan
    return (count - 1) / ((last_ns - first_ns) * 1.0e-9)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("net", help="network interface connected to the Go2")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--low-topic", default="rt/lowstate")
    parser.add_argument("--sport-topic", default="rt/sportmodestate")
    parser.add_argument("--mode-label", required=True, help="e.g. sport or lowlevel")
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.duration <= 0.0:
        raise ValueError("--duration must be positive")
    ChannelFactoryInitialize(args.domain, args.net)
    probe = Probe()
    low_subscriber = ChannelSubscriber(args.low_topic, LowStateGo)
    sport_subscriber = ChannelSubscriber(args.sport_topic, SportModeState_)
    low_subscriber.Init(probe.low_handler, 10)
    sport_subscriber.Init(probe.sport_handler, 10)
    print(
        f"Read-only telemetry probe: mode={args.mode_label} "
        f"duration={args.duration}s",
        flush=True,
    )
    time.sleep(args.duration)

    with probe.lock:
        report = {
            "mode_label": args.mode_label,
            "network_interface": args.net,
            "domain": args.domain,
            "duration_s": args.duration,
            "low_topic": args.low_topic,
            "sport_topic": args.sport_topic,
            "low_state": {
                "callbacks": probe.low_count,
                "callback_rate_hz": rate(
                    probe.low_count, probe.low_first_ns, probe.low_last_ns
                ),
                "tick_minimum": min(probe.low_ticks) if probe.low_ticks else None,
                "tick_maximum": max(probe.low_ticks) if probe.low_ticks else None,
                "foot_force_raw": summarize_channels(probe.raw),
                "foot_force_estimated": summarize_channels(probe.estimated),
            },
            "sport_state": {
                "callbacks": probe.sport_count,
                "callback_rate_hz": rate(
                    probe.sport_count, probe.sport_first_ns, probe.sport_last_ns
                ),
                "available": probe.sport_count > 0,
                "position": summarize_sport_field(probe.sport_samples, "position"),
                "velocity": summarize_sport_field(probe.sport_samples, "velocity"),
                "yaw_speed": summarize_sport_field(
                    probe.sport_samples, "yaw_speed"
                ),
                "body_height": summarize_sport_field(
                    probe.sport_samples, "body_height"
                ),
                "foot_force": summarize_sport_field(
                    probe.sport_samples, "foot_force"
                ),
                "foot_position_body": summarize_sport_field(
                    probe.sport_samples, "foot_position_body"
                ),
                "foot_speed_body": summarize_sport_field(
                    probe.sport_samples, "foot_speed_body"
                ),
                "mode_values": sorted(
                    {sample["mode"] for sample in probe.sport_samples}
                ),
                "gait_type_values": sorted(
                    {sample["gait_type"] for sample in probe.sport_samples}
                ),
                "first_samples": probe.sport_samples[:5],
            },
        }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
