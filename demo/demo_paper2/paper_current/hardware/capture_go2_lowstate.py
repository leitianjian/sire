#!/usr/bin/env python3
"""Capture raw Go2 LowState for offline kinematic-estimator validation.

This diagnostic publishes no command and does not switch robot modes.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np

import deploy_go2_paper as paper
from deploy_go2_segmented import RawLowStateRecorder
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_ as LowStateGo


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("net", help="network interface connected to the Go2")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--low-topic", default="rt/lowstate")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--queue-size", type=int, default=32768)
    parser.add_argument("--flush-every", type=int, default=250)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.duration <= 0.0:
        raise ValueError("--duration must be positive")
    output = args.output.expanduser().resolve()
    if output.exists():
        raise FileExistsError(output)

    sink = paper.AsyncJsonlLogger(output, args.queue_size, args.flush_every)
    recorder = RawLowStateRecorder(sink)
    start_ns = time.monotonic_ns()
    recorder.start(
        protocol_id="read_only_diagnostic",
        segment_id="stationary_lowstate_capture",
        repeat=0,
        policy_seed=0,
        checkpoint_iteration=0,
        trial_start_ns=start_ns,
    )
    recorder.update_context(
        phase="read_only_capture",
        phase_index=0,
        planned_normalized_command=np.zeros(3, dtype=np.float32),
    )

    ChannelFactoryInitialize(args.domain, args.net)
    subscriber = ChannelSubscriber(args.low_topic, LowStateGo)

    def handler(message: LowStateGo) -> None:
        recorder.submit(message, time.monotonic_ns(), time.time_ns())

    subscriber.Init(handler, 10)
    try:
        time.sleep(args.duration)
    finally:
        recorder.stop()
        sink.close()

    elapsed_s = (time.monotonic_ns() - start_ns) * 1.0e-9
    summary = {
        "output": str(output),
        "duration_s": elapsed_s,
        "samples_written": sink.written,
        "samples_dropped": sink.dropped,
        "mean_rate_hz": sink.written / elapsed_s,
        "read_only": True,
    }
    print(json.dumps(summary, indent=2))
    if sink.written < 2:
        raise RuntimeError("fewer than two LowState samples were captured")
    if sink.dropped:
        raise RuntimeError("raw LowState samples were dropped")


if __name__ == "__main__":
    main()
