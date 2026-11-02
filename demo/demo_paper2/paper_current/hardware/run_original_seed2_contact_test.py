#!/usr/bin/env python3
"""Run the original seed2 collection and summarize its existing foot-force log."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time


BASELINE_POLICY_SHA256 = "8ea73ce49a5be53dbaa9cd1aede4cf9f01821577944fc3385402ae6c01f399df"


class Channels:
    def __init__(self):
        self.count = 0
        self.minimum = [math.inf] * 4
        self.maximum = [-math.inf] * 4
        self.total = [0.0] * 4
        self.zero = [0] * 4

    def update(self, values):
        if len(values) != 4 or not all(math.isfinite(float(v)) for v in values):
            raise ValueError("Invalid foot-force array")
        self.count += 1
        for i, value in enumerate(values):
            value = float(value)
            self.minimum[i] = min(self.minimum[i], value)
            self.maximum[i] = max(self.maximum[i], value)
            self.total[i] += value
            self.zero[i] += value == 0

    def report(self):
        if not self.count:
            return {"samples": 0}
        return {
            "samples": self.count,
            "minimum": self.minimum,
            "maximum": self.maximum,
            "mean": [v / self.count for v in self.total],
            "zero_fraction": [v / self.count for v in self.zero],
            "observed_variation": [lo != hi for lo, hi in zip(self.minimum, self.maximum)],
        }


def summarize(session_dir):
    metadata_paths = list(session_dir.glob("*.metadata.json"))
    if len(metadata_paths) != 1:
        raise RuntimeError(f"Expected exactly one metadata file in {session_dir}")
    metadata_path = metadata_paths[0]
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    # Resolve within the selected session, including when logs were downloaded.
    raw_path = session_dir / Path(metadata["raw_lowstate_log_path"]).name
    raw = Channels()
    estimated = Channels()
    phases = {}
    malformed = 0
    with raw_path.open(encoding="utf-8") as stream:
        for line in stream:
            try:
                sample = json.loads(line)
            except json.JSONDecodeError:
                malformed += 1
                continue
            raw_values = sample["foot_force_raw_indices"]
            estimated_values = sample["foot_force_est_indices"]
            raw.update(raw_values)
            estimated.update(estimated_values)
            phase = str(sample.get("phase", "unknown"))
            phase_channels = phases.setdefault(phase, (Channels(), Channels()))
            phase_channels[0].update(raw_values)
            phase_channels[1].update(estimated_values)
    if not raw.count:
        raise RuntimeError("No raw LowState samples were recorded")
    report = {
        "session_id": metadata["session_id"],
        "policy_path": metadata["policy_path"],
        "policy_sha256": metadata["policy_sha256"],
        "original_seed2_policy": metadata["policy_sha256"] == BASELINE_POLICY_SHA256,
        "duration_s": metadata.get("duration_s"),
        "exit_reason": metadata.get("exit_reason"),
        "recording_integrity_ok": metadata.get("recording_integrity_ok"),
        "raw_lowstate_mean_rate_hz": metadata.get("raw_lowstate_mean_rate_hz"),
        "raw_samples_dropped": metadata.get("raw_lowstate_samples_dropped"),
        "malformed_json_lines": malformed,
        "channel_order": "raw indices 0..3; physical feet not confirmed",
        "units": "device integer readings; not assumed to be newtons",
        "raw": raw.report(),
        "estimated": estimated.report(),
        "by_command_phase": {
            name: {"raw": values[0].report(), "estimated": values[1].report()}
            for name, values in phases.items()
        },
        "foot_contact_valid": False,
        "interpretation": "Signal variation does not establish calibrated contact. All-zero channels remain unknown.",
    }
    output = metadata_path.with_name(metadata_path.name[:-len(".metadata.json")] + ".foot_contact_summary.json")
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print("\nFoot-force summary (indices 0,1,2,3; order unconfirmed):", flush=True)
    print(json.dumps({key: report[key] for key in (
        "session_id", "original_seed2_policy", "duration_s", "recording_integrity_ok",
        "raw_lowstate_mean_rate_hz", "raw", "estimated")}, indent=2), flush=True)
    print(f"Summary saved: {output}", flush=True)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-id", default=None)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--notes", default="original seed2; foot contact diagnostic; flat indoor floor")
    parser.add_argument("--analyze-session", type=Path, help="Only analyze an existing session; no robot I/O")
    args = parser.parse_args()
    if args.analyze_session is not None:
        summarize(args.analyze_session.resolve())
        return 0
    root = Path(__file__).resolve().parent
    launcher = root / "run_operator_collection.sh"
    policy = root / "policies/sire_seeds_1_4_20260924/sire_seed_2_model_1200_jit.pt"
    if not launcher.is_file() or not policy.is_file():
        parser.error("Run this entry from the robot deployment directory containing the original launcher and policies")
    if hashlib.sha256(policy.read_bytes()).hexdigest() != BASELINE_POLICY_SHA256:
        parser.error("Original seed2 policy hash mismatch; refusing to start")
    session_id = args.session_id or time.strftime("%Y%m%d_%H%M%S_original_seed2_contact")
    if args.repeat < 1 or not session_id or not session_id[0].isascii() or not session_id[0].isalnum() or any(
        not c.isascii() or not (c.isalnum() or c in "._-") for c in session_id
    ):
        parser.error("Invalid repeat or session ID")
    session_dir = root / "operator_hardware_logs" / session_id
    if session_dir.exists():
        parser.error(f"Session already exists: {session_dir}")
    print("Original seed2 iteration1200. Existing controller/config/raw logger are reused.", flush=True)
    print("Startup may StandDown/ReleaseMode before START. Support robot before starting.", flush=True)
    print("START -> 2s default-pose transition -> policy/joystick control immediately.", flush=True)
    print("Neutral sticks request zero velocity; the policy still runs. SELECT saves and enters damping.", flush=True)
    print("Raw foot-force data is already recorded; summary follows after SELECT. No second terminal needed.", flush=True)
    result = subprocess.run([
        "bash", str(launcher), "2", str(args.repeat), session_id, args.notes,
    ], cwd=root, check=False)
    try:
        summarize(session_dir)
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        print(f"Summary unavailable: {error}; check session logs: {session_dir}", flush=True)
        return result.returncode or 1
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
