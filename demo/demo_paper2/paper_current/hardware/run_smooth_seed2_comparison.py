#!/usr/bin/env python3
"""Collect smooth seed2 with the existing controller; compare after download."""
from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-id", default=None)
    parser.add_argument("--repeat", type=int, default=1)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    session_id = args.session_id or time.strftime("%Y%m%d_%H%M%S_smooth_seed2_comparison")
    print("Uses original seed2 control/config/joystick/raw data flow with the smooth policy.", flush=True)
    print("Support robot: startup can StandDown before START; SELECT ends with damping.", flush=True)
    print("After START: 2s default pose, then policy immediately. Neutral sticks command zero velocity.", flush=True)
    print("Robot side only collects data; comparison is performed locally after download.", flush=True)
    print("Use comparable floor, payload and command amplitudes; include stand, forward/backward and turns.", flush=True)
    result = subprocess.run([
        "bash", str(root / "run_smooth_seed2_operator_collection.sh"), str(args.repeat), session_id,
        "flat indoor floor; smooth collection for local comparison",
    ], cwd=root, check=False)
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
