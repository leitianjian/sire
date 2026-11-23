"""
Play back a Sire training recording JSON in meshcat.

Usage:
    python test/test_visualize_recording.py
    python test/test_visualize_recording.py --recording logs/rough_go2/exp48/vis/recording_10.json
    python test/test_visualize_recording.py --recording path/to/recording_100.json --speed 0.5
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_TEST_DIR = Path(__file__).resolve().parent
_SIREGYM_DIR = _TEST_DIR.parent
_DEMO_PY = _SIREGYM_DIR.parent
sys.path.insert(0, str(_DEMO_PY))

import sire


def main():
    p = argparse.ArgumentParser(description="Play back a Sire recording JSON in meshcat")
    p.add_argument("--recording", type=str,
                   default=str(_SIREGYM_DIR / "scripts" / "SireRLGym" / "logs" / "rough_go2" / "exp48" / "vis" / "recording_10.json"),
                   help="Path to recording_xxx.json file")
    p.add_argument("--resource_path", type=str, default=None,
                   help="Mesh resource directory (default: auto-detect dogRL/)")
    args = p.parse_args()

    # ---- Load recording ----
    rec_path = Path(args.recording)
    if not rec_path.exists():
        # try relative paths
        for base in [_SIREGYM_DIR, Path.cwd()]:
            p2 = base / args.recording
            if p2.exists():
                rec_path = p2
                break
        else:
            print(f"Recording not found: {args.recording}")
            sys.exit(1)

    print(f"[vis] Loading: {rec_path}")
    with open(rec_path) as f:
        recording = json.load(f)

    nlinks = recording["nlinks"]
    display_init = recording["display_init"]
    print(display_init.keys())
    print({k: v for k, v in display_init["geometry_pool"][-1].items() if k != "heights"})
    frames = recording["frames"]
    n_frames = len(frames.get("timeIndex", []))
    print(f"[vis] Bodies: {nlinks},  Frames: {n_frames}")

    # ---- Synthetic timestamps: 0.001 s per frame ----
    # The recorder's timeIndex is unreliable (timer resets, multiple episodes).
    # Use frame count as ground truth — each frame = one integration step ≈ 0.001 s.
    dt_frame = 0.001
    duration = n_frames * dt_frame
    frames["timeIndex"] = [i * dt_frame for i in range(n_frames)]
    print(f"[vis] duration: {duration:.3f}s  ({n_frames} frames × {dt_frame}s)")

    # ---- Auto-detect resource path ----
    # resource_path = args.resource_path
    # if resource_path is None:
    #     candidate = _DEMO_PY / "sirePaperDogRL"
    #     if candidate.is_dir():
    #         resource_path = str(candidate)
    # if resource_path is None:
    #     candidate = _DEMO_PY / "dogRL"
    #     if candidate.is_dir():
    #         resource_path = str(candidate)
    # if resource_path is None or not Path(resource_path).is_dir():
    #     print(f"[vis] Resource path not found. Provide --resource_path")
    #     sys.exit(1)
    # print(f"[vis] Resource path: {resource_path}")
    resource_path = "D:/code/sire/demo/demo_python/dogRL"

    # ---- Meshcat visualization ----
    try:
        import meshcat
    except ImportError:
        print("[vis] meshcat not installed. Run: pip install meshcat")
        sys.exit(1)

    vis = meshcat.Visualizer()

    # Init robot geometry
    sire.robotInit(nlinks, resource_path, display_init, vis)

    # Play frames
    sire.animateRobotByRecords(nlinks, frames, 10000, vis)

    input("[vis] Press Enter to exit...")


if __name__ == "__main__":
    main()
