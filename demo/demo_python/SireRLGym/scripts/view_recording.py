"""
View a saved meshcat recording JSON produced by ``train.py --visualize_interval N``.

Recordings are saved to ``<log_dir>/vis/``
(e.g. ``SireRLGym/logs/rough_go2/exp1/vis/``).

Usage:
    cd D:/code/sire/demo/demo_python/SireRLGym
    python scripts/train.py --task go2 --log_dir logs --visualize_interval 10
    python scripts/view_recording.py logs/rough_go2/exp1/vis/recording_0.json --resource_path ../dogRL
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description="View a saved Sire RL recording in meshcat.")
    p.add_argument('recording', type=str, help='Path to recording_<iter>.json')
    p.add_argument('--resource_path', type=str, required=True,
                   help='Directory with meshcat visual assets (e.g. D:/code/sire/demo_python/dogRL)')
    p.add_argument('--loop', type=int, default=None,
                   help='Number of animation loops (default: infinite, press Ctrl+C to stop)')
    args = p.parse_args()

    recording_path = Path(args.recording)
    if not recording_path.exists():
        print(f"Recording not found: {recording_path}")
        sys.exit(1)

    resource_path = args.resource_path
    if not resource_path or not Path(resource_path).is_dir():
        print(f"[view] Resource path not found: {resource_path}")
        print("[view] Provide --resource_path pointing to the directory with meshcat assets (e.g. dogRL/).")
        sys.exit(1)

    with open(recording_path) as f:
        recording = json.load(f)

    nlinks = recording['nlinks']
    display_init = recording['display_init']
    frames = recording['frames']
    num_frames = len(frames.get('timeIndex', []))

    print(f"[view] Recording: {num_frames} frames, {nlinks} links, resource_path={resource_path}")

    try:
        import sire
        import meshcat
    except ImportError:
        print("[view] Required packages not installed. Run:")
        print("    pip install meshcat")
        print("  and ensure `import sire` works.")
        sys.exit(1)

    vis = meshcat.Visualizer()
    vis.open()

    sire.robotInit(nlinks, resource_path, display_init, vis)

    print(f"\n  Open http://localhost:7000/static/ in your browser")
    print("  Press Ctrl+C to stop.\n")

    try:
        loops = args.loop  # None → infinite
        while loops is None or loops > 0:
            sire.animateRobotByRecords(nlinks, frames, 1000, vis)
            if loops is not None:
                loops -= 1
                if loops <= 0:
                    break
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[view] Stopped.")
    except Exception as e:
        print(f"[view] Error during animation: {e}")


if __name__ == '__main__':
    main()
