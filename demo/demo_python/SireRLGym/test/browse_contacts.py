"""
Browse ``contactPairResults`` from a recording JSON.

Usage:
    python test/browse_contacts.py SireRLGym/logs/rough_go2/exp31/vis/recording_70.json
    python test/browse_contacts.py SireRLGym/logs/rough_go2/exp31/vis/recording_70.json --parts FL_calf

Prints a frame-by-frame summary of which part pairs are in contact.
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path


def _summarise_all_frames(frames: dict):
    """Print contact pairs for every frame."""
    cpr_list = frames.get('contactPairResults', [])
    time_idx = frames.get('timeIndex', [])
    n_frames = len(cpr_list) if cpr_list else 0

    if n_frames == 0:
        print("No frames with contact data.")
        return

    total_contacts = 0
    for i in range(n_frames):
        t = time_idx[i] if i < len(time_idx) else '?'
        cr = cpr_list[i]
        if not cr:
            continue
        total_contacts += len(cr)
        print(f"[frame {i:4d}  t={t:.6f}] {len(cr):3d} contact(s)")
        for entry in cr:
            # each entry: (pa, pb, fx, fy, fz, px, py, pz)
            pa, pb, fx, fy, fz, px, py, pz = entry
            mag = (fx * fx + fy * fy + fz * fz) ** 0.5
            print(f"          part {pa:2d} ↔ {pb:2d}  |F|={mag:.3f}  pos=({px:.3f}, {py:.3f}, {pz:.3f})")

    print(f"\n{total_contacts} total contact entries across {n_frames} frames")


def _summarise_by_part(frames: dict, target_name: str):
    """Print only frames where a part containing ``target_name`` appears."""
    cpr_list = frames.get('contactPairResults', [])
    time_idx = frames.get('timeIndex', [])
    part_pq = frames.get('partPq', [])
    n_frames = len(cpr_list) if cpr_list else 0

    count = 0
    for i in range(n_frames):
        cr = cpr_list[i]
        if not cr:
            continue
        for entry in cr:
            pa, pb, fx, fy, fz, px, py, pz = entry
            if target_name.lower() not in f"{pa}{pb}":
                continue
            t = time_idx[i] if i < len(time_idx) else '?'
            pq = part_pq[i] if i < len(part_pq) else None
            z_a = pq[pa][2] if pq and pa < len(pq) else '?'
            z_b = pq[pb][2] if pq and pb < len(pq) else '?'

            mag = (fx * fx + fy * fy + fz * fz) ** 0.5
            print(f"[frame {i:4d}  t={t:.6f}] part {pa:2d}(z={z_a}) ↔ {pb:2d}(z={z_b})  |F|={mag:.3f}")
            count += 1

    print(f"\n{count} entries matching '{target_name}' across {n_frames} frames")


def main():
    p = argparse.ArgumentParser(description="Browse contact pair results from a recording JSON")
    p.add_argument('recording', type=str, help='Path to recording_<iter>.json')
    p.add_argument('--parts', type=str, default=None,
                   help='Filter: only show contacts involving a part name (e.g. FL_calf)')
    p.add_argument('--dump', type=str, default=None,
                   help='Dump a per-body contact force summary to a text file')
    args = p.parse_args()

    recording_path = Path(args.recording)
    if not recording_path.exists():
        print(f"Recording not found: {recording_path}")
        sys.exit(1)

    with open(recording_path) as f:
        recording = json.load(f)

    frames = recording.get('frames', {})
    print(f"Loaded: {len(frames.get('timeIndex', []))} frames, "
          f"{len(frames.get('contactPairResults', []))} contact-pair snapshots")
    for contactResult in frames.get('contactPairResults', []):
        print(f"{contactResult}")
    if args.parts is not None:
        _summarise_by_part(frames, args.parts)
    else:
        _summarise_all_frames(frames)


if __name__ == '__main__':
    main()
