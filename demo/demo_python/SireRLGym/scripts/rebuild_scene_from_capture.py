from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[2]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

import numpy as np

from SireRLGym.scene_curriculum.reconstructor import (
    ReconstructionConfig,
    RobustTerrainReconstructor,
    load_depth_capture,
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Rebuild a terrain scene from a saved realsense_capture.npz using the robust reconstructor.")
    parser.add_argument("capture_path", type=str, help="Path to realsense_capture.npz")
    parser.add_argument("--output-dir", type=str, default=None, help="Output directory, defaults to capture file parent")
    parser.add_argument("--resolution", type=float, default=0.03)
    parser.add_argument("--x-min", type=float, default=0.0)
    parser.add_argument("--x-max", type=float, default=1.5)
    parser.add_argument("--y-min", type=float, default=-1.5)
    parser.add_argument("--y-max", type=float, default=1.5)
    parser.add_argument("--obstacle-height-threshold", type=float, default=None)
    parser.add_argument("--focus-corridor-half-width", type=float, default=None)
    parser.add_argument("--mad-reject-m", type=float, default=None)
    parser.add_argument("--bilateral-sigma-r", type=float, default=None)
    parser.add_argument("--min-frames-seen", type=int, default=None)
    parser.add_argument("--no-bilateral", action="store_true", help="Disable bilateral smoothing passes.")
    parser.add_argument("--inspect-dir", type=str, default=None, help="Directory to dump debug previews.")
    return parser.parse_args()


def _build_config(args: argparse.Namespace) -> ReconstructionConfig:
    defaults = ReconstructionConfig()
    overrides: dict[str, float] = dict(
        world_x_min=float(args.x_min),
        world_x_max=float(args.x_max),
        world_y_min=float(args.y_min),
        world_y_max=float(args.y_max),
        resolution=float(args.resolution),
    )
    if args.obstacle_height_threshold is not None:
        overrides["obstacle_height_threshold"] = float(args.obstacle_height_threshold)
    if args.focus_corridor_half_width is not None:
        overrides["focus_corridor_half_width"] = float(args.focus_corridor_half_width)
    if args.mad_reject_m is not None:
        overrides["mad_reject_m"] = float(args.mad_reject_m)
    if args.bilateral_sigma_r is not None:
        overrides["bilateral_sigma_r_m"] = float(args.bilateral_sigma_r)
    if args.min_frames_seen is not None:
        overrides["min_frames_seen"] = int(args.min_frames_seen)
    if args.no_bilateral:
        overrides["bilateral_passes"] = 0
    fields = {f.name: getattr(defaults, f.name) for f in defaults.__dataclass_fields__.values()}
    fields.update(overrides)
    return ReconstructionConfig(**fields)


def _write_inspect(scene, per_cell_seen: np.ndarray, output_dir: Path, cfg: ReconstructionConfig) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output_dir.mkdir(parents=True, exist_ok=True)
    height = scene.height_map
    valid = scene.valid_mask.astype(bool)
    confidence = scene.confidence_map
    extent = (
        cfg.world_x_min,
        cfg.world_x_max,
        cfg.world_y_min,
        cfg.world_y_max,
    )

    vmax = float(height[valid].max()) if valid.any() else cfg.max_height_above_ground
    plt.figure(figsize=(7, 5))
    plt.imshow(height, origin="lower", extent=extent, cmap="terrain", vmin=0.0, vmax=max(vmax, 1e-3))
    plt.colorbar(label="height above ground (m)")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("heightmap")
    plt.tight_layout()
    plt.savefig(output_dir / "heightmap.png", dpi=160)
    plt.close()

    plt.figure(figsize=(7, 5))
    plt.imshow(valid.astype(np.float32), origin="lower", extent=extent, cmap="gray", vmin=0.0, vmax=1.0)
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("valid_mask")
    plt.tight_layout()
    plt.savefig(output_dir / "valid_mask.png", dpi=160)
    plt.close()

    plt.figure(figsize=(7, 5))
    plt.imshow(confidence, origin="lower", extent=extent, cmap="viridis", vmin=0.0, vmax=1.0)
    plt.colorbar(label="confidence")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("confidence_map")
    plt.tight_layout()
    plt.savefig(output_dir / "confidence.png", dpi=160)
    plt.close()

    masked_height = np.where(valid, height, np.nan)
    gx = np.gradient(masked_height, cfg.resolution, axis=1)
    plt.figure(figsize=(7, 5))
    gx_abs_max = float(np.nanmax(np.abs(gx))) if np.any(valid) else 1.0
    gx_abs_max = max(gx_abs_max, 1e-3)
    plt.imshow(gx, origin="lower", extent=extent, cmap="coolwarm", vmin=-gx_abs_max, vmax=gx_abs_max)
    plt.colorbar(label="dz/dx")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("gradient_x")
    plt.tight_layout()
    plt.savefig(output_dir / "gradient_x.png", dpi=160)
    plt.close()

    plt.figure(figsize=(7, 5))
    plt.imshow(per_cell_seen, origin="lower", extent=extent, cmap="magma")
    plt.colorbar(label="n_frames_seen")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("per_cell_frames_seen")
    plt.tight_layout()
    plt.savefig(output_dir / "per_cell_frames_seen.png", dpi=160)
    plt.close()

    valid_heights = height[valid]
    if valid_heights.size:
        hist_counts, hist_edges = np.histogram(valid_heights, bins=min(40, max(8, int(np.sqrt(valid_heights.size)))))
        height_hist = {
            "counts": hist_counts.astype(int).tolist(),
            "bin_edges": hist_edges.astype(float).tolist(),
        }
        h_min = float(valid_heights.min())
        h_max = float(valid_heights.max())
        h_median = float(np.median(valid_heights))
    else:
        height_hist = {"counts": [], "bin_edges": []}
        h_min = h_max = h_median = 0.0

    summary = {
        "valid_cells": int(valid.sum()),
        "total_cells": int(valid.size),
        "valid_height_min": h_min,
        "valid_height_max": h_max,
        "valid_height_median": h_median,
        "valid_height_histogram": height_hist,
        "per_cell_seen_max": int(per_cell_seen.max()) if per_cell_seen.size else 0,
        "per_cell_seen_mean_over_valid": float(per_cell_seen[valid].mean()) if valid.any() else 0.0,
        "metadata": scene.metadata,
        "config": {k: v for k, v in scene.metadata.items() if k.startswith(("mad_", "bilateral_", "min_frames", "spatial_", "lateral_"))},
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")


def _per_cell_frames_seen(frames, cfg: ReconstructionConfig) -> np.ndarray:
    from mujoco_legged_gym.scene_curriculum.reconstructor import _frame_to_world_points, _points_in_bounds

    width_cells = int(np.ceil((cfg.world_x_max - cfg.world_x_min) / cfg.resolution))
    height_cells = int(np.ceil((cfg.world_y_max - cfg.world_y_min) / cfg.resolution))
    seen = np.zeros((height_cells, width_cells), dtype=np.int32)
    for frame in frames:
        pts = _frame_to_world_points(frame)
        mask = _points_in_bounds(pts, cfg)
        pts = pts[mask]
        if pts.shape[0] == 0:
            continue
        cols = np.floor((pts[:, 0] - cfg.world_x_min) / cfg.resolution).astype(np.int64)
        rows = np.floor((pts[:, 1] - cfg.world_y_min) / cfg.resolution).astype(np.int64)
        rows = np.clip(rows, 0, height_cells - 1)
        cols = np.clip(cols, 0, width_cells - 1)
        flat = rows * width_cells + cols
        uniq = np.unique(flat)
        frame_mask = np.zeros(seen.size, dtype=bool)
        frame_mask[uniq] = True
        seen += frame_mask.reshape(height_cells, width_cells).astype(np.int32)
    return seen


def main() -> int:
    args = _parse_args()
    capture_path = Path(args.capture_path).expanduser().resolve()
    if not capture_path.exists():
        raise FileNotFoundError(f"Capture file not found: {capture_path}")

    frames, capture_metadata = load_depth_capture(capture_path)
    cfg = _build_config(args)
    reconstructor = RobustTerrainReconstructor(cfg)
    scene = reconstructor.reconstruct(frames)

    output_dir = Path(args.output_dir).expanduser().resolve() if args.output_dir else capture_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    scene_path = scene.save(output_dir / "reconstructed_scene.npz")

    inspect_dir = Path(args.inspect_dir).expanduser().resolve() if args.inspect_dir else None
    if inspect_dir is not None:
        per_cell_seen = _per_cell_frames_seen(frames, cfg)
        _write_inspect(scene, per_cell_seen, inspect_dir, cfg)

    valid = scene.valid_mask.astype(bool)
    height = scene.height_map
    hmin = float(height[valid].min()) if valid.any() else 0.0
    hmax = float(height[valid].max()) if valid.any() else 0.0
    print(f"frames={len(frames)} capture_source={capture_metadata.get('source', '?')}")
    print(f"terrain_scene={scene_path}")
    print(f"valid_cells={int(valid.sum())}/{valid.size}")
    print(f"height_range=[{hmin:.4f}, {hmax:.4f}] m")
    if inspect_dir is not None:
        print(f"inspect_dir={inspect_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
