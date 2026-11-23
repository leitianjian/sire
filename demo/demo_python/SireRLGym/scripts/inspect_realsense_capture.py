from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Inspect a raw realsense_capture.npz by exporting point clouds and preview images.")
    parser.add_argument("capture_npz", type=Path, help="Path to realsense_capture.npz")
    parser.add_argument("--frame-index", type=int, default=None, help="Frame index to inspect. Defaults to the middle frame.")
    parser.add_argument("--pixel-stride", type=int, default=4, help="Subsample every N pixels when exporting/plotting.")
    parser.add_argument("--fuse-frame-stride", type=int, default=6, help="Subsample every N frames when creating fused previews.")
    parser.add_argument("--output-dir", type=Path, default=None, help="Output directory. Defaults to <npz_stem>_inspect next to the npz.")
    return parser.parse_args()


def _load_capture(npz_path: Path):
    payload = np.load(npz_path, allow_pickle=False)
    metadata = json.loads(str(payload["metadata_json"])) if "metadata_json" in payload else {}
    return (
        payload["depth_stack"].astype(np.float32),
        payload["pose_stack"].astype(np.float32),
        payload["timestamps"].astype(np.float32),
        payload["intrinsics"].astype(np.float32),
        metadata,
    )


def _backproject_depth_to_world(depth_m: np.ndarray, intrinsics: np.ndarray, camera_to_world: np.ndarray, pixel_stride: int) -> np.ndarray:
    width, height, fx, fy, cx, cy, depth_min, depth_max = intrinsics.tolist()
    step = max(1, int(pixel_stride))
    sampled = depth_m[::step, ::step]
    v_idx, u_idx = np.nonzero((sampled > depth_min) & (sampled < depth_max))
    if v_idx.size == 0:
        return np.zeros((0, 3), dtype=np.float32)
    z = sampled[v_idx, u_idx]
    u = u_idx.astype(np.float32) * step
    v = v_idx.astype(np.float32) * step
    x = (u - cx) / fx * z
    y = (v - cy) / fy * z
    points_cam = np.stack([x, y, z], axis=1)
    rotation = camera_to_world[:3, :3]
    translation = camera_to_world[:3, 3]
    return points_cam @ rotation.T + translation


def _save_ply(points_world: np.ndarray, path: Path) -> None:
    with path.open("w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {points_world.shape[0]}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for px, py, pz in points_world:
            f.write(f"{px:.6f} {py:.6f} {pz:.6f}\n")


def _save_depth_preview(depth_m: np.ndarray, intrinsics: np.ndarray, path: Path) -> None:
    depth_min = float(intrinsics[6])
    depth_max = float(intrinsics[7])
    valid = (depth_m > depth_min) & (depth_m < depth_max)
    vis = np.zeros((*depth_m.shape, 3), dtype=np.uint8)
    vis[:] = (25, 25, 25)
    if np.any(valid):
        vals = depth_m[valid]
        norm = np.clip((vals - vals.min()) / max(float(vals.max() - vals.min()), 1e-6), 0.0, 1.0)
        colors = np.stack(
            [
                (255 * (1.0 - norm)).astype(np.uint8),
                (200 * (1.0 - np.abs(norm - 0.5) * 2.0)).clip(0, 255).astype(np.uint8),
                (255 * norm).astype(np.uint8),
            ],
            axis=1,
        )
        vis[valid] = colors
    plt.figure(figsize=(8, 6))
    plt.imshow(vis)
    plt.axis("off")
    plt.tight_layout()
    plt.savefig(path, dpi=180, bbox_inches="tight")
    plt.close()


def _save_scatter_projections(points: np.ndarray, prefix: Path) -> None:
    if points.shape[0] == 0:
        return

    z = points[:, 2]
    zmin = float(z.min())
    zmax = float(z.max())

    plt.figure(figsize=(7, 6))
    plt.scatter(points[:, 0], points[:, 1], c=z, s=1, cmap="terrain")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Top view (x-y), colored by z")
    plt.axis("equal")
    plt.colorbar(label="z (m)")
    plt.tight_layout()
    plt.savefig(prefix.with_name(prefix.name + "_xy.png"), dpi=180)
    plt.close()

    plt.figure(figsize=(7, 4.5))
    plt.scatter(points[:, 0], points[:, 2], c=z, s=1, cmap="terrain")
    plt.xlabel("x (m)")
    plt.ylabel("z (m)")
    plt.title("Side view (x-z)")
    plt.colorbar(label="z (m)")
    plt.tight_layout()
    plt.savefig(prefix.with_name(prefix.name + "_xz.png"), dpi=180)
    plt.close()

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=z, s=1, cmap="terrain", depthshade=False)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.set_title("3D scatter")
    ax.view_init(elev=28, azim=-58)
    fig.tight_layout()
    fig.savefig(prefix.with_name(prefix.name + "_3d.png"), dpi=180)
    plt.close(fig)


def main() -> int:
    args = _parse_args()
    capture_npz = args.capture_npz.expanduser().resolve()
    if not capture_npz.exists():
        raise FileNotFoundError(f"Capture npz not found: {capture_npz}")

    output_dir = args.output_dir.expanduser().resolve() if args.output_dir else capture_npz.with_suffix("")
    output_dir = output_dir.parent / f"{capture_npz.stem}_inspect" if args.output_dir is None else output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    depth_stack, pose_stack, timestamps, intrinsics, metadata = _load_capture(capture_npz)
    num_frames = int(depth_stack.shape[0])
    frame_index = args.frame_index if args.frame_index is not None else num_frames // 2
    frame_index = max(0, min(num_frames - 1, int(frame_index)))

    frame_points = _backproject_depth_to_world(
        depth_stack[frame_index],
        intrinsics,
        pose_stack[frame_index],
        pixel_stride=args.pixel_stride,
    )
    _save_ply(frame_points, output_dir / f"frame_{frame_index:04d}_world.ply")
    _save_depth_preview(depth_stack[frame_index], intrinsics, output_dir / f"frame_{frame_index:04d}_depth.png")
    _save_scatter_projections(frame_points, output_dir / f"frame_{frame_index:04d}")

    fused_points = []
    for idx in range(0, num_frames, max(1, int(args.fuse_frame_stride))):
        pts = _backproject_depth_to_world(
            depth_stack[idx],
            intrinsics,
            pose_stack[idx],
            pixel_stride=args.pixel_stride,
        )
        if pts.shape[0] > 0:
            fused_points.append(pts)
    fused_points = np.concatenate(fused_points, axis=0) if fused_points else np.zeros((0, 3), dtype=np.float32)
    _save_ply(fused_points, output_dir / "fused_world.ply")
    _save_scatter_projections(fused_points, output_dir / "fused")

    summary = {
        "capture_npz": str(capture_npz),
        "num_frames": num_frames,
        "frame_index": frame_index,
        "pixel_stride": int(args.pixel_stride),
        "fuse_frame_stride": int(args.fuse_frame_stride),
        "frame_points": int(frame_points.shape[0]),
        "fused_points": int(fused_points.shape[0]),
        "frame_world_min": frame_points.min(axis=0).tolist() if frame_points.size else None,
        "frame_world_max": frame_points.max(axis=0).tolist() if frame_points.size else None,
        "fused_world_min": fused_points.min(axis=0).tolist() if fused_points.size else None,
        "fused_world_max": fused_points.max(axis=0).tolist() if fused_points.size else None,
        "metadata": metadata,
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print(f"inspect_output={output_dir}", flush=True)
    print(f"frame_points={frame_points.shape[0]} fused_points={fused_points.shape[0]}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
