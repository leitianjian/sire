from __future__ import annotations

"""
场景重建模块 (Scene Reconstructor)。
该文件主要用于处理传感器（如深度相机）输入的数据，将其转换为 3D 高度图 (Height map) 以及地形场景 (TerrainScene)。
在现实世界中训练或部署强化学习智能体时，机器人需要理解周边的物理环境；
Reconstructor 就是用来把原始像素数据还原成便于强化学习处理的空间物理结构。
"""

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import numpy as np
from scipy.ndimage import binary_closing, distance_transform_edt, label

from .types import TerrainScene


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    depth_min: float
    depth_max: float


@dataclass(frozen=True)
class DepthFrame:
    depth_m: np.ndarray
    intrinsics: CameraIntrinsics
    camera_to_world: np.ndarray
    timestamp_s: float = 0.0
    frame_id: str = ""


@dataclass(frozen=True)
class ReconstructionConfig:
    world_x_min: float = 0.0
    world_x_max: float = 1.5
    world_y_min: float = -1.5
    world_y_max: float = 1.5
    resolution: float = 0.03
    z_clip_min: float = -0.20
    z_clip_max: float = 0.60
    max_height_above_ground: float = 0.40
    obstacle_height_threshold: float = 0.05
    focus_corridor_half_width: float = 0.35
    # Robustness
    min_frames_seen: int = 2
    mad_reject_m: float = 0.035
    spatial_outlier_m: float = 0.06
    strong_support: int = 4
    # Bilateral
    bilateral_sigma_s_cells: float = 1.5
    bilateral_sigma_r_m: float = 0.02
    bilateral_passes: int = 2
    bilateral_window: int = 5
    # Ground RANSAC
    ground_ransac_iters: int = 200
    ground_inlier_m: float = 0.015
    ground_low_quartile: float = 0.25
    ground_max_slope: float = 0.15
    # Corridor keep
    lateral_keep_m: float = 0.25
    # Shadow fill (geometric occlusion reasoning from single-view capture)
    shadow_fill_enabled: bool = True
    shadow_fill_samples: int = 48
    shadow_fill_min_occluder_height: float = 0.03
    # Fill holes
    hole_closing_radius: int = 1


class RobustTerrainReconstructor:
    def __init__(self, config: ReconstructionConfig):
        self.config = config

    def reconstruct(self, frames: list[DepthFrame]) -> TerrainScene:
        cfg = self.config
        width_cells = int(np.ceil((cfg.world_x_max - cfg.world_x_min) / cfg.resolution))
        height_cells = int(np.ceil((cfg.world_y_max - cfg.world_y_min) / cfg.resolution))
        frame_count = len(frames)

        per_frame = np.full((frame_count, height_cells, width_cells), np.nan, dtype=np.float32)
        frame_point_counts: list[int] = []
        for i, frame in enumerate(frames):
            pts_world = _frame_to_world_points(frame)
            kept = _points_in_bounds(pts_world, cfg)
            pts_world = pts_world[kept]
            frame_point_counts.append(int(pts_world.shape[0]))
            if pts_world.shape[0] == 0:
                continue
            cols = np.floor((pts_world[:, 0] - cfg.world_x_min) / cfg.resolution).astype(np.int64)
            rows = np.floor((pts_world[:, 1] - cfg.world_y_min) / cfg.resolution).astype(np.int64)
            rows = np.clip(rows, 0, height_cells - 1)
            cols = np.clip(cols, 0, width_cells - 1)
            per_frame[i] = _per_frame_cell_median(rows, cols, pts_world[:, 2].astype(np.float32), height_cells, width_cells)

        valid_per_cell = ~np.isnan(per_frame)
        n_frames_seen = valid_per_cell.sum(axis=0).astype(np.int32)
        with np.errstate(invalid="ignore", all="ignore"):
            z_agg = np.nanmedian(per_frame, axis=0)
            mad = np.nanmedian(np.abs(per_frame - z_agg[None]), axis=0)
        z_agg = np.where(np.isfinite(z_agg), z_agg, 0.0).astype(np.float32)
        valid = (n_frames_seen >= cfg.min_frames_seen)
        if cfg.mad_reject_m > 0:
            mad_ok = ~np.isfinite(mad) | (mad <= cfg.mad_reject_m)
            valid &= mad_ok

        z_for_spatial = np.where(valid, z_agg, np.nan).astype(np.float32)
        local_med = _nanmedian_filter(z_for_spatial, size=3)
        dev = np.abs(z_for_spatial - local_med)
        spatial_outlier = np.isfinite(dev) & (dev > cfg.spatial_outlier_m) & (n_frames_seen < cfg.strong_support)
        valid &= ~spatial_outlier
        z_agg = np.where(valid, z_agg, 0.0).astype(np.float32)

        z_smooth = z_agg
        for _ in range(int(cfg.bilateral_passes)):
            z_smooth = _bilateral_filter(
                z_smooth,
                valid,
                window=int(cfg.bilateral_window),
                sigma_s=float(cfg.bilateral_sigma_s_cells),
                sigma_r=float(cfg.bilateral_sigma_r_m),
            )

        plane_coeffs, ground_at_origin = _fit_ground_plane(z_smooth, valid, cfg)
        x_coords = cfg.world_x_min + (np.arange(width_cells, dtype=np.float64) + 0.5) * cfg.resolution
        y_coords = cfg.world_y_min + (np.arange(height_cells, dtype=np.float64) + 0.5) * cfg.resolution
        plane_z = (
            plane_coeffs[0] * x_coords[None, :]
            + plane_coeffs[1] * y_coords[:, None]
            + plane_coeffs[2]
        ).astype(np.float32)
        z_rel = np.clip(z_smooth - plane_z, 0.0, cfg.max_height_above_ground).astype(np.float32)

        corridor_center = np.abs(y_coords)[:, None]
        corridor_band = (corridor_center <= float(cfg.focus_corridor_half_width))
        extended_band = (corridor_center <= float(cfg.focus_corridor_half_width + cfg.lateral_keep_m))
        corridor_band = np.broadcast_to(corridor_band, valid.shape)
        extended_band = np.broadcast_to(extended_band, valid.shape)
        in_corridor = valid & corridor_band
        outside_candidates = valid & ~corridor_band & extended_band
        all_candidates = in_corridor | outside_candidates
        structure8 = np.ones((3, 3), dtype=bool)
        if np.any(all_candidates):
            labels, _ = label(all_candidates, structure=structure8)
            touching = np.unique(labels[in_corridor])
            touching = touching[touching > 0]
            valid = np.isin(labels, touching) if touching.size > 0 else np.zeros_like(valid)
        else:
            valid = np.zeros_like(valid)

        shadow_stats: dict[str, Any] = {
            "enabled": bool(cfg.shadow_fill_enabled),
            "shadow_filled_cells": 0,
            "fov_outside_cells": 0,
            "candidate_cells": 0,
        }
        if cfg.shadow_fill_enabled and frames and np.any(valid):
            shadow_z, shadow_mask, shadow_stats = _compute_shadow_fill(
                z_rel=z_rel,
                valid=valid,
                extended_band=np.asarray(extended_band, dtype=bool),
                cfg=cfg,
                frame=frames[0],
                x_coords=x_coords,
                y_coords=y_coords,
                ground_at_origin=float(ground_at_origin),
                plane_z=plane_z,
            )
            if np.any(shadow_mask):
                z_rel = np.where(shadow_mask, shadow_z, z_rel).astype(np.float32)
                valid = valid | shadow_mask

        if cfg.hole_closing_radius > 0 and np.any(valid):
            struct = np.ones((2 * int(cfg.hole_closing_radius) + 1,) * 2, dtype=bool)
            closed = binary_closing(valid, structure=struct)
            newly_valid = closed & ~valid
            if np.any(newly_valid):
                _, indices = distance_transform_edt(~valid, return_indices=True)
                src_rows = indices[0][newly_valid]
                src_cols = indices[1][newly_valid]
                z_rel[newly_valid] = z_rel[src_rows, src_cols]
            valid = closed

        height_map = np.where(valid, z_rel, 0.0).astype(np.float32)

        max_frames_seen = max(1, int(n_frames_seen.max()))
        mad_ref = max(float(cfg.mad_reject_m), 1e-4)
        mad_clean = np.where(np.isfinite(mad), mad, 0.0).astype(np.float32)
        confidence = np.clip(
            (np.maximum(n_frames_seen.astype(np.float32) - 1.0, 0.0) / max(max_frames_seen - 1, 1))
            * np.exp(-mad_clean / mad_ref),
            0.0,
            1.0,
        ).astype(np.float32)
        confidence *= valid.astype(np.float32)

        finite_mad_vals = mad[np.isfinite(mad)]
        robust_stats = {
            "median_mad": float(np.median(finite_mad_vals)) if finite_mad_vals.size else 0.0,
            "cells_above_threshold": int(np.sum(valid & (height_map >= cfg.obstacle_height_threshold))),
            "valid_cells": int(valid.sum()),
            "max_frames_seen": int(max_frames_seen),
        }
        metadata: dict[str, Any] = {
            "num_frames": int(frame_count),
            "valid_frames": int(sum(1 for n in frame_point_counts if n > 0)),
            "frame_point_counts": frame_point_counts,
            "world_bounds": {
                "x_min": float(cfg.world_x_min),
                "x_max": float(cfg.world_x_max),
                "y_min": float(cfg.world_y_min),
                "y_max": float(cfg.world_y_max),
            },
            "resolution": float(cfg.resolution),
            "ground_height_offset": float(ground_at_origin),
            "ground_plane_coeffs": [float(plane_coeffs[0]), float(plane_coeffs[1]), float(plane_coeffs[2])],
            "max_height_above_ground": float(cfg.max_height_above_ground),
            "obstacle_height_threshold": float(cfg.obstacle_height_threshold),
            "focus_corridor_half_width": float(cfg.focus_corridor_half_width),
            "apply_ground_alignment": True,
            "reconstruction_mode": "robust_dense_v1",
            "mad_reject_m": float(cfg.mad_reject_m),
            "min_frames_seen": int(cfg.min_frames_seen),
            "bilateral_sigma_s_cells": float(cfg.bilateral_sigma_s_cells),
            "bilateral_sigma_r_m": float(cfg.bilateral_sigma_r_m),
            "bilateral_passes": int(cfg.bilateral_passes),
            "spatial_outlier_m": float(cfg.spatial_outlier_m),
            "lateral_keep_m": float(cfg.lateral_keep_m),
            "shadow_fill": shadow_stats,
            "robust_stats": robust_stats,
        }

        return TerrainScene(
            height_map=height_map,
            valid_mask=valid.astype(bool),
            confidence_map=confidence,
            resolution=float(cfg.resolution),
            origin_xy=(float(cfg.world_x_min), float(cfg.world_y_min)),
            metadata=metadata,
        )


def _frame_to_world_points(frame: DepthFrame) -> np.ndarray:
    intr = frame.intrinsics
    depth = frame.depth_m.astype(np.float32)
    valid = (depth > intr.depth_min) & (depth < intr.depth_max)
    if not np.any(valid):
        return np.zeros((0, 3), dtype=np.float32)
    v_idx, u_idx = np.nonzero(valid)
    z = depth[v_idx, u_idx]
    x = (u_idx.astype(np.float32) - intr.cx) / intr.fx * z
    y = (v_idx.astype(np.float32) - intr.cy) / intr.fy * z
    pts_cam = np.stack([x, y, z], axis=1)
    R = frame.camera_to_world[:3, :3].astype(np.float32)
    t = frame.camera_to_world[:3, 3].astype(np.float32)
    return pts_cam @ R.T + t


def _points_in_bounds(pts: np.ndarray, cfg: ReconstructionConfig) -> np.ndarray:
    if pts.shape[0] == 0:
        return np.zeros((0,), dtype=bool)
    return (
        (pts[:, 0] >= cfg.world_x_min)
        & (pts[:, 0] < cfg.world_x_max)
        & (pts[:, 1] >= cfg.world_y_min)
        & (pts[:, 1] < cfg.world_y_max)
        & (pts[:, 2] >= cfg.z_clip_min)
        & (pts[:, 2] <= cfg.z_clip_max)
    )


def _per_frame_cell_median(rows: np.ndarray, cols: np.ndarray, vals: np.ndarray, H: int, W: int) -> np.ndarray:
    out = np.full((H, W), np.nan, dtype=np.float32)
    if rows.size == 0:
        return out
    flat = rows.astype(np.int64) * np.int64(W) + cols.astype(np.int64)
    order = np.argsort(flat, kind="stable")
    flat_sorted = flat[order]
    vals_sorted = vals[order]
    uniq, first_idx = np.unique(flat_sorted, return_index=True)
    groups = np.split(vals_sorted, first_idx[1:])
    medians = np.fromiter((float(np.median(g)) for g in groups), dtype=np.float32, count=uniq.size)
    out.reshape(-1)[uniq] = medians
    return out


def _nanmedian_filter(arr: np.ndarray, size: int = 3) -> np.ndarray:
    pad = size // 2
    padded = np.pad(arr, pad, mode="constant", constant_values=np.nan)
    H, W = arr.shape
    neigh = np.stack(
        [padded[i : i + H, j : j + W] for i in range(size) for j in range(size)],
        axis=0,
    )
    with np.errstate(invalid="ignore", all="ignore"):
        return np.nanmedian(neigh, axis=0).astype(np.float32)


def _bilateral_filter(
    z: np.ndarray,
    valid: np.ndarray,
    *,
    window: int,
    sigma_s: float,
    sigma_r: float,
) -> np.ndarray:
    pad = window // 2
    z_padded = np.pad(z, pad, mode="edge").astype(np.float32)
    v_padded = np.pad(valid.astype(np.float32), pad, mode="constant", constant_values=0.0)
    H, W = z.shape
    num = np.zeros((H, W), dtype=np.float32)
    den = np.zeros((H, W), dtype=np.float32)
    two_s2 = 2.0 * max(sigma_s, 1e-3) ** 2
    two_r2 = 2.0 * max(sigma_r, 1e-4) ** 2
    for i in range(window):
        for j in range(window):
            di = float(i - pad)
            dj = float(j - pad)
            spatial = float(np.exp(-(di * di + dj * dj) / two_s2))
            z_shift = z_padded[i : i + H, j : j + W]
            v_shift = v_padded[i : i + H, j : j + W]
            range_w = np.exp(-((z - z_shift) ** 2) / two_r2)
            w = (spatial * range_w * v_shift).astype(np.float32)
            num += w * z_shift
            den += w
    with np.errstate(invalid="ignore"):
        out = np.where(den > 0, num / np.maximum(den, 1e-8), z)
    return np.where(valid, out, z).astype(np.float32)


def _compute_shadow_fill(
    *,
    z_rel: np.ndarray,
    valid: np.ndarray,
    extended_band: np.ndarray,
    cfg: ReconstructionConfig,
    frame: DepthFrame,
    x_coords: np.ndarray,
    y_coords: np.ndarray,
    ground_at_origin: float,
    plane_z: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    H, W = z_rel.shape
    shadow_z = np.zeros_like(z_rel)
    shadow_mask = np.zeros_like(valid)

    candidate_mask = (~valid) & extended_band
    stats: dict[str, Any] = {
        "enabled": True,
        "candidate_cells": int(candidate_mask.sum()),
        "shadow_filled_cells": 0,
        "fov_outside_cells": 0,
    }
    if not np.any(candidate_mask):
        return shadow_z, shadow_mask, stats

    intr = frame.intrinsics
    pose = frame.camera_to_world.astype(np.float64)
    R_cw = pose[:3, :3]
    cam_pos = pose[:3, 3]
    R_wc = R_cw.T

    rows, cols = np.nonzero(candidate_mask)
    N = rows.size
    x_cells = x_coords[cols]
    y_cells = y_coords[rows]
    z_cells_world = plane_z[rows, cols].astype(np.float64)

    pts_world = np.stack([x_cells, y_cells, z_cells_world], axis=1)
    diffs = pts_world - cam_pos[None, :]
    pts_cam = diffs @ R_wc.T
    z_cam = pts_cam[:, 2]
    in_front = z_cam > 1e-6
    safe_z = np.where(in_front, z_cam, 1.0)
    u = intr.fx * pts_cam[:, 0] / safe_z + intr.cx
    v = intr.fy * pts_cam[:, 1] / safe_z + intr.cy
    in_fov = in_front & (u >= 0.0) & (u < float(intr.width)) & (v >= 0.0) & (v < float(intr.height))

    stats["fov_outside_cells"] = int(np.sum(~in_fov))
    if not np.any(in_fov):
        return shadow_z, shadow_mask, stats

    K = int(max(8, cfg.shadow_fill_samples))
    ts = np.linspace(0.05, 0.95, K, dtype=np.float64)
    dx = x_cells - cam_pos[0]
    dy = y_cells - cam_pos[1]
    dz = z_cells_world - cam_pos[2]
    sx = cam_pos[0] + ts[None, :] * dx[:, None]
    sy = cam_pos[1] + ts[None, :] * dy[:, None]
    sz = cam_pos[2] + ts[None, :] * dz[:, None]

    col_idx = np.floor((sx - cfg.world_x_min) / cfg.resolution).astype(np.int64)
    row_idx = np.floor((sy - cfg.world_y_min) / cfg.resolution).astype(np.int64)
    in_bounds = (row_idx >= 0) & (row_idx < H) & (col_idx >= 0) & (col_idx < W)
    row_c = np.clip(row_idx, 0, H - 1)
    col_c = np.clip(col_idx, 0, W - 1)

    sample_valid = valid[row_c, col_c] & in_bounds
    sample_z_world = plane_z[row_c, col_c] + z_rel[row_c, col_c]
    sample_rel = z_rel[row_c, col_c]

    min_occluder = float(cfg.shadow_fill_min_occluder_height)
    blocked = sample_valid & (sample_z_world > sz) & (sample_rel >= min_occluder)
    any_blocked = np.any(blocked, axis=1)

    occluder_rel = np.where(blocked, sample_rel, -np.inf)
    max_rel = np.max(occluder_rel, axis=1)
    max_rel = np.where(any_blocked, max_rel, 0.0)

    fill = in_fov & any_blocked
    if np.any(fill):
        shadow_mask[rows[fill], cols[fill]] = True
        shadow_z[rows[fill], cols[fill]] = np.clip(
            max_rel[fill], 0.0, float(cfg.max_height_above_ground)
        ).astype(np.float32)
    stats["shadow_filled_cells"] = int(shadow_mask.sum())
    return shadow_z, shadow_mask, stats


def _fit_ground_plane(
    z: np.ndarray,
    valid: np.ndarray,
    cfg: ReconstructionConfig,
) -> tuple[tuple[float, float, float], float]:
    zeros = (0.0, 0.0, 0.0)
    if not np.any(valid):
        return zeros, 0.0
    rows, cols = np.nonzero(valid)
    zs = z[rows, cols].astype(np.float64)
    if zs.size < 3:
        mean_z = float(zs.mean())
        return (0.0, 0.0, mean_z), mean_z
    quantile = float(np.clip(cfg.ground_low_quartile, 0.05, 0.95))
    q_value = float(np.quantile(zs, quantile))
    low_mask = zs <= q_value
    low_rows = rows[low_mask]
    low_cols = cols[low_mask]
    low_zs = zs[low_mask]
    if low_zs.size < 3:
        mean_z = float(low_zs.mean()) if low_zs.size else float(zs.mean())
        return (0.0, 0.0, mean_z), mean_z

    xs = cfg.world_x_min + (low_cols.astype(np.float64) + 0.5) * cfg.resolution
    ys = cfg.world_y_min + (low_rows.astype(np.float64) + 0.5) * cfg.resolution
    pts = np.stack([xs, ys, low_zs], axis=1)
    N = pts.shape[0]
    rng = np.random.default_rng(42)
    best_inliers = -1
    best_plane: tuple[float, float, float] = (0.0, 0.0, float(np.median(low_zs)))
    iters = int(max(1, cfg.ground_ransac_iters))
    for _ in range(iters):
        idx = rng.choice(N, size=3, replace=False)
        sample = pts[idx]
        A = np.stack([sample[:, 0], sample[:, 1], np.ones(3)], axis=1)
        try:
            coeffs = np.linalg.solve(A, sample[:, 2])
        except np.linalg.LinAlgError:
            continue
        pred = coeffs[0] * pts[:, 0] + coeffs[1] * pts[:, 1] + coeffs[2]
        inliers = int(np.sum(np.abs(pts[:, 2] - pred) < cfg.ground_inlier_m))
        if inliers > best_inliers:
            best_inliers = inliers
            best_plane = (float(coeffs[0]), float(coeffs[1]), float(coeffs[2]))

    a, b, c = best_plane
    pred = a * pts[:, 0] + b * pts[:, 1] + c
    inlier_mask = np.abs(pts[:, 2] - pred) < cfg.ground_inlier_m
    if int(inlier_mask.sum()) >= 3:
        A = np.stack([pts[inlier_mask, 0], pts[inlier_mask, 1], np.ones(int(inlier_mask.sum()))], axis=1)
        try:
            coeffs, *_ = np.linalg.lstsq(A, pts[inlier_mask, 2], rcond=None)
            best_plane = (float(coeffs[0]), float(coeffs[1]), float(coeffs[2]))
        except np.linalg.LinAlgError:
            pass

    a, b, c = best_plane
    if abs(a) > cfg.ground_max_slope or abs(b) > cfg.ground_max_slope:
        constant = float(np.median(low_zs))
        best_plane = (0.0, 0.0, constant)

    return best_plane, float(best_plane[2])


def load_depth_capture(capture_path: str | Path) -> tuple[list[DepthFrame], dict[str, Any]]:
    payload = np.load(Path(capture_path), allow_pickle=False)
    intr_arr = payload["intrinsics"].astype(np.float32)
    intrinsics = CameraIntrinsics(
        width=int(intr_arr[0]),
        height=int(intr_arr[1]),
        fx=float(intr_arr[2]),
        fy=float(intr_arr[3]),
        cx=float(intr_arr[4]),
        cy=float(intr_arr[5]),
        depth_min=float(intr_arr[6]),
        depth_max=float(intr_arr[7]),
    )
    depth_stack = payload["depth_stack"].astype(np.float32)
    pose_stack = payload["pose_stack"].astype(np.float32)
    timestamps = payload["timestamps"].astype(np.float32) if "timestamps" in payload else np.zeros(depth_stack.shape[0], dtype=np.float32)
    try:
        metadata = json.loads(str(payload["metadata_json"])) if "metadata_json" in payload else {}
    except (ValueError, TypeError):
        metadata = {}
    frames = [
        DepthFrame(
            depth_m=depth_stack[i],
            intrinsics=intrinsics,
            camera_to_world=pose_stack[i],
            timestamp_s=float(timestamps[i]),
            frame_id=f"capture_{i:04d}",
        )
        for i in range(depth_stack.shape[0])
    ]
    return frames, metadata
