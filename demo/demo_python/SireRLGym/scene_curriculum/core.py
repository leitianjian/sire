from __future__ import annotations

"""
场景课程学习 (Scene Curriculum Learning) 的核心组件。
在强化学习中，课程学习 (Curriculum Learning) 是一种训练策略，它让智能体 (Agent) 
从简单的任务开始学习，然后逐渐增加任务的难度。这就像人类上学一样，先学简单的算术，再学复杂的微积分。
在这个文件中，包含了地形障碍物高度计算、课程难度缩放比例控制等重要函数。
"""

import json
import time
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

from .types import SceneCurriculumArtifacts, SceneCurriculumLevel, SceneCurriculumMetadata, SceneObstacleStats, TerrainScene

DEFAULT_CURRICULUM_LEVEL_SCALES: tuple[float, ...] = (0.35, 0.5, 0.65, 0.8, 1.0)
DEFAULT_SUCCESS_X_MARGIN = 0.25


def target_heights_to_level_scales(target_heights, max_height: float) -> list[float]:
    """
    将目标高度 (target_heights) 转换为课程级别缩放比例 (level scales, 范围 0 到 1)。
    在强化学习训练初期，我们会通过缩放因子 (scale) 降低障碍物的实际高度，
    让智能体更容易通过；随着训练进行，缩放因子逐渐增大至 1.0，恢复真实地形。
    """
    if not target_heights:
        return []
    max_height = float(max_height)
    if max_height <= 1e-8:
        raise ValueError("Cannot derive curriculum scales because detected obstacle height is too small.")
    scales = []
    for height in target_heights:
        scales.append(float(np.clip(float(height) / max_height, 0.0, 1.0)))
    return scales


def auto_curriculum_target_heights(max_height: float, *, base_height: float = 0.04, step_height: float = 0.02) -> list[float]:
    max_height = float(max_height)
    base_height = float(base_height)
    step_height = float(step_height)
    if max_height <= 1e-8:
        return []
    if max_height <= base_height + 1e-6:
        return [max_height]

    targets = []
    current = base_height
    while current < max_height - 1e-6:
        targets.append(round(current, 6))
        current += step_height
    targets.append(round(max_height, 6))
    return targets


def load_terrain_scene(npz_path: str | Path) -> TerrainScene:
    payload = np.load(Path(npz_path), allow_pickle=False)
    metadata = json.loads(str(payload["metadata_json"]))
    origin_xy = tuple(float(v) for v in payload["origin_xy"].tolist())
    return TerrainScene(
        height_map=payload["height_map"].astype(np.float32),
        valid_mask=payload["valid_mask"].astype(bool),
        confidence_map=payload["confidence_map"].astype(np.float32),
        resolution=float(payload["resolution"]),
        origin_xy=(origin_xy[0], origin_xy[1]),
        metadata=metadata,
    )


def load_scene_curriculum_metadata(curriculum_dir: str | Path) -> SceneCurriculumMetadata:
    metadata_path = Path(curriculum_dir) / "scene_curriculum_metadata.json"
    payload = json.loads(metadata_path.read_text(encoding="utf-8"))
    return SceneCurriculumMetadata.from_dict(payload)


def _connected_components(mask: np.ndarray) -> list[np.ndarray]:
    visited = np.zeros_like(mask, dtype=bool)
    components: list[np.ndarray] = []
    rows, cols = mask.shape
    for start_row, start_col in np.argwhere(mask):
        if visited[start_row, start_col]:
            continue
        stack = [(int(start_row), int(start_col))]
        visited[start_row, start_col] = True
        coords: list[tuple[int, int]] = []
        while stack:
            row, col = stack.pop()
            coords.append((row, col))
            for d_row, d_col in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                next_row = row + d_row
                next_col = col + d_col
                if next_row < 0 or next_row >= rows or next_col < 0 or next_col >= cols:
                    continue
                if visited[next_row, next_col] or not mask[next_row, next_col]:
                    continue
                visited[next_row, next_col] = True
                stack.append((next_row, next_col))
        component = np.zeros_like(mask, dtype=bool)
        rr, cc = zip(*coords)
        component[np.asarray(rr), np.asarray(cc)] = True
        components.append(component)
    return components


def _estimate_ground_height(height_map: np.ndarray, valid_mask: np.ndarray, percentile: float = 10.0) -> float:
    valid_heights = height_map[valid_mask]
    if valid_heights.size == 0:
        return 0.0
    return float(np.percentile(valid_heights, percentile))


def _resolve_ground_height(scene: TerrainScene, percentile: float = 10.0) -> float:
    metadata = scene.metadata or {}
    # Reconstructed scenes may already be aligned so that the traversable ground
    # is z=0 in the exported height map. In that case, re-estimating "ground"
    # from the valid heights can incorrectly treat the obstacle top as the
    # baseline when most valid points lie on the obstacle/platform itself.
    if bool(metadata.get("apply_ground_alignment", False)):
        return 0.0
    return _estimate_ground_height(scene.height_map, scene.valid_mask, percentile=percentile)


def _centerline_clearance_m(y_min: float, y_max: float) -> float:
    if y_min <= 0.0 <= y_max:
        return 0.0
    if y_max < 0.0:
        return abs(y_max)
    return abs(y_min)


def _recommended_lateral_error_threshold(obstacle_width: float, patch_width: float) -> float:
    max_allowed = max(0.25, 0.5 * patch_width - 0.05)
    return float(np.clip(0.5 * obstacle_width + 0.1, 0.35, max_allowed))


def _occupied_bounds_from_level_scene(
    scene: TerrainScene,
    occupied_threshold: float,
    *,
    support_ratio: float = 0.25,
    min_support_cells: int = 3,
) -> tuple[float, float, float, float] | None:
    occupied = scene.valid_mask & (scene.height_map > occupied_threshold)
    if not np.any(occupied):
        return None
    row_counts = occupied.sum(axis=1)
    row_support = max(min_support_cells, int(np.ceil(float(row_counts.max()) * support_ratio)))
    supported_rows = np.flatnonzero(row_counts >= row_support)
    if supported_rows.size == 0:
        supported_rows = np.flatnonzero(row_counts > 0)
    rows, cols = np.nonzero(occupied)
    supported_cols = np.unique(cols)
    x_start = float(scene.origin_xy[0] + supported_cols.min() * scene.resolution)
    x_end = float(scene.origin_xy[0] + (supported_cols.max() + 1) * scene.resolution)
    y_min = float(scene.origin_xy[1] + supported_rows.min() * scene.resolution)
    y_max = float(scene.origin_xy[1] + (supported_rows.max() + 1) * scene.resolution)
    return x_start, x_end, y_min, y_max


def analyze_scene_for_curriculum(
    scene: TerrainScene,
    *,
    obstacle_height_threshold: float | None = None,
    focus_corridor_half_width: float | None = None,
    min_component_cells: int | None = None,
    level_scales: list[float] | tuple[float, ...] | None = None,
    base_scene_npz: str = "base_scene.npz",
) -> SceneCurriculumMetadata:
    scene_rows, scene_cols = scene.height_map.shape
    patch_length, patch_width = scene.size_xy
    threshold = float(obstacle_height_threshold if obstacle_height_threshold is not None else 0.05)
    corridor_half_width = float(focus_corridor_half_width if focus_corridor_half_width is not None else max(0.2, 0.3 * patch_width))
    component_min_cells = int(min_component_cells if min_component_cells is not None else 12)
    scales = tuple(float(v) for v in (level_scales or DEFAULT_CURRICULUM_LEVEL_SCALES))

    ground_height = _resolve_ground_height(scene)
    rel_height = np.clip(scene.height_map - ground_height, 0.0, None)
    y_coords = scene.origin_xy[1] + (np.arange(scene_rows, dtype=np.float32) + 0.5) * scene.resolution
    focus_mask = np.abs(y_coords[:, None]) <= corridor_half_width
    obstacle_seed = scene.valid_mask & focus_mask & (rel_height >= threshold)
    components = [comp for comp in _connected_components(obstacle_seed) if int(comp.sum()) >= component_min_cells]

    if not components:
        valid_heights = scene.height_map[scene.valid_mask]
        is_uniform_valid_height = valid_heights.size > 0 and float(valid_heights.max() - valid_heights.min()) <= 1e-5
        metadata_ground = 0.0 if bool((scene.metadata or {}).get("apply_ground_alignment", False)) else float((scene.metadata or {}).get("ground_height_offset", 0.0))
        # Some reconstructed npz files only store the obstacle body in valid_mask and
        # leave the surrounding plane implicit. In that case the percentile-based
        # ground estimator collapses to the obstacle top, so fall back to using the
        # metadata ground offset and treat the valid_mask itself as the obstacle seed.
        if is_uniform_valid_height and float(valid_heights.max()) > metadata_ground + threshold:
            ground_height = metadata_ground
            rel_height = np.clip(scene.height_map - ground_height, 0.0, None)
            obstacle_seed = scene.valid_mask & (rel_height >= threshold)
            components = [comp for comp in _connected_components(obstacle_seed) if int(comp.sum()) >= component_min_cells]

    if not components:
        return SceneCurriculumMetadata(
            curriculum_applicable=False,
            analysis_version="scene_curriculum_v1",
            forward_axis="+x",
            ground_height=ground_height,
            obstacle_height_threshold=threshold,
            focus_corridor_half_width=corridor_half_width,
            min_component_cells=component_min_cells,
            resolution=scene.resolution,
            origin_xy=scene.origin_xy,
            scene_shape=(scene_rows, scene_cols),
            patch_length=patch_length,
            patch_width=patch_width,
            level_scales=scales,
            base_scene_npz=base_scene_npz,
        )

    center_row = float((-scene.origin_xy[1]) / scene.resolution)
    best_component: np.ndarray | None = None
    best_score: tuple[float, float, float, float] | None = None
    for component in components:
        rows, cols = np.nonzero(component)
        min_col = float(cols.min())
        mean_row = float(rows.mean())
        height_p95 = float(np.percentile(rel_height[component], 95))
        cell_count = float(component.sum())
        score = (min_col, abs(mean_row - center_row), -height_p95, -cell_count)
        if best_score is None or score < best_score:
            best_component = component
            best_score = score

    if best_component is None or not np.any(best_component):
        return SceneCurriculumMetadata(
            curriculum_applicable=False,
            analysis_version="scene_curriculum_v1",
            forward_axis="+x",
            ground_height=ground_height,
            obstacle_height_threshold=threshold,
            focus_corridor_half_width=corridor_half_width,
            min_component_cells=component_min_cells,
            resolution=scene.resolution,
            origin_xy=scene.origin_xy,
            scene_shape=(scene_rows, scene_cols),
            patch_length=patch_length,
            patch_width=patch_width,
            level_scales=scales,
            base_scene_npz=base_scene_npz,
        )

    rows, cols = np.nonzero(best_component)
    min_row = int(rows.min())
    max_row = int(rows.max())
    min_col = int(cols.min())
    max_col = int(cols.max())
    y_min = scene.origin_xy[1] + min_row * scene.resolution
    y_max = scene.origin_xy[1] + (max_row + 1) * scene.resolution
    x_start = scene.origin_xy[0] + min_col * scene.resolution
    x_end = scene.origin_xy[0] + (max_col + 1) * scene.resolution
    bbox_cell_count = int((max_row - min_row + 1) * (max_col - min_col + 1))
    stats = SceneObstacleStats(
        height_max=float(rel_height[best_component].max()),
        height_p95=float(np.percentile(rel_height[best_component], 95)),
        x_start=float(x_start),
        x_end=float(x_end),
        y_min=float(y_min),
        y_max=float(y_max),
        width=float(y_max - y_min),
        length=float(x_end - x_start),
        centerline_clearance=float(_centerline_clearance_m(y_min, y_max)),
        coverage_ratio=float(best_component.sum() / max(1, bbox_cell_count)),
        cell_count=int(best_component.sum()),
        bbox_cell_count=bbox_cell_count,
        mask=best_component.copy(),
    )
    return SceneCurriculumMetadata(
        curriculum_applicable=True,
        analysis_version="scene_curriculum_v1",
        forward_axis="+x",
        ground_height=ground_height,
        obstacle_height_threshold=threshold,
        focus_corridor_half_width=max(corridor_half_width, 0.5 * stats.width + scene.resolution),
        min_component_cells=component_min_cells,
        resolution=scene.resolution,
        origin_xy=scene.origin_xy,
        scene_shape=(scene_rows, scene_cols),
        patch_length=patch_length,
        patch_width=patch_width,
        level_scales=scales,
        base_scene_npz=base_scene_npz,
        obstacle_stats=stats,
    )


def _binary_dilation(mask: np.ndarray, radius: int = 1) -> np.ndarray:
    out = mask.astype(bool, copy=True)
    for _ in range(max(0, radius)):
        padded = np.pad(out, 1, mode="constant", constant_values=False)
        dilated = np.zeros_like(out, dtype=bool)
        for row_offset in range(3):
            for col_offset in range(3):
                dilated |= padded[row_offset:row_offset + out.shape[0], col_offset:col_offset + out.shape[1]]
        out = dilated
    return out


def _binary_erosion(mask: np.ndarray, radius: int = 1) -> np.ndarray:
    out = mask.astype(bool, copy=True)
    for _ in range(max(0, radius)):
        padded = np.pad(out, 1, mode="constant", constant_values=False)
        eroded = np.ones_like(out, dtype=bool)
        for row_offset in range(3):
            for col_offset in range(3):
                eroded &= padded[row_offset:row_offset + out.shape[0], col_offset:col_offset + out.shape[1]]
        out = eroded
    return out


def _box_blur(height_map: np.ndarray, passes: int = 1) -> np.ndarray:
    out = height_map.astype(np.float32, copy=True)
    for _ in range(max(1, passes)):
        padded = np.pad(out, 1, mode="edge")
        acc = np.zeros_like(out, dtype=np.float32)
        for row_offset in range(3):
            for col_offset in range(3):
                acc += padded[row_offset:row_offset + out.shape[0], col_offset:col_offset + out.shape[1]]
        out = acc / 9.0
    return out


def _soften_heightfield_edges(height_map: np.ndarray, active_mask: np.ndarray) -> np.ndarray:
    if not np.any(active_mask):
        return height_map
    inner_mask = _binary_erosion(active_mask, radius=1)
    boundary_band = _binary_dilation(active_mask, radius=1) & ~inner_mask
    if not np.any(boundary_band):
        return height_map
    smoothed = _box_blur(height_map, passes=1)
    softened = height_map.astype(np.float32, copy=True)
    softened[boundary_band] = smoothed[boundary_band]
    return softened


def generate_scene_curriculum(
    scene: TerrainScene,
    metadata: SceneCurriculumMetadata,
    level_scales: list[float] | tuple[float, ...] | None = None,
) -> list[TerrainScene]:
    if not metadata.curriculum_applicable or metadata.obstacle_stats is None:
        return []

    obstacle_mask = metadata.obstacle_stats.mask
    scales = tuple(float(v) for v in (level_scales or metadata.level_scales))
    ground_height = float(metadata.ground_height)
    rel_height = np.clip(scene.height_map - ground_height, 0.0, None)
    non_primary_obstacle_mask = (~obstacle_mask) & scene.valid_mask & (rel_height >= metadata.obstacle_height_threshold)
    obstacle_height_max = float(metadata.obstacle_stats.height_max)
    level_scenes: list[TerrainScene] = []
    for level_idx, scale in enumerate(scales):
        # Cap the real geometry at target_h = scale * height_max instead of scaling it uniformly.
        # This preserves the true slope / riser geometry and only flattens cells above the cap,
        # so level 0 is a short genuine ramp up to target_h followed by a plateau, not a shrunken
        # miniature of the full obstacle.
        target_h = scale * obstacle_height_max
        capped_height = rel_height.copy()
        capped_height[obstacle_mask] = np.minimum(rel_height[obstacle_mask], target_h)
        # Keep the curriculum focused on a single obstacle by flattening other tall regions.
        capped_height[non_primary_obstacle_mask] = 0.0
        active_mask = scene.valid_mask & (capped_height > 1e-6)
        capped_height = _soften_heightfield_edges(capped_height, active_mask)
        level_scenes.append(
            TerrainScene(
                height_map=capped_height.astype(np.float32),
                valid_mask=scene.valid_mask.copy(),
                confidence_map=scene.confidence_map.copy(),
                resolution=scene.resolution,
                origin_xy=scene.origin_xy,
                metadata={
                    **scene.metadata,
                    "scene_curriculum": {
                        "analysis_version": metadata.analysis_version,
                        "level_index": level_idx,
                        "level_scale": scale,
                        "target_height": target_h,
                    },
                },
            )
        )
    return level_scenes


def _normalize_height_map(height_map: np.ndarray) -> tuple[np.ndarray, float]:
    peak = float(np.max(height_map))
    if peak <= 1e-8:
        return np.zeros_like(height_map, dtype=np.uint8), 1e-4
    normalized = np.clip(np.round(255.0 * height_map / peak), 0, 255).astype(np.uint8)
    return normalized, peak


def _export_level_xml(scene: TerrainScene, output_dir: Path, model_name: str) -> tuple[Path, Path]:
    normalized, peak_height = _normalize_height_map(scene.height_map)
    png_path = output_dir / f"{model_name}.png"
    Image.fromarray(normalized, mode="L").save(png_path)
    width_m, length_m = scene.size_xy
    center_x = scene.origin_xy[0] + width_m * 0.5
    center_y = scene.origin_xy[1] + length_m * 0.5
    extent = max(width_m, length_m) * 0.6
    plane_size = max(width_m, length_m) * 1.5
    xml_path = output_dir / f"{model_name}.xml"
    xml_path.write_text(
        f"""<mujoco model="{model_name}">
  <statistic center="{center_x:.4f} {center_y:.4f} 0.1500" extent="{extent:.4f}"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.2 0.2 0.2" specular="0.8 0.8 0.8"/>
    <rgba haze="0.12 0.14 0.18 1"/>
    <global azimuth="135" elevation="-28" offwidth="1600" offheight="1200"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.17 0.2 0.24" rgb2="0.02 0.03 0.05" width="512" height="3072"/>
    <texture type="2d" name="groundplane_tex" builtin="checker" mark="edge" rgb1="0.30 0.34 0.30" rgb2="0.20 0.22 0.20" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane_mat" texture="groundplane_tex" texuniform="true" texrepeat="8 8" reflectance="0.08"/>
    <material name="terrain_mat" rgba="0.70 0.74 0.62 1" reflectance="0.10"/>
    <hfield name="terrain_hf" file="{png_path.name}" size="{width_m * 0.5:.4f} {length_m * 0.5:.4f} {peak_height:.4f} 0.05"/>
  </asset>
  <worldbody>
    <light pos="1 0 4" dir="0 0 -1" directional="true"/>
    <geom name="groundplane" type="plane" material="groundplane_mat" pos="{center_x:.4f} {center_y:.4f} -0.001" size="{plane_size:.4f} {plane_size:.4f} 0.1" friction="1.0 0.1 0.1"/>
    <geom name="terrain" type="hfield" hfield="terrain_hf" material="terrain_mat" pos="{center_x:.4f} {center_y:.4f} 0.0" friction="1.0 0.1 0.1"/>
  </worldbody>
</mujoco>
""",
        encoding="utf-8",
    )
    return xml_path, png_path


def export_scene_curriculum(
    level_scenes: list[TerrainScene],
    output_dir: str | Path,
    scene_name: str,
    metadata: SceneCurriculumMetadata,
    *,
    base_scene: TerrainScene,
    success_x_margin: float = DEFAULT_SUCCESS_X_MARGIN,
) -> SceneCurriculumArtifacts:
    output_dir_path = Path(output_dir)
    output_dir_path.mkdir(parents=True, exist_ok=True)

    base_scene_path = base_scene.save(output_dir_path / metadata.base_scene_npz)
    if not metadata.curriculum_applicable or metadata.obstacle_stats is None:
        aggregate_payload = metadata.to_dict()
        aggregate_payload["export_time_s"] = time.time()
        metadata_path = output_dir_path / "scene_curriculum_metadata.json"
        metadata_path.write_text(json.dumps(aggregate_payload, indent=2, ensure_ascii=True), encoding="utf-8")
        return SceneCurriculumArtifacts(
            output_dir=output_dir_path,
            base_scene_path=base_scene_path,
            metadata_path=metadata_path,
            level_scene_paths=(),
            level_xml_paths=(),
            level_png_paths=(),
        )

    levels: list[SceneCurriculumLevel] = []
    level_scene_paths: list[Path] = []
    level_xml_paths: list[Path] = []
    level_png_paths: list[Path] = []
    for level_idx, (scale, level_scene) in enumerate(zip(metadata.level_scales, level_scenes)):
        scene_npz_name = f"scene_level_{level_idx:02d}.npz"
        scene_xml_name = f"scene_level_{level_idx:02d}.xml"
        scene_png_name = f"scene_level_{level_idx:02d}.png"
        scene_path = level_scene.save(output_dir_path / scene_npz_name)
        xml_path, png_path = _export_level_xml(level_scene, output_dir_path, f"scene_level_{level_idx:02d}")
        threshold_height = float(np.percentile(level_scene.height_map[level_scene.height_map > 0.0], 95)) if np.any(level_scene.height_map > 0.0) else metadata.obstacle_stats.height_p95 * scale
        effective_wall_threshold = 0.03
        bounds = _occupied_bounds_from_level_scene(level_scene, occupied_threshold=effective_wall_threshold)
        if bounds is None:
            bounds = _occupied_bounds_from_level_scene(level_scene, occupied_threshold=0.02)
        if bounds is None:
            bounds = (
                metadata.obstacle_stats.x_start,
                metadata.obstacle_stats.x_end,
                metadata.obstacle_stats.y_min,
                metadata.obstacle_stats.y_max,
            )
        x_start, x_end, y_min, y_max = bounds
        local_x_start = x_start - metadata.origin_xy[0]
        local_x_end = x_end - metadata.origin_xy[0]
        local_y_min = y_min - metadata.origin_xy[1]
        local_y_max = y_max - metadata.origin_xy[1]
        local_y_center = 0.5 * (y_min + y_max) - metadata.origin_xy[1]
        threshold_width = float(y_max - y_min)
        corridor_width = threshold_width + 0.20
        level_record = SceneCurriculumLevel(
            level_index=level_idx,
            level_scale=scale,
            threshold_height=threshold_height,
            threshold_start_x=local_x_start,
            threshold_end_x=local_x_end,
            threshold_width=threshold_width,
            threshold_y_min=local_y_min,
            threshold_y_max=local_y_max,
            corridor_center_y=local_y_center,
            corridor_width=corridor_width,
            success_x=local_x_end + success_x_margin,
            lateral_error_threshold=_recommended_lateral_error_threshold(threshold_width, metadata.patch_width),
            recommended_use="final_eval" if level_idx == len(level_scenes) - 1 else ("validation" if level_idx == len(level_scenes) - 2 else "training"),
            scene_npz=scene_npz_name,
            scene_xml=scene_xml_name,
            scene_png=scene_png_name,
        )
        levels.append(level_record)
        level_scene_paths.append(scene_path)
        level_xml_paths.append(xml_path)
        level_png_paths.append(png_path)

    metadata_with_levels = replace(metadata, levels=tuple(levels))
    aggregate_payload = metadata_with_levels.to_dict()
    aggregate_payload["export_time_s"] = time.time()
    aggregate_payload["scene_name"] = scene_name
    metadata_path = output_dir_path / "scene_curriculum_metadata.json"
    metadata_path.write_text(json.dumps(aggregate_payload, indent=2, ensure_ascii=True), encoding="utf-8")
    return SceneCurriculumArtifacts(
        output_dir=output_dir_path,
        base_scene_path=base_scene_path,
        metadata_path=metadata_path,
        level_scene_paths=tuple(level_scene_paths),
        level_xml_paths=tuple(level_xml_paths),
        level_png_paths=tuple(level_png_paths),
    )


def apply_scene_curriculum_to_env_cfg(
    env_cfg,
    curriculum_dir: str | Path,
    *,
    forced_level: int | None = None,
    single_level_preview: bool = False,
):
    metadata = load_scene_curriculum_metadata(curriculum_dir)
    if not metadata.curriculum_applicable:
        raise ValueError(f"Scene curriculum at {curriculum_dir} is not applicable for curriculum training.")

    levels = list(metadata.levels)
    preview_level_idx = None
    if single_level_preview:
        preview_level_idx = len(levels) - 1 if forced_level is None else int(np.clip(forced_level, 0, len(levels) - 1))
        levels = [levels[preview_level_idx]]
        forced_level = 0

    env_cfg.terrain.mesh_type = "trimesh"
    env_cfg.terrain.terrain_type_mode = "scene_curriculum"
    env_cfg.terrain.scene_curriculum_dir = str(Path(curriculum_dir).resolve())
    env_cfg.terrain.scene_curriculum_single_level = preview_level_idx
    env_cfg.terrain.scene_curriculum_content_length = float(metadata.patch_length)
    env_cfg.terrain.scene_curriculum_content_width = float(metadata.patch_width)
    env_cfg.terrain.num_rows = len(levels)
    env_cfg.terrain.num_cols = 1
    min_patch_length = float(metadata.patch_length) + max(0.5, float(getattr(env_cfg.terrain, "threshold_offset_x", 1.5)))
    min_patch_width = max(
        float(metadata.patch_width) + 0.2,
        float(getattr(env_cfg.terrain, "corridor_width", metadata.patch_width)) + 0.2,
    )
    env_cfg.terrain.terrain_length = max(float(getattr(env_cfg.terrain, "terrain_length", metadata.patch_length)), min_patch_length)
    env_cfg.terrain.terrain_width = max(float(getattr(env_cfg.terrain, "terrain_width", metadata.patch_width)), min_patch_width)
    env_cfg.terrain.threshold_height_levels = [float(level.threshold_height) for level in levels]
    if levels:
        env_cfg.terrain.threshold_height = float(levels[0].threshold_height)
    if hasattr(env_cfg, "local_task") and levels:
        threshold_level = levels[-1]
        if forced_level is not None:
            threshold_level = levels[int(np.clip(forced_level, 0, len(levels) - 1))]
        env_cfg.local_task.lateral_error_threshold = float(threshold_level.lateral_error_threshold)
        if forced_level is not None:
            env_cfg.local_task.forced_terrain_level = True
            env_cfg.local_task.forced_terrain_level_index = int(np.clip(forced_level, 0, len(levels) - 1))
    return metadata


def build_scene_curriculum_from_npz(
    scene_npz: str | Path,
    output_root: str | Path,
    *,
    scene_name: str | None = None,
    target_heights: list[float] | tuple[float, ...] | None = None,
    level_scales: list[float] | tuple[float, ...] | None = None,
    obstacle_height_threshold: float | None = None,
    focus_corridor_half_width: float | None = None,
    min_component_cells: int | None = None,
):
    scene_path = Path(scene_npz).expanduser().resolve()
    scene = load_terrain_scene(scene_path)
    scene_name = scene_name or f"{scene_path.stem}_auto"
    output_dir = Path(output_root).expanduser().resolve() / scene_name

    base_metadata = analyze_scene_for_curriculum(
        scene,
        obstacle_height_threshold=obstacle_height_threshold,
        focus_corridor_half_width=focus_corridor_half_width,
        min_component_cells=min_component_cells,
        level_scales=list(DEFAULT_CURRICULUM_LEVEL_SCALES),
    )
    if base_metadata.obstacle_stats is None:
        raise ValueError(f"No primary obstacle detected in scene npz: {scene_path}")

    if target_heights is not None:
        resolved_target_heights = list(target_heights)
        resolved_level_scales = target_heights_to_level_scales(resolved_target_heights, base_metadata.obstacle_stats.height_max)
    elif level_scales is not None:
        resolved_target_heights = []
        resolved_level_scales = [float(v) for v in level_scales]
    else:
        resolved_target_heights = auto_curriculum_target_heights(base_metadata.obstacle_stats.height_max)
        resolved_level_scales = target_heights_to_level_scales(resolved_target_heights, base_metadata.obstacle_stats.height_max)

    metadata = analyze_scene_for_curriculum(
        scene,
        obstacle_height_threshold=obstacle_height_threshold,
        focus_corridor_half_width=focus_corridor_half_width,
        min_component_cells=min_component_cells,
        level_scales=resolved_level_scales,
    )
    level_scenes = generate_scene_curriculum(scene, metadata, level_scales=resolved_level_scales)
    artifacts = export_scene_curriculum(level_scenes, output_dir, scene_name, metadata, base_scene=scene)
    exported_metadata = load_scene_curriculum_metadata(artifacts.output_dir)
    return exported_metadata, artifacts, resolved_target_heights
