from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np


@dataclass
class TerrainScene:
    height_map: np.ndarray
    valid_mask: np.ndarray
    confidence_map: np.ndarray
    resolution: float
    origin_xy: tuple[float, float]
    metadata: dict[str, Any] = field(default_factory=dict)

    @property
    def size_xy(self) -> tuple[float, float]:
        rows, cols = self.height_map.shape
        return cols * self.resolution, rows * self.resolution

    def save(self, path: str | Path) -> Path:
        out_path = Path(path)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            out_path,
            height_map=self.height_map.astype(np.float32),
            valid_mask=self.valid_mask.astype(np.uint8),
            confidence_map=self.confidence_map.astype(np.float32),
            resolution=np.float32(self.resolution),
            origin_xy=np.asarray(self.origin_xy, dtype=np.float32),
            metadata_json=json.dumps(self.metadata, ensure_ascii=True),
        )
        return out_path


@dataclass(frozen=True)
class SceneObstacleStats:
    height_max: float
    height_p95: float
    x_start: float
    x_end: float
    y_min: float
    y_max: float
    width: float
    length: float
    centerline_clearance: float
    coverage_ratio: float
    cell_count: int
    bbox_cell_count: int
    mask: np.ndarray = field(repr=False, compare=False)

    def to_dict(self) -> dict[str, Any]:
        return {
            "height_max": self.height_max,
            "height_p95": self.height_p95,
            "x_start": self.x_start,
            "x_end": self.x_end,
            "y_min": self.y_min,
            "y_max": self.y_max,
            "width": self.width,
            "length": self.length,
            "centerline_clearance": self.centerline_clearance,
            "coverage_ratio": self.coverage_ratio,
            "cell_count": self.cell_count,
            "bbox_cell_count": self.bbox_cell_count,
        }


@dataclass(frozen=True)
class SceneCurriculumLevel:
    level_index: int
    level_scale: float
    threshold_height: float
    threshold_start_x: float
    threshold_end_x: float
    threshold_width: float
    threshold_y_min: float
    threshold_y_max: float
    corridor_center_y: float
    corridor_width: float
    success_x: float
    lateral_error_threshold: float
    recommended_use: str
    scene_npz: str
    scene_xml: str
    scene_png: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "level_index": self.level_index,
            "level_scale": self.level_scale,
            "threshold_height": self.threshold_height,
            "threshold_start_x": self.threshold_start_x,
            "threshold_end_x": self.threshold_end_x,
            "threshold_width": self.threshold_width,
            "threshold_y_min": self.threshold_y_min,
            "threshold_y_max": self.threshold_y_max,
            "corridor_center_y": self.corridor_center_y,
            "corridor_width": self.corridor_width,
            "success_x": self.success_x,
            "lateral_error_threshold": self.lateral_error_threshold,
            "recommended_use": self.recommended_use,
            "scene_npz": self.scene_npz,
            "scene_xml": self.scene_xml,
            "scene_png": self.scene_png,
        }


@dataclass(frozen=True)
class SceneCurriculumMetadata:
    curriculum_applicable: bool
    analysis_version: str
    forward_axis: str
    ground_height: float
    obstacle_height_threshold: float
    focus_corridor_half_width: float
    min_component_cells: int
    resolution: float
    origin_xy: tuple[float, float]
    scene_shape: tuple[int, int]
    patch_length: float
    patch_width: float
    level_scales: tuple[float, ...]
    base_scene_npz: str
    obstacle_stats: SceneObstacleStats | None = None
    levels: tuple[SceneCurriculumLevel, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return {
            "curriculum_applicable": self.curriculum_applicable,
            "analysis_version": self.analysis_version,
            "forward_axis": self.forward_axis,
            "ground_height": self.ground_height,
            "obstacle_height_threshold": self.obstacle_height_threshold,
            "focus_corridor_half_width": self.focus_corridor_half_width,
            "min_component_cells": self.min_component_cells,
            "resolution": self.resolution,
            "origin_xy": list(self.origin_xy),
            "scene_shape": list(self.scene_shape),
            "patch_length": self.patch_length,
            "patch_width": self.patch_width,
            "level_scales": list(self.level_scales),
            "base_scene_npz": self.base_scene_npz,
            "obstacle_stats": self.obstacle_stats.to_dict() if self.obstacle_stats is not None else None,
            "num_levels": len(self.levels),
            "levels": [level.to_dict() for level in self.levels],
        }

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "SceneCurriculumMetadata":
        obstacle_payload = payload.get("obstacle_stats")
        obstacle_stats = None
        if obstacle_payload is not None:
            obstacle_stats = SceneObstacleStats(
                height_max=float(obstacle_payload["height_max"]),
                height_p95=float(obstacle_payload["height_p95"]),
                x_start=float(obstacle_payload["x_start"]),
                x_end=float(obstacle_payload["x_end"]),
                y_min=float(obstacle_payload["y_min"]),
                y_max=float(obstacle_payload["y_max"]),
                width=float(obstacle_payload["width"]),
                length=float(obstacle_payload["length"]),
                centerline_clearance=float(obstacle_payload["centerline_clearance"]),
                coverage_ratio=float(obstacle_payload["coverage_ratio"]),
                cell_count=int(obstacle_payload["cell_count"]),
                bbox_cell_count=int(obstacle_payload["bbox_cell_count"]),
                mask=np.zeros(tuple(payload["scene_shape"]), dtype=bool),
            )
        levels = tuple(
            SceneCurriculumLevel(
                level_index=int(level["level_index"]),
                level_scale=float(level["level_scale"]),
                threshold_height=float(level["threshold_height"]),
                threshold_start_x=float(level["threshold_start_x"]),
                threshold_end_x=float(level["threshold_end_x"]),
                threshold_width=float(level["threshold_width"]),
                threshold_y_min=float(level.get("threshold_y_min", level["corridor_center_y"] - 0.5 * level.get("corridor_width", level["threshold_width"]))),
                threshold_y_max=float(level.get("threshold_y_max", level["corridor_center_y"] + 0.5 * level.get("corridor_width", level["threshold_width"]))),
                corridor_center_y=float(level["corridor_center_y"]),
                corridor_width=float(level.get("corridor_width", level["threshold_width"])),
                success_x=float(level["success_x"]),
                lateral_error_threshold=float(level["lateral_error_threshold"]),
                recommended_use=str(level["recommended_use"]),
                scene_npz=str(level["scene_npz"]),
                scene_xml=str(level["scene_xml"]),
                scene_png=str(level["scene_png"]),
            )
            for level in payload.get("levels", [])
        )
        return cls(
            curriculum_applicable=bool(payload["curriculum_applicable"]),
            analysis_version=str(payload["analysis_version"]),
            forward_axis=str(payload["forward_axis"]),
            ground_height=float(payload["ground_height"]),
            obstacle_height_threshold=float(payload["obstacle_height_threshold"]),
            focus_corridor_half_width=float(payload["focus_corridor_half_width"]),
            min_component_cells=int(payload["min_component_cells"]),
            resolution=float(payload["resolution"]),
            origin_xy=(float(payload["origin_xy"][0]), float(payload["origin_xy"][1])),
            scene_shape=(int(payload["scene_shape"][0]), int(payload["scene_shape"][1])),
            patch_length=float(payload["patch_length"]),
            patch_width=float(payload["patch_width"]),
            level_scales=tuple(float(v) for v in payload.get("level_scales", [])),
            base_scene_npz=str(payload["base_scene_npz"]),
            obstacle_stats=obstacle_stats,
            levels=levels,
        )


@dataclass(frozen=True)
class SceneCurriculumArtifacts:
    output_dir: Path
    base_scene_path: Path
    metadata_path: Path
    level_scene_paths: tuple[Path, ...]
    level_xml_paths: tuple[Path, ...]
    level_png_paths: tuple[Path, ...]
