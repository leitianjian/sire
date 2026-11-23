from __future__ import annotations

import math
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Dict, Tuple

import numpy as np
from PIL import Image

from RLGym.scene_curriculum import load_scene_curriculum_metadata, load_terrain_scene


@dataclass(frozen=True)
class TerrainPatch:
    row: int
    col: int
    terrain_type: str
    start_x: float
    start_y: float
    center_x: float
    center_y: float
    spawn_x: float
    spawn_y: float
    spawn_z: float
    difficulty: float
    metadata: Dict[str, object]


class TerrainLayout:
    def __init__(self, cfg, num_envs: int):
        self.cfg = cfg
        self.num_envs = num_envs
        self.num_rows = int(cfg.num_rows)
        self.num_cols = int(cfg.num_cols)
        self.patch_length = float(cfg.terrain_length)
        self.patch_width = float(cfg.terrain_width)
        self.total_length = self.patch_length * self.num_rows
        self.total_width = self.patch_width * self.num_cols
        self.border = float(cfg.border_size)
        self.spawn_offset_x = float(getattr(cfg, 'spawn_offset_x', 1.2))
        self.spawn_offset_y = float(getattr(cfg, 'spawn_offset_y', self.patch_width * 0.5))
        self.base_half_height = float(getattr(cfg, 'base_half_height', 0.04))
        self.slope_half_height = float(getattr(cfg, 'slope_half_height', 0.06))
        self.rng = np.random.default_rng(int(getattr(cfg, 'terrain_seed', 1234)))
        self.terrain_type_mode = str(getattr(cfg, 'terrain_type_mode', 'slope')).lower()
        if self.terrain_type_mode not in {'slope', 'heightfield', 'threshold', 'scene_curriculum'}:
            raise ValueError(
                f"Unsupported terrain_type_mode='{self.terrain_type_mode}'. "
                "Use 'slope', 'heightfield', 'threshold', or 'scene_curriculum'."
            )

        self.generated_dir = Path(tempfile.gettempdir()) / 'mujoco_rl_terrain'
        self.generated_dir.mkdir(parents=True, exist_ok=True)

        self.slope_range_deg = tuple(float(v) for v in getattr(cfg, 'slope_range_deg', [4.0, 12.0]))
        self.slope_angle_override = getattr(cfg, 'slope_angle_override_deg', None)
        self.heightfield_height_range = tuple(
            float(v) for v in getattr(cfg, 'heightfield_height_range', getattr(cfg, 'rough_height_range', [0.01, 0.05]))
        )
        self.heightfield_height_override = getattr(cfg, 'heightfield_height_override', None)
        self.heightfield_nrow = int(getattr(cfg, 'heightfield_nrow', 257))
        self.heightfield_ncol = int(getattr(cfg, 'heightfield_ncol', 257))
        self.heightfield_smooth_steps = int(getattr(cfg, 'heightfield_smooth_steps', 12))
        self.heightfield_spawn_flat_radius = int(getattr(cfg, 'heightfield_spawn_flat_radius', 4))
        self.threshold_height = float(getattr(cfg, 'threshold_height', 0.15))
        threshold_height_levels = getattr(cfg, 'threshold_height_levels', None)
        self.threshold_height_levels = None if threshold_height_levels is None else [float(v) for v in threshold_height_levels]
        self.threshold_depth = float(getattr(cfg, 'threshold_depth', 0.15))
        self.threshold_width = float(getattr(cfg, 'threshold_width', 0.9))
        self.threshold_offset_x = float(getattr(cfg, 'threshold_offset_x', 2.0))
        self.corridor_width = float(getattr(cfg, 'corridor_width', self.threshold_width))
        self.enable_corridor_walls = bool(getattr(cfg, 'enable_corridor_walls', True))
        self.corridor_wall_height = float(getattr(cfg, 'corridor_wall_height', 0.35))
        self.corridor_wall_thickness = float(getattr(cfg, 'corridor_wall_thickness', 0.08))
        self.corridor_margin = float(getattr(cfg, 'corridor_margin', 0.1))
        self.scene_curriculum_dir = getattr(cfg, 'scene_curriculum_dir', None)
        self.scene_curriculum_single_level = getattr(cfg, 'scene_curriculum_single_level', None)
        self.scene_curriculum_content_length = getattr(cfg, 'scene_curriculum_content_length', None)
        self.scene_curriculum_content_width = getattr(cfg, 'scene_curriculum_content_width', None)
        self.scene_curriculum_metadata = None
        self.scene_curriculum_levels = []
        if self.terrain_type_mode == 'scene_curriculum':
            self._configure_scene_curriculum_from_dir()

        self.patch_map: Dict[Tuple[int, int], TerrainPatch] = {}
        self.env_origins = np.zeros((self.num_rows, self.num_cols, 3), dtype=np.float32)
        self.global_heightfield: np.ndarray | None = None
        self.global_slope_angle_deg: float | None = None
        self.global_slope: float | None = None
        self._build_layout()

    def _build_layout(self) -> None:
        for row in range(self.num_rows):
            difficulty = row / max(1, self.num_rows - 1)
            for col in range(self.num_cols):
                start_x = self.border + row * self.patch_length
                start_y = self.border + col * self.patch_width
                center_x = start_x + self.patch_length * 0.5
                center_y = start_y + self.patch_width * 0.5
                spawn_x = start_x + self.spawn_offset_x
                spawn_y = start_y + self.spawn_offset_y
                self.patch_map[(row, col)] = TerrainPatch(
                    row=row,
                    col=col,
                    terrain_type=self.terrain_type_mode,
                    start_x=start_x,
                    start_y=start_y,
                    center_x=center_x,
                    center_y=center_y,
                    spawn_x=spawn_x,
                    spawn_y=spawn_y,
                    spawn_z=0.0,
                    difficulty=difficulty,
                    metadata={},
                )

        if self.terrain_type_mode == 'slope':
            self._finalize_slope_layout()
        elif self.terrain_type_mode == 'heightfield':
            self._finalize_heightfield_layout()
        elif self.terrain_type_mode == 'scene_curriculum':
            self._finalize_scene_curriculum_layout()
        else:
            self._finalize_threshold_layout()

    def _configure_scene_curriculum_from_dir(self) -> None:
        if not self.scene_curriculum_dir:
            raise ValueError("terrain_type_mode='scene_curriculum' requires cfg.scene_curriculum_dir")
        metadata = load_scene_curriculum_metadata(self.scene_curriculum_dir)
        if not metadata.curriculum_applicable:
            raise ValueError(f"Scene curriculum at {self.scene_curriculum_dir} is not applicable.")
        self.scene_curriculum_metadata = metadata
        self.scene_curriculum_levels = list(metadata.levels)
        if self.scene_curriculum_single_level is not None:
            level_idx = int(np.clip(int(self.scene_curriculum_single_level), 0, len(self.scene_curriculum_levels) - 1))
            self.scene_curriculum_levels = [self.scene_curriculum_levels[level_idx]]
        self.num_rows = len(self.scene_curriculum_levels)
        self.num_cols = 1
        self.scene_curriculum_content_length = float(
            self.scene_curriculum_content_length if self.scene_curriculum_content_length is not None else metadata.patch_length
        )
        self.scene_curriculum_content_width = float(
            self.scene_curriculum_content_width if self.scene_curriculum_content_width is not None else metadata.patch_width
        )
        self.patch_length = max(float(self.patch_length), self.scene_curriculum_content_length)
        self.patch_width = max(float(self.patch_width), self.scene_curriculum_content_width)
        self.total_length = self.patch_length * self.num_rows
        self.total_width = self.patch_width * self.num_cols
        self.spawn_offset_y = 0.5 * self.patch_width
        self.threshold_height_levels = [float(level.threshold_height) for level in self.scene_curriculum_levels]

    def _sample_scene_height(self, scene, local_x: float, local_y: float) -> float:
        x_alpha = (local_x - scene.origin_xy[0]) / max(scene.resolution, 1e-6)
        y_alpha = (local_y - scene.origin_xy[1]) / max(scene.resolution, 1e-6)
        col = int(np.clip(round(x_alpha), 0, scene.height_map.shape[1] - 1))
        row = int(np.clip(round(y_alpha), 0, scene.height_map.shape[0] - 1))
        return float(scene.height_map[row, col])

    def _resolve_slope_angle_deg(self) -> float:
        if self.slope_angle_override is not None:
            return float(self.slope_angle_override)
        return float(self.slope_range_deg[0])

    def _resolve_heightfield_amplitude(self) -> float:
        if self.heightfield_height_override is not None:
            return float(self.heightfield_height_override)
        return float(self.heightfield_height_range[0])

    def _finalize_slope_layout(self) -> None:
        angle_deg = self._resolve_slope_angle_deg()
        slope = math.tan(math.radians(angle_deg))
        self.global_slope_angle_deg = angle_deg
        self.global_slope = slope

        for key, patch in list(self.patch_map.items()):
            spawn_z = slope * max(0.0, patch.spawn_x - self.border)
            metadata = {
                'angle_deg': angle_deg,
                'slope': slope,
                'rise_total': slope * self.total_length,
                'terrain_length_total': self.total_length,
                'terrain_width_total': self.total_width,
            }
            self.patch_map[key] = replace(patch, spawn_z=spawn_z, metadata=metadata)
            self.env_origins[patch.row, patch.col] = np.array([patch.spawn_x, patch.spawn_y, spawn_z], dtype=np.float32)

    def _finalize_heightfield_layout(self) -> None:
        amplitude = self._resolve_heightfield_amplitude()
        heights = self._generate_heightfield(amplitude)

        for patch in self.patch_map.values():
            spawn_ix = self._world_to_heightfield_index(patch.spawn_x, self.border, self.total_length, self.heightfield_ncol)
            spawn_iy = self._world_to_heightfield_index(patch.spawn_y, self.border, self.total_width, self.heightfield_nrow)
            flat_r = self.heightfield_spawn_flat_radius
            x0 = max(0, spawn_ix - flat_r)
            x1 = min(self.heightfield_ncol, spawn_ix + flat_r + 1)
            y0 = max(0, spawn_iy - flat_r)
            y1 = min(self.heightfield_nrow, spawn_iy + flat_r + 1)
            spawn_height = float(heights[spawn_iy, spawn_ix])
            heights[y0:y1, x0:x1] = spawn_height

        self.global_heightfield = heights
        peak_height = float(np.max(heights))
        valley_height = float(np.min(heights))
        for key, patch in list(self.patch_map.items()):
            spawn_ix = self._world_to_heightfield_index(patch.spawn_x, self.border, self.total_length, self.heightfield_ncol)
            spawn_iy = self._world_to_heightfield_index(patch.spawn_y, self.border, self.total_width, self.heightfield_nrow)
            spawn_z = float(heights[spawn_iy, spawn_ix])
            metadata = {
                'height_amplitude': amplitude,
                'peak_height': peak_height,
                'valley_height': valley_height,
                'heightfield_nrow': self.heightfield_nrow,
                'heightfield_ncol': self.heightfield_ncol,
                'smooth_steps': self.heightfield_smooth_steps,
            }
            self.patch_map[key] = replace(patch, spawn_z=spawn_z, metadata=metadata)
            self.env_origins[patch.row, patch.col] = np.array([patch.spawn_x, patch.spawn_y, spawn_z], dtype=np.float32)

    def _finalize_threshold_layout(self) -> None:
        for key, patch in list(self.patch_map.items()):
            threshold_height = self._resolve_threshold_height(patch.row)
            threshold_start_x = patch.start_x + self.threshold_offset_x
            threshold_end_x = threshold_start_x + self.threshold_depth
            corridor_center_y = patch.start_y + self.patch_width * 0.5
            corridor_half_width = 0.5 * self.corridor_width
            success_x = threshold_end_x + float(getattr(self.cfg, 'success_x_margin', 0.5))
            metadata = {
                'threshold_height': threshold_height,
                'threshold_depth': self.threshold_depth,
                'threshold_width': self.threshold_width,
                'threshold_start_x': threshold_start_x,
                'threshold_end_x': threshold_end_x,
                'threshold_center_x': 0.5 * (threshold_start_x + threshold_end_x),
                'corridor_center_y': corridor_center_y,
                'corridor_width': self.corridor_width,
                'corridor_half_width': corridor_half_width,
                'success_x': success_x,
            }
            self.patch_map[key] = replace(patch, spawn_z=0.0, metadata=metadata)
            self.env_origins[patch.row, patch.col] = np.array([patch.spawn_x, corridor_center_y, 0.0], dtype=np.float32)

    def _finalize_scene_curriculum_layout(self) -> None:
        if self.scene_curriculum_metadata is None:
            raise ValueError("Scene curriculum metadata not loaded.")

        resolution = float(self.scene_curriculum_metadata.resolution)
        scene_rows = self.scene_curriculum_metadata.scene_shape[0]
        scene_cols = self.scene_curriculum_metadata.scene_shape[1]
        patch_rows = int(round(self.patch_width / resolution))
        patch_cols = int(round(self.patch_length / resolution))
        global_rows = patch_rows * self.num_cols
        global_cols = patch_cols * self.num_rows
        heights = np.zeros((global_rows, global_cols), dtype=np.float32)

        ref_level = self.scene_curriculum_levels[0]
        scene_offset_x = np.clip(
            self.threshold_offset_x - float(ref_level.threshold_start_x),
            0.0,
            max(0.0, self.patch_length - self.scene_curriculum_content_length),
        )
        scene_offset_y = np.clip(
            0.5 * (self.patch_width - self.scene_curriculum_content_width),
            0.0,
            max(0.0, self.patch_width - self.scene_curriculum_content_width),
        )
        scene_row_offset = int(round(scene_offset_y / resolution))
        scene_col_offset = int(round(scene_offset_x / resolution))

        for key, patch in list(self.patch_map.items()):
            level = self.scene_curriculum_levels[patch.row]
            scene = load_terrain_scene(Path(self.scene_curriculum_dir) / level.scene_npz)
            row_offset = patch.col * patch_rows + scene_row_offset
            col_offset = patch.row * patch_cols + scene_col_offset
            heights[row_offset:row_offset + scene_rows, col_offset:col_offset + scene_cols] = scene.height_map

            threshold_start_x = patch.start_x + scene_offset_x + level.threshold_start_x
            threshold_end_x = patch.start_x + scene_offset_x + level.threshold_end_x
            threshold_center_x = 0.5 * (threshold_start_x + threshold_end_x)
            # MuJoCo hfield image rows are visually flipped relative to our raw scene row indexing,
            # so convert local y coordinates into the rendered patch frame before building task metadata.
            rendered_local_y_min = self.scene_curriculum_content_width - float(level.threshold_y_max)
            rendered_local_y_max = self.scene_curriculum_content_width - float(level.threshold_y_min)
            threshold_y_min = patch.start_y + scene_offset_y + rendered_local_y_min
            threshold_y_max = patch.start_y + scene_offset_y + rendered_local_y_max
            threshold_center_y = 0.5 * (threshold_y_min + threshold_y_max)
            spawn_scene_x = float(np.clip(self.spawn_offset_x - scene_offset_x, scene.origin_xy[0], scene.origin_xy[0] + self.scene_curriculum_content_length))
            rendered_spawn_local_y = float(np.clip(self.spawn_offset_y - scene_offset_y, 0.0, self.scene_curriculum_content_width))
            raw_spawn_local_y = self.scene_curriculum_content_width - rendered_spawn_local_y
            spawn_scene_y = float(np.clip(scene.origin_xy[1] + raw_spawn_local_y, scene.origin_xy[1], scene.origin_xy[1] + self.scene_curriculum_content_width))
            spawn_z = self._sample_scene_height(scene, spawn_scene_x, spawn_scene_y)
            metadata = {
                'threshold_height': float(level.threshold_height),
                'threshold_depth': float(level.threshold_end_x - level.threshold_start_x),
                'threshold_width': float(level.threshold_width),
                'threshold_start_x': float(threshold_start_x),
                'threshold_end_x': float(threshold_end_x),
                'threshold_y_min': float(threshold_y_min),
                'threshold_y_max': float(threshold_y_max),
                'threshold_center_x': float(threshold_center_x),
                'corridor_center_y': float(threshold_center_y),
                'corridor_width': float(level.corridor_width),
                'corridor_half_width': 0.5 * float(level.corridor_width),
                'success_x': float(patch.start_x + scene_offset_x + level.success_x),
                'recommended_lateral_error_threshold': float(level.lateral_error_threshold),
                'scene_level_index': int(level.level_index),
                'scene_level_scale': float(level.level_scale),
                'scene_offset_x': float(scene_offset_x),
                'scene_offset_y': float(scene_offset_y),
            }
            self.patch_map[key] = replace(
                patch,
                terrain_type='scene_curriculum',
                spawn_y=threshold_center_y,
                spawn_z=spawn_z,
                metadata=metadata,
            )
            self.env_origins[patch.row, patch.col] = np.array([patch.spawn_x, threshold_center_y, spawn_z], dtype=np.float32)

        self.global_heightfield = heights

    def _resolve_threshold_height(self, row: int) -> float:
        if not self.threshold_height_levels:
            return self.threshold_height
        idx = int(np.clip(row, 0, len(self.threshold_height_levels) - 1))
        return float(self.threshold_height_levels[idx])

    def _generate_heightfield(self, amplitude: float) -> np.ndarray:
        heights = self.rng.normal(0.0, 1.0, size=(self.heightfield_nrow, self.heightfield_ncol)).astype(np.float32)
        for _ in range(max(1, self.heightfield_smooth_steps)):
            heights = (
                heights
                + np.roll(heights, 1, axis=0)
                + np.roll(heights, -1, axis=0)
                + np.roll(heights, 1, axis=1)
                + np.roll(heights, -1, axis=1)
            ) / 5.0
        heights -= float(np.min(heights))
        peak = float(np.max(heights))
        if peak > 1e-6:
            heights /= peak
        heights *= float(amplitude)
        return heights

    def _generate_slope_heightfield(self) -> np.ndarray:
        x_coords = np.linspace(0.0, self.total_length, self.heightfield_ncol, dtype=np.float32)
        heights_1d = self.global_slope * x_coords
        return np.repeat(heights_1d[np.newaxis, :], self.heightfield_nrow, axis=0)

    def _world_to_heightfield_index(self, coord: float, terrain_start: float, terrain_extent: float, num_samples: int) -> int:
        alpha = (coord - terrain_start) / max(1e-6, terrain_extent)
        return int(np.clip(round(alpha * (num_samples - 1)), 0, num_samples - 1))

    def write_scene(self, base_scene_path: Path) -> Path:
        tree = ET.parse(base_scene_path)
        root = tree.getroot()
        for include in root.findall('include'):
            include_file = include.get('file')
            if include_file:
                include_path = (base_scene_path.parent / include_file).resolve()
                include.set('file', str(self._materialize_include(include_path)))
        asset = root.find('asset')
        if asset is None:
            asset = ET.SubElement(root, 'asset')
        worldbody = root.find('worldbody')
        if worldbody is None:
            worldbody = ET.SubElement(root, 'worldbody')

        for geom in list(worldbody.findall('geom')):
            if geom.get('name') == 'floor' or geom.get('type') == 'plane':
                worldbody.remove(geom)

        self._append_assets(asset)
        if self.terrain_type_mode == 'slope':
            self._append_global_slope(asset, worldbody)
        elif self.terrain_type_mode == 'heightfield':
            self._append_global_heightfield(asset, worldbody)
        elif self.terrain_type_mode == 'scene_curriculum':
            self._append_scene_curriculum_course(asset, worldbody)
        else:
            self._append_threshold_course(worldbody)

        self._indent_xml(root)
        with tempfile.NamedTemporaryFile(prefix='terrain_scene_', suffix='.xml', delete=False, dir=str(self.generated_dir)) as tmp:
            tree.write(tmp.name, encoding='unicode')
            return Path(tmp.name)

    def _materialize_include(self, include_path: Path) -> Path:
        tree = ET.parse(include_path)
        root = tree.getroot()

        compiler = root.find('compiler')
        meshdir = self._resolve_compiler_dir(include_path, compiler, 'meshdir')
        texturedir = self._resolve_compiler_dir(include_path, compiler, 'texturedir')
        asset = root.find('asset')
        if asset is not None:
            for tag, base_dir in (('mesh', meshdir), ('texture', texturedir), ('hfield', include_path.parent)):
                for elem in asset.findall(tag):
                    file_attr = elem.get('file')
                    if file_attr and not Path(file_attr).is_absolute():
                        resolved = (base_dir / file_attr).resolve()
                        elem.set('file', str(resolved))
        if compiler is not None:
            for attr in ('meshdir', 'texturedir'):
                if attr in compiler.attrib:
                    compiler.attrib.pop(attr)

        for include in root.findall('include'):
            nested = include.get('file')
            if nested:
                include.set('file', str(self._materialize_include((include_path.parent / nested).resolve())))

        self._indent_xml(root)
        with tempfile.NamedTemporaryFile(prefix=f'{include_path.stem}_', suffix='.xml', delete=False, dir=str(self.generated_dir)) as tmp:
            tree.write(tmp.name, encoding='unicode')
            return Path(tmp.name)

    def _resolve_compiler_dir(self, include_path: Path, compiler, attr: str) -> Path:
        if compiler is None:
            return include_path.parent
        rel_dir = compiler.get(attr)
        if not rel_dir:
            return include_path.parent
        return (include_path.parent / rel_dir).resolve()

    def _append_assets(self, asset) -> None:
        existing_materials = {elem.get('name') for elem in asset.findall('material')}
        if 'terrain_slope' not in existing_materials:
            ET.SubElement(asset, 'material', name='terrain_slope', rgba='0.66 0.57 0.43 1', reflectance='0.06')
        if 'terrain_heightfield' not in existing_materials:
            ET.SubElement(asset, 'material', name='terrain_heightfield', rgba='0.43 0.49 0.39 1', reflectance='0.04')

    def _append_global_slope(self, asset, worldbody) -> None:
        heights = self._generate_slope_heightfield()
        peak_height = max(1e-4, float(np.max(heights)))
        image_path = self._write_heightfield_png(heights)
        hfield_name = 'terrain_slope_hfield'
        center_x = self.border + self.total_length * 0.5
        center_y = self.border + self.total_width * 0.5

        ET.SubElement(
            asset,
            'hfield',
            name=hfield_name,
            file=str(image_path),
            size=f'{self.total_length * 0.5:.4f} {self.total_width * 0.5:.4f} {peak_height:.4f} 0.02',
        )
        ET.SubElement(
            worldbody,
            'geom',
            name='terrain_slope',
            type='hfield',
            hfield=hfield_name,
            pos=f'{center_x:.4f} {center_y:.4f} 0.0000',
            material='terrain_slope',
            friction='1.0 0.1 0.1',
        )

    def _append_global_heightfield(self, asset, worldbody) -> None:
        heights = self.global_heightfield
        peak_height = max(1e-4, float(np.max(heights)))
        image_path = self._write_heightfield_png(heights)
        hfield_name = 'terrain_hfield'
        center_x = self.border + self.total_length * 0.5
        center_y = self.border + self.total_width * 0.5

        ET.SubElement(
            asset,
            'hfield',
            name=hfield_name,
            file=str(image_path),
            size=f'{self.total_length * 0.5:.4f} {self.total_width * 0.5:.4f} {peak_height:.4f} 0.02',
        )
        ET.SubElement(
            worldbody,
            'geom',
            name='terrain_heightfield',
            type='hfield',
            hfield=hfield_name,
            pos=f'{center_x:.4f} {center_y:.4f} 0.0000',
            material='terrain_heightfield',
            friction='1.0 0.1 0.1',
        )

    def _append_scene_curriculum_course(self, asset, worldbody) -> None:
        center_x = self.border + self.total_length * 0.5
        center_y = self.border + self.total_width * 0.5
        ET.SubElement(
            worldbody,
            'geom',
            name='terrain_floor',
            type='plane',
            pos=f'{center_x:.4f} {center_y:.4f} 0.0000',
            size=f'{self.total_length:.4f} {self.total_width:.4f} 0.1000',
            rgba='0.55 0.55 0.55 1',
            friction='1.0 0.1 0.1',
        )
        for patch in self.patch_map.values():
            meta = patch.metadata
            level = self.scene_curriculum_levels[patch.row]
            scene = load_terrain_scene(Path(self.scene_curriculum_dir) / level.scene_npz)
            scene_width, scene_length = scene.size_xy
            peak_height = max(1e-4, float(np.max(scene.height_map)))
            image_path = self._write_heightfield_png(scene.height_map)
            hfield_name = f'scene_curriculum_hfield_r{patch.row}_c{patch.col}'
            ET.SubElement(
                asset,
                'hfield',
                name=hfield_name,
                file=str(image_path),
                size=f'{scene_width * 0.5:.4f} {scene_length * 0.5:.4f} {peak_height:.4f} 0.02',
            )
            scene_center_x = patch.start_x + float(meta.get('scene_offset_x', 0.0)) + scene_width * 0.5
            scene_center_y = patch.start_y + float(meta.get('scene_offset_y', 0.0)) + scene_length * 0.5
            ET.SubElement(
                worldbody,
                'geom',
                name=f'scene_curriculum_r{patch.row}_c{patch.col}',
                type='hfield',
                hfield=hfield_name,
                pos=f'{scene_center_x:.4f} {scene_center_y:.4f} 0.0000',
                material='terrain_heightfield',
                friction='1.0 0.1 0.1',
            )
        if not self.enable_corridor_walls:
            return
        for patch in self.patch_map.values():
            meta = patch.metadata
            threshold_y_min = float(meta.get('threshold_y_min', meta['corridor_center_y'] - 0.5 * meta.get('threshold_width', self.corridor_width)))
            threshold_y_max = float(meta.get('threshold_y_max', meta['corridor_center_y'] + 0.5 * meta.get('threshold_width', self.corridor_width)))
            wall_half_length = 0.5 * self.patch_length
            wall_half_width = 0.5 * self.corridor_wall_thickness
            wall_half_height = 0.5 * self.corridor_wall_height
            wall_center_x = patch.start_x + wall_half_length
            left_wall_y = threshold_y_min - self.corridor_margin - wall_half_width
            right_wall_y = threshold_y_max + self.corridor_margin + wall_half_width
            for wall_name, wall_y in (
                (f'corridor_left_r{patch.row}_c{patch.col}', left_wall_y),
                (f'corridor_right_r{patch.row}_c{patch.col}', right_wall_y),
            ):
                ET.SubElement(
                    worldbody,
                    'geom',
                    name=wall_name,
                    type='box',
                    pos=f'{wall_center_x:.4f} {wall_y:.4f} {wall_half_height:.4f}',
                    size=f'{wall_half_length:.4f} {wall_half_width:.4f} {wall_half_height:.4f}',
                    rgba='0.35 0.35 0.38 1',
                    friction='1.0 0.1 0.1',
                )

    def _append_threshold_course(self, worldbody) -> None:
        center_x = self.border + self.total_length * 0.5
        center_y = self.border + self.total_width * 0.5
        ET.SubElement(
            worldbody,
            'geom',
            name='terrain_floor',
            type='plane',
            pos=f'{center_x:.4f} {center_y:.4f} 0.0000',
            size=f'{self.total_length:.4f} {self.total_width:.4f} 0.1000',
            rgba='0.55 0.55 0.55 1',
            friction='1.0 0.1 0.1',
        )

        for patch in self.patch_map.values():
            meta = patch.metadata
            threshold_center_x = float(meta['threshold_center_x'])
            threshold_center_y = float(meta['corridor_center_y'])
            threshold_half_depth = 0.5 * self.threshold_depth
            threshold_half_width = 0.5 * self.threshold_width
            threshold_half_height = 0.5 * float(meta['threshold_height'])
            ET.SubElement(
                worldbody,
                'geom',
                name=f'threshold_r{patch.row}_c{patch.col}',
                type='box',
                pos=f'{threshold_center_x:.4f} {threshold_center_y:.4f} {threshold_half_height:.4f}',
                size=f'{threshold_half_depth:.4f} {threshold_half_width:.4f} {threshold_half_height:.4f}',
                rgba='0.72 0.48 0.26 1',
                friction='1.2 0.1 0.1',
            )

            if not self.enable_corridor_walls:
                continue

            wall_half_length = 0.5 * self.patch_length
            wall_half_width = 0.5 * self.corridor_wall_thickness
            wall_half_height = 0.5 * self.corridor_wall_height
            wall_center_x = patch.start_x + wall_half_length
            left_wall_y = threshold_center_y - 0.5 * self.corridor_width - wall_half_width - self.corridor_margin
            right_wall_y = threshold_center_y + 0.5 * self.corridor_width + wall_half_width + self.corridor_margin
            for wall_name, wall_y in (
                (f'corridor_left_r{patch.row}_c{patch.col}', left_wall_y),
                (f'corridor_right_r{patch.row}_c{patch.col}', right_wall_y),
            ):
                ET.SubElement(
                    worldbody,
                    'geom',
                    name=wall_name,
                    type='box',
                    pos=f'{wall_center_x:.4f} {wall_y:.4f} {wall_half_height:.4f}',
                    size=f'{wall_half_length:.4f} {wall_half_width:.4f} {wall_half_height:.4f}',
                    rgba='0.35 0.35 0.38 1',
                    friction='1.0 0.1 0.1',
                )

    def _write_heightfield_png(self, heights: np.ndarray) -> Path:
        peak = float(np.max(heights))
        if peak <= 1e-6:
            normalized = np.zeros_like(heights, dtype=np.uint8)
        else:
            normalized = np.clip(np.round(255.0 * heights / peak), 0, 255).astype(np.uint8)
        with tempfile.NamedTemporaryFile(prefix='terrain_hf_', suffix='.png', delete=False, dir=str(self.generated_dir)) as tmp:
            Image.fromarray(normalized, mode='L').save(tmp.name)
            return Path(tmp.name)

    def _indent_xml(self, elem, level: int = 0) -> None:
        indent = '\n' + level * '  '
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = indent + '  '
            for child in elem:
                self._indent_xml(child, level + 1)
            if not elem[-1].tail or not elem[-1].tail.strip():
                elem[-1].tail = indent
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = indent
