"""Shared file paths and calibrated transforms for scanned Sire environments."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Optional

import numpy as np


def _numeric_setting(config, name, default, shape):
    value = config.get(name, default)
    values = value if isinstance(value, list) else [value]
    if any(type(item) not in (int, float) for item in values):
        raise ValueError(f"'{name}' must contain finite numbers")
    try:
        array = np.asarray(value, dtype=np.float64)
    except (OverflowError, ValueError) as exc:
        raise ValueError(f"'{name}' must contain finite numbers") from exc
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError(f"'{name}' must have shape {shape} and contain finite numbers")
    return array


def _asset_path(config, name, directory, suffixes):
    value = config.get(name)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"'{name}' must specify a file path")
    path = (directory / Path(value).expanduser()).resolve()
    if path.suffix.lower() not in suffixes:
        raise ValueError(f"'{name}' must be a {' or '.join(suffixes)} file: {path}")
    if not path.is_file():
        raise ValueError(f"'{name}' file does not exist: {path}")
    return path


def _unit_quaternions(quats, name):
    magnitude = np.max(np.abs(quats), axis=-1, keepdims=True)
    if np.any(magnitude == 0):
        raise ValueError(f"{name} must contain nonzero WXYZ quaternions")
    scaled = quats / magnitude
    return scaled / np.linalg.norm(scaled, axis=-1, keepdims=True)


def _asset_pose(config):
    position = _numeric_setting(config, "position", [0, 0, 0], (3,))
    quat = _numeric_setting(config, "rotation_wxyz", [1, 0, 0, 0], (4,))
    scale = float(_numeric_setting(config, "scale", 1, ()))
    if scale <= 0:
        raise ValueError("'scale' must be strictly positive")
    w, x, y, z = _unit_quaternions(quat, "'rotation_wxyz'")
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = position
    return transform, scale


@dataclass(frozen=True)
class SceneAsset:
    """Aligned appearance, depth, and collision files for a static scene."""

    source: Path
    depth_mesh: Path
    collision_mesh: Path
    gaussians: Optional[Path]
    pose: np.ndarray
    scale: float
    sh_degree: Optional[int]
    radius_clip: float

    @classmethod
    def load(cls, path: str | Path) -> SceneAsset:
        """Resolve files and the rigid asset pose from one scene JSON."""
        path = Path(path).expanduser().resolve()
        try:
            config = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(config, dict):
                raise ValueError("scene JSON must be an object")
            depth_mesh = _asset_path(config, "depth_mesh", path.parent, (".ply", ".obj"))
            collision_mesh = (_asset_path(config, "collision_mesh", path.parent, (".ply", ".obj", ".stl"))
                              if "collision_mesh" in config else depth_mesh)
            gaussians = (_asset_path(config, "gaussians", path.parent, (".ply",))
                         if "gaussians" in config else None)
            pose, scale = _asset_pose(config)
            degree = config.get("sh_degree")
            if "sh_degree" in config and (type(degree) is not int or degree < 0):
                raise ValueError("'sh_degree' must be a nonnegative integer")
            radius_clip = float(_numeric_setting(config, "radius_clip", 0, ()))
            if radius_clip < 0:
                raise ValueError("'radius_clip' must be nonnegative")
        except (OSError, ValueError) as error:
            raise ValueError(f"Invalid scene asset {path}: {error}") from error
        pose.setflags(write=False)
        return cls(path, depth_mesh, collision_mesh, gaussians, pose, scale, degree, radius_clip)

    @property
    def asset_to_world(self) -> np.ndarray:
        """Return the 4x4 transform including uniform asset scale."""
        transform = self.pose.copy()
        transform[:3, :3] *= self.scale
        return transform

    def replace_collision(self, simulator, geometry_id: int):
        """Replace an anchored world geometry before Simulator.init()."""
        return simulator.physicsEngine().replaceStaticMeshGeometry(
            geometry_id, str(self.collision_mesh), self.scale, self.pose.ravel().tolist()
        )
