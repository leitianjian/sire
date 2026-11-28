"""Load INRIA 3D Gaussian Splatting assets for the RGB-D viewer."""

from __future__ import annotations

from dataclasses import dataclass
from math import isqrt
from pathlib import Path

import numpy as np

from ..scene import SceneAsset, _unit_quaternions


@dataclass
class SplatScene:
    means: np.ndarray
    quats: np.ndarray
    scales: np.ndarray
    opacities: np.ndarray
    colors: np.ndarray
    asset_to_world: np.ndarray
    depth_mesh: Path
    sh_degree: int
    radius_clip: float


def _read_vertices(path):
    try:
        from plyfile import PlyData, PlyParseError
    except ImportError as exc:
        raise ImportError(
            "Reading Gaussian splat assets requires plyfile. Install it with "
            "`pip install plyfile`."
        ) from exc

    try:
        vertices = PlyData.read(str(path))["vertex"].data
    except (OSError, ValueError, KeyError, PlyParseError) as exc:
        raise ValueError(f"Could not read Gaussian vertices from {path}: {exc}") from exc
    if len(vertices) == 0:
        raise ValueError(f"Gaussian PLY contains no vertices: {path}")
    return vertices


def _columns(vertices, names):
    missing = [name for name in names if name not in vertices.dtype.names]
    if missing:
        raise ValueError(f"missing Gaussian vertex properties: {', '.join(missing)}")
    if any(vertices.dtype[name].kind not in "iuf" for name in names):
        raise ValueError(f"Gaussian vertex properties must be numeric scalars: {', '.join(names)}")
    array = np.column_stack([vertices[name] for name in names]).astype(np.float64)
    if not np.isfinite(array).all():
        raise ValueError(f"Gaussian vertex properties must be finite: {', '.join(names)}")
    return array


def _load_gaussians(path, requested_degree):
    vertices = _read_vertices(path)
    try:
        means = _columns(vertices, ("x", "y", "z"))
        rotations = _columns(vertices, tuple(f"rot_{i}" for i in range(4)))
        quats = _unit_quaternions(rotations, "rot_0 through rot_3")
        log_scales = _columns(vertices, tuple(f"scale_{i}" for i in range(3)))
        with np.errstate(over="ignore", under="ignore"):
            scales = np.exp(log_scales)
        opacity_logits = _columns(vertices, ("opacity",))[:, 0]
        opacities = np.exp(-np.logaddexp(0, -opacity_logits))

        dc = _columns(vertices, tuple(f"f_dc_{i}" for i in range(3)))
        rest = [name for name in vertices.dtype.names if name.startswith("f_rest_")]
        coefficient_count = 1 + len(rest) // 3
        bands = isqrt(coefficient_count)
        if len(rest) % 3 or bands * bands != coefficient_count:
            raise ValueError("f_rest_* properties must contain complete RGB SH bands")
        available_degree = bands - 1
        degree = available_degree if requested_degree is None else requested_degree
        if degree > available_degree:
            raise ValueError(
                f"'sh_degree' requests {degree}, but this PLY has degree {available_degree}"
            )
        if rest:
            higher = _columns(vertices, tuple(f"f_rest_{i}" for i in range(len(rest))))
            higher = higher.reshape(len(vertices), 3, coefficient_count - 1).transpose(0, 2, 1)
            colors = np.concatenate((dc[:, None, :], higher), axis=1)
        else:
            colors = dc[:, None, :]
        colors = colors[:, : (degree + 1) ** 2, :]

        with np.errstate(over="ignore", under="ignore"):
            arrays = tuple(np.ascontiguousarray(array, dtype=np.float32) for array in (
                means, quats, scales, opacities, colors
            ))
        if not all(np.isfinite(array).all() for array in arrays):
            raise ValueError("Gaussian positions, scales and SH coefficients must fit in float32")
        if np.any(arrays[2] <= 0):
            raise ValueError("Gaussian scales must remain strictly positive in float32")
    except ValueError as exc:
        raise ValueError(f"Invalid Gaussian PLY {path}: {exc}") from exc
    return (*arrays, degree)


def load_splat_scene(path: str | Path) -> SplatScene:
    """Read a JSON scene and load Gaussian arrays in their original asset frame."""
    scene = SceneAsset.load(path)
    if scene.gaussians is None:
        raise ValueError(f"Scene asset {scene.source} has no 'gaussians' PLY")
    means, quats, scales, opacities, colors, degree = _load_gaussians(scene.gaussians, scene.sh_degree)
    return SplatScene(
        means=means,
        quats=quats,
        scales=scales,
        opacities=opacities,
        colors=colors,
        asset_to_world=scene.asset_to_world,
        depth_mesh=scene.depth_mesh,
        sh_degree=degree,
        radius_clip=scene.radius_clip,
    )
