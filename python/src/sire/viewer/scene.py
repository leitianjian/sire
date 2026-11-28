"""Convert Sire's visual geometry and live body poses into renderer inputs."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import sire


@dataclass(frozen=True)
class RenderItem:
    body: Optional[str]
    local_pose: np.ndarray
    color: tuple[float, float, float]
    mesh: Optional[Path] = None
    size: Optional[tuple[float, float, float]] = None
    checker: bool = False


class SireScene:
    """Build visual items from the model's mesh and box geometries.

    With a mesh directory, resolve OBJ resources by basename in that directory.
    Otherwise, use the resource path stored on the model.
    An optional style(item, geometry) callback can customize each render item.
    """

    def __init__(self, model, mesh_directory=None, *, style=None):
        mesh_directory = Path(mesh_directory) if mesh_directory is not None else None
        self.parts = {model.link(i).name: model.link(i) for i in range(model.nbody)}
        self.items = []
        for name, part in self.parts.items():
            for geometry in part.geometryPool:
                local_pose = np.asarray(geometry.getPm()).reshape(4, 4)
                if isinstance(geometry, sire.MeshGeometry):
                    mesh = Path(geometry.meshShape.resourcePath())
                    if mesh_directory is not None:
                        mesh = mesh_directory / mesh.name
                    if not mesh.is_file():
                        raise FileNotFoundError(f"Sire mesh resource {mesh} is missing")
                    item = RenderItem(name, local_pose, (0.66, 0.69, 0.72), mesh=mesh)
                elif isinstance(geometry, sire.BoxGeometry):
                    size = tuple(geometry.boxShape.side())
                    item = RenderItem(name, local_pose, (0.85, 0.30, 0.09), size=size)
                else:
                    continue
                self.items.append(item if style is None else style(item, geometry))

    def poses(self):
        return {name: np.asarray(part.getPm()).reshape(4, 4) for name, part in self.parts.items()}
