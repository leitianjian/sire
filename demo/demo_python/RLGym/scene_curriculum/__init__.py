from pathlib import Path
from typing import Optional

from .core import (
    DEFAULT_CURRICULUM_LEVEL_SCALES,
    analyze_scene_for_curriculum,
    apply_scene_curriculum_to_env_cfg,
    auto_curriculum_target_heights,
    build_scene_curriculum_from_npz,
    export_scene_curriculum,
    generate_scene_curriculum,
    load_scene_curriculum_metadata,
    load_terrain_scene,
    target_heights_to_level_scales,
)
from .types import SceneCurriculumArtifacts, SceneCurriculumLevel, SceneCurriculumMetadata, SceneObstacleStats, TerrainScene

SCENE_RECONSTRUCTIONS_DIR = Path(__file__).resolve().parents[2] / "resources" / "scene_reconstructions"


def resolve_scene_npz(name_or_path: Optional[str]) -> Optional[Path]:
    """Resolve a scene npz reference.

    - Empty / None -> None.
    - Absolute path -> used as-is.
    - Bare filename (no path separator) -> looked up under resources/scene_reconstructions/.
    - Relative path with separators -> resolved against repo root (backward compat).
    """
    if not name_or_path:
        return None
    raw = str(name_or_path).strip()
    if not raw:
        return None
    candidate = Path(raw).expanduser()
    if candidate.is_absolute():
        resolved = candidate
    elif raw == candidate.name:
        resolved = SCENE_RECONSTRUCTIONS_DIR / candidate.name
    else:
        repo_root = Path(__file__).resolve().parents[2]
        resolved = (repo_root / candidate).resolve()
    if not resolved.exists():
        raise FileNotFoundError(
            f"Scene reconstruction npz not found: {resolved} "
            f"(looked up from '{raw}', scene_reconstructions dir={SCENE_RECONSTRUCTIONS_DIR})"
        )
    return resolved


__all__ = [
    "DEFAULT_CURRICULUM_LEVEL_SCALES",
    "SCENE_RECONSTRUCTIONS_DIR",
    "TerrainScene",
    "SceneObstacleStats",
    "SceneCurriculumLevel",
    "SceneCurriculumMetadata",
    "SceneCurriculumArtifacts",
    "load_terrain_scene",
    "load_scene_curriculum_metadata",
    "analyze_scene_for_curriculum",
    "target_heights_to_level_scales",
    "auto_curriculum_target_heights",
    "build_scene_curriculum_from_npz",
    "generate_scene_curriculum",
    "export_scene_curriculum",
    "apply_scene_curriculum_to_env_cfg",
    "resolve_scene_npz",
]
