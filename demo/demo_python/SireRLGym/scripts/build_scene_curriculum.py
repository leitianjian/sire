from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[2]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from SireRLGym.scene_curriculum import (
    analyze_scene_for_curriculum,
    auto_curriculum_target_heights,
    build_scene_curriculum_from_npz,
    load_scene_curriculum_metadata,
    load_terrain_scene,
)


def _parse_float_list(value: str | None):
    if value is None:
        return None
    values = []
    for item in str(value).split(","):
        item = item.strip()
        if not item:
            continue
        values.append(float(item))
    return values or None


def parse_args():
    parser = argparse.ArgumentParser(description="Build a scene curriculum directory from a TerrainScene npz.")
    parser.add_argument("--scene-npz", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=Path("resources/scene_curricula"))
    parser.add_argument("--scene-name", type=str, default=None)
    parser.add_argument("--level-scales", type=str, default=None, help="Comma-separated scales. If omitted, auto-generate 0.04m start + 0.02m increments up to max.")
    parser.add_argument(
        "--target-heights",
        type=str,
        default=None,
        help="Comma-separated target obstacle heights in meters. Overrides auto curriculum and --level-scales.",
    )
    parser.add_argument("--obstacle-height-threshold", type=float, default=None)
    parser.add_argument("--focus-corridor-half-width", type=float, default=None)
    parser.add_argument("--min-component-cells", type=int, default=None)
    return parser.parse_args()


def main():
    args = parse_args()
    scene_path = args.scene_npz.expanduser().resolve()
    if not scene_path.exists():
        raise FileNotFoundError(f"Scene npz not found: {scene_path}")
    scene_name = args.scene_name or scene_path.stem
    output_dir = (args.output_dir.expanduser().resolve() / scene_name)
    requested_target_heights = _parse_float_list(args.target_heights)
    requested_level_scales = _parse_float_list(args.level_scales)

    target_heights = requested_target_heights
    if target_heights is None and requested_level_scales is None:
        base_metadata = analyze_scene_for_curriculum(
            load_terrain_scene(scene_path),
            obstacle_height_threshold=args.obstacle_height_threshold,
            focus_corridor_half_width=args.focus_corridor_half_width,
            min_component_cells=args.min_component_cells,
        )
        if base_metadata.obstacle_stats is None:
            raise ValueError("Automatic curriculum generation requires a detected primary obstacle.")
        target_heights = auto_curriculum_target_heights(base_metadata.obstacle_stats.height_max)

    exported_metadata, artifacts, resolved_target_heights = build_scene_curriculum_from_npz(
        scene_path,
        args.output_dir,
        scene_name=scene_name,
        target_heights=target_heights,
        level_scales=requested_level_scales,
        obstacle_height_threshold=args.obstacle_height_threshold,
        focus_corridor_half_width=args.focus_corridor_half_width,
        min_component_cells=args.min_component_cells,
    )

    print(f"scene_curriculum scene={scene_path}")
    print(
        f"scene_curriculum_summary applicable={exported_metadata.curriculum_applicable} "
        f"levels={len(exported_metadata.levels)} output_dir={artifacts.output_dir}"
    )
    if exported_metadata.obstacle_stats is not None:
        stats = exported_metadata.obstacle_stats
        print(
            "scene_curriculum_obstacle "
            f"height_max={stats.height_max:.4f} height_p95={stats.height_p95:.4f} "
            f"x_start={stats.x_start:.4f} x_end={stats.x_end:.4f} "
            f"y_min={stats.y_min:.4f} y_max={stats.y_max:.4f} width={stats.width:.4f} length={stats.length:.4f}"
        )
    if exported_metadata.levels:
        if resolved_target_heights:
            print(f"scene_curriculum_targets heights={','.join(f'{v:.4f}' for v in resolved_target_heights)}")
        for level in exported_metadata.levels:
            print(
                "scene_curriculum_level "
                f"idx={level.level_index} scale={level.level_scale:.2f} "
                f"threshold_height={level.threshold_height:.4f} "
                f"scene_npz={level.scene_npz}"
            )


if __name__ == "__main__":
    main()
