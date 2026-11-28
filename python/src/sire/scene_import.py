"""Import calibrated static scene assets into the Sire scene format."""

from __future__ import annotations

import argparse
import json
import math
from numbers import Integral
import os
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np


def _file(path: str | Path, label: str) -> Path:
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f"{label} file does not exist: {path}")
    return path


def _destination(path: str | Path) -> Path:
    path = Path(path).expanduser().resolve()
    if path.suffix.lower() != ".json":
        raise ValueError(f"Scene destination must be a JSON path: {path}")
    if path.exists() and not path.is_file():
        raise ValueError(f"Scene destination is not a file: {path}")
    return path


def _relative(path: Path, destination: Path) -> str:
    return Path(os.path.relpath(path, destination.parent)).as_posix()


def _positive(value: float, label: str) -> float:
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label} must be finite and positive") from exc
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{label} must be finite and positive")
    return value


def _vector(values, length: int, label: str) -> list[float]:
    try:
        values = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label} must contain {length} finite numbers") from exc
    if len(values) != length or not all(math.isfinite(value) for value in values):
        raise ValueError(f"{label} must contain {length} finite numbers")
    return list(values)


def _write_manifest(path: Path, values: dict) -> None:
    path.write_text(json.dumps(values, indent=2) + "\n", encoding="utf-8")


def _open3d():
    try:
        import open3d
    except ImportError as exc:
        raise ImportError("Mesh processing requires Open3D. Install it with `pip install open3d`.") from exc
    return open3d


def simplify_collision(mesh_path: str | Path, destination: str | Path, *, target_triangles: int) -> Path:
    """Decimate a collision mesh without changing its asset coordinate frame."""
    source = _file(mesh_path, "collision mesh")
    destination = Path(destination).expanduser().resolve()
    if destination.suffix.lower() != ".ply" or source == destination:
        raise ValueError("Choose a separate PLY file for the simplified mesh")
    if isinstance(target_triangles, bool) or not isinstance(target_triangles, Integral) or target_triangles <= 0:
        raise ValueError("target_triangles must be a positive integer")

    o3d = _open3d()
    mesh = o3d.io.read_triangle_mesh(str(source))
    if not mesh.has_triangles():
        raise ValueError(f"Collision mesh has no triangles: {source}")
    mesh.remove_duplicated_vertices()
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_unreferenced_vertices()
    if len(mesh.triangles) > target_triangles:
        mesh = mesh.simplify_quadric_decimation(int(target_triangles))
    if not mesh.has_triangles():
        raise ValueError("Simplification left no triangles")
    destination.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(dir=destination.parent) as staging:
        pending = Path(staging) / destination.name
        if not o3d.io.write_triangle_mesh(str(pending), mesh):
            raise OSError(f"Could not write simplified mesh: {destination}")
        pending.replace(destination)
    return destination


def import_gs_real2sim(
    output_dir: str | Path,
    object_name: str,
    destination: str | Path,
    *,
    position=(0, 0, 0),
    rotation_wxyz=(1, 0, 0, 0),
    scale=1,
) -> Path:
    """Write a scene referencing the aligned OBJ and 3DGS PLY from GS-Real2Sim.

    Supply the measured pose and meters per asset unit when the original asset
    frame differs from Sire world coordinates. Both exports use this transform.
    """
    if not object_name or object_name in (".", "..") or "/" in object_name or "\\" in object_name:
        raise ValueError("object_name must be a single GS-Real2Sim asset name")

    assets = Path(output_dir).expanduser() / "3d_assets"
    mesh = _file(assets / f"{object_name}.obj", "GS-Real2Sim OBJ")
    gaussians = _file(assets / f"{object_name}.ply", "GS-Real2Sim Gaussian PLY")
    destination = _destination(destination)
    position = _vector(position, 3, "position")
    rotation_wxyz = _vector(rotation_wxyz, 4, "rotation_wxyz")
    if not any(rotation_wxyz):
        raise ValueError("rotation_wxyz must be a nonzero WXYZ quaternion")
    scale = _positive(scale, "scale")

    mesh_path = _relative(mesh, destination)
    values = {
        "gaussians": _relative(gaussians, destination),
        "depth_mesh": mesh_path,
        "collision_mesh": mesh_path,
        "position": position,
        "rotation_wxyz": rotation_wxyz,
        "scale": scale,
    }
    destination.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(dir=destination.parent) as staging:
        pending = Path(staging) / destination.name
        _write_manifest(pending, values)
        pending.replace(destination)
    return destination


def _capture(path: Path):
    with np.load(path, allow_pickle=False) as data:
        required = ("depth", "camera_to_world", "intrinsics")
        missing = [name for name in required if name not in data]
        if missing:
            raise ValueError(f"RGB-D capture is missing: {', '.join(missing)}")
        depth = data["depth"]
        camera_to_world = np.asarray(data["camera_to_world"], dtype=float)
        intrinsics = np.asarray(data["intrinsics"], dtype=float)
        rgb = data["rgb"] if "rgb" in data else None

    if depth.ndim != 3 or not all(depth.shape) or depth.dtype.kind != "f":
        raise ValueError("depth must be a nonempty floating-point [T,H,W] array in meters")
    frames, height, width = depth.shape
    if rgb is not None and (rgb.shape != (frames, height, width, 3) or rgb.dtype != np.uint8):
        raise ValueError("rgb must be a uint8 [T,H,W,3] array matching depth")
    if camera_to_world.shape != (frames, 4, 4) or not np.isfinite(camera_to_world).all():
        raise ValueError("camera_to_world must be finite with shape [T,4,4]")
    rotation = camera_to_world[:, :3, :3]
    if (not np.allclose(camera_to_world[:, 3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(rotation @ rotation.transpose(0, 2, 1), np.eye(3), atol=1e-5)
            or not np.allclose(np.linalg.det(rotation), 1, atol=1e-5)):
        raise ValueError("camera_to_world must contain rigid OpenCV camera poses")
    if intrinsics.shape == (4,):
        intrinsics = np.broadcast_to(intrinsics, (frames, 4))
    if (intrinsics.shape != (frames, 4) or not np.isfinite(intrinsics).all()
            or np.any(intrinsics[:, :2] <= 0)):
        raise ValueError("intrinsics must be [fx,fy,cx,cy] or [T,4] with positive focal lengths")
    return depth, camera_to_world, intrinsics, rgb


def fuse_rgbd(
    capture_path: str | Path,
    destination: str | Path,
    *,
    gaussians: str | Path | None = None,
    voxel_size: float = 0.02,
    sdf_trunc: float = 0.06,
    depth_trunc: float = 5.0,
) -> Path:
    """Fuse calibrated metric RGB-D frames into a world-space triangle mesh.

    The input NPZ contains depth [T,H,W] in meters, camera_to_world [T,4,4]
    in the OpenCV camera convention, intrinsics [4] or [T,4] in fx,fy,cx,cy
    order, and optional uint8 rgb [T,H,W,3]. RGB affects mesh vertex colors.
    An optional external 3DGS PLY must already use the same metric world frame.
    """
    capture_path = _file(capture_path, "RGB-D capture")
    if capture_path.suffix.lower() != ".npz":
        raise ValueError(f"RGB-D capture must be an NPZ file: {capture_path}")
    destination = _destination(destination)
    mesh_path = destination.with_suffix(".mesh.ply")
    voxel_size = _positive(voxel_size, "voxel_size")
    sdf_trunc = _positive(sdf_trunc, "sdf_trunc")
    depth_trunc = _positive(depth_trunc, "depth_trunc")
    values = {"depth_mesh": mesh_path.name, "collision_mesh": mesh_path.name}
    if gaussians is not None:
        gaussian_path = _file(gaussians, "Gaussian PLY")
        if gaussian_path.suffix.lower() != ".ply":
            raise ValueError(f"Gaussian asset must be a PLY file: {gaussian_path}")
        if gaussian_path == mesh_path.resolve():
            raise ValueError(f"Fused mesh would overwrite the supplied Gaussian PLY: {gaussian_path}")
        values["gaussians"] = _relative(gaussian_path, destination)

    depth, poses, intrinsics, rgb = _capture(capture_path)
    o3d = _open3d()

    integ = o3d.pipelines.integration
    color = integ.TSDFVolumeColorType.NoColor if rgb is None else integ.TSDFVolumeColorType.RGB8
    volume = integ.ScalableTSDFVolume(
        voxel_length=voxel_size, sdf_trunc=sdf_trunc, color_type=color,
    )
    height, width = depth.shape[1:]
    blank_rgb = np.zeros((height, width, 3), dtype=np.uint8) if rgb is None else None
    for index, (frame, pose, intr) in enumerate(zip(depth, poses, intrinsics)):
        valid = np.isfinite(frame) & (frame > 0) & (frame < depth_trunc)
        if not valid.any():
            continue
        frame = np.where(valid, frame, 0).astype(np.float32)
        rgb_frame = blank_rgb if rgb is None else np.ascontiguousarray(rgb[index])
        image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(rgb_frame), o3d.geometry.Image(frame),
            depth_scale=1.0, depth_trunc=depth_trunc, convert_rgb_to_intensity=False,
        )
        calibration = o3d.camera.PinholeCameraIntrinsic(width, height, *intr)
        volume.integrate(image, calibration, np.linalg.inv(pose))

    mesh = volume.extract_triangle_mesh()
    if not mesh.has_triangles():
        raise ValueError("RGB-D fusion produced no triangles. Check camera poses and valid depths.")
    destination.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(dir=destination.parent) as staging:
        pending_mesh = Path(staging) / mesh_path.name
        pending_manifest = Path(staging) / destination.name
        if not o3d.io.write_triangle_mesh(str(pending_mesh), mesh):
            raise OSError(f"Could not write fused mesh: {mesh_path}")
        _write_manifest(pending_manifest, values)
        pending_mesh.replace(mesh_path)
        pending_manifest.replace(destination)
    return destination


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    gs = commands.add_parser("gs-real2sim", help="import a GS-Real2Sim static object")
    gs.add_argument("output_dir")
    gs.add_argument("object_name")
    gs.add_argument("destination")
    gs.add_argument("--position", type=float, nargs=3, default=(0, 0, 0), metavar=("X", "Y", "Z"))
    gs.add_argument("--rotation-wxyz", type=float, nargs=4, default=(1, 0, 0, 0), metavar=("W", "X", "Y", "Z"))
    gs.add_argument("--scale", type=float, default=1, help="meters per asset unit")
    rgbd = commands.add_parser("fuse-rgbd", help="fuse a calibrated RGB-D NPZ capture")
    rgbd.add_argument("capture_path")
    rgbd.add_argument("destination")
    rgbd.add_argument("--gaussians", help="optional external 3DGS PLY in the same metric world frame")
    rgbd.add_argument("--voxel-size", type=float, default=0.02, help="TSDF voxel size in meters")
    rgbd.add_argument("--sdf-trunc", type=float, default=0.06, help="TSDF truncation in meters")
    rgbd.add_argument("--depth-trunc", type=float, default=5.0, help="maximum valid depth in meters")
    simplify = commands.add_parser("simplify-collision", help="decimate an existing collision mesh")
    simplify.add_argument("mesh_path")
    simplify.add_argument("destination")
    simplify.add_argument("--target-triangles", type=int, required=True)
    args = vars(parser.parse_args(argv))
    command = args.pop("command")
    importer = {"gs-real2sim": import_gs_real2sim, "fuse-rgbd": fuse_rgbd,
                "simplify-collision": simplify_collision}[command]
    try:
        path = importer(**args)
    except (ValueError, OSError, ImportError) as exc:
        parser.error(str(exc))
    print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
