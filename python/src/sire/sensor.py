"""Simulation-time RGB-D camera independent of the rendering backend."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import floor, isfinite, radians, tan
from numbers import Integral
from typing import Callable, Mapping, Optional, Tuple

import numpy as np


Pinhole = Tuple[float, float, float, float]


@dataclass(frozen=True)
class CameraConfig:
    """Camera optics and pose relative to its parent, or to the world.

    Poses map local coordinates into the parent frame. The camera uses +X
    right, +Y up, and -Z forward. The pinhole model uses pixel coordinates
    from the top-left image corner, with pixel centers at half-integers.
    Explicit intrinsics override the vertical field of view in degrees.
    Clipping planes are distances in meters along the optical axis.
    """

    width: int = 640
    height: int = 480
    fps: float = 30.0
    fov_y: Optional[float] = 60.0
    intrinsics: Optional[Pinhole] = None
    near: float = 0.05
    far: float = 20.0
    parent: Optional[str] = None
    local_pose: np.ndarray = field(default_factory=lambda: np.eye(4), compare=False)

    def __post_init__(self) -> None:
        if any(not isinstance(size, Integral) or isinstance(size, bool) or size <= 0
               for size in (self.width, self.height)):
            raise ValueError("Camera width and height must be positive integers")
        if not isfinite(self.fps) or self.fps <= 0:
            raise ValueError("Camera FPS must be positive and finite")
        if not (0 < self.near < self.far and isfinite(self.far)):
            raise ValueError("Camera clipping planes must satisfy 0 < near < far < infinity")
        if self.intrinsics is None:
            if self.fov_y is None or not 0 < self.fov_y < 180:
                raise ValueError("Camera vertical field of view must lie between 0 and 180 degrees")
        else:
            intrinsics = tuple(float(value) for value in self.intrinsics)
            if len(intrinsics) != 4 or not all(map(isfinite, intrinsics)) or min(intrinsics[:2]) <= 0:
                raise ValueError("Camera intrinsics require positive focal lengths and a finite principal point")
            object.__setattr__(self, "intrinsics", intrinsics)
        pose = np.array(self.local_pose, dtype=float, copy=True)
        if pose.shape != (4, 4) or not np.all(np.isfinite(pose)):
            raise ValueError("Camera local pose must be a finite 4x4 matrix")
        pose.setflags(write=False)
        object.__setattr__(self, "local_pose", pose)

    @property
    def pinhole(self) -> Pinhole:
        """Return effective (fx, fy, cx, cy) in pixels."""
        if self.intrinsics is not None:
            return self.intrinsics
        focal_length = self.height / (2 * tan(radians(self.fov_y) / 2))
        return focal_length, focal_length, self.width / 2, self.height / 2


@dataclass(frozen=True)
class CameraFrame:
    """RGB and optical-axis depth arrays in top-to-bottom image order.

    Use the boolean valid mask to identify pixels with a depth measurement.
    Simulation time is in seconds, and frame IDs start at zero after reset.
    """

    rgb: np.ndarray
    depth: np.ndarray
    valid: np.ndarray
    sim_time: float
    frame_id: int


FrameArrays = Tuple[np.ndarray, np.ndarray, np.ndarray]
Render = Callable[[CameraConfig, np.ndarray], FrameArrays]
DeferredRender = Callable[[CameraConfig, np.ndarray], Callable[[], FrameArrays]]


class Camera:
    """Capture RGB-D frames using render(config, world_pose)."""

    def __init__(self, config: CameraConfig, render: Render, *, deferred_render: Optional[DeferredRender] = None):
        self.config = config
        self._render = render
        self._deferred_render = deferred_render
        self.reset()

    def reset(self) -> None:
        """Clear the cached frame and start a new sampling clock."""
        self._latest: Optional[CameraFrame] = None
        self._pending = None
        self._frame_id = 0
        self._clock_origin: Optional[float] = None
        self._last_tick = -1
        self._last_sample_time: Optional[float] = None

    def read(self) -> Optional[CameraFrame]:
        """Return the latest frame, materializing a deferred sample if present."""
        if self._pending is not None:
            sim_time, frame_id, readback = self._pending
            self._latest = self._make_frame(readback(), sim_time, frame_id)
            self._pending = None
        return self._latest

    def capture(self, sim_time: float, body_poses: Optional[Mapping[str, np.ndarray]] = None) -> CameraFrame:
        """Render immediately without changing the periodic sampling clock."""
        sim_time = float(sim_time)
        if not isfinite(sim_time):
            raise ValueError("Camera simulation time must be finite")
        frame = self._make_frame(self._render(self.config, self._world_pose(body_poses)),
                                 sim_time, self._frame_id)
        self._latest = frame
        self._pending = None
        self._frame_id += 1
        return frame

    def _world_pose(self, body_poses):
        world_pose = self.config.local_pose
        parent = self.config.parent
        if parent is not None:
            if body_poses is None or parent not in body_poses:
                raise KeyError(f"Missing camera parent pose for {parent!r}")
            parent_pose = np.asarray(body_poses[parent], dtype=float)
            if parent_pose.shape != (4, 4) or not np.all(np.isfinite(parent_pose)):
                raise ValueError(f"Camera parent {parent!r} must have a finite 4x4 pose")
            world_pose = parent_pose @ world_pose

        return world_pose

    def _make_frame(self, arrays, sim_time, frame_id):
        rgb, depth, valid = arrays
        rgb = np.array(rgb, dtype=np.uint8, copy=True)
        depth = np.array(depth, dtype=np.float32, copy=True)
        valid = np.array(valid, dtype=bool, copy=True)
        shape = (self.config.height, self.config.width)
        if rgb.shape != shape + (3,) or depth.shape != shape or valid.shape != shape:
            raise ValueError("Camera renderer returned arrays that do not match the configured resolution")

        return CameraFrame(rgb, depth, valid, sim_time, frame_id)

    def update(self, sim_time: float, body_poses: Optional[Mapping[str, np.ndarray]] = None) -> Optional[CameraFrame]:
        """Sample when due. Skip missed ticks and stamp frames with actual simulation time."""
        return self._sample(sim_time, body_poses, self.capture)

    def update_deferred(self, sim_time: float, body_poses: Optional[Mapping[str, np.ndarray]] = None) -> bool:
        """Render when due and defer copying the latest sample until read()."""
        if self._deferred_render is None:
            raise RuntimeError("Camera requires deferred_render to use update_deferred()")
        return bool(self._sample(sim_time, body_poses, self._render_pending))

    def _render_pending(self, sim_time, body_poses):
        readback = self._deferred_render(self.config, self._world_pose(body_poses))
        self._pending = sim_time, self._frame_id, readback
        self._frame_id += 1
        return True

    def _sample(self, sim_time, body_poses, sample):
        sim_time = float(sim_time)
        if not isfinite(sim_time):
            raise ValueError("Camera simulation time must be finite")
        if self._last_sample_time is not None and sim_time <= self._last_sample_time:
            return None

        origin = sim_time if self._clock_origin is None else self._clock_origin
        # Tolerate round-off at integer ticks without accumulating period error.
        tick = floor((sim_time - origin) * self.config.fps + 1e-9)
        if tick <= self._last_tick:
            return None

        frame = sample(sim_time, body_poses)
        self._clock_origin = origin
        self._last_tick = tick
        self._last_sample_time = sim_time
        return frame


@dataclass(frozen=True)
class BatchCameraFrame:
    """CUDA tensors in top-to-bottom image order.

    RGB is uint8 [N, H, W, 3]. Optical-axis depth is float32 [N, H, W]
    in meters. The boolean valid mask [N, H, W] identifies measured pixels.
    """

    rgb: "torch.Tensor"
    depth: "torch.Tensor"
    valid: "torch.Tensor"


class BatchCamera:
    """Render one RGB-D camera per environment of a CUDA simulator batch.

    The camera uploads a shared copy of the visual geometry from a SireScene.
    A SceneAsset or JSON path adds static depth geometry and optional Gaussian
    appearance. Each instance uses the same CameraConfig across environments.
    Use a separate instance for each camera mount or optics. The caller schedules
    capture explicitly. BatchCamera can be used as a context manager.
    """

    def __init__(self, scene, config: CameraConfig, num_envs: int, *, device="cuda:0",
                 chunk_size: int = 64, scene_asset=None):
        from threading import Lock

        from .native import sire as native

        try:
            make_camera = native._SireCudaBatchCamera
        except AttributeError as error:
            raise RuntimeError("BatchCamera requires Sire built with CUDA camera support") from error

        import torch

        from .viewer.renderer import _box_vertices, _load_depth_mesh

        for name, value in (("num_envs", num_envs), ("chunk_size", chunk_size)):
            if not isinstance(value, Integral) or isinstance(value, bool) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        device = torch.device(device)
        if device.type != "cuda":
            raise ValueError("BatchCamera requires a CUDA device")
        if device.index is None:
            device = torch.device("cuda", torch.cuda.current_device())

        body_indices = {name: index for index, name in enumerate(scene.parts)}
        parent = -1 if config.parent is None else body_indices[config.parent]
        self._gaussians = None
        self._foreground = None
        view_items = list(scene.items)
        if scene_asset is not None:
            from .scene import SceneAsset
            from .viewer.scene import RenderItem

            asset = scene_asset if isinstance(scene_asset, SceneAsset) else SceneAsset.load(scene_asset)
            room = RenderItem(None, asset.asset_to_world, (0.24, 0.24, 0.24), mesh=asset.depth_mesh)
            view_items.insert(0, room)
            if asset.gaussians is not None:
                from .viewer._gaussian_rasterizer import _GaussianRasterizer

                self._gaussians = _GaussianRasterizer.for_scene(asset.source, device)
        else:
            room = None
        meshes, items = [], []
        mesh_indices = {}
        for item in view_items:
            key = item.mesh.resolve() if item.mesh is not None else tuple(item.size)
            if key not in mesh_indices:
                mesh_indices[key] = len(meshes)
                vertices = _load_depth_mesh(key) if item.mesh is not None else _box_vertices(key)
                meshes.append(vertices)
            body = -1 if item.body is None else body_indices[item.body]
            entry = (mesh_indices[key], body, np.ravel(item.local_pose).tolist(), item.color, item.checker)
            items.append(entry + (item is room,) if self._gaussians is not None else entry)

        self.config = config
        self.num_envs = int(num_envs)
        self.device = device
        self._lock = Lock()
        self._torch = torch
        self._parent = parent
        self._chunk_size = min(chunk_size, self.num_envs)
        self._camera = make_camera(device.index, self.num_envs, len(body_indices), parent,
                                   config.width, config.height, self._chunk_size,
                                   config.pinhole, config.near, config.far,
                                   config.local_pose.ravel().tolist(), meshes, items)
        shape = (self.num_envs, config.height, config.width)
        self._frame = BatchCameraFrame(
            torch.empty(shape + (3,), dtype=torch.uint8, device=device),
            torch.empty(shape, dtype=torch.float32, device=device),
            torch.empty(shape, dtype=torch.bool, device=device),
        )
        if self._gaussians is not None:
            self._foreground = torch.empty(shape, dtype=torch.bool, device=device)
            self._mount = torch.tensor(config.local_pose, dtype=torch.float64, device=device)
        self._ready = torch.cuda.Event()
        self._ready.record(torch.cuda.current_stream(device))

    def capture(self, poses: "torch.Tensor") -> BatchCameraFrame:
        """Capture from contiguous float64 CUDA poses [N, bodies, 3, 4].

        Pose matrices are row-major in model body order, as returned by the
        CUDA batch stepper. Capture uses the current PyTorch CUDA stream for
        this device. Order pose writes and output reads on that stream or
        synchronize access across streams. Keep the pose storage and its
        producer alive until capture completes on the stream.

        Each call reuses the same frame tensors. Clone them before the next
        capture if the frame must be retained. Finish any reads on other
        streams before the next capture reuses the buffers.
        """
        with self._lock:
            if self._camera is None:
                raise RuntimeError("BatchCamera is closed")
            stream = self._torch.cuda.current_stream(self.device)
            stream.wait_event(self._ready)
            frame = self._frame
            for output in (frame.rgb, frame.depth, frame.valid):
                output.record_stream(stream)
            if isinstance(poses, self._torch.Tensor) and poses.device == self.device:
                poses.record_stream(stream)
            try:
                if self._gaussians is None:
                    self._camera.capture(poses, frame.rgb, frame.depth, frame.valid, stream.cuda_stream)
                else:
                    self._foreground.record_stream(stream)
                    self._camera.capture(poses, frame.rgb, frame.depth, frame.valid,
                                         stream.cuda_stream, self._foreground)
                    self._composite_gaussians(poses)
            finally:
                self._ready.record(stream)
            return frame

    def _composite_gaussians(self, poses):
        torch = self._torch
        with torch.inference_mode():
            for start in range(0, self.num_envs, self._chunk_size):
                rows = slice(start, min(start + self._chunk_size, self.num_envs))
                count = rows.stop - rows.start
                world = self._mount.expand(count, -1, -1)
                if self._parent >= 0:
                    world = world.clone()
                    world[:, :3, :] = poses[rows, self._parent] @ self._mount
                rgb, alpha = self._gaussians.render(self.config, world)
                mesh = self._frame.rgb[rows]
                color = rgb.mul_(255).addcmul_(1 - alpha, mesh).round_().clamp_(0, 255)
                mesh.copy_(torch.where(self._foreground[rows, ..., None], mesh, color))

    def close(self) -> None:
        """Wait for the last capture and release the renderer."""
        with self._lock:
            if self._camera is not None:
                self._ready.synchronize()
                self._camera = None
                self._frame = None
                self._foreground = None
                self._gaussians = None

    def __enter__(self) -> BatchCamera:
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
