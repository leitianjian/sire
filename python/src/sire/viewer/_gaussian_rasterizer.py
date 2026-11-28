"""Shared CUDA rasterization of a static Gaussian scene."""

from pathlib import Path
from threading import Lock
from weakref import WeakValueDictionary

import numpy as np

from ._splat_asset import load_splat_scene


class _GaussianRasterizer:
    _scenes = WeakValueDictionary()
    _scenes_lock = Lock()

    @classmethod
    def for_scene(cls, scene_path, device):
        """Share GPU assets while a renderer holds the resolved scene and device."""
        try:
            import torch
            from gsplat import rasterization
        except ImportError as error:
            raise RuntimeError("Install the Sire photoreal extra and CUDA PyTorch to use 3DGS.") from error
        device = torch.device(device)
        if device.type != "cuda" or not torch.cuda.is_available():
            raise RuntimeError("Photoreal rendering requires an available CUDA device.")
        if device.index is None:
            device = torch.device("cuda", torch.cuda.current_device())
        path = Path(scene_path).expanduser().resolve()
        key = (path, device)
        with cls._scenes_lock:
            renderer = cls._scenes.get(key)
            if renderer is None:
                renderer = cls(load_splat_scene(path), device, torch, rasterization)
                cls._scenes[key] = renderer
        return renderer

    def __init__(self, scene, device, torch, rasterization):
        self.depth_mesh = scene.depth_mesh
        self.asset_to_world = scene.asset_to_world
        self._device = device
        self._torch = torch
        self._rasterize = rasterization
        self._means = self._tensor(scene.means)
        self._quats = self._tensor(scene.quats)
        self._scales = self._tensor(scene.scales)
        self._opacities = self._tensor(scene.opacities)
        self._colors = self._tensor(scene.colors)
        self._sh_degree = scene.sh_degree
        self._radius_clip = scene.radius_clip
        linear = scene.asset_to_world[:3, :3]
        self._meters_per_unit = float(np.hypot.reduce(linear[:, 0]))
        self._asset_from_world = self._tensor(np.linalg.inv(scene.asset_to_world))
        # Convert OpenGL's +Y up and -Z forward to gsplat's +Y down and +Z
        # forward while removing the similarity scale from the camera basis.
        scale = self._meters_per_unit
        self._camera_basis = self._tensor([scale, -scale, -scale])
        self._intrinsics = {}

    def _tensor(self, array):
        return self._torch.as_tensor(array, dtype=self._torch.float32, device=self._device)

    def render(self, config, world_poses):
        """Render CUDA OpenGL poses [N, 4, 4] as premultiplied RGB and alpha."""
        torch = self._torch
        if world_poses.device != self._device:
            raise ValueError(f"Gaussian camera poses must be on {self._device}.")
        if world_poses.ndim != 3 or world_poses.shape[1:] != (4, 4):
            raise ValueError("Gaussian camera poses must have shape [N, 4, 4].")
        with torch.inference_mode(), torch.cuda.device(self._device):
            count = world_poses.shape[0]
            if count == 0:
                return (
                    self._means.new_empty((0, config.height, config.width, 3)),
                    self._means.new_empty((0, config.height, config.width, 1)),
                )
            asset_pose = self._asset_from_world @ world_poses.to(dtype=torch.float32)
            # Keep the Gaussians and their spherical harmonics in the asset frame.
            asset_pose[:, :3, :3] *= self._camera_basis
            view, _ = torch.linalg.inv_ex(asset_pose)
            pinhole = config.pinhole
            intrinsics = self._intrinsics.get(pinhole)
            if intrinsics is None:
                fx, fy, cx, cy = pinhole
                intrinsics = self._tensor([[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]])
                self._intrinsics[pinhole] = intrinsics
            intrinsics = intrinsics.expand(count, -1, -1).contiguous()
            rgb, alpha, _ = self._rasterize(
                self._means, self._quats, self._scales, self._opacities, self._colors,
                view.contiguous(), intrinsics, config.width, config.height,
                near_plane=config.near / self._meters_per_unit,
                far_plane=config.far / self._meters_per_unit,
                sh_degree=self._sh_degree, radius_clip=self._radius_clip,
            )
            return rgb, alpha

    def capture(self, config, world_pose):
        """Adapt one Viewer pose to its CUDA uint8 RGBA texture format."""
        torch = self._torch
        with torch.inference_mode():
            rgb, alpha = self.render(config, self._tensor(np.array(world_pose[None], copy=True)))
            rgba = torch.cat((rgb[0], alpha[0]), dim=-1)
            return rgba.clamp_(0, 1).mul_(255).round_().to(torch.uint8)

    @property
    def current_stream(self):
        return self._torch.cuda.current_stream(self._device).cuda_stream

    @property
    def device_index(self):
        return self._device.index
