"""Verify the 3DGS and mesh passes agree on projection and occlusion."""

from pathlib import Path
from dataclasses import replace
import json
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

import numpy as np

try:
    import gsplat
    from plyfile import PlyData, PlyElement
    import sire
    import torch
    from sire.sensor import BatchCamera, CameraConfig
    from sire.viewer import RenderItem, graphics_context
    from sire.viewer.photoreal import PhotorealRenderer
except ImportError:
    torch = None


@unittest.skipUnless(torch is not None and torch.cuda.is_available(),
                     "requires the Sire photoreal extra and a CUDA device")
class PhotorealRendererTest(unittest.TestCase):
    def test_splat_alignment_and_opaque_depth(self):
        with TemporaryDirectory() as directory:
            path = Path(directory)
            fields = ("x", "y", "z", "scale_0", "scale_1", "scale_2", "rot_0", "rot_1", "rot_2",
                      "rot_3", "opacity", "f_dc_0", "f_dc_1", "f_dc_2")
            vertices = np.array(
                [(0.2, 0.25, -2.0, -1.65, -1.65, -1.65, 1., 0., 0., 0., 10., -1.77, -1.77, 1.77)],
                dtype=[(field, "f4") for field in fields],
            )
            PlyData([PlyElement.describe(vertices, "vertex")]).write(path / "splat.ply")
            (path / "wall.obj").write_text(
                "v -1.4 -1.2 -2\nv 1.4 -1.2 -2\nv 1.4 1.2 -2\nv -1.4 1.2 -2\nf 1 2 3 4\n"
            )
            (path / "scene.json").write_text(json.dumps({
                "gaussians": "splat.ply", "depth_mesh": "wall.obj",
                "position": [0.15, 0, -0.3], "scale": 1.5,
            }))
            box = RenderItem("box", np.eye(4), (0.9, 0.1, 0.1), size=(0.35, 0.35, 0.4))
            config = CameraConfig(96, 64, intrinsics=(80, 80, 48, 32), near=0.1, far=8.0)
            with graphics_context(headless=True):
                renderer = PhotorealRenderer([box], path / "scene.json")
                try:
                    pose = np.eye(4)
                    pose[:3, 3] = (0.23, 0.19, -1.7)
                    foreground, depth_front, valid_front = renderer.capture({"box": pose}, config, np.eye(4))
                    pose[:3, 3] = (0.57, 0.48, -4.2)
                    background, depth_back, valid_back = renderer.capture({"box": pose}, config, np.eye(4))
                finally:
                    renderer.close()

            if hasattr(sire.sire, "_SireCudaBatchCamera"):
                scene = SimpleNamespace(parts={"wrist": None, "box": None}, items=[box])
                wrist_config = replace(config, parent="wrist")
                poses = torch.eye(4, dtype=torch.float64, device="cuda")[:3].repeat(3, 2, 1, 1)
                poses[:, 1, :, 3] = torch.tensor(
                    [[0.23, 0.19, -1.7], [0.57, 0.48, -4.2], [0.57, 0.48, -4.2]],
                    device="cuda", dtype=torch.float64,
                )
                poses[2, 0, 0, 3] = 0.3
                with BatchCamera(scene, config, 3, scene_asset=path / "scene.json", chunk_size=2) as fixed:
                    with BatchCamera(scene, wrist_config, 3, scene_asset=path / "scene.json",
                                     chunk_size=2) as wrist:
                        fixed_frame = fixed.capture(poses)
                        wrist_frame = wrist.capture(poses)
                        self.assertTrue(all(t.is_cuda for t in (
                            wrist_frame.rgb, wrist_frame.depth, wrist_frame.valid,
                        )))
                        np.testing.assert_array_equal(fixed_frame.valid[0].cpu(), valid_front)
                        np.testing.assert_array_equal(wrist_frame.valid[1].cpu(), valid_back)
                        np.testing.assert_allclose(wrist_frame.depth[0].cpu(), depth_front, atol=1e-5)
                        np.testing.assert_allclose(wrist_frame.depth[1].cpu(), depth_back, atol=1e-5)
                        self.assertGreater(int(wrist_frame.rgb[0, 23, 59, 0]),
                                           int(wrist_frame.rgb[0, 23, 59, 2]) + 25)
                        self.assertGreater(int(wrist_frame.rgb[1, 23, 59, 2]), 190)
                        self.assertLess(int(wrist_frame.rgb[2, 23, 59, 2]),
                                        int(fixed_frame.rgb[2, 23, 59, 2]) - 30)

        row, col = 23, 59
        self.assertTrue(valid_front[row, col] and valid_back[row, col])
        self.assertAlmostEqual(float(depth_front[row, col]), 1.5, places=4)
        self.assertAlmostEqual(float(depth_back[row, col]), 3.3, places=4)
        self.assertGreater(int(foreground[row, col, 0]), int(foreground[row, col, 2]) + 25)
        self.assertGreater(int(background[row, col, 2]), int(background[40, col, 2]) + 80)
        self.assertGreater(int(background[row, col, 2]), 190)


if __name__ == "__main__":
    unittest.main()
