"""GPU regression tests for batch camera projection, chunking, and stream order."""

import unittest
from types import SimpleNamespace

import numpy as np

try:
    import sire
    import torch
    from sire.sensor import BatchCamera, CameraConfig
    from sire.viewer import MeshRenderer, RenderItem, graphics_context
except ImportError:
    sire = None


@unittest.skipUnless(
    sire is not None and hasattr(sire.sire, "_SireCudaBatchCamera") and torch.cuda.is_available(),
    "the installed Sire SDK needs the native EGL/CUDA camera and a CUDA device",
)
class CudaBatchCameraTest(unittest.TestCase):
    def test_chunked_views_match_individual_captures(self):
        scene = SimpleNamespace(
            parts={"mount": None, "box": None},
            items=[RenderItem("box", np.eye(4), (0.85, 0.3, 0.09), size=(0.6, 0.5, 0.3))],
        )
        config = CameraConfig(63, 47, intrinsics=(43, 40, 29, 21),
                              near=0.1, far=8., parent="mount")
        poses = np.tile(np.eye(4), (5, 2, 1, 1))
        poses[:, 1, 2, 3] = -np.array([1.65, 2.2, 3.0, 5.0, 9.0])
        poses[:, 0, 0, 3] = [0., -0.2, 0.2, 0.4, 0.]
        poses[2, 0, :2, :2] = [[0.8, -0.6], [0.6, 0.8]]

        # The current single-scene camera serves as the projection and depth reference.
        with graphics_context(headless=True):
            renderer = MeshRenderer(scene.items)
            try:
                expected = [renderer.capture(dict(zip(scene.parts, item)), config,
                                             item[0] @ config.local_pose) for item in poses]
            finally:
                renderer.close()

        stream = torch.cuda.Stream(device=0)
        with BatchCamera(scene, config, len(poses), chunk_size=3) as camera:
            with torch.cuda.stream(stream):
                device_poses = torch.tensor(poses[:, :, :3, :].copy(), device="cuda:0")
                frame = camera.capture(device_poses)
                self.assertTrue(frame.rgb.is_cuda and frame.depth.is_cuda and frame.valid.is_cuda)
                rgb, depth, valid = (item.cpu().numpy() for item in (frame.rgb, frame.depth, frame.valid))

        for index, (ref_rgb, ref_depth, ref_valid) in enumerate(expected):
            np.testing.assert_array_equal(valid[index], ref_valid)
            np.testing.assert_allclose(rgb[index], ref_rgb, rtol=0, atol=1)
            np.testing.assert_allclose(depth[index], ref_depth, rtol=1e-5, atol=1e-4)
        np.testing.assert_allclose(depth[0, 21, 29], 1.5, atol=1e-5)
        self.assertFalse(valid[-1].any())


if __name__ == "__main__":
    unittest.main()
