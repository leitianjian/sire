"""CUDA Go2 regression for startup and masked-reset domain sampling."""

import unittest

import torch
import sire

from SireRLGym.utils.task_registry import make_env_cfg, make_env_from_cfg


@unittest.skipUnless(
    torch.cuda.is_available() and hasattr(sire, "SireCudaBatchStepper"),
    "requires the native Sire CUDA backend and a CUDA device",
)
class SireCudaDomainRandomizationTest(unittest.TestCase):
    def test_per_environment_samples_at_startup_and_masked_reset(self):
        for randomize_on_reset in (False, True):
            with self.subTest(randomize_on_reset=randomize_on_reset):
                cfg = make_env_cfg("go2")
                cfg.env.num_envs = 6
                cfg.sim.sire_cuda_streams = 2
                cfg.domain_rand.randomize_friction = True
                cfg.domain_rand.randomize_base_mass = True
                cfg.domain_rand.randomize_on_reset = randomize_on_reset
                cfg.domain_rand.friction_range = [0.3, 0.8]
                cfg.domain_rand.added_mass_range = [-0.3, 1.0]
                torch.manual_seed(7291)
                env = make_env_from_cfg("go2", cfg, sire_backend="cuda", sim_device="cuda:0")
                indices = torch.tensor([1, 4], device=env.device)
                keep = torch.tensor([0, 2, 3, 5], device=env.device)
                parameters = (env.friction_coeffs, env.base_mass_offsets)
                originals = tuple(values.clone() for values in parameters)
                env.reset_idx(indices)
                bounds = (cfg.domain_rand.friction_range, cfg.domain_rand.added_mass_range)
                for values, original, (low, high) in zip(parameters, originals, bounds):
                    self.assertEqual(values.device, env.device)
                    self.assertTrue(torch.all((values >= low) & (values <= high)).item())
                    self.assertGreater(torch.unique(original).numel(), 1)
                    torch.testing.assert_close(values[keep], original[keep], atol=0, rtol=0)
                    if randomize_on_reset:
                        self.assertTrue(torch.all(values[indices] != original[indices]).item())
                    else:
                        torch.testing.assert_close(values[indices], original[indices], atol=0, rtol=0)


if __name__ == "__main__":
    unittest.main()
