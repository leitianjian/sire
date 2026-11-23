"""Small adapter/export regressions; no batch training is started by these tests."""
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import mujoco
import numpy as np
import torch

from deploy_mujoco import Controller, Go2Simulation, ROOT, joystick_command, load_jit, rotation_matrix
from export_policy import export


POLICY = ROOT / "sim2sim/policies/exp24/model_1000_jit.pt"
XML = ROOT / "demo/demo_python/resources/robots/go2/flat.xml"


class Sim2SimTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)
        cls.sim = Go2Simulation(POLICY, XML, 0.002)

    def setUp(self):
        self.sim.reset()

    def test_joint_order_and_observation_blocks(self):
        sim = self.sim
        np.testing.assert_array_equal(sim.qpos_ids, [7, 10, 13, 16, 8, 11, 14, 17, 9, 12, 15, 18])
        np.testing.assert_array_equal(sim.actuator_ids, [0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11])
        delta = np.arange(12) * 0.01
        velocity = np.arange(12) * 0.1
        sim.data.qpos[sim.qpos_ids] = sim.default_q + delta
        sim.data.qvel[sim.qvel_ids] = velocity
        sim.action = np.arange(12, dtype=np.float32) * 0.03
        obs = sim.observation([0.2, -0.3, 0.7])
        self.assertEqual(obs.shape, (45,))
        np.testing.assert_allclose(obs[6:9], [0.4, -0.6, 0.175])
        np.testing.assert_allclose(obs[9:21], delta, atol=1e-7)
        np.testing.assert_allclose(obs[21:33], velocity * 0.05, atol=1e-7)
        np.testing.assert_array_equal(obs[33:], sim.action)

    def test_yaw_and_tilt_do_not_double_rotate_angular_velocity(self):
        sim = self.sim
        for yaw in [0, 90, 180, 270, 370, 720]:
            angle = np.deg2rad(yaw) / 2
            yaw_q = np.array([np.cos(angle), 0, 0, np.sin(angle)])
            tilt_q = np.array([np.cos(0.15), np.sin(0.15), 0, 0])
            quat = np.empty(4)
            mujoco.mju_mulQuat(quat, yaw_q, tilt_q)
            sim.data.qpos[3:7] = quat
            rot = rotation_matrix(quat)
            local_linear = np.array([0.6, -0.2, 0.1])
            local_angular = np.array([0.2, -0.3, 0.7])
            sim.data.qvel[:3] = rot @ local_linear
            sim.data.qvel[3:6] = local_angular
            mujoco.mj_forward(sim.model, sim.data)
            _, _, linear, angular = sim.state()
            mj_local_velocity = np.empty(6)
            mujoco.mj_objectVelocity(sim.model, sim.data, mujoco.mjtObj.mjOBJ_BODY, sim.base_body, mj_local_velocity, 1)
            np.testing.assert_allclose(linear, local_linear, atol=1e-12)
            np.testing.assert_allclose(angular, mj_local_velocity[:3], atol=1e-12)
            obs = sim.observation([0, 0, 0.8])
            np.testing.assert_allclose(obs[:3], local_angular * 0.25, atol=1e-7)
            np.testing.assert_allclose(obs[3:6], rot.T @ [0, 0, -1], atol=1e-7)
            self.assertAlmostEqual(float(obs[8]), 0.2, places=6)

    def test_reference_joystick_axes_signs_deadzone(self):
        args = ([1, 0, 3], [-1, -1, -1], [0.5, 0.5, 1.0], 0.1)
        np.testing.assert_array_equal(joystick_command([0, 0, -1, 0, 0, -1], *args), [0, 0, 0])
        np.testing.assert_allclose(joystick_command([-0.4, -1, -1, -0.8, 0, -1], *args), [0.5, 0.2, 0.8])
        np.testing.assert_array_equal(joystick_command([0.09, -0.09, -1, 0.09, 0, -1], *args), [0, 0, 0])

    def test_detach_clears_cached_commands(self):
        controller = Controller.__new__(Controller)
        detached = []
        js = SimpleNamespace(get_instance_id=lambda: 7, quit=lambda: detached.append(True))
        pg = SimpleNamespace(JOYDEVICEREMOVED=99,
                             event=SimpleNamespace(pump=lambda: None, get=lambda: [SimpleNamespace(type=99, instance_id=7)]))
        controller.pg, controller.js = pg, js
        controller.previous_buttons = {0}
        controller.last_probe = float("inf")
        command, pressed, axes = controller.poll()
        np.testing.assert_array_equal(command, [0, 0, 0])
        self.assertIsNone(controller.js)
        self.assertEqual(pressed, set())
        self.assertEqual(detached, [True])

    def test_reset_and_control_period(self):
        self.sim.step([0.3, 0, 0.4])
        self.assertAlmostEqual(self.sim.data.time, 0.02)
        self.assertTrue(np.isfinite(self.sim.observation([0, 0, 0])).all())
        self.sim.reset()
        self.assertEqual(self.sim.data.time, 0)
        np.testing.assert_array_equal(self.sim.action, np.zeros(12))
        np.testing.assert_allclose(self.sim.data.qpos[self.sim.qpos_ids], self.sim.default_q)

    def test_jit_deployment_does_not_load_checkpoint_or_yaml_from_training(self):
        with patch("torch.load", side_effect=AssertionError("Runtime must not load a checkpoint")):
            actor, profile = load_jit(POLICY)
            with torch.inference_mode():
                action = actor(torch.zeros(1, 45))
        self.assertEqual(action.shape, (1, 12))
        self.assertEqual(profile["iteration"], 1000)
        self.assertTrue(all(name.startswith("actor.") for name in actor.state_dict()))

    def test_export_matches_existing_actor_and_rejects_overwrite(self):
        with tempfile.TemporaryDirectory(prefix="sire_sim2sim_export_") as temp:
            output = Path(temp) / "policy.pt"
            export(ROOT / "logs/flat_go2/exp24/model_1000.pt", output)
            new_actor, profile = load_jit(output)
            with torch.inference_mode():
                obs = self.sim.observation([0.3, -0.2, 0.7])
                tensor = torch.from_numpy(obs)[None]
                torch.testing.assert_close(new_actor(tensor), self.sim.actor(tensor), rtol=0, atol=0)
            self.assertEqual(profile["policy_joint_names"], list(self.sim.names))
            with self.assertRaises(FileExistsError):
                export(ROOT / "logs/flat_go2/exp24/model_1000.pt", output)


if __name__ == "__main__":
    unittest.main()
