"""Regression coverage for the native Sire RL batch training path.

Run from the repository root with no extra test dependency:
  PYTHONPATH=python/src:demo/demo_python \
    .venv/bin/python demo/demo_python/SireRLGym/test/test_sire_batch_training.py
"""

from __future__ import annotations

import numpy as np
import torch
import unittest

from SireRLGym.utils.task_registry import make_env_cfg, make_env_from_cfg
from SireRLGym.utils.math import quat_rotate_inverse


def _make_env(num_envs: int = 2, threads: int = 2, rough_terrain: bool = False):
    cfg = make_env_cfg("go2")
    cfg.env.num_envs = num_envs
    cfg.sim.sire_batch_threads = threads
    cfg.sim.sire_diagnostics = False
    cfg.noise.add_noise = False
    cfg.init_state.init_yaw_range = [0.0, 0.0]
    if rough_terrain:
        cfg.terrain.mesh_type = "trimesh"
        cfg.terrain.measure_heights = True
    torch.manual_seed(12345)
    return make_env_from_cfg("go2", cfg, headless=True)


def _reset_deterministically(env, seed: int = 24680):
    ids = torch.arange(env.num_envs, device=env.device)
    torch.manual_seed(seed)
    env.reset_idx(ids)
    return ids


def _snapshot(env):
    return {
        "root_states": env.root_states.clone(),
        "dof_pos": env.dof_pos.clone(),
        "dof_vel": env.dof_vel.clone(),
        "torques": env.torques.clone(),
        "contact_forces": env.contact_forces.clone(),
        "feet_pos_world": env.feet_pos_world.clone(),
        "body_ground_contact": env.body_ground_contact.clone(),
        "foot_ground_contact": env.foot_ground_contact.clone(),
        "obs_buf": env.obs_buf.clone(),
        "rew_buf": env.rew_buf.clone(),
    }


class SireBatchTrainingTest(unittest.TestCase):
    def test_effort_guard_and_joint_limit_method_are_configurable(self):
        env = _make_env(num_envs=1, threads=1)
        engine = env.sire_simulators[0].physicsEngine()
        self.assertEqual(engine.jointLimitMethod, "shifted_ncp")
        for method in ("projection", "disabled", "shifted_ncp"):
            engine.jointLimitMethod = method
            self.assertEqual(engine.jointLimitMethod, method)
        # The ARIS reflection setter currently translates invalid C++ property
        # values to RuntimeError, while direct pybind setters use ValueError.
        with self.assertRaises((ValueError, RuntimeError)):
            engine.jointLimitMethod = "unknown"

        actuator = env.sire_models[0].motionPool()[int(env._motion_idx[0])]
        actuator.desiredValue = 1e9
        actuator.forward()
        self.assertAlmostEqual(actuator.appliedValue, actuator.maxForce)

        actuator.mp = actuator.maxPosition + 1e-3
        actuator.mv = 1.0
        self.assertTrue(actuator.enforcePositionLimits())
        self.assertAlmostEqual(actuator.mp, actuator.maxPosition)
        self.assertEqual(actuator.mv, 0.0)

    def test_history_does_not_change_physics(self):
        env = _make_env(num_envs=1, threads=1)
        stepper = env._sire_batch_stepper
        actions = np.zeros((1, env.num_actions), dtype=np.float32)
        snapshots = []
        for enabled in (True, False):
            _reset_deterministically(env)
            stepper.setHistoryRecording(enabled)
            for _ in range(8):
                stepper.step(actions)
            snapshots.append([np.array(value, copy=True) for value in stepper.outputs()])
        for recorded, unrecorded in zip(*snapshots):
            np.testing.assert_allclose(recorded, unrecorded, rtol=1e-5, atol=1e-6)

    def test_control_boundary_and_history_opt_in(self):
        env = _make_env(num_envs=1, threads=1)
        stepper = env._sire_batch_stepper
        loop = env.sire_sim_loops[0]
        actions = np.zeros((1, env.num_actions), dtype=np.float32)
        for _ in range(3):
            before = loop.simTime()
            outputs = stepper.step(actions)
            self.assertAlmostEqual(loop.simTime() - before, env.dt, delta=1e-6)
            self.assertAlmostEqual(outputs[-1][0], loop.simTime() - before)
            self.assertTrue(loop.headerIsCtrl())
            self.assertEqual(len(loop.recordsToJson()['timeIndex']), 1)
        state_before = np.array(stepper.outputs()[0], copy=True)
        before = loop.simTime()
        stepper.setHistoryRecording(True)
        self.assertEqual(loop.simTime(), before)
        np.testing.assert_array_equal(stepper.outputs()[0], state_before)
        stepper.step(actions)
        self.assertGreater(len(loop.recordsToJson()['timeIndex']), 1)
        before = loop.simTime()
        stepper.setHistoryRecording(False)
        self.assertEqual(loop.simTime(), before)
        stepper.step(actions)
        self.assertEqual(len(loop.recordsToJson()['timeIndex']), 1)

    def test_reset_refreshes_body_observations(self):
        env = _make_env()
        env.base_lin_vel[:] = 99.0
        env.base_ang_vel[:] = 99.0
        env.projected_gravity[:] = 99.0
        env.actions[:] = 1.0
        ids = torch.tensor([1], device=env.device)
        env.reset_idx(ids)
        q = env.root_states[ids, 3:7]
        torch.testing.assert_close(env.base_lin_vel[ids],
                                   quat_rotate_inverse(q, env.root_states[ids, 7:10]))
        torch.testing.assert_close(env.base_ang_vel[ids],
                                   quat_rotate_inverse(q, env.root_states[ids, 10:13]))
        torch.testing.assert_close(env.projected_gravity[ids],
                                   quat_rotate_inverse(q, env.gravity_vec[ids]))
        self.assertTrue(torch.all(env.base_lin_vel[0] == 99.0))
        self.assertTrue(torch.all(env.actions[ids] == 0.0))
        # Repeated IDs must not dispatch two workers against the same model.
        env._sire_batch_stepper.reset(np.array([1, 1], dtype=np.int64))
        self.assertEqual(env.sire_sim_loops[1].simTime(), 0.0)

    def test_batch_matches_legacy_step_and_reuses_outputs(self):
        # One environment isolates implementation equivalence from any
        # underlying solver-level cross-thread nondeterminism.
        env = _make_env(num_envs=1, threads=1)
        _reset_deterministically(env)
        initial = env.root_states.clone()
        self.assertTrue(torch.equal(initial[:, :2], torch.zeros_like(initial[:, :2])))

        actions = torch.linspace(-0.2, 0.2, 12).repeat(env.num_envs, 1)
        output_ids_before = tuple(id(array) for array in env._sire_batch_stepper.outputs())
        env.stepSireBatch(actions)
        batch = _snapshot(env)
        batch_times = [loop.simTime() for loop in env.sire_sim_loops]
        output_ids_after = tuple(id(array) for array in env._sire_batch_stepper.outputs())
        self.assertEqual(output_ids_before, output_ids_after)

        _reset_deterministically(env)
        self.assertTrue(torch.equal(env.root_states, initial))
        env.legacySireStep(actions)
        legacy = _snapshot(env)
        legacy_times = [loop.simTime() for loop in env.sire_sim_loops]

        for name in (
            "root_states",
            "dof_pos",
            "dof_vel",
            "torques",
            "contact_forces",
            "feet_pos_world",
            "obs_buf",
            "rew_buf",
        ):
            torch.testing.assert_close(
                batch[name], legacy[name], rtol=1e-5, atol=1e-6, msg=name
            )
        self.assertTrue(
            torch.equal(batch["body_ground_contact"], legacy["body_ground_contact"])
        )
        self.assertTrue(
            torch.equal(batch["foot_ground_contact"], legacy["foot_ground_contact"])
        )
        np.testing.assert_allclose(batch_times, legacy_times, rtol=0.0, atol=1e-12)
        self.assertEqual(env._sire_batch_stepper.threadCount, 1)
        self.assertEqual(env._sire_batch_stepper.workerCount, 0)
        self.assertGreaterEqual(env._sire_batch_stepper.dispatchCount, 4)

    def test_reset_and_exception_semantics(self):
        env = _make_env()
        env.step(torch.zeros(env.num_envs, env.num_actions))
        time_before = [loop.simTime() for loop in env.sire_sim_loops]
        env.reset_idx(torch.tensor([1], dtype=torch.long, device=env.device))
        time_after_one_reset = [loop.simTime() for loop in env.sire_sim_loops]
        self.assertAlmostEqual(time_after_one_reset[0], time_before[0])
        self.assertAlmostEqual(time_after_one_reset[1], 0.0)

        env.resetSireRecorders()
        self.assertEqual(
            [loop.simTime() for loop in env.sire_sim_loops], time_after_one_reset
        )
        for loop in env.sire_sim_loops:
            # resetRecorders keeps one empty continuation frame because the
            # native contact solver writes records.back() on its next event.
            self.assertEqual(len(loop.recordsToJson()["timeIndex"]), 1)

        # A step after the rollout reset must be valid (the old implementation
        # wrote through records.back() on an empty vector here).
        env.step(torch.zeros(env.num_envs, env.num_actions))
        self.assertEqual(len(env.sire_sim_loops[1].recordsToJson()["timeIndex"]), 1)

        actions = torch.zeros(env.num_envs, env.num_actions)
        actions[1, 3] = torch.nan
        with self.assertRaises(RuntimeError) as context:
            env.step(actions)
        message = str(context.exception)
        for expected in ("env_id=1", "sim_time=", "pq=[", "mp=[", "actions=["):
            self.assertIn(expected, message)

    def test_terrain_boundary_is_a_per_env_timeout(self):
        env = _make_env(rough_terrain=True)
        env.contact_forces.zero_()
        env.episode_length_buf.zero_()
        env.root_states[:, :2].zero_()
        env.root_states[1, 0] = env.terrain.patch_length + 1.0
        env.check_termination()
        self.assertFalse(bool(env.time_out_buf[0]))
        self.assertFalse(bool(env.reset_buf[0]))
        self.assertTrue(bool(env.time_out_buf[1]))
        self.assertTrue(bool(env.reset_buf[1]))

    def test_native_physics_failure_is_a_per_env_terminal(self):
        env = _make_env()
        env.contact_forces.zero_()
        env.episode_length_buf.zero_()
        env.root_states[:, 2] = 0.34
        env._sire_physics_failure_buf.zero_()
        env._sire_physics_failure_buf[1] = True

        env.check_termination()

        self.assertFalse(bool(env.reset_buf[0]))
        self.assertTrue(bool(env.reset_buf[1]))
        self.assertFalse(bool(env.time_out_buf[1]))

    def test_go2_action_guard_limits_pd_target_offset(self):
        env = _make_env(num_envs=1, threads=1)
        self.assertEqual(env.cfg.normalization.clip_actions, 4.0)
        self.assertLessEqual(
            env.cfg.normalization.clip_actions * env.cfg.control.action_scale,
            1.0,
        )

    def test_low_base_does_not_terminate_but_excessive_roll_does(self):
        env = _make_env()
        env.step(torch.zeros(env.num_envs, env.num_actions))
        env.contact_forces.zero_()
        env.episode_length_buf.zero_()
        env.root_states[:, 2] = 0.34
        env.root_states[1, 2] = 0.01

        env.check_termination()
        self.assertFalse(bool(env.reset_buf[0]))
        self.assertFalse(bool(env.reset_buf[1]))

        # Sire quaternions are [x, y, z, w]; roll=0.9 rad exceeds 0.8.
        env.root_states[1, 3:7] = torch.tensor(
            [np.sin(0.45), 0.0, 0.0, np.cos(0.45)], dtype=torch.float
        )
        env.check_termination()
        self.assertFalse(bool(env.reset_buf[0]))
        self.assertTrue(bool(env.reset_buf[1]))
        self.assertFalse(bool(env.time_out_buf[1]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
