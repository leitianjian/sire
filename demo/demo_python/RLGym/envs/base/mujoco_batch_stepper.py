from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from typing import Sequence

import mujoco
import numpy as np


class MuJoCoBatchStepper:
    """Persistent threaded MuJoCo stepping and state collection."""

    def __init__(
        self,
        models: Sequence[mujoco.MjModel],
        datas: Sequence[mujoco.MjData],
        *,
        actuator_ids: np.ndarray,
        qpos_adr: np.ndarray,
        qvel_adr: np.ndarray,
        root_qpos_adr: int,
        root_qvel_adr: int,
        feet_body_ids: np.ndarray,
        world_body_id: int,
        num_bodies: int,
        decimation: int,
        thread_count: int,
        control_type: str,
        action_scale: float,
        p_gains: np.ndarray,
        d_gains: np.ndarray,
        default_dof_pos: np.ndarray,
        torque_limits: np.ndarray,
    ):
        if len(models) != len(datas):
            raise ValueError('models and datas must have the same length')
        if len(datas) == 0:
            raise ValueError('MuJoCoBatchStepper requires at least one environment')

        self.models = models
        self.datas = datas
        self.num_envs = len(datas)
        self.actuator_ids = np.asarray(actuator_ids, dtype=np.int32)
        self.qpos_adr = np.asarray(qpos_adr, dtype=np.int32)
        self.qvel_adr = np.asarray(qvel_adr, dtype=np.int32)
        self.root_qpos_adr = int(root_qpos_adr)
        self.root_qvel_adr = int(root_qvel_adr)
        self.feet_body_ids = np.asarray(feet_body_ids, dtype=np.int32)
        self.world_body_id = int(world_body_id)
        self.decimation = int(decimation)
        self.control_type = str(control_type)
        self.action_scale = float(action_scale)

        num_dof = len(self.qpos_adr)
        self.p_gains = self._control_vector(p_gains, 'p_gains', num_dof)
        self.d_gains = self._control_vector(d_gains, 'd_gains', num_dof)
        self.default_dof_pos = self._control_vector(
            default_dof_pos, 'default_dof_pos', num_dof
        )
        self.torque_limits = self._control_vector(
            torque_limits, 'torque_limits', num_dof
        )
        if self.control_type not in {'P', 'T'}:
            raise ValueError(
                f'MuJoCoBatchStepper supports P or T control, got {self.control_type!r}'
            )
        if np.any(self.torque_limits <= 0.0):
            raise ValueError('torque_limits must be positive')

        if self.control_type == 'P':
            self._configure_native_pd_actuators()

        requested_threads = max(1, int(thread_count))
        self.thread_count = min(requested_threads, self.num_envs)
        self.worker_count = self.thread_count
        self._executor = ThreadPoolExecutor(
            max_workers=self.thread_count,
            thread_name_prefix='mujoco-batch',
        )
        boundaries = np.linspace(0, self.num_envs, self.thread_count + 1, dtype=np.int32)
        self._ranges = [
            (int(boundaries[i]), int(boundaries[i + 1]))
            for i in range(self.thread_count)
            if boundaries[i] < boundaries[i + 1]
        ]

        num_feet = len(self.feet_body_ids)
        self.root_states = np.empty((self.num_envs, 13), dtype=np.float32)
        self.dof_pos = np.empty((self.num_envs, num_dof), dtype=np.float32)
        self.dof_vel = np.empty((self.num_envs, num_dof), dtype=np.float32)
        self.contact_forces = np.zeros((self.num_envs, num_bodies, 3), dtype=np.float32)
        self.feet_pos_world = np.empty((self.num_envs, num_feet, 3), dtype=np.float32)
        self.body_ground_contact = np.zeros((self.num_envs, num_bodies), dtype=np.bool_)
        self.foot_ground_contact = np.zeros((self.num_envs, num_feet), dtype=np.bool_)
        self.torques = np.zeros((self.num_envs, num_dof), dtype=np.float32)

    @staticmethod
    def _control_vector(values: np.ndarray, name: str, size: int) -> np.ndarray:
        result = np.asarray(values, dtype=np.float64).reshape(-1)
        if result.shape != (size,):
            raise ValueError(f'{name} must have shape {(size,)}, got {result.shape}')
        if not np.all(np.isfinite(result)):
            raise ValueError(f'{name} contains NaN or Inf')
        return result

    def _configure_native_pd_actuators(self) -> None:
        """Configure MuJoCo to evaluate PD force at every physics substep."""
        configured_model_ids = set()
        for model in self.models:
            model_identity = id(model)
            if model_identity in configured_model_ids:
                continue
            configured_model_ids.add(model_identity)
            for dof_id, actuator_id in enumerate(self.actuator_ids):
                actuator_id = int(actuator_id)
                kp = self.p_gains[dof_id]
                kd = self.d_gains[dof_id]
                limit = self.torque_limits[dof_id]
                model.actuator_gaintype[actuator_id] = mujoco.mjtGain.mjGAIN_FIXED
                model.actuator_biastype[actuator_id] = mujoco.mjtBias.mjBIAS_AFFINE
                model.actuator_gainprm[actuator_id] = 0.0
                model.actuator_biasprm[actuator_id] = 0.0
                model.actuator_gainprm[actuator_id, 0] = kp
                model.actuator_biasprm[actuator_id, 1] = -kp
                model.actuator_biasprm[actuator_id, 2] = -kd
                model.actuator_forcelimited[actuator_id] = 1
                model.actuator_forcerange[actuator_id] = (-limit, limit)

    def _run_ranges(self, function, *args) -> None:
        futures = [
            self._executor.submit(function, start, end, *args)
            for start, end in self._ranges
        ]
        for future in futures:
            future.result()

    def step(self, actions: np.ndarray) -> np.ndarray:
        actions = np.asarray(actions)
        expected_shape = (self.num_envs, len(self.actuator_ids))
        if actions.shape != expected_shape:
            raise ValueError(f'expected actions shaped {expected_shape}, got {actions.shape}')
        if not np.all(np.isfinite(actions)):
            raise ValueError('actions contain NaN or Inf')
        self._run_ranges(self._step_range, actions)
        return self.torques

    def _step_range(self, start: int, end: int, actions: np.ndarray) -> None:
        for env_id in range(start, end):
            model = self.models[env_id]
            data = self.datas[env_id]
            scaled_action = actions[env_id] * self.action_scale
            if self.control_type == 'P':
                # The affine actuator evaluates kp*(target-q)-kd*qvel inside
                # every mj_step, using that substep's current joint state.
                data.ctrl[self.actuator_ids] = self.default_dof_pos + scaled_action
            else:
                data.ctrl[self.actuator_ids] = np.clip(
                    scaled_action, -self.torque_limits, self.torque_limits
                )
            for _ in range(self.decimation):
                mujoco.mj_step(model, data)
            self.torques[env_id] = data.actuator_force[self.actuator_ids]

    def collect(self):
        self._run_ranges(self._collect_range)
        return (
            self.root_states,
            self.dof_pos,
            self.dof_vel,
            self.contact_forces,
            self.feet_pos_world,
            self.body_ground_contact,
            self.foot_ground_contact,
        )

    def _collect_range(self, start: int, end: int) -> None:
        cf_buf = np.zeros(6, dtype=np.float64)
        for env_id in range(start, end):
            model = self.models[env_id]
            data = self.datas[env_id]
            qpos = data.qpos
            qvel = data.qvel

            root = self.root_states[env_id]
            root[:3] = qpos[self.root_qpos_adr:self.root_qpos_adr + 3]
            root[3:7] = qpos[self.root_qpos_adr + 3:self.root_qpos_adr + 7]
            root[7:13] = qvel[self.root_qvel_adr:self.root_qvel_adr + 6]
            self.dof_pos[env_id] = qpos[self.qpos_adr]
            self.dof_vel[env_id] = qvel[self.qvel_adr]
            self.feet_pos_world[env_id] = data.xpos[self.feet_body_ids]

            env_contact_forces = self.contact_forces[env_id]
            env_body_contact = self.body_ground_contact[env_id]
            env_contact_forces.fill(0.0)
            env_body_contact.fill(False)
            for contact_id in range(data.ncon):
                contact = data.contact[contact_id]
                body1 = int(model.geom_bodyid[contact.geom1])
                body2 = int(model.geom_bodyid[contact.geom2])
                mujoco.mj_contactForce(model, data, contact_id, cf_buf)
                force_world = contact.frame.reshape(3, 3) @ cf_buf[:3]
                env_contact_forces[body1] -= force_world
                env_contact_forces[body2] += force_world
                if body1 == self.world_body_id and body2 != self.world_body_id:
                    env_body_contact[body2] = True
                elif body2 == self.world_body_id and body1 != self.world_body_id:
                    env_body_contact[body1] = True

            self.foot_ground_contact[env_id] = env_body_contact[self.feet_body_ids]

    def close(self) -> None:
        self._executor.shutdown(wait=True)
