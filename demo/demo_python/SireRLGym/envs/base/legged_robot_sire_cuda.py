"""GPU-resident Sire physics for the flat, headless Go2 locomotion task."""

from __future__ import annotations

import torch
import sire

from SireRLGym.envs.base.legged_robot_sire import LeggedRobotSire
from SireRLGym.utils.math import quat_mul, quat_rotate_inverse, torch_rand_float


def _rotation_to_quaternion(rotation):
    """Convert a batch of rotation matrices to ARIS's scalar-last quaternions."""
    r00, r01, r02 = rotation[:, 0].unbind(dim=1)
    r10, r11, r12 = rotation[:, 1].unbind(dim=1)
    r20, r21, r22 = rotation[:, 2].unbind(dim=1)
    xx = 1 + r00 - r11 - r22
    yy = 1 - r00 + r11 - r22
    zz = 1 - r00 - r11 + r22
    ww = 1 + r00 + r11 + r22
    xy, xz, yz = r01 + r10, r02 + r20, r12 + r21
    xw, yw, zw = r21 - r12, r02 - r20, r10 - r01

    # Use the largest component so the representation stays accurate near pi.
    candidates = torch.stack(
        (
            torch.stack((xx, xy, xz, xw), dim=1),
            torch.stack((xy, yy, yz, yw), dim=1),
            torch.stack((xz, yz, zz, zw), dim=1),
            torch.stack((xw, yw, zw, ww), dim=1),
        ),
        dim=1,
    )
    largest = candidates.diagonal(dim1=1, dim2=2).argmax(dim=1)
    quat = candidates.gather(1, largest[:, None, None].expand(-1, 1, 4)).squeeze(1)
    quat = torch.nn.functional.normalize(quat, dim=1)
    return torch.where(quat[:, 3:4] < 0.0, -quat, quat)


def _quaternion_to_rotation(quaternion):
    x, y, z, w = torch.nn.functional.normalize(quaternion, dim=1).unbind(dim=1)
    return torch.stack(
        (
            1 - 2 * (y * y + z * z),
            2 * (x * y - z * w),
            2 * (x * z + y * w),
            2 * (x * y + z * w),
            1 - 2 * (x * x + z * z),
            2 * (y * z - x * w),
            2 * (x * z - y * w),
            2 * (y * z + x * w),
            1 - 2 * (x * x + y * y),
        ),
        dim=1,
    ).reshape(-1, 3, 3)


def _zero_masked(tensor, mask):
    tensor.masked_fill_(mask.reshape(-1, *([1] * (tensor.ndim - 1))), 0)


class _ShardedCudaBatchStepper:
    def __init__(self, device, steppers):
        self._device = device
        self._inputs_ready = torch.cuda.Event()
        self._shards = []
        shard_states = []
        start = 0
        for stepper in steppers:
            stop = start + stepper.numEnvs
            self._shards.append((
                slice(start, stop), stepper, torch.cuda.Stream(device=device), torch.cuda.Event()
            ))
            shard_states.append({
                name: torch.as_tensor(view, device=device)
                for name, view in stepper.outputs().items()
            })
            start = stop
        self._state = {
            name: torch.empty((start, *values.shape[1:]), dtype=values.dtype, device=device)
            for name, values in shard_states[0].items()
        }
        self._gather_buffers = [
            (output, [state[name] for state in shard_states])
            for name, output in self._state.items()
        ]
        self._gather()

    def _gather(self):
        for output, shards in self._gather_buffers:
            torch.cat(shards, dim=0, out=output)

    def _enqueue(self, method, inputs, stream):
        caller = torch.cuda.current_stream(self._device)
        if stream != caller.cuda_stream:
            caller = (torch.cuda.ExternalStream(stream, device=self._device) if stream
                      else torch.cuda.default_stream(self._device))
        # Fence input production and the previous gather before any shard writes again.
        self._inputs_ready.record(caller)
        for group, stepper, worker, done in self._shards:
            worker.wait_event(self._inputs_ready)
            for tensor in inputs:
                if tensor is not None:
                    tensor.record_stream(worker)
            getattr(stepper, method)(
                *(tensor[group] if tensor is not None else None for tensor in inputs),
                worker.cuda_stream,
            )
            done.record(worker)
        for _, _, _, done in self._shards:
            caller.wait_event(done)
        with torch.cuda.stream(caller):
            self._gather()

    def step(self, actions=None, stream=0):
        self._enqueue("step", (actions,), stream)

    def reset(
        self, mask=None, base_poses=None, base_velocities=None,
        joint_positions=None, joint_rates=None, stream=0,
    ):
        self._enqueue("reset", (mask, base_poses, base_velocities, joint_positions, joint_rates),
                      stream)

    def outputs(self):
        return self._state


class LeggedRobotSireCuda(LeggedRobotSire):
    def _simulation_device(self, sim_device):
        device = torch.device(sim_device)
        if device.type != "cuda" or not torch.cuda.is_available():
            raise ValueError("The Sire CUDA environment requires an available CUDA simulation device")
        if device.index is None:
            device = torch.device("cuda", torch.cuda.current_device())
        return device

    def _num_sire_instances(self):
        return 1

    def create_sim(self):
        cfg = self.cfg
        if (
            not self.headless
            or cfg.asset.name != "go2"
            or cfg.terrain.mesh_type != "plane"
            or cfg.terrain.measure_heights
            or cfg.terrain.curriculum
            or cfg.commands.curriculum
            or cfg.control.control_type != "P"
        ):
            raise ValueError("Sire CUDA supports headless flat Go2 with P control")
        if (
            cfg.domain_rand.randomize_friction
            or cfg.domain_rand.randomize_base_mass
            or cfg.domain_rand.push_robots
        ):
            raise ValueError("Sire CUDA requires fixed friction and mass with robot pushes disabled")
        if getattr(cfg.sim, "sire_diagnostics", False):
            raise ValueError("Sire CUDA does not support CPU history diagnostics")
        super().create_sim()

    def _initialize_batch_stepper(self):
        if not hasattr(sire, "SireCudaBatchStepper"):
            raise RuntimeError("This Sire installation was built without the CUDA batch stepper")
        requested_streams = getattr(self.cfg.sim, "sire_cuda_streams", 4)
        if requested_streams < 1:
            raise ValueError("sim.sire_cuda_streams must be positive")
        self.sire_cuda_streams = min(requested_streams, self.num_envs)
        shard_size, extra = divmod(self.num_envs, self.sire_cuda_streams)
        stepper_args = (
            float(self.cfg.sim.dt),
            float(self.dt),
            self._motion_idx.tolist(),
            self.p_gains.tolist(),
            self.d_gains.tolist(),
            self.default_dof_pos.squeeze(0).tolist(),
            float(self.cfg.control.action_scale),
            self.device.index,
        )
        steppers = [
            sire.SireCudaBatchStepper(self.sire_simulators[0], shard_size + (index < extra),
                                    *stepper_args)
            for index in range(self.sire_cuda_streams)
        ]
        first_stepper = steppers[0]
        if (
            first_stepper.bodyCount != self.num_bodies
            or first_stepper.jointCount != self.num_dof
        ):
            raise RuntimeError("Sire CUDA model dimensions do not match the Go2 task")

        self._root_body = first_stepper.rootBody
        if self.sire_cuda_streams == 1:
            self._sire_batch_stepper = first_stepper
            self._cuda_views = first_stepper.outputs()
            self._cuda_state = {
                name: torch.as_tensor(view, device=self.device)
                for name, view in self._cuda_views.items()
            }
        else:
            self._sire_batch_stepper = _ShardedCudaBatchStepper(self.device, steppers)
            self._cuda_state = self._sire_batch_stepper.outputs()
        self._policy_motion_ids = torch.as_tensor(
            self._motion_idx, dtype=torch.long, device=self.device
        )
        self._motion_policy_ids = torch.as_tensor(
            self._motion_idx_inv, dtype=torch.long, device=self.device
        )
        self._zero_joint_rates = torch.zeros(
            (self.num_envs, self.num_dof), dtype=torch.float64, device=self.device
        )
        self._previous_cuda_stream = torch.cuda.current_stream(self.device)
        self._command_period = max(1, int(self.cfg.commands.resampling_time / self.dt))
        third_command = "heading" if self.cfg.commands.heading_command else "ang_vel_yaw"
        bounds = [self.command_ranges[name] for name in ("lin_vel_x", "lin_vel_y", third_command)]
        self._command_min = torch.tensor(
            [bound[0] for bound in bounds], device=self.device, dtype=torch.float32
        )
        self._command_span = torch.tensor(
            [bound[1] - bound[0] for bound in bounds], device=self.device, dtype=torch.float32
        )

    def _ordered_stream(self):
        stream = torch.cuda.current_stream(self.device)
        if stream != self._previous_cuda_stream:
            stream.wait_stream(self._previous_cuda_stream)
            self._previous_cuda_stream = stream
        return stream

    def _refresh_sim_tensors_sire(self):
        if not hasattr(self, "_cuda_state"):
            # The base class initializes derived quantities before constructing the stepper.
            self.root_states[:, 6] = 1.0
            return
        state = self._cuda_state
        base_pose = state["poses"][:, self._root_body]
        base_twist = state["velocities"][:, self._root_body]
        translation = base_pose[:, :, 3]
        angular = base_twist[:, 3:]
        linear_at_base = base_twist[:, :3] + torch.cross(angular, translation, dim=1)
        root = torch.cat(
            (translation, _rotation_to_quaternion(base_pose[:, :, :3]), linear_at_base, angular),
            dim=1,
        ).float()
        positions = state["positions"].index_select(1, self._policy_motion_ids).float()
        rates = state["rates"].index_select(1, self._policy_motion_ids).float()
        torques = state["motor_torques"].index_select(1, self._policy_motion_ids).float()
        forces = state["contact_wrenches"][:, :, :3].float()
        feet = state["poses"][:, :, :, 3].index_select(1, self.feet_indices).float()
        failed = state["status"] != 0
        for values in (root, positions, rates, torques, forces, feet):
            failed |= ~torch.isfinite(values).flatten(1).all(dim=1)
        self._sire_physics_failure_buf.copy_(failed)

        self.root_states.copy_(torch.where(failed[:, None], self.base_init_state, root))
        self.dof_pos.copy_(torch.where(failed[:, None], self.default_dof_pos, positions))
        self.dof_vel.copy_(rates)
        self.torques.copy_(torques)
        self.contact_forces.copy_(forces)
        self.feet_pos_world.copy_(feet)
        self.body_ground_contact.copy_(state["ground_contacts"] & ~failed[:, None])
        self.foot_ground_contact.copy_(self.body_ground_contact.index_select(1, self.feet_indices))
        for values in (self.dof_vel, self.torques, self.contact_forces, self.feet_pos_world):
            _zero_masked(values, failed)

    def _update_base_frame(self):
        # base_quat aliases the quaternion columns of root_states in the base class.
        self.base_lin_vel.copy_(quat_rotate_inverse(self.base_quat, self.root_states[:, 7:10]))
        self.base_ang_vel.copy_(quat_rotate_inverse(self.base_quat, self.root_states[:, 10:13]))
        self.projected_gravity.copy_(quat_rotate_inverse(self.base_quat, self.gravity_vec))

    def _resample_commands_masked(self, mask, initialize=False):
        heading = self.cfg.commands.heading_command
        samples = self._command_min + self._command_span * torch.rand(
            (self.num_envs, 3), device=self.device, dtype=self.commands.dtype
        )
        linear = samples[:, :2]
        linear *= (torch.linalg.vector_norm(linear, dim=1) > 0.2)[:, None]
        targets = torch.zeros_like(self.command_targets)
        targets[:, :2] = linear
        targets[:, 3 if heading else 2] = samples[:, 2]

        prob_zero = float(getattr(self.cfg.commands, "prob_zero_command", 0.0))
        prob_negative = float(getattr(self.cfg.commands, "prob_negative_command", 0.0))
        if prob_zero > 0.0 or prob_negative > 0.0:
            draws = torch.rand(self.num_envs, device=self.device)
            zero = draws < prob_zero
            negative = (draws >= prob_zero) & (draws < prob_zero + prob_negative)
            low, high = getattr(self.cfg.commands, "negative_lin_vel_x_range", (-0.3, -0.1))
            targets[:, 0] = torch.where(
                negative, torch_rand_float(low, high, (self.num_envs,), self.device), targets[:, 0]
            )
            targets[:, 1].masked_fill_(negative, 0.0)
            targets[:, :3].masked_fill_(zero[:, None], 0.0)
            if heading:
                targets[:, 3] = torch.where(zero | negative, self._current_heading(), targets[:, 3])

        self.command_targets.copy_(torch.where(mask[:, None], targets, self.command_targets))
        if initialize:
            desired = self._desired_velocity_commands()
            self.commands[:, :3].copy_(torch.where(mask[:, None], desired, self.commands[:, :3]))
            if heading:
                self.commands[:, 3].copy_(torch.where(mask, self.command_targets[:, 3], self.commands[:, 3]))

    def _post_physics_step_callback(self):
        self._resample_commands_masked(self.episode_length_buf.remainder(self._command_period) == 0)
        self._update_smoothed_commands()

    def _reset_masked(self, mask, stream):
        cfg = self.cfg.init_state
        positions = self.default_dof_pos * torch_rand_float(
            0.5, 1.5, (self.num_envs, self.num_dof), self.device
        )
        joint_positions = positions.index_select(1, self._motion_policy_ids).double().contiguous()
        world_position = self.base_init_state[:3] + self._physics_origins
        quaternion = self.base_init_state[3:7].expand(self.num_envs, -1)
        if cfg.init_yaw_range[0] != 0.0 or cfg.init_yaw_range[1] != 0.0:
            half_yaw = 0.5 * torch_rand_float(*cfg.init_yaw_range, (self.num_envs,), self.device)
            zeros = torch.zeros_like(half_yaw)
            yaw_quat = torch.stack((zeros, zeros, half_yaw.sin(), half_yaw.cos()), dim=1)
            quaternion = quat_mul(yaw_quat, quaternion)
        base_poses = torch.cat(
            (_quaternion_to_rotation(quaternion.double()), world_position.double().unsqueeze(2)),
            dim=2,
        ).contiguous()
        point_velocity = torch_rand_float(-0.5, 0.5, (self.num_envs, 6), self.device).double()
        angular = point_velocity[:, 3:]
        linear = point_velocity[:, :3] - torch.cross(angular, world_position.double(), dim=1)
        base_velocities = torch.cat((linear, angular), dim=1).contiguous()
        self._sire_batch_stepper.reset(
            mask, base_poses, base_velocities, joint_positions, self._zero_joint_rates,
            stream.cuda_stream,
        )

        self._refresh_sim_tensors_sire()
        self._update_base_frame()
        self._resample_commands_masked(mask, initialize=True)
        self.last_feet_pos_world.copy_(
            torch.where(mask[:, None, None], self.feet_pos_world, self.last_feet_pos_world)
        )
        for values in (
            self.actions, self.last_actions, self.last_dof_vel, self.last_root_vel,
            self.feet_air_time, self.last_contacts, self.last_slip_contacts,
            self.episode_length_buf, self.episode_foot_contact_sums, self.episode_hip_abs_sums,
            *self.episode_sums.values(),
        ):
            _zero_masked(values, mask)

    @torch.no_grad()
    def reset_idx(self, env_ids):
        stream = self._ordered_stream()
        if isinstance(env_ids, torch.Tensor) and env_ids.dtype == torch.bool:
            if env_ids.shape != self.reset_buf.shape:
                raise ValueError("Reset mask must have shape [num_envs]")
            mask = env_ids.to(self.device).contiguous()
        else:
            mask = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
            mask[env_ids] = True
        self.reset_buf |= mask
        self._reset_masked(mask, stream)

    def _episode_metrics(self, mask, failure_count):
        count = mask.sum(dtype=self.rew_buf.dtype)
        denominator = count.clamp_min(1)
        length = self.episode_length_buf.clamp_min(1)
        duration = length * self.dt

        def mean_completed(values):
            valid = mask & torch.isfinite(values)
            return torch.where(valid, values, 0.0).sum() / denominator

        metrics = {"episode_count": count, "physics_failure_count": failure_count}
        for name, sums in self.episode_sums.items():
            metrics["rew_" + name] = mean_completed(sums / duration)
        for index, name in enumerate(self.foot_names):
            metrics[f"contact_ratio_{name}"] = mean_completed(
                self.episode_foot_contact_sums[:, index] / length
            )
        for index, name in enumerate(self.hip_names):
            metrics[f"hip_offset_{name}"] = mean_completed(
                self.episode_hip_abs_sums[:, index] / length
            )
        return metrics

    @torch.no_grad()
    def stepSireBatch(self, actions):
        stream = self._ordered_stream()
        if actions.device != self.device or actions.shape != self.actions.shape:
            raise ValueError("Actions must have shape [num_envs, num_actions] on the simulation device")
        limit = self.cfg.normalization.clip_actions
        self.actions.copy_(actions.detach().clamp(-limit, limit))
        self._sire_batch_stepper.step(self.actions, stream.cuda_stream)

        self.episode_length_buf += 1
        self.common_step_counter += 1
        self._refresh_sim_tensors_sire()
        self._update_base_frame()
        self._post_physics_step_callback()
        self._update_episode_diagnostics()
        self.check_termination()
        self.time_out_buf &= ~self._sire_physics_failure_buf
        self.compute_reward()

        failed = self._sire_physics_failure_buf | ~torch.isfinite(self.rew_buf)
        self.reset_buf |= failed
        self.time_out_buf &= ~failed
        self.rew_buf.masked_fill_(failed, 0.0)
        failure_count = failed.sum(dtype=self.rew_buf.dtype)
        self.extras = {
            "episode": self._episode_metrics(self.reset_buf, failure_count),
            "sire_physics_failure_count": failure_count,
        }
        if self.cfg.env.send_timeouts:
            self.extras["time_outs"] = self.time_out_buf
        self._reset_masked(self.reset_buf, stream)
        self._sire_physics_failure_buf.copy_(failed)

        self.compute_observations()
        self.last_actions.copy_(self.actions)
        self.last_dof_vel.copy_(self.dof_vel)
        self.last_root_vel.copy_(self.root_states[:, 7:13])
        self.last_feet_pos_world.copy_(self.feet_pos_world)
        return self._clip_and_collect_step_result()

    def setSireHistoryRecording(self, enabled):
        if enabled:
            raise ValueError("Sire CUDA does not support CPU history recording")

    def resetSireRecorders(self):
        """The CUDA stepper has no CPU recorders."""
