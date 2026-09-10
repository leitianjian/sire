from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Tuple

import mujoco
import mujoco.viewer
import numpy as np
import torch

from rsl_rl.env import VecEnv

from RLGym import ROOT_DIR
from RLGym.envs.base.legged_robot_config import LeggedRobotCfg
from RLGym.utils.helpers import class_to_dict
from RLGym.utils.math import quat_apply, quat_apply_yaw, quat_mul, quat_rotate_inverse, torch_rand_float, wrap_to_pi
from RLGym.utils.terrain import TerrainLayout


class LeggedRobot(VecEnv):
    """
    VecEnv (向量化环境），用于在物理引擎中同时、并行地模拟多个足式机器人。
    它不仅负责推进物理仿真（physics engine），还负责计算强化学习所需的核心信号：
    - 奖励 (rewards)：告诉智能体当前表现有多好；
    - 观测 (observations)：智能体用来做决策的自身及环境状态输入。
    此外，它还处理领域随机化（Domain Randomizations，比如随机推力）和地形难度的课程学习。
    通过并行提供大批量的状态、接收大批量的动作，大大加速了 RL 策略的训练。
    """
    def __init__(self, cfg: LeggedRobotCfg, sim_params=None, physics_engine=None, sim_device='cpu', headless=True):
        self.cfg = cfg
        self.sim_params = sim_params
        self.physics_engine = physics_engine
        self.device = torch.device('cpu')
        self.headless = headless

        self.height_samples = None
        self.debug_viz = False
        self.init_done = False
        self.viewer = None
        self.viewer_camera_initialized = False

        self._parse_cfg(self.cfg)
        self.create_sim()
        self._sync_dt_with_model()
        self._init_buffers()
        self._prepare_reward_function()
        self.reset_idx(torch.arange(self.num_envs, device=self.device))
        self.compute_observations()
        self.init_done = True

    def step(self, actions):
        """
        在向量化环境中执行一个“步（step）”：也就是“环境接收动作并反馈下一个状态”。
        接收神经网络输出的大批量动作（batched actions），将它们截断、缩放，并转化为关节扭矩。
        然后推进 Mujoco 物理仿真器向前运行一段时间，最后更新并返回下一步的环境状态。
        """
        clip_actions = self.cfg.normalization.clip_actions
        self.actions = torch.clip(actions, -clip_actions, clip_actions).to(self.device)

        for _ in range(self.cfg.control.decimation):
            self.torques = self._compute_torques(self.actions).view(self.torques.shape)
            for i in range(self.num_envs):
                self.datas[i].ctrl[self.actuator_ids_np] = self.torques[i].cpu().numpy()
                mujoco.mj_step(self.model, self.datas[i])

        self.post_physics_step()

        clip_obs = self.cfg.normalization.clip_observations
        self.obs_buf = torch.clip(self.obs_buf, -clip_obs, clip_obs)
        if self.privileged_obs_buf is not None:
            self.privileged_obs_buf = torch.clip(self.privileged_obs_buf, -clip_obs, clip_obs)

        return self.obs_buf, self.privileged_obs_buf, self.rew_buf, self.reset_buf, self.extras

    def post_physics_step(self):
        """
        在每个物理仿真步骤之后被调用，处理强化学习训练所必须的“内务工作”：
        更新观测/状态缓冲区、计算智能体的奖励（rewards）、判断当前回合是否失败/结束（termination）、
        施加随机物理扰动（pushes），以及为策略重新分配运动指令（如随机生成下一步的速度命令）。
        """
        self.extras = {}
        self.episode_length_buf += 1
        self.common_step_counter += 1

        self._refresh_sim_tensors()

        self.base_quat[:] = self.root_states[:, 3:7]
        self.base_lin_vel[:] = quat_rotate_inverse(self.base_quat, self.root_states[:, 7:10])
        self.base_ang_vel[:] = quat_rotate_inverse(self.base_quat, self.root_states[:, 10:13])
        self.projected_gravity[:] = quat_rotate_inverse(self.base_quat, self.gravity_vec)

        self._post_physics_step_callback()
        self._update_episode_diagnostics()
        self._post_physics_step_tasks()

        self.check_termination()
        self.compute_reward()
        env_ids = self.reset_buf.nonzero(as_tuple=False).flatten()
        self.reset_idx(env_ids)
        self.compute_observations()

        self.last_actions[:] = self.actions[:]
        self.last_dof_vel[:] = self.dof_vel[:]
        self.last_root_vel[:] = self.root_states[:, 7:13]
        self.last_feet_pos_world[:] = self.feet_pos_world[:]

    def check_termination(self):
        base_contacts = torch.norm(self.contact_forces[:, self.termination_contact_indices, :], dim=-1) > 1.0
        self.reset_buf = torch.any(base_contacts, dim=1)
        # MuJoCo free-joint quaternions are scalar-first [w, x, y, z].
        w, x, y, z = self.base_quat.unbind(dim=1)
        roll = torch.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = torch.asin(torch.clamp(2.0 * (w * y - z * x), -1.0, 1.0))
        self.reset_buf |= (torch.abs(pitch) > 1.0) | (torch.abs(roll) > 0.8)
        self.time_out_buf = self.episode_length_buf > self.max_episode_length
        self.reset_buf |= self.time_out_buf
        self._update_task_termination()

    def reset_idx(self, env_ids):
        if len(env_ids) == 0:
            return

        if self.cfg.terrain.curriculum:
            self._update_terrain_curriculum(env_ids)
        if self.cfg.commands.curriculum and (self.common_step_counter % int(self.max_episode_length) == 0):
            self.update_command_curriculum(env_ids)

        self._reset_dofs(env_ids)
        self._reset_root_states(env_ids)
        self._resample_commands(env_ids, initialize=True)

        ep_len = self.episode_length_buf[env_ids].float().clamp(min=1.0)
        ep_duration = (ep_len * self.dt).clamp(min=self.dt)

        self.last_actions[env_ids] = 0.0
        self.last_dof_vel[env_ids] = 0.0
        self.feet_air_time[env_ids] = 0.0
        self.episode_length_buf[env_ids] = 0
        self.reset_buf[env_ids] = 1

        self.extras['episode'] = {}
        self.extras['episode']['episode_count'] = float(len(env_ids))
        for key in self.episode_sums.keys():
            # Log reward terms as average contribution per second over the actual episode duration.
            self.extras['episode']['rew_' + key] = torch.mean(self.episode_sums[key][env_ids] / ep_duration)
            self.episode_sums[key][env_ids] = 0.0
        if len(env_ids) > 0:
            for foot_i, foot_name in enumerate(self.foot_names):
                ratio = self.episode_foot_contact_sums[env_ids, foot_i] / ep_len
                self.extras['episode'][f'contact_ratio_{foot_name}'] = torch.mean(ratio)
                self.episode_foot_contact_sums[env_ids, foot_i] = 0.0
            for hip_i, hip_name in enumerate(self.hip_names):
                offset = self.episode_hip_abs_sums[env_ids, hip_i] / ep_len
                self.extras['episode'][f'hip_offset_{hip_name}'] = torch.mean(offset)
                self.episode_hip_abs_sums[env_ids, hip_i] = 0.0

        if self.cfg.terrain.curriculum:
            self.extras['episode']['terrain_level'] = torch.mean(self.terrain_levels.float())
        if self.cfg.commands.curriculum:
            self.extras['episode']['max_command_x'] = self.command_ranges['lin_vel_x'][1]
        if self.cfg.env.send_timeouts:
            self.extras['time_outs'] = self.time_out_buf
        self._update_task_episode_extras(env_ids)
        self._reset_task_buffers(env_ids)

        # Hip coupling diagnostics for gait debugging.
        # self.extras['episode'].update(self._compute_hip_debug_stats())

    def compute_reward(self):
        """
        计算奖励值（Reward），这是强化学习算法优化策略的唯一目标。
        它会便利所有注册好的奖励函数（例如：鼓励速度跟踪目标的奖励，惩罚能耗、碰撞的奖励），
        按各自配置好的权重系数进行缩放，并将总和累加到 `rew_buf`（奖励缓冲区）中。
        """
        self.rew_buf[:] = 0.0
        for i in range(len(self.reward_functions)):
            name = self.reward_names[i]
            rew = self.reward_functions[i]() * self.reward_scales[name]
            self.rew_buf += rew
            self.episode_sums[name] += rew

        if self.cfg.rewards.only_positive_rewards:
            self.rew_buf[:] = torch.clip(self.rew_buf[:], min=0.0)

        if 'termination' in self.reward_scales:
            rew = self._reward_termination() * self.reward_scales['termination']
            self.rew_buf += rew
            self.episode_sums['termination'] += rew

    def compute_observations(self):
        """
        收集各种传感器反馈，为策略网络（Policy）拼装出观测向量（Observation Vector）。
        这相当于智能体的“眼睛”和“本体感觉”，通常包括：基座角速度、重力投影（以反映倾斜角度）、
        期望的运动速度命令、各关节的位置与速度，以及上一步执行的控制动作。
        此外还会添加噪声，以模拟真实物理世界中不完美的传感器测量。
        """
        actor_obs = torch.cat(
            (
                self.base_ang_vel * self.obs_scales.ang_vel,
                self.projected_gravity,
                self.commands[:, :3] * self.commands_scale,
                (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,
                self.dof_vel * self.obs_scales.dof_vel,
                self.actions,
            ),
            dim=-1,
        )
        full_obs = torch.cat(
            (
                self.base_lin_vel * self.obs_scales.lin_vel,
                actor_obs,
            ),
            dim=-1,
        )
        if self.cfg.terrain.measure_heights:
            heights = torch.clip(self.root_states[:, 2].unsqueeze(1) - 0.5 - self.measured_heights, -1, 1.0) * self.obs_scales.height_measurements
            full_obs = torch.cat((full_obs, heights), dim=-1)

        # Support both legacy 235-dim privileged observations and newer 263-dim
        # checkpoints by appending critic-only extras when requested.
        if self.num_privileged_obs is not None and self.num_privileged_obs > full_obs.shape[1]:
            privileged_extras = torch.cat(
                (
                    torch.norm(self.contact_forces[:, self.feet_indices, :], dim=-1) * 1e-3,
                    self.torques / torch.clamp(self.torque_limits, min=1e-6),
                    (self.last_dof_vel - self.dof_vel) / max(self.dt, 1e-6) * 1e-4,
                ),
                dim=-1,
            )
            full_obs = torch.cat((full_obs, privileged_extras), dim=-1)

        self.obs_buf = actor_obs[:, :self.num_obs]

        if self.add_noise:
            self.obs_buf += (2 * torch.rand_like(self.obs_buf) - 1) * self.noise_scale_vec

        if self.privileged_obs_buf is not None:
            self.privileged_obs_buf[:] = full_obs[:, :self.num_privileged_obs]

    def create_sim(self):
        self.up_axis_idx = 2
        self.terrain = None
        if self.cfg.terrain.mesh_type in ['heightfield', 'trimesh']:
            self.terrain = TerrainLayout(self.cfg.terrain, self.cfg.env.num_envs)
        self._create_envs()

    # def render(self):
    #     if self.headless:
    #         return
    #     if self.viewer is None:
    #         self.viewer = mujoco.viewer.launch_passive(self.model, self.datas[0])
    #         self.viewer_camera_initialized = False
    #     if not self.viewer.is_running():
    #         return

    #     base_pos = self.root_states[0, :3].detach().cpu().numpy()
    #     terrain_length = float(getattr(self.cfg.terrain, 'terrain_length', 10.0))
    #     terrain_width = float(getattr(self.cfg.terrain, 'terrain_width', 10.0))
    #     distance = max(6.0, 0.9 * max(terrain_length, terrain_width))

    #     self.viewer.cam.lookat[:] = base_pos
    #     self.viewer.cam.lookat[2] += 0.2
    #     if not self.viewer_camera_initialized:
    #         self.viewer.cam.distance = distance
    #         self.viewer.cam.elevation = -30.0
    #         self.viewer.cam.azimuth = 135.0
    #         self.viewer_camera_initialized = True
    #     self.viewer.sync()

    def _post_physics_step_callback(self):
        env_ids = (self.episode_length_buf % int(self.cfg.commands.resampling_time / self.dt) == 0).nonzero(as_tuple=False).flatten()
        self._resample_commands(env_ids)
        self._update_smoothed_commands()

        if self.cfg.terrain.measure_heights:
            self.measured_heights = self._get_heights()
        if self.cfg.domain_rand.push_robots and (self.common_step_counter % int(self.cfg.domain_rand.push_interval) == 0):
            self._push_robots()

    def _current_heading(self, env_ids=None):
        quaternions = self.base_quat if env_ids is None else self.base_quat[env_ids]
        forward = quat_apply(
            quaternions,
            self.forward_vec if env_ids is None else self.forward_vec[env_ids],
        )
        return torch.atan2(forward[:, 1], forward[:, 0])

    def _desired_velocity_commands(self, env_ids=None):
        targets = self.command_targets if env_ids is None else self.command_targets[env_ids]
        desired = targets[:, :3].clone()
        if self.cfg.commands.heading_command:
            desired[:, 2] = torch.clip(
                0.5 * wrap_to_pi(targets[:, 3] - self._current_heading(env_ids)),
                -1.0,
                1.0,
            )
        return desired

    def _update_smoothed_commands(self):
        desired = self._desired_velocity_commands()
        if not self.cfg.commands.smooth_commands:
            self.commands[:, :3] = desired
        else:
            max_delta = torch.tensor(
                [
                    self.cfg.commands.lin_vel_slew_rate * self.dt,
                    self.cfg.commands.lin_vel_slew_rate * self.dt,
                    self.cfg.commands.ang_vel_yaw_slew_rate * self.dt,
                ],
                dtype=self.commands.dtype,
                device=self.device,
            )
            delta = torch.clamp(desired - self.commands[:, :3], -max_delta, max_delta)
            self.commands[:, :3] += delta
        if self.cfg.commands.heading_command:
            self.commands[:, 3] = self.command_targets[:, 3]

    def _resample_commands(self, env_ids, initialize=False):
        if len(env_ids) == 0:
            return
        self.command_targets[env_ids, 0] = torch_rand_float(self.command_ranges['lin_vel_x'][0], self.command_ranges['lin_vel_x'][1], (len(env_ids), 1), device=self.device).squeeze(1)
        self.command_targets[env_ids, 1] = torch_rand_float(self.command_ranges['lin_vel_y'][0], self.command_ranges['lin_vel_y'][1], (len(env_ids), 1), device=self.device).squeeze(1)
        if self.cfg.commands.heading_command:
            self.command_targets[env_ids, 3] = torch_rand_float(self.command_ranges['heading'][0], self.command_ranges['heading'][1], (len(env_ids), 1), device=self.device).squeeze(1)
        else:
            self.command_targets[env_ids, 2] = torch_rand_float(self.command_ranges['ang_vel_yaw'][0], self.command_ranges['ang_vel_yaw'][1], (len(env_ids), 1), device=self.device).squeeze(1)

        self.command_targets[env_ids, :2] *= (torch.norm(self.command_targets[env_ids, :2], dim=1) > 0.2).unsqueeze(1)

        prob_zero = float(getattr(self.cfg.commands, 'prob_zero_command', 0.0))
        prob_neg = float(getattr(self.cfg.commands, 'prob_negative_command', 0.0))
        if prob_zero > 0.0 or prob_neg > 0.0:
            draws = torch.rand(len(env_ids), device=self.device)
            zero_mask = draws < prob_zero
            neg_mask = (draws >= prob_zero) & (draws < prob_zero + prob_neg)
            if zero_mask.any():
                zero_ids = env_ids[zero_mask]
                self.command_targets[zero_ids, :3] = 0.0
                if self.cfg.commands.heading_command:
                    self.command_targets[zero_ids, 3] = self._current_heading(zero_ids)
            if neg_mask.any():
                neg_ids = env_ids[neg_mask]
                neg_range = getattr(self.cfg.commands, 'negative_lin_vel_x_range', [-0.3, -0.1])
                count = int(neg_mask.sum().item())
                self.command_targets[neg_ids, 0] = torch_rand_float(
                    float(neg_range[0]), float(neg_range[1]),
                    (count, 1), device=self.device,
                ).squeeze(1)
                self.command_targets[neg_ids, 1] = 0.0
                if self.cfg.commands.heading_command:
                    self.command_targets[neg_ids, 3] = self._current_heading(neg_ids)

        if initialize:
            self.commands[env_ids, :3] = self._desired_velocity_commands(env_ids)
            if self.cfg.commands.heading_command:
                self.commands[env_ids, 3] = self.command_targets[env_ids, 3]

    def _compute_torques(self, actions):
        actions_scaled = actions * self.cfg.control.action_scale
        control_type = self.cfg.control.control_type
        if control_type == 'P':
            torques = self.p_gains * (actions_scaled + self.default_dof_pos - self.dof_pos) - self.d_gains * self.dof_vel
        elif control_type == 'V':
            torques = self.p_gains * (actions_scaled - self.dof_vel) - self.d_gains * (self.dof_vel - self.last_dof_vel) / self.dt
        elif control_type == 'T':
            torques = actions_scaled
        else:
            raise NameError(f'Unknown controller type: {control_type}')
        return torch.clip(torques, -self.torque_limits, self.torque_limits)

    def _reset_dofs(self, env_ids):
        self.dof_pos[env_ids] = self.default_dof_pos * torch_rand_float(0.5, 1.5, (len(env_ids), self.num_dof), device=self.device)
        self.dof_vel[env_ids] = 0.0

        for eid in env_ids.tolist():
            d = self.datas[eid]
            d.qpos[self.qpos_adr_np] = self.dof_pos[eid].cpu().numpy()
            d.qvel[self.qvel_adr_np] = 0.0
            mujoco.mj_forward(self.model, d)

    def _reset_root_states(self, env_ids):
        if self.custom_origins:
            self.root_states[env_ids] = self.base_init_state
            self.root_states[env_ids, :3] += self.env_origins[env_ids]
            rand_x_low, rand_x_high = self.cfg.terrain.spawn_rand_x_range
            rand_y_low, rand_y_high = self.cfg.terrain.spawn_rand_y_range
            self.root_states[env_ids, 0] += torch_rand_float(rand_x_low, rand_x_high, (len(env_ids), 1), device=self.device).squeeze(1)
            self.root_states[env_ids, 1] += torch_rand_float(rand_y_low, rand_y_high, (len(env_ids), 1), device=self.device).squeeze(1)
        else:
            self.root_states[env_ids] = self.base_init_state
            self.root_states[env_ids, :3] += self.env_origins[env_ids]

        yaw_low, yaw_high = self.cfg.init_state.init_yaw_range
        if yaw_low != 0.0 or yaw_high != 0.0:
            yaw = torch_rand_float(yaw_low, yaw_high, (len(env_ids), 1), device=self.device).squeeze(1)
            half_yaw = 0.5 * yaw
            yaw_quat = torch.stack(
                (
                    torch.cos(half_yaw),
                    torch.zeros_like(half_yaw),
                    torch.zeros_like(half_yaw),
                    torch.sin(half_yaw),
                ),
                dim=1,
            )
            self.root_states[env_ids, 3:7] = quat_mul(yaw_quat, self.root_states[env_ids, 3:7])

        self.root_states[env_ids, 7:13] = torch_rand_float(-0.5, 0.5, (len(env_ids), 6), device=self.device)

        for eid in env_ids.tolist():
            d = self.datas[eid]
            root = self.root_states[eid]
            d.qpos[self.root_qpos_adr_np : self.root_qpos_adr_np + 3] = root[:3].cpu().numpy()
            d.qpos[self.root_qpos_adr_np + 3 : self.root_qpos_adr_np + 7] = root[3:7].cpu().numpy()
            d.qvel[self.root_qvel_adr_np : self.root_qvel_adr_np + 6] = root[7:13].cpu().numpy()
            mujoco.mj_forward(self.model, d)

    def _push_robots(self):
        max_vel = self.cfg.domain_rand.max_push_vel_xy
        self.root_states[:, 7:9] = torch_rand_float(-max_vel, max_vel, (self.num_envs, 2), device=self.device)
        for i in range(self.num_envs):
            self.datas[i].qvel[self.root_qvel_adr_np : self.root_qvel_adr_np + 2] = self.root_states[i, 7:9].cpu().numpy()

    def _update_terrain_curriculum(self, env_ids):
        if not self.init_done or not self.custom_origins or not hasattr(self, 'terrain_origins'):
            return
        distance = torch.norm(self.root_states[env_ids, :2] - self.env_origins[env_ids, :2], dim=1)
        move_up = distance > (self.terrain.patch_length * 0.5)
        move_down = distance < torch.norm(self.commands[env_ids, :2], dim=1) * self.max_episode_length_s * 0.5
        move_down &= ~move_up
        self.terrain_levels[env_ids] += move_up.long() - move_down.long()
        self.terrain_levels[env_ids] = torch.where(
            self.terrain_levels[env_ids] >= self.max_terrain_level,
            torch.randint_like(self.terrain_levels[env_ids], self.max_terrain_level),
            torch.clamp(self.terrain_levels[env_ids], min=0),
        )
        self.env_origins[env_ids] = self.terrain_origins[self.terrain_levels[env_ids], self.terrain_types[env_ids]]

    def update_command_curriculum(self, env_ids):
        if 'tracking_lin_vel' not in self.episode_sums:
            return
        if torch.mean(self.episode_sums['tracking_lin_vel'][env_ids]) / self.max_episode_length > 0.8 * self.reward_scales['tracking_lin_vel']:
            self.command_ranges['lin_vel_x'][0] = np.clip(self.command_ranges['lin_vel_x'][0] - 0.5, -self.cfg.commands.max_curriculum, 0.0)
            self.command_ranges['lin_vel_x'][1] = np.clip(self.command_ranges['lin_vel_x'][1] + 0.5, 0.0, self.cfg.commands.max_curriculum)

    def _get_noise_scale_vec(self, cfg):
        noise_vec = torch.zeros(self.num_obs, device=self.device)
        self.add_noise = self.cfg.noise.add_noise
        noise_scales = self.cfg.noise.noise_scales
        noise_level = self.cfg.noise.noise_level

        for start, end, value in (
            (0, 3, noise_scales.ang_vel * noise_level * self.obs_scales.ang_vel),
            (3, 6, noise_scales.gravity * noise_level),
            (6, 9, 0.0),
            (9, 21, noise_scales.dof_pos * noise_level * self.obs_scales.dof_pos),
            (21, 33, noise_scales.dof_vel * noise_level * self.obs_scales.dof_vel),
            (33, 45, 0.0),
        ):
            if start < self.num_obs:
                noise_vec[start:min(end, self.num_obs)] = value
        return noise_vec

    def _init_buffers(self):
        self._refresh_sim_tensors()

        self.common_step_counter = 0
        self.extras = {}
        self.noise_scale_vec = self._get_noise_scale_vec(self.cfg)
        self.gravity_vec = torch.tensor([0.0, 0.0, -1.0], device=self.device).repeat((self.num_envs, 1))
        self.forward_vec = torch.tensor([1.0, 0.0, 0.0], device=self.device).repeat((self.num_envs, 1))
        self.base_quat = self.root_states[:, 3:7]

        self.torques = torch.zeros(self.num_envs, self.num_actions, dtype=torch.float, device=self.device)
        self.p_gains = torch.zeros(self.num_actions, dtype=torch.float, device=self.device)
        self.d_gains = torch.zeros(self.num_actions, dtype=torch.float, device=self.device)
        self.actions = torch.zeros(self.num_envs, self.num_actions, dtype=torch.float, device=self.device)
        self.last_actions = torch.zeros(self.num_envs, self.num_actions, dtype=torch.float, device=self.device)
        self.last_dof_vel = torch.zeros_like(self.dof_vel)
        self.last_root_vel = torch.zeros_like(self.root_states[:, 7:13])

        self.commands = torch.zeros(self.num_envs, self.cfg.commands.num_commands, dtype=torch.float, device=self.device)
        self.command_targets = torch.zeros_like(self.commands)
        self.commands_scale = torch.tensor([self.obs_scales.lin_vel, self.obs_scales.lin_vel, self.obs_scales.ang_vel], device=self.device)
        self.feet_air_time = torch.zeros(self.num_envs, self.feet_indices.shape[0], dtype=torch.float, device=self.device)
        self.last_contacts = torch.zeros(self.num_envs, len(self.feet_indices), dtype=torch.bool, device=self.device)
        self.last_feet_pos_world = torch.zeros(self.num_envs, len(self.feet_indices), 3, dtype=torch.float, device=self.device)
        self.episode_foot_contact_sums = torch.zeros(self.num_envs, len(self.feet_indices), dtype=torch.float, device=self.device)

        self.base_lin_vel = quat_rotate_inverse(self.base_quat, self.root_states[:, 7:10])
        self.base_ang_vel = quat_rotate_inverse(self.base_quat, self.root_states[:, 10:13])
        self.projected_gravity = quat_rotate_inverse(self.base_quat, self.gravity_vec)

        if self.cfg.terrain.measure_heights:
            self.height_points = self._init_height_points()
            self.measured_heights = self._get_heights()
        else:
            self.measured_heights = torch.zeros(self.num_envs, 1, device=self.device)

        self.default_dof_pos = torch.zeros(self.num_dof, dtype=torch.float, device=self.device)
        for i in range(self.num_dof):
            name = self.dof_names[i]
            angle = self.cfg.init_state.default_joint_angles[name]
            self.default_dof_pos[i] = angle
            found = False
            for dof_name in self.cfg.control.stiffness.keys():
                if dof_name in name:
                    self.p_gains[i] = self.cfg.control.stiffness[dof_name]
                    self.d_gains[i] = self.cfg.control.damping[dof_name]
                    found = True
            if not found and self.cfg.control.control_type in ['P', 'V']:
                self.p_gains[i] = 0.0
                self.d_gains[i] = 0.0
        self.default_dof_pos = self.default_dof_pos.unsqueeze(0)
        hip_ids = [i for i, name in enumerate(self.dof_names) if 'hip_joint' in name]
        self.hip_indices = torch.tensor(hip_ids, dtype=torch.long, device=self.device)
        self.episode_hip_abs_sums = torch.zeros(self.num_envs, len(self.hip_indices), dtype=torch.float, device=self.device)

        self.obs_buf = torch.zeros(self.num_envs, self.num_obs, dtype=torch.float, device=self.device)
        self.privileged_obs_buf = None if self.num_privileged_obs is None else torch.zeros(self.num_envs, self.num_privileged_obs, dtype=torch.float, device=self.device)
        self.rew_buf = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.reset_buf = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self.time_out_buf = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.episode_length_buf = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)

        self.compute_observations()
        self.last_feet_pos_world[:] = self.feet_pos_world[:]

    def _prepare_reward_function(self):
        for key in list(self.reward_scales.keys()):
            scale = self.reward_scales[key]
            if scale == 0:
                self.reward_scales.pop(key)
            else:
                self.reward_scales[key] *= self.dt

        self.reward_functions = []
        self.reward_names = []
        for name, _ in self.reward_scales.items():
            if name == 'termination':
                continue
            self.reward_names.append(name)
            self.reward_functions.append(getattr(self, '_reward_' + name))

        self.episode_sums = {
            name: torch.zeros(self.num_envs, dtype=torch.float, device=self.device) for name in self.reward_scales.keys()
        }

    def _create_envs(self):
        model_path = Path(self.cfg.asset.file.format(LEGGED_GYM_ROOT_DIR=str(ROOT_DIR)))
        if not model_path.exists():
            raise FileNotFoundError(f'MJCF file not found: {model_path}')

        self.num_envs = self.cfg.env.num_envs
        self.num_obs = self.cfg.env.num_observations
        self.num_privileged_obs = self.cfg.env.num_privileged_obs
        self.num_actions = self.cfg.env.num_actions

        self.datas: List[mujoco.MjData] = []
        self.generated_model_path = model_path

        if self.terrain is not None:
            self.generated_model_path = self.terrain.write_scene(model_path)

        base_model = mujoco.MjModel.from_xml_path(str(self.generated_model_path))
        base_model.opt.timestep = float(self.cfg.sim.dt)
        self.model = base_model
        self.models: List[mujoco.MjModel] = [self.model for _ in range(self.num_envs)]
        self.num_dof = self.num_actions
        self.num_dofs = self.num_actions
        self.num_bodies = base_model.nbody

        self.dof_names = [k for k in self.cfg.init_state.default_joint_angles.keys()]
        self.actuator_names = [n.replace('_joint', '') for n in self.dof_names]

        self.joint_ids_np = np.array([mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in self.dof_names], dtype=np.int32)
        self.actuator_ids_np = np.array([mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in self.actuator_names], dtype=np.int32)

        self.qpos_adr_np = base_model.jnt_qposadr[self.joint_ids_np]
        self.qvel_adr_np = base_model.jnt_dofadr[self.joint_ids_np]

        self.root_joint_id_np = mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_JOINT, 'root')
        self.root_qpos_adr_np = int(base_model.jnt_qposadr[self.root_joint_id_np])
        self.root_qvel_adr_np = int(base_model.jnt_dofadr[self.root_joint_id_np])

        self.base_body_id_np = mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_BODY, 'base')
        self.world_body_id_np = 0
        self._ray_geomgroup = np.ones(6, dtype=np.uint8)
        self._ray_down_vec = np.array([0.0, 0.0, -1.0], dtype=np.float64)
        self._ray_height_margin = 5.0

        body_names = [mujoco.mj_id2name(base_model, mujoco.mjtObj.mjOBJ_BODY, i) or '' for i in range(base_model.nbody)]
        feet_names = [s for s in body_names if self.cfg.asset.foot_name in s]
        if len(feet_names) == 0:
            raise RuntimeError(
                f"No foot bodies found with foot_name='{self.cfg.asset.foot_name}'. "
                "Please define explicit *_foot bodies in XML."
            )
        penalized_contact_names = []
        for name in self.cfg.asset.penalize_contacts_on:
            penalized_contact_names.extend([s for s in body_names if name in s])
        termination_contact_names = []
        for name in self.cfg.asset.terminate_after_contacts_on:
            termination_contact_names.extend([s for s in body_names if name in s])

        feet_names = sorted(set(feet_names))
        penalized_contact_names = sorted(set(penalized_contact_names))
        termination_contact_names = sorted(set(termination_contact_names))
        self.foot_names = [name.lower().replace('_foot', '') for name in feet_names]

        feet_ids = [mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_BODY, n) for n in feet_names]
        penalized_ids = [mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_BODY, n) for n in penalized_contact_names]
        # Do not penalize designated feet contacts (equivalent to LeggedGym where feet are excluded from collision penalty).
        feet_id_set = set(feet_ids)
        penalized_ids = [pid for pid in penalized_ids if pid not in feet_id_set]

        self.feet_indices = torch.tensor(feet_ids, dtype=torch.long, device=self.device)
        self.feet_indices_np = np.array(feet_ids, dtype=np.int32)
        self.penalised_contact_indices = torch.tensor(penalized_ids, dtype=torch.long, device=self.device)
        self.termination_contact_indices = torch.tensor(
            [mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_BODY, n) for n in termination_contact_names], dtype=torch.long, device=self.device
        )

        if self.termination_contact_indices.numel() == 0:
            self.termination_contact_indices = torch.tensor([self.base_body_id_np], dtype=torch.long, device=self.device)

        self.dof_pos_limits = torch.tensor(base_model.jnt_range[self.joint_ids_np], dtype=torch.float, device=self.device)
        self.torque_limits = torch.tensor(base_model.jnt_actfrcrange[self.joint_ids_np, 1], dtype=torch.float, device=self.device)
        # MuJoCo MJCF does not expose per-joint velocity limits in this model; use a conservative default.
        self.dof_vel_limits = torch.full((self.num_dof,), 100.0, dtype=torch.float, device=self.device)
        self.hip_names = [name.lower().replace('_joint', '') for name in self.dof_names if 'hip_joint' in name]

        self.base_body_mass = float(base_model.body_mass[self.base_body_id_np])
        self._flat_world_plane_only = self._detect_flat_world_plane_only(base_model)

        if self.cfg.domain_rand.randomize_friction:
            fr = np.random.uniform(self.cfg.domain_rand.friction_range[0], self.cfg.domain_rand.friction_range[1])
            self.model.geom_friction[:, 0] = fr

        if self.cfg.domain_rand.randomize_base_mass:
            delta = np.random.uniform(self.cfg.domain_rand.added_mass_range[0], self.cfg.domain_rand.added_mass_range[1])
            self.model.body_mass[self.base_body_id_np] = max(0.1, self.base_body_mass + delta)

        self._get_env_origins()
        base_init_state_list = self.cfg.init_state.pos + [self.cfg.init_state.rot[3], self.cfg.init_state.rot[0], self.cfg.init_state.rot[1], self.cfg.init_state.rot[2]] + self.cfg.init_state.lin_vel + self.cfg.init_state.ang_vel
        self.base_init_state = torch.tensor(base_init_state_list, device=self.device, dtype=torch.float)

        for _ in range(self.num_envs):
            self.datas.append(mujoco.MjData(self.model))

    def _detect_flat_world_plane_only(self, model):
        world_geom_ids = np.where(model.geom_bodyid == self.world_body_id_np)[0]
        if len(world_geom_ids) == 0:
            return False
        world_geom_types = model.geom_type[world_geom_ids]
        if not np.all(world_geom_types == mujoco.mjtGeom.mjGEOM_PLANE):
            return False
        world_geom_quats = model.geom_quat[world_geom_ids]
        identity_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        return bool(np.allclose(world_geom_quats, identity_quat, atol=1e-8))

    def _get_env_origins(self):
        if self.terrain is not None:
            self.custom_origins = True
            self.env_origins = torch.zeros(self.cfg.env.num_envs, 3, device=self.device)
            max_init_level = min(self.cfg.terrain.max_init_terrain_level, self.cfg.terrain.num_rows - 1)
            if not self.cfg.terrain.curriculum:
                max_init_level = self.cfg.terrain.num_rows - 1
            self.terrain_levels = torch.randint(0, max_init_level + 1, (self.num_envs,), device=self.device)
            envs_per_col = max(1, math.ceil(self.num_envs / self.cfg.terrain.num_cols))
            self.terrain_types = torch.div(
                torch.arange(self.num_envs, device=self.device),
                envs_per_col,
                rounding_mode='floor',
            ).clamp(max=self.cfg.terrain.num_cols - 1).to(torch.long)
            self.max_terrain_level = self.cfg.terrain.num_rows
            self.terrain_origins = torch.from_numpy(self.terrain.env_origins).to(self.device).to(torch.float)
            self.env_origins[:] = self.terrain_origins[self.terrain_levels, self.terrain_types]
            return

        self.custom_origins = False
        self.env_origins = torch.zeros(self.cfg.env.num_envs, 3, device=self.device)
        num_cols = np.floor(np.sqrt(self.cfg.env.num_envs))
        num_rows = np.ceil(self.cfg.env.num_envs / num_cols)
        xx, yy = torch.meshgrid(torch.arange(int(num_rows), device=self.device), torch.arange(int(num_cols), device=self.device), indexing='ij')
        spacing = self.cfg.env.env_spacing
        self.env_origins[:, 0] = spacing * xx.flatten()[: self.cfg.env.num_envs]
        self.env_origins[:, 1] = spacing * yy.flatten()[: self.cfg.env.num_envs]
        self.env_origins[:, 2] = 0.0
        self.terrain_levels = torch.zeros(self.cfg.env.num_envs, dtype=torch.long, device=self.device)

    def _parse_cfg(self, cfg):
        sim_dt = cfg.sim.dt if self.sim_params is None else self.sim_params.dt
        self.dt = self.cfg.control.decimation * sim_dt
        soft_dof_pos_limit = float(self.cfg.rewards.soft_dof_pos_limit)
        if not 0.0 <= soft_dof_pos_limit <= 1.0:
            raise ValueError("soft_dof_pos_limit must be in [0, 1]")
        for name in ("lin_vel_slew_rate", "ang_vel_yaw_slew_rate"):
            value = float(getattr(self.cfg.commands, name))
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        self.obs_scales = self.cfg.normalization.obs_scales
        self.reward_scales = class_to_dict(self.cfg.rewards.scales)
        self.command_ranges = class_to_dict(self.cfg.commands.ranges)

        if self.cfg.terrain.mesh_type not in ['heightfield', 'trimesh']:
            self.cfg.terrain.curriculum = False

        self.max_episode_length_s = self.cfg.env.episode_length_s
        self.max_episode_length = int(np.ceil(self.max_episode_length_s / self.dt))
        self.cfg.domain_rand.push_interval = int(np.ceil(self.cfg.domain_rand.push_interval_s / self.dt))

    def _sync_dt_with_model(self):
        # Use MuJoCo model timestep as source of truth to avoid cfg/xml mismatch.
        if hasattr(self, 'model'):
            sim_dt = float(self.model.opt.timestep)
            self.dt = self.cfg.control.decimation * sim_dt
            self.max_episode_length = int(np.ceil(self.max_episode_length_s / self.dt))
            self.cfg.domain_rand.push_interval = int(np.ceil(self.cfg.domain_rand.push_interval_s / self.dt))

    def _init_height_points(self):
        y = torch.tensor(self.cfg.terrain.measured_points_y, device=self.device)
        x = torch.tensor(self.cfg.terrain.measured_points_x, device=self.device)
        grid_x, grid_y = torch.meshgrid(x, y, indexing='ij')

        self.num_height_points = grid_x.numel()
        points = torch.zeros(self.num_envs, self.num_height_points, 3, device=self.device)
        points[:, :, 0] = grid_x.flatten()
        points[:, :, 1] = grid_y.flatten()
        return points

    def _get_heights(self, env_ids=None):
        if env_ids is None:
            env_ids_tensor = torch.arange(self.num_envs, device=self.device)
        else:
            env_ids_tensor = env_ids if isinstance(env_ids, torch.Tensor) else torch.as_tensor(env_ids, device=self.device, dtype=torch.long)

        num_height_points = getattr(self, 'num_height_points', 1)
        if num_height_points == 0:
            return torch.zeros(len(env_ids_tensor), 0, dtype=torch.float, device=self.device)

        if self._flat_world_plane_only:
            return torch.zeros(len(env_ids_tensor), num_height_points, dtype=torch.float, device=self.device)

        base_quat = self.base_quat[env_ids_tensor]
        height_points = self.height_points[env_ids_tensor]
        world_points = quat_apply_yaw(base_quat.repeat_interleave(self.num_height_points, dim=0), height_points.reshape(-1, 3))
        world_points = world_points.reshape(len(env_ids_tensor), self.num_height_points, 3)
        world_points += self.root_states[env_ids_tensor, :3].unsqueeze(1)

        heights = torch.zeros(len(env_ids_tensor), self.num_height_points, dtype=torch.float, device=self.device)
        geomid = np.zeros(1, dtype=np.int32)

        for out_i, env_id in enumerate(env_ids_tensor.tolist()):
            data = self.datas[env_id]
            points_np = world_points[out_i].cpu().numpy()
            ray_origin_z = float(max(self.root_states[env_id, 2].item() + self._ray_height_margin, 5.0))
            for point_i in range(self.num_height_points):
                ray_origin = np.array([points_np[point_i, 0], points_np[point_i, 1], ray_origin_z], dtype=np.float64)
                dist = mujoco.mj_ray(
                    self.model,
                    data,
                    ray_origin,
                    self._ray_down_vec,
                    self._ray_geomgroup,
                    1,
                    self.base_body_id_np,
                    geomid,
                )
                if dist >= 0.0:
                    heights[out_i, point_i] = ray_origin_z - float(dist)

        return heights

    def _reward_lin_vel_z(self):
        return torch.square(self.base_lin_vel[:, 2])

    def _reward_ang_vel_xy(self):
        return torch.sum(torch.square(self.base_ang_vel[:, :2]), dim=1)

    def _reward_orientation(self):
        return torch.sum(torch.square(self.projected_gravity[:, :2]), dim=1)

    def _reward_base_height(self):
        base_height = torch.mean(self.root_states[:, 2].unsqueeze(1) - self.measured_heights, dim=1)
        return torch.square(base_height - self.cfg.rewards.base_height_target)

    def _reward_torques(self):
        return torch.sum(torch.square(self.torques), dim=1)

    def _reward_dof_vel(self):
        return torch.sum(torch.square(self.dof_vel), dim=1)

    def _reward_dof_acc(self):
        return torch.sum(torch.square((self.last_dof_vel - self.dof_vel) / self.dt), dim=1)

    def _reward_action_rate(self):
        return torch.sum(torch.square(self.last_actions - self.actions), dim=1)

    def _reward_action_magnitude(self):
        """Penalize sustained large policy outputs, including constant saturation."""
        return torch.sum(torch.square(self.actions), dim=1)

    def _reward_collision(self):
        if self.penalised_contact_indices.numel() == 0:
            return torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        return torch.sum(
            (torch.norm(self.contact_forces[:, self.penalised_contact_indices, :], dim=-1) > 0.1).float(),
            dim=1,
        )

    def _reward_termination(self):
        return self.reset_buf * (~self.time_out_buf)

    def _reward_dof_pos_limits(self):
        lower = self.dof_pos_limits[:, 0]
        upper = self.dof_pos_limits[:, 1]
        midpoint = 0.5 * (lower + upper)
        soft_half_range = (
            0.5
            * (upper - lower)
            * float(self.cfg.rewards.soft_dof_pos_limit)
        )
        soft_lower = midpoint - soft_half_range
        soft_upper = midpoint + soft_half_range
        below = (soft_lower - self.dof_pos).clip(min=0.0)
        above = (self.dof_pos - soft_upper).clip(min=0.0)
        return torch.sum(below + above, dim=1)

    def _reward_dof_vel_limits(self):
        return torch.sum(
            (torch.abs(self.dof_vel) - self.dof_vel_limits * self.cfg.rewards.soft_dof_vel_limit).clip(min=0.0, max=1.0),
            dim=1,
        )

    def _reward_torque_limits(self):
        return torch.sum(
            (torch.abs(self.torques) - self.torque_limits * self.cfg.rewards.soft_torque_limit).clip(min=0.0),
            dim=1,
        )

    def _reward_tracking_lin_vel(self):
        lin_vel_error = torch.sum(torch.square(self.commands[:, :2] - self.base_lin_vel[:, :2]), dim=1)
        return torch.exp(-lin_vel_error / self.cfg.rewards.tracking_sigma)

    def _reward_tracking_ang_vel(self):
        ang_vel_error = torch.square(self.commands[:, 2] - self.base_ang_vel[:, 2])
        return torch.exp(-ang_vel_error / self.cfg.rewards.tracking_sigma)

    def _reward_feet_air_time(self):
        contact = self.foot_ground_contact
        contact_filt = torch.logical_or(contact, self.last_contacts)
        self.last_contacts = contact
        first_contact = (self.feet_air_time > 0.0) * contact_filt
        self.feet_air_time += self.dt
        rew_air_time = torch.sum((self.feet_air_time - 0.5) * first_contact, dim=1)
        rew_air_time *= torch.norm(self.commands[:, :2], dim=1) > 0.1
        self.feet_air_time *= ~contact_filt
        return rew_air_time

    def _reward_stumble(self):
        return torch.any(
            torch.norm(self.contact_forces[:, self.feet_indices, :2], dim=2)
            > 5 * torch.abs(self.contact_forces[:, self.feet_indices, 2]),
            dim=1,
        )

    def _reward_stand_still(self):
        return torch.sum(torch.abs(self.dof_pos - self.default_dof_pos), dim=1) * (torch.norm(self.commands[:, :2], dim=1) < 0.1)

    def _reward_hip_pos(self):
        if self.hip_indices.numel() == 0:
            return torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        hip_pos = self.dof_pos[:, self.hip_indices]
        hip_default = self.default_dof_pos[:, self.hip_indices]
        return torch.sum(torch.square(hip_pos - hip_default), dim=1)

    def _update_episode_diagnostics(self):
        if self.feet_indices.numel() > 0:
            self.episode_foot_contact_sums += self.foot_ground_contact.float()
        if self.hip_indices.numel() > 0:
            hip_pos = self.dof_pos[:, self.hip_indices]
            hip_default = self.default_dof_pos[:, self.hip_indices]
            self.episode_hip_abs_sums += torch.abs(hip_pos - hip_default)

    def _post_physics_step_tasks(self):
        return None

    def _update_task_termination(self):
        return None

    def _update_task_episode_extras(self, env_ids):
        return None

    def _reset_task_buffers(self, env_ids):
        return None

    def _reward_feet_contact_forces(self):
        return torch.sum(
            (torch.norm(self.contact_forces[:, self.feet_indices, :], dim=-1) - self.cfg.rewards.max_contact_force).clip(min=0.0),
            dim=1,
        )

    def reset(self, env_ids=None):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        self.reset_idx(env_ids)
        self.compute_observations()
        return self.obs_buf, self.privileged_obs_buf

    def get_observations(self):
        return self.obs_buf

    def get_privileged_observations(self):
        return self.privileged_obs_buf

    def _refresh_sim_tensors(self):
        self.root_states = torch.zeros(self.num_envs, 13, dtype=torch.float, device=self.device)
        self.dof_pos = torch.zeros(self.num_envs, self.num_dof, dtype=torch.float, device=self.device)
        self.dof_vel = torch.zeros(self.num_envs, self.num_dof, dtype=torch.float, device=self.device)
        self.contact_forces = torch.zeros(self.num_envs, self.num_bodies, 3, dtype=torch.float, device=self.device)
        self.feet_pos_world = torch.zeros(self.num_envs, len(self.feet_indices), 3, dtype=torch.float, device=self.device)
        self.body_ground_contact = torch.zeros(self.num_envs, self.num_bodies, dtype=torch.bool, device=self.device)
        self.foot_ground_contact = torch.zeros(self.num_envs, len(self.feet_indices), dtype=torch.bool, device=self.device)
        cf_buf = np.zeros(6, dtype=np.float64)

        for i in range(self.num_envs):
            d = self.datas[i]
            qpos = d.qpos
            qvel = d.qvel

            self.root_states[i, :3] = torch.tensor(qpos[self.root_qpos_adr_np : self.root_qpos_adr_np + 3], dtype=torch.float)
            self.root_states[i, 3:7] = torch.tensor(qpos[self.root_qpos_adr_np + 3 : self.root_qpos_adr_np + 7], dtype=torch.float)
            self.root_states[i, 7:13] = torch.tensor(qvel[self.root_qvel_adr_np : self.root_qvel_adr_np + 6], dtype=torch.float)

            self.dof_pos[i] = torch.tensor(qpos[self.qpos_adr_np], dtype=torch.float)
            self.dof_vel[i] = torch.tensor(qvel[self.qvel_adr_np], dtype=torch.float)
            self.feet_pos_world[i] = torch.tensor(d.xpos[self.feet_indices_np], dtype=torch.float, device=self.device)

            for c in range(d.ncon):
                con = d.contact[c]
                b1 = self.model.geom_bodyid[con.geom1]
                b2 = self.model.geom_bodyid[con.geom2]
                mujoco.mj_contactForce(self.model, d, c, cf_buf)
                frame = con.frame.reshape(3, 3)
                force_world = frame @ cf_buf[:3]
                f_world = torch.tensor(force_world, dtype=torch.float, device=self.device)
                self.contact_forces[i, b1] -= f_world
                self.contact_forces[i, b2] += f_world
                if b1 == self.world_body_id_np and b2 != self.world_body_id_np:
                    self.body_ground_contact[i, b2] = True
                elif b2 == self.world_body_id_np and b1 != self.world_body_id_np:
                    self.body_ground_contact[i, b1] = True

            if len(self.feet_indices_np) > 0:
                self.foot_ground_contact[i] = self.body_ground_contact[i, self.feet_indices]
