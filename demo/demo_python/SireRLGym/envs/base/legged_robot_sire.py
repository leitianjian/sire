"""
LeggedRobotSire — Sire-backed version of LeggedRobot.

Inherits all RL logic (rewards, observations, buffers, curriculum, terrain)
from LeggedRobot.  Only overrides the physics-engine-specific methods.

Usage:
    from SireRLGym.envs.base.legged_robot_sire import LeggedRobotSire
    env = LeggedRobotSire(cfg, headless=True)
    env._create_sire_envs()                    # build Sire sim instances
    obs, priv, rew, reset, extras = env.step(actions)  # uses Sire backend
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import torch
import sire
import math

from rsl_rl.env import VecEnv

from SireRLGym import ROOT_DIR
from SireRLGym.envs.base.legged_robot_config import LeggedRobotCfg
from SireRLGym.utils.helpers import class_to_dict
from SireRLGym.utils.math import (
    quat_apply,
    quat_apply_yaw,
    quat_mul,
    quat_rotate_inverse,
    torch_rand_float,
    wrap_to_pi,
)
from SireRLGym.utils.terrain import TerrainLayout

from SireRLGym.utils.height_query import bilinear_height


# ═══════════════════════════════════════════════════════════════════
#  Helper: count elements in a Sire PointerArray (no .size() / len())
# ═══════════════════════════════════════════════════════════════════
def _pointer_array_len(pa) -> int:
    """Return number of elements in a Sire PointerArray by probing."""
    n = 0
    while True:
        try:
            if pa[n] is None:
                break
            n += 1
        except Exception:
            break
    return n


class LeggedRobotSire(VecEnv):
    """
    VecEnv (向量化环境），用于在物理引擎中同时、并行地模拟多个足式机器人。
    它不仅负责推进物理仿真（physics engine），还负责计算强化学习所需的核心信号：
    - 奖励 (rewards)：告诉智能体当前表现有多好；
    - 观测 (observations)：智能体用来做决策的自身及环境状态输入。
    此外，它还处理领域随机化（Domain Randomizations，比如随机推力）和地形难度的课程学习。
    通过并行提供大批量的状态、接收大批量的动作，大大加速了 RL 策略的训练。
    """

    def __init__(
        self,
        cfg: LeggedRobotCfg,
        sim_params=None,
        physics_engine=None,
        sim_device="cpu",
        headless=True,
    ):
        self.cfg = cfg
        self.sim_params = sim_params
        self.physics_engine = physics_engine
        self.device = torch.device("cpu")
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

    # ------------------------------------------------------------------
    #  Main simulation step
    # ------------------------------------------------------------------
    def step(self, actions):
        """
        Sire version — per-environment time-driven loop.

        MuJoCo (all envs same clock):
            for _ in range(decimation):
                torques = _compute_torques(actions)     # batch [E, A]
                for i in range(num_envs):
                    set ctrl[i] = torques[i]
                    mj_step(model, data[i])             # fixed sim_dt

        Sire (each env has its own clock):
            for i in range(num_envs):
                while not headerIsCtrl():
                    _update_actuator_torque(i)  # re-PD from current mp/mv
                    handleContact()
                _update_actuator_torque(i)      # ctrl event
                handleContact()

        Key fix: torques are recomputed EVERY event (step AND ctrl),
        just like MuJoCo recomputes torques every decimation substep
        and dog.py recomputes tau every simulator.integrate() call.
        """
        clip_actions = self.cfg.normalization.clip_actions
        self.actions = torch.clip(actions, -clip_actions, clip_actions).to(self.device)
        self._sire_physics_failures = getattr(self, '_sire_physics_failures', [])
        self._sire_physics_failures.clear()

        for i in range(self.num_envs):
            sl = self.sire_sim_loops[i]

            try:
                # Process all non-ctrl (step) events.
                # Recompute PD torque from current Sire joint state
                # before EVERY handleContact, matching MuJoCo/dog.py.
                while not sl.headerIsCtrl():
                    self._update_actuator_torque(i)
                    t0 = sl.simTime()
                    sl.handleContact()
                    self._sire_dt_actual[i] = sl.simTime() - t0

                # Process the ctrl event — also recompute torque first.
                self._update_actuator_torque(i)
                t0 = sl.simTime()
                sl.handleContact()
                self._sire_dt_actual[i] = sl.simTime() - t0
            except Exception as e:
                self._sire_physics_failures.append(i)
                print(f"[Sire physics reset] env {i}: {type(e).__name__}: {e}", flush=True)
                self._reinit_sire_env(i)

        # ---- post-step (once per control interval) ----
        self._post_physics_step_sire()
        clip_obs = self.cfg.normalization.clip_observations
        self.obs_buf = torch.clip(self.obs_buf, -clip_obs, clip_obs)
        if self.privileged_obs_buf is not None:
            self.privileged_obs_buf = torch.clip(
                self.privileged_obs_buf,
                -clip_obs,
                clip_obs,
            )
        return (
            self.obs_buf,
            self.privileged_obs_buf,
            self.rew_buf,
            self.reset_buf,
            self.extras,
        )

    def post_physics_step(self):
        """
        在每个物理仿真步骤之后被调用，处理强化学习训练所必须的“内务工作”：
        更新观测/状态缓冲区、计算智能体的奖励（rewards）、判断当前回合是否失败/结束（termination）、
        施加随机物理扰动（pushes），以及为策略重新分配运动指令（如随机生成下一步的速度命令）。
        """
        self._post_physics_step_sire()

    def check_termination(self):
        base_contacts = (
            torch.norm(
                self.contact_forces[:, self.termination_contact_indices, :], dim=-1
            )
            > 1.0
        )
        self.reset_buf = torch.any(base_contacts, dim=1)
        self.time_out_buf = self.episode_length_buf > self.max_episode_length
        self.reset_buf |= self.time_out_buf
        self._update_task_termination()

    def reset_idx(self, env_ids):
        if len(env_ids) == 0:
            return

        if self.cfg.terrain.curriculum:
            self._update_terrain_curriculum(env_ids)
        if self.cfg.commands.curriculum and (
            self.common_step_counter % int(self.max_episode_length) == 0
        ):
            self.update_command_curriculum(env_ids)
        # for i in env_ids.tolist():
        #     self.sire_simulators[i].reset()
        self._reset_dofs(env_ids)
        self._reset_root_states(env_ids)

        # ---- settling phase: land robot with zero actions before training ----
        # Without this, random PD torques launch the base upward before feet
        # ever touch ground, causing immediate termination and zero learning.
        # self._settle_sire_envs(env_ids)

        self._resample_commands(env_ids)

        ep_len = self.episode_length_buf[env_ids].float().clamp(min=1.0)
        ep_duration = (ep_len * self.dt).clamp(min=self.dt)

        self.last_actions[env_ids] = 0.0
        self.last_dof_vel[env_ids] = 0.0
        self.feet_air_time[env_ids] = 0.0
        self.episode_length_buf[env_ids] = 0
        self.reset_buf[env_ids] = 1

        self.extras["episode"] = {}
        self.extras["episode"]["episode_count"] = float(len(env_ids))
        for key in self.episode_sums.keys():
            # Log reward terms as average contribution per second over the actual episode duration.
            self.extras["episode"]["rew_" + key] = torch.mean(
                self.episode_sums[key][env_ids] / ep_duration
            )
            self.episode_sums[key][env_ids] = 0.0
        if len(env_ids) > 0:
            for foot_i, foot_name in enumerate(self.foot_names):
                ratio = self.episode_foot_contact_sums[env_ids, foot_i] / ep_len
                self.extras["episode"][f"contact_ratio_{foot_name}"] = torch.mean(ratio)
                self.episode_foot_contact_sums[env_ids, foot_i] = 0.0
            for hip_i, hip_name in enumerate(self.hip_names):
                offset = self.episode_hip_abs_sums[env_ids, hip_i] / ep_len
                self.extras["episode"][f"hip_offset_{hip_name}"] = torch.mean(offset)
                self.episode_hip_abs_sums[env_ids, hip_i] = 0.0

        if self.cfg.terrain.curriculum:
            self.extras["episode"]["terrain_level"] = torch.mean(
                self.terrain_levels.float()
            )
        if self.cfg.commands.curriculum:
            self.extras["episode"]["max_command_x"] = self.command_ranges["lin_vel_x"][
                1
            ]
        if self.cfg.env.send_timeouts:
            self.extras["time_outs"] = self.time_out_buf
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

        if "termination" in self.reward_scales:
            rew = self._reward_termination() * self.reward_scales["termination"]
            self.rew_buf += rew
            self.episode_sums["termination"] += rew

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
            heights = (
                torch.clip(
                    self.root_states[:, 2].unsqueeze(1) - 0.5 - self.measured_heights,
                    -1,
                    1.0,
                )
                * self.obs_scales.height_measurements
            )
            full_obs = torch.cat((full_obs, heights), dim=-1)

        # Support both legacy 235-dim privileged observations and newer 263-dim
        # checkpoints by appending critic-only extras when requested.
        if (
            self.num_privileged_obs is not None
            and self.num_privileged_obs > full_obs.shape[1]
        ):
            privileged_extras = torch.cat(
                (
                    torch.norm(self.contact_forces[:, self.feet_indices, :], dim=-1)
                    * 1e-3,
                    self.torques / torch.clamp(self.torque_limits, min=1e-6),
                    (self.last_dof_vel - self.dof_vel) / max(self.dt, 1e-6) * 1e-4,
                ),
                dim=-1,
            )
            full_obs = torch.cat((full_obs, privileged_extras), dim=-1)

        self.obs_buf = actor_obs[:, : self.num_obs]

        if self.add_noise:
            self.obs_buf += (
                2 * torch.rand_like(self.obs_buf) - 1
            ) * self.noise_scale_vec

        if self.privileged_obs_buf is not None:
            n = min(full_obs.shape[1], self.num_privileged_obs)
            self.privileged_obs_buf[:, :n] = full_obs[:, :n]

    # ------------------------------------------------------------------
    #  Simulation creation
    # ------------------------------------------------------------------
    def create_sim(self):
        self.up_axis_idx = 2
        self.terrain = None
        if self.cfg.terrain.mesh_type in ["heightfield", "trimesh"]:
            self.terrain = TerrainLayout(self.cfg.terrain, self.cfg.env.num_envs)
        self._create_sire_envs()

    def _post_physics_step_callback(self):
        env_ids = (
            (
                self.episode_length_buf
                % int(self.cfg.commands.resampling_time / self.dt)
                == 0
            )
            .nonzero(as_tuple=False)
            .flatten()
        )
        self._resample_commands(env_ids)

        if self.cfg.commands.heading_command:
            forward = quat_apply(self.base_quat, self.forward_vec)
            heading = torch.atan2(forward[:, 1], forward[:, 0])
            self.commands[:, 2] = torch.clip(
                0.5 * wrap_to_pi(self.commands[:, 3] - heading), -1.0, 1.0
            )

        if self.cfg.terrain.measure_heights:
            self.measured_heights = self._get_heights()
        if self.cfg.domain_rand.push_robots and (
            self.common_step_counter % int(self.cfg.domain_rand.push_interval) == 0
        ):
            self._push_robots()

    def _resample_commands(self, env_ids):
        if len(env_ids) == 0:
            return
        self.commands[env_ids, 0] = torch_rand_float(
            self.command_ranges["lin_vel_x"][0],
            self.command_ranges["lin_vel_x"][1],
            (len(env_ids), 1),
            device=self.device,
        ).squeeze(1)
        self.commands[env_ids, 1] = torch_rand_float(
            self.command_ranges["lin_vel_y"][0],
            self.command_ranges["lin_vel_y"][1],
            (len(env_ids), 1),
            device=self.device,
        ).squeeze(1)
        if self.cfg.commands.heading_command:
            self.commands[env_ids, 3] = torch_rand_float(
                self.command_ranges["heading"][0],
                self.command_ranges["heading"][1],
                (len(env_ids), 1),
                device=self.device,
            ).squeeze(1)
        else:
            self.commands[env_ids, 2] = torch_rand_float(
                self.command_ranges["ang_vel_yaw"][0],
                self.command_ranges["ang_vel_yaw"][1],
                (len(env_ids), 1),
                device=self.device,
            ).squeeze(1)

        self.commands[env_ids, :2] *= (
            torch.norm(self.commands[env_ids, :2], dim=1) > 0.2
        ).unsqueeze(1)

        prob_zero = float(getattr(self.cfg.commands, "prob_zero_command", 0.0))
        prob_neg = float(getattr(self.cfg.commands, "prob_negative_command", 0.0))
        if prob_zero > 0.0 or prob_neg > 0.0:
            draws = torch.rand(len(env_ids), device=self.device)
            zero_mask = draws < prob_zero
            neg_mask = (draws >= prob_zero) & (draws < prob_zero + prob_neg)
            if zero_mask.any():
                zero_ids = env_ids[zero_mask]
                self.commands[zero_ids, :2] = 0.0
                self.commands[zero_ids, 2] = 0.0
                if self.cfg.commands.heading_command:
                    self.commands[zero_ids, 3] = 0.0
            if neg_mask.any():
                neg_ids = env_ids[neg_mask]
                neg_range = getattr(
                    self.cfg.commands, "negative_lin_vel_x_range", [-0.3, -0.1]
                )
                count = int(neg_mask.sum().item())
                self.commands[neg_ids, 0] = torch_rand_float(
                    float(neg_range[0]),
                    float(neg_range[1]),
                    (count, 1),
                    device=self.device,
                ).squeeze(1)
                self.commands[neg_ids, 1] = 0.0
                if self.cfg.commands.heading_command:
                    self.commands[neg_ids, 3] = 0.0

    # ------------------------------------------------------------------
    #  Single-environment torque update (reads current Sire mp/mv)
    #  Called EVERY event (step + ctrl), matching MuJoCo/dog.py.
    # ------------------------------------------------------------------
    def _update_actuator_torque(self, env_idx: int):
        """Read current mp/mv from Sire model, compute PD torque, set desiredValue."""
        model = self.sire_models[env_idx]
        action_i = self.actions[env_idx]
        action_scaled = action_i * self.cfg.control.action_scale
        control_type = self.cfg.control.control_type
        default_pos = self.default_dof_pos.squeeze(0)  # [A]
        torque = torch.zeros(self.num_actions, device=self.device)
        for j, dof_name in enumerate(self.dof_names):
            mot_idx = self._sire_dof_to_motion[dof_name]
            mot = model.motionPool()[mot_idx]
            mp = float(mot.mp)
            mv = float(mot.mv)
            if control_type == "P":
                t = (self.p_gains[j].item() * (action_scaled[j].item() + default_pos[j].item() - mp)
                     - self.d_gains[j].item() * mv)
            elif control_type == "V":
                t = (self.p_gains[j].item() * (action_scaled[j].item() - mv)
                     - self.d_gains[j].item() * (mv - float(self.last_dof_vel[env_idx, j])) / max(self.dt, 1e-6))
            elif control_type == "T":
                t = action_scaled[j].item()
            else:
                t = 0.0
            t = max(float(-self.torque_limits[j]), min(float(self.torque_limits[j]), t))
            torque[j] = t
            if isinstance(mot, sire.ActuatorSISO):
                mot.desiredValue = t
        self.torques[env_idx] = torque

    # ------------------------------------------------------------------
    #  Reset helpers
    # ------------------------------------------------------------------
    def _reset_dofs(self, env_ids):
        """Sire version: write joint positions through MotionPool."""
        self.dof_pos[env_ids] = self.default_dof_pos * torch_rand_float(
            0.5,
            1.5,
            (len(env_ids), self.num_dof),
            device=self.device,
        )
        self.dof_vel[env_ids] = 0.0
        for eid in env_ids.tolist():
            m = self.sire_models[eid]
            for j, dof_name in enumerate(self.dof_names):
                mot_idx = self._sire_dof_to_motion[dof_name]
                m.motionPool()[mot_idx].mp = float(self.dof_pos[eid, j])
                m.motionPool()[mot_idx].mv = 0.0
            m.forwardKinematics()
            m.forwardKinematicsVel()

    def _reset_root_states(self, env_ids):
        """Sire version: write root pose through body pq."""
        # (identical randomisation logic as parent, only setter differs)
        if self.custom_origins:
            self.root_states[env_ids] = self.base_init_state
            self.root_states[env_ids, :3] += self.env_origins[env_ids]
            rx = torch_rand_float(
                *self.cfg.terrain.spawn_rand_x_range,
                (len(env_ids), 1),
                device=self.device,
            ).squeeze(1)
            ry = torch_rand_float(
                *self.cfg.terrain.spawn_rand_y_range,
                (len(env_ids), 1),
                device=self.device,
            ).squeeze(1)
            self.root_states[env_ids, 0] += rx
            self.root_states[env_ids, 1] += ry
        else:
            self.root_states[env_ids] = self.base_init_state
            self.root_states[env_ids, :3] += self.env_origins[env_ids]

        yl, yh = self.cfg.init_state.init_yaw_range
        if yl != 0.0 or yh != 0.0:
            yaw = torch_rand_float(
                yl,
                yh,
                (len(env_ids), 1),
                device=self.device,
            ).squeeze(1)
            hy = 0.5 * yaw
            # Aris scalar-last convention: [qx, qy, qz, qw] = [0, 0, sin(hy), cos(hy)]
            yq = torch.stack(
                (
                    torch.zeros_like(hy),
                    torch.zeros_like(hy),
                    torch.sin(hy),
                    torch.cos(hy),
                ),
                dim=1,
            )
            self.root_states[env_ids, 3:7] = quat_mul(
                yq,
                self.root_states[env_ids, 3:7],
            )

        self.root_states[env_ids, 7:13] = torch_rand_float(
            -0.5,
            0.5,
            (len(env_ids), 6),
            device=self.device,
        )

        for eid in env_ids.tolist():
            m = self.sire_models[eid]
            r = self.root_states[eid]
            # root_states uses Aris convention: [x,y,z, qx,qy,qz,qw, vx,vy,vz,wx,wy,wz]
            m.partPool()[1].pq = r[:7].cpu().numpy()
            m.partPool()[1].vs = r[7:13].cpu().numpy()
            m.forwardKinematics()
            m.forwardKinematicsVel()

    def _settle_sire_envs(self, env_ids, settle_steps=50):
        """Run physics for `settle_steps` control intervals with zero actions.

        After reset, the robot is spawned above ground and random PD torques
        can launch the base upward before feet ever touch ground.  Zero-action
        settling lets gravity bring the robot to a stable standing posture,
        after which training data collection (with random/policy actions)
        produces meaningful rewards.
        """
        if len(env_ids) == 0 or settle_steps <= 0:
            return

        # Temporarily zero actions for the settling envs.
        saved = self.actions[env_ids].clone()
        self.actions[env_ids] = 0.0

        for _ in range(settle_steps):
            for eid in env_ids.tolist():
                sl = self.sire_sim_loops[eid]
                try:
                    while not sl.headerIsCtrl():
                        self._update_actuator_torque(eid)
                        sl.handleContact()
                    self._update_actuator_torque(eid)
                    sl.handleContact()
                    # print(sl.simTime())
                except Exception as e:
                    print(f"[Sire settle] env {eid}: {e}", flush=True)
                    self._reinit_sire_env(eid)

        self.actions[env_ids] = saved

        # Sync settled Sire model state → torch buffers so downstream code
        # (observations, rewards) sees the post-landing configuration.
        self._refresh_sim_tensors_sire()

    def _push_robots(self):
        max_vel = self.cfg.domain_rand.max_push_vel_xy
        self.root_states[:, 7:9] = torch_rand_float(
            -max_vel, max_vel, (self.num_envs, 2), device=self.device
        )
        for i in range(self.num_envs):
            vs = self.sire_models[i].partPool()[1].vs
            vs[:2] = self.root_states[i, 7:9].cpu().numpy()
            self.sire_models[i].partPool()[1].vs = vs

    def _update_terrain_curriculum(self, env_ids):
        if (
            not self.init_done
            or not self.custom_origins
            or not hasattr(self, "terrain_origins")
        ):
            return
        distance = torch.norm(
            self.root_states[env_ids, :2] - self.env_origins[env_ids, :2], dim=1
        )
        move_up = distance > (self.terrain.patch_length * 0.5)
        move_down = (
            distance
            < torch.norm(self.commands[env_ids, :2], dim=1)
            * self.max_episode_length_s
            * 0.5
        )
        move_down &= ~move_up
        self.terrain_levels[env_ids] += move_up.long() - move_down.long()
        self.terrain_levels[env_ids] = torch.where(
            self.terrain_levels[env_ids] >= self.max_terrain_level,
            torch.randint_like(self.terrain_levels[env_ids], self.max_terrain_level),
            torch.clamp(self.terrain_levels[env_ids], min=0),
        )
        self.env_origins[env_ids] = self.terrain_origins[
            self.terrain_levels[env_ids], self.terrain_types[env_ids]
        ]

    def update_command_curriculum(self, env_ids):
        if "tracking_lin_vel" not in self.episode_sums:
            return
        if (
            torch.mean(self.episode_sums["tracking_lin_vel"][env_ids])
            / self.max_episode_length
            > 0.8 * self.reward_scales["tracking_lin_vel"]
        ):
            self.command_ranges["lin_vel_x"][0] = np.clip(
                self.command_ranges["lin_vel_x"][0] - 0.5,
                -self.cfg.commands.max_curriculum,
                0.0,
            )
            self.command_ranges["lin_vel_x"][1] = np.clip(
                self.command_ranges["lin_vel_x"][1] + 0.5,
                0.0,
                self.cfg.commands.max_curriculum,
            )

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
                noise_vec[start : min(end, self.num_obs)] = value
        return noise_vec

    def _init_buffers(self):
        self._refresh_sim_tensors_sire()

        self.common_step_counter = 0
        self.extras = {}
        self.noise_scale_vec = self._get_noise_scale_vec(self.cfg)
        self.gravity_vec = torch.tensor([0.0, 0.0, -1.0], device=self.device).repeat(
            (self.num_envs, 1)
        )
        self.forward_vec = torch.tensor([1.0, 0.0, 0.0], device=self.device).repeat(
            (self.num_envs, 1)
        )
        self.base_quat = self.root_states[:, 3:7]

        self.torques = torch.zeros(
            self.num_envs, self.num_actions, dtype=torch.float, device=self.device
        )
        self.p_gains = torch.zeros(
            self.num_actions, dtype=torch.float, device=self.device
        )
        self.d_gains = torch.zeros(
            self.num_actions, dtype=torch.float, device=self.device
        )
        self.actions = torch.zeros(
            self.num_envs, self.num_actions, dtype=torch.float, device=self.device
        )
        self.last_actions = torch.zeros(
            self.num_envs, self.num_actions, dtype=torch.float, device=self.device
        )
        self.last_dof_vel = torch.zeros_like(self.dof_vel)
        self.last_root_vel = torch.zeros_like(self.root_states[:, 7:13])

        self.commands = torch.zeros(
            self.num_envs,
            self.cfg.commands.num_commands,
            dtype=torch.float,
            device=self.device,
        )
        self.commands_scale = torch.tensor(
            [self.obs_scales.lin_vel, self.obs_scales.lin_vel, self.obs_scales.ang_vel],
            device=self.device,
        )
        self.feet_air_time = torch.zeros(
            self.num_envs,
            self.feet_indices.shape[0],
            dtype=torch.float,
            device=self.device,
        )
        self.last_contacts = torch.zeros(
            self.num_envs, len(self.feet_indices), dtype=torch.bool, device=self.device
        )
        self.last_feet_pos_world = torch.zeros(
            self.num_envs,
            len(self.feet_indices),
            3,
            dtype=torch.float,
            device=self.device,
        )
        self.episode_foot_contact_sums = torch.zeros(
            self.num_envs, len(self.feet_indices), dtype=torch.float, device=self.device
        )

        # Quaternion convention: Aris scalar-last [qx,qy,qz,qw].
        # All math functions in SireRLGym.utils.math use this convention.
        # root_states[i, 3:7] = [qx, qy, qz, qw] matches Part.pq[3:].
        self.base_lin_vel = quat_rotate_inverse(
            self.base_quat, self.root_states[:, 7:10]
        )
        self.base_ang_vel = quat_rotate_inverse(
            self.base_quat, self.root_states[:, 10:13]
        )
        self.projected_gravity = quat_rotate_inverse(self.base_quat, self.gravity_vec)

        if self.cfg.terrain.measure_heights:
            self.height_points = self._init_height_points()
            self.measured_heights = self._get_heights()
        else:
            self.measured_heights = torch.zeros(self.num_envs, 1, device=self.device)

        self.default_dof_pos = torch.zeros(
            self.num_dof, dtype=torch.float, device=self.device
        )
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
            if not found and self.cfg.control.control_type in ["P", "V"]:
                self.p_gains[i] = 0.0
                self.d_gains[i] = 0.0
        self.default_dof_pos = self.default_dof_pos.unsqueeze(0)
        hip_ids = [i for i, name in enumerate(self.dof_names) if "hip_joint" in name]
        self.hip_indices = torch.tensor(hip_ids, dtype=torch.long, device=self.device)
        self.episode_hip_abs_sums = torch.zeros(
            self.num_envs, len(self.hip_indices), dtype=torch.float, device=self.device
        )

        self.obs_buf = torch.zeros(
            self.num_envs, self.num_obs, dtype=torch.float, device=self.device
        )
        self.privileged_obs_buf = (
            None
            if self.num_privileged_obs is None
            else torch.zeros(
                self.num_envs,
                max(self.num_obs, self.num_privileged_obs),
                dtype=torch.float,
                device=self.device,
            )
        )
        self.rew_buf = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.reset_buf = torch.zeros(
            self.num_envs, dtype=torch.long, device=self.device
        )
        self.time_out_buf = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self.episode_length_buf = torch.zeros(
            self.num_envs, dtype=torch.long, device=self.device
        )

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
            if name == "termination":
                continue
            self.reward_names.append(name)
            self.reward_functions.append(getattr(self, "_reward_" + name))

        self.episode_sums = {
            name: torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
            for name in self.reward_scales.keys()
        }

    def _reinit_sire_env(self, env_idx: int) -> None:
        """Re-create a single Sire environment after physics crash (like MuJoCo reset)."""
        sim = sire.Simulator()
        sire.fromXmlFile(sim, str(self.generated_model_path))
        model = sim.model()
        sloop = sim.simulationLoop()
        sloop.deltaT = float(self.cfg.sim.dt)
        sloop.ctrlT = float(self.cfg.sim.dt) * self.cfg.control.decimation
        sim.init()

        pe = sim.physicsEngine()
        gmap = {}
        gp = pe.geometryPool
        for idx in range(pe.numGeometries()):
            g = gp[idx]
            if g is not None:
                gmap[int(g.id)] = int(g.prtId)

        self.sire_simulators[env_idx] = sim
        self.sire_models[env_idx] = model
        self.sire_sim_loops[env_idx] = sloop
        self.sire_physics[env_idx] = pe
        self._sire_geom_to_part[env_idx] = gmap
        self._sire_dt_actual.pop(env_idx, None)

        # Reset env state tensors
        env_ids = torch.tensor([env_idx], device=self.device)
        self._reset_dofs(env_ids)
        self._reset_root_states(env_ids)
        self.episode_length_buf[env_idx] = 0
        self.reset_buf[env_idx] = 1

    # ------------------------------------------------------------------
    #  Sire environment construction
    # ------------------------------------------------------------------
    def _create_sire_envs(self):
        """
        Build one Sire Simulator per parallel environment.
        """

        # Use sirePaperDogRL model by default, fallback to resources/
        model_path = str(ROOT_DIR / "sirePaperDogRL" / "go2_rai_foot.xml")
        if not Path(model_path).exists():
            model_path = str(ROOT_DIR / "resources" / "go2SRCF.xml")
        if not Path(model_path).exists():
            raise FileNotFoundError(
                f"Sire model not found: {model_path}.  Run scripts/mjcf2srdf first."
            )

        self.sire_simulators = []
        self.sire_models = []
        self.sire_sim_loops = []
        self._sire_dt_actual = {}  # per-env actual dt (pybind11 objects can't store arbitrary attrs)
        # TODO [5]: store physics engine per env
        self.sire_physics = []
        # TODO [6]: build geometry→part mapping per env for ground contact
        self._sire_geom_to_part = []
        self._sire_part_name_to_idx = {}
        self.generated_model_path = model_path

        self.num_envs = self.cfg.env.num_envs
        self.num_obs = self.cfg.env.num_observations
        self.num_privileged_obs = self.cfg.env.num_privileged_obs
        self.num_actions = self.cfg.env.num_actions

        # TODO: Terrain的写入需要处理
        if self.terrain is not None:
            self.generated_model_path = self.terrain.write_scene(model_path)
        base_simulator = sire.Simulator()
        sire.fromXmlFile(base_simulator, str(self.generated_model_path))
        base_simulator.simulationLoop().deltaT = float(self.cfg.sim.dt)
        self.simulator = base_simulator
        # base_model.opt.timestep = float(self.cfg.sim.dt)
        for i in range(self.num_envs):
            sim = sire.Simulator()
            sire.fromXmlFile(sim, str(self.generated_model_path))
            model = sim.model()
            # name → part_id map (first env only)
            # TODO: 可能要修改
            if i == 0:
                for pid in range(model.nbody):
                    self._sire_part_name_to_idx[model.partPool()[pid].name] = pid

            # event-handler map for variable-step contact simulation
            sloop = sim.simulationLoop()
            sloop.deltaT = float(self.cfg.sim.dt)
            sloop.ctrlT = float(self.cfg.sim.dt) * self.cfg.control.decimation

            sim.init()

            # TODO [5][6]: populate per-env physics engine and geometry→part map
            pe = sim.physicsEngine()
            self.sire_physics.append(pe)
            self.sire_simulators.append(sim)
            self.sire_models.append(model)
            self.sire_sim_loops.append(sloop)

            # Build geometry_id → part_id map for ground contact detection
            gmap = {}
            gp = pe.geometryPool
            for idx in range(_pointer_array_len(gp)):
                g = gp[idx]
                if g is not None:
                    gmap[int(g.id)] = int(g.prtId)
            self._sire_geom_to_part.append(gmap)

        # --- post Sire model creation: build Sire-native mappings ----------
        self.num_dof = self.num_actions
        self.num_dofs = self.num_actions
        self.num_bodies = base_simulator.model().nbody

        self.dof_names = [k for k in self.cfg.init_state.default_joint_angles.keys()]
        self.actuator_names = [n.replace("_joint", "") for n in self.dof_names]

        # ── Motion ↔ Joint name mapping (Sire motion names are "actuator_N") ─
        # JointPool and MotionPool share the same index order for Go2.
        # Build dof_name → motion_idx dict for robust access.
        m0 = self.sire_models[0]
        self._sire_dof_to_motion: dict[str, int] = {}
        for mot_idx in range(m0.numMotions()):
            joint = m0.jointPool()[mot_idx]
            if joint.name in self.dof_names:
                self._sire_dof_to_motion[joint.name] = mot_idx
        # Verify all dof_names are mapped
        for dof_name in self.dof_names:
            if dof_name not in self._sire_dof_to_motion:
                raise RuntimeError(
                    f"Joint '{dof_name}' not found in Sire MotionPool. "
                    f"Available motions: {[m0.motion(i).name for i in range(m0.numMotions())]}")


        # ── Sire body / part name lookups (replaces mj_name2id for bodies) ─
        # partPool()[0] = ground (id=0, same as MuJoCo world_body_id=0)
        m0 = self.sire_models[0]
        sire_part_names = []
        for pid in range(m0.nbody):
            sire_part_names.append(m0.partPool()[pid].name)

        # feet / penalized / termination contacts — identified by Sire part names
        self.foot_names_mujoco = []  # MuJoCo-style foot names (e.g. "FL_foot")
        feet_names_raw = []
        for name in sire_part_names:
            if self.cfg.asset.foot_name in name:
                feet_names_raw.append(name)
        if len(feet_names_raw) == 0:
            # fallback: use calf as foot proxy
            for name in sire_part_names:
                if "calf" in name:
                    feet_names_raw.append(name)
        feet_names_raw = sorted(set(feet_names_raw))
        self.foot_names = [n.lower().replace("_calf", "") for n in feet_names_raw]

        penalized_names = []
        for name in self.cfg.asset.penalize_contacts_on:
            for sn in sire_part_names:
                if name in sn and sn not in feet_names_raw:
                    penalized_names.append(sn)
        termination_names = []
        for name in self.cfg.asset.terminate_after_contacts_on:
            for sn in sire_part_names:
                if name in sn:
                    termination_names.append(sn)

        # Sire part IDs (equivalent to MuJoCo body IDs)
        self.feet_indices = torch.tensor(
            [self._sire_part_name_to_idx[n] for n in feet_names_raw],
            dtype=torch.long, device=self.device,
        )
        self.feet_indices_np = np.array(
            [self._sire_part_name_to_idx[n] for n in feet_names_raw],
            dtype=np.int32,
        )
        penalized_ids = sorted(set(
            self._sire_part_name_to_idx[n] for n in penalized_names
        ))
        self.penalised_contact_indices = torch.tensor(
            penalized_ids, dtype=torch.long, device=self.device,
        )
        termination_ids = sorted(set(
            self._sire_part_name_to_idx[n] for n in termination_names
        ))
        self.termination_contact_indices = torch.tensor(
            termination_ids, dtype=torch.long, device=self.device,
        )
        if self.termination_contact_indices.numel() == 0:
            self.base_body_pid = self._sire_part_name_to_idx.get("base", 1)
            self.termination_contact_indices = torch.tensor(
                [self.base_body_pid], dtype=torch.long, device=self.device,
            )
        else:
            self.base_body_pid = self._sire_part_name_to_idx.get("base", 1)

        self.world_body_id_np = 0  # ground is always partPool()[0]



        # ── Joint & torque limits (Go2 model — hardcoded for Sire) ────────
        # Sire SRCF XML does not expose joint range/force limits natively yet.
        # These values are from the Go2 MuJoCo model and match the actual
        # Unitree Go2 mechanical specs.
        _GO2_JOINT_LIMITS = {
            # hip: ±1.0472 rad (60°), thigh: [-1.5708, 3.4907], calf: [-2.7227, 0.83776]
            "FL_hip_joint":   [-1.0472, 1.0472, 23.7],
            "FL_thigh_joint": [-1.5708, 3.4907, 23.7],
            "FL_calf_joint":  [-2.7227, 0.83776, 35.55],
            "FR_hip_joint":   [-1.0472, 1.0472, 23.7],
            "FR_thigh_joint": [-1.5708, 3.4907, 23.7],
            "FR_calf_joint":  [-2.7227, 0.83776, 35.55],
            "RL_hip_joint":   [-1.0472, 1.0472, 23.7],
            "RL_thigh_joint": [-0.5236, 4.5379, 23.7],
            "RL_calf_joint":  [-2.7227, 0.83776, 35.55],
            "RR_hip_joint":   [-1.0472, 1.0472, 23.7],
            "RR_thigh_joint": [-0.5236, 4.5379, 23.7],
            "RR_calf_joint":  [-2.7227, 0.83776, 35.55],
        }
        limits_list = []
        torque_list = []
        for name in self.dof_names:
            entry = _GO2_JOINT_LIMITS.get(name)
            if entry is not None:
                limits_list.append([entry[0], entry[1]])
                torque_list.append(entry[2])
            else:
                # Unknown joint — wide limits
                limits_list.append([-3.1416, 3.1416])
                torque_list.append(20.0)
        self.dof_pos_limits = torch.tensor(limits_list, dtype=torch.float, device=self.device)
        self.torque_limits = torch.tensor(torque_list, dtype=torch.float, device=self.device)
        self.base_body_mass = 6.921

        self.dof_vel_limits = torch.full(
            (self.num_dof,), 100.0, dtype=torch.float, device=self.device,
        )
        self.hip_names = [
            name.lower().replace("_joint", "")
            for name in self.dof_names
            if "hip_joint" in name
        ]

        # ── Domain randomization on MuJoCo model (TODO: port to Sire) ────────
        self._flat_world_plane_only = self._detect_flat_world_plane_only()

        self._get_env_origins()
        base_init_state_list = (
            self.cfg.init_state.pos
            + self.cfg.init_state.rot   # [qx, qy, qz, qw] in Aris convention
            + self.cfg.init_state.lin_vel
            + self.cfg.init_state.ang_vel
        )
        self.base_init_state = torch.tensor(
            base_init_state_list, device=self.device, dtype=torch.float,
        )
        # ── diagnostic: dump config mappings ──
        self._print_config_diagnostic(sire_part_names, feet_names_raw,
                                       penalized_names, termination_names)
        # No MuJoCo MjData — Sire manages state internally via partPool / motionPool


    def _print_config_diagnostic(self, sire_part_names, feet_names_raw,
                                  penalized_names, termination_names):
        """Print part pool name → ID mapping and key config indices."""
        print("=" * 70)
        print("Sire Part Pool (name -> ID):")
        for pid in range(self.num_bodies):
            pname = sire_part_names[pid]
            tags = []
            if pid in self.feet_indices.tolist():       tags.append("FOOT")
            if pid in self.penalised_contact_indices.tolist(): tags.append("PENALTY")
            if pid in self.termination_contact_indices.tolist(): tags.append("TERMINATE")
            tag_str = " [" + ", ".join(tags) + "]" if tags else ""
            print(f"  part[{pid:2d}] = {pname:20s}{tag_str}")

        print(f"\nfeet_indices                     = {self.feet_indices.tolist()}")
        print(f"penalised_contact_indices        = {self.penalised_contact_indices.tolist()}")
        print(f"termination_contact_indices      = {self.termination_contact_indices.tolist()}")
        print(f"base_body_pid                    = {self.base_body_pid}")
        print(f"world_body_id_np                 = {self.world_body_id_np}")
        print(f"feet_names_raw                   = {feet_names_raw}")
        print(f"feet_names                       = {self.foot_names}")
        print(f"penalized_names                  = {penalized_names}")
        print(f"termination_names                = {termination_names}")
        print(f"dof_names                        = {self.dof_names}")
        print(f"num_obs / num_priv_obs / num_act = {self.num_obs} / {self.num_privileged_obs} / {self.num_actions}")
        print(f"_sire_dof_to_motion              = {self._sire_dof_to_motion}")
        print("=" * 70, flush=True)

    def _detect_flat_world_plane_only(self) -> bool:
        """检测实际 Sire 物理场景中地面是否有高度变化。

        等价于 MuJoCo 版：遍历 physicsEngine.geometryPool()，
        检查是否存在 Mesh 或 HeightField 类型的几何体。
        """
        if not self.sire_physics:
            return True
        pe = self.sire_physics[0]
        gp = pe.geometryPool
        for i in range(_pointer_array_len(gp)):
            geo = gp[i]
            shape = getattr(geo, "shape", None)
            if shape is not None:
                shape_type = type(shape).__name__
                if "Mesh" in shape_type or "HeightField" in shape_type:
                    return False
        return True

    def _get_env_origins(self):
        """为每个并行环境分配世界坐标系下的初始位置（spawn point）。

        有 terrain 时：从 TerrainLayout 的地形网格中随机选择 (row, col)，
        读取对应的 env_origins（含 z=地面高度）。
        无 terrain 时：将环境排列在均匀网格上，间距 = env_spacing，z=0。
        """
        if self.terrain is not None:
            self.custom_origins = True
            self.env_origins = torch.zeros(self.cfg.env.num_envs, 3, device=self.device)
            max_init_level = min(
                self.cfg.terrain.max_init_terrain_level, self.cfg.terrain.num_rows - 1
            )
            if not self.cfg.terrain.curriculum:
                max_init_level = self.cfg.terrain.num_rows - 1
            self.terrain_levels = torch.randint(
                0, max_init_level + 1, (self.num_envs,), device=self.device
            )
            envs_per_col = max(1, math.ceil(self.num_envs / self.cfg.terrain.num_cols))
            self.terrain_types = (
                torch.div(
                    torch.arange(self.num_envs, device=self.device),
                    envs_per_col,
                    rounding_mode="floor",
                )
                .clamp(max=self.cfg.terrain.num_cols - 1)
                .to(torch.long)
            )
            self.max_terrain_level = self.cfg.terrain.num_rows
            self.terrain_origins = (
                torch.from_numpy(self.terrain.env_origins)
                .to(self.device)
                .to(torch.float)
            )
            self.env_origins[:] = self.terrain_origins[
                self.terrain_levels, self.terrain_types
            ]
            return

        self.custom_origins = False
        self.env_origins = torch.zeros(self.cfg.env.num_envs, 3, device=self.device)
        num_cols = np.floor(np.sqrt(self.cfg.env.num_envs))
        num_rows = np.ceil(self.cfg.env.num_envs / num_cols)
        xx, yy = torch.meshgrid(
            torch.arange(int(num_rows), device=self.device),
            torch.arange(int(num_cols), device=self.device),
            indexing="ij",
        )
        spacing = self.cfg.env.env_spacing
        self.env_origins[:, 0] = spacing * xx.flatten()[: self.cfg.env.num_envs]
        self.env_origins[:, 1] = spacing * yy.flatten()[: self.cfg.env.num_envs]
        self.env_origins[:, 2] = 0.0
        self.terrain_levels = torch.zeros(
            self.cfg.env.num_envs, dtype=torch.long, device=self.device
        )

    def _parse_cfg(self, cfg):
        sim_dt = cfg.sim.dt if self.sim_params is None else self.sim_params.dt
        self.dt = self.cfg.control.decimation * sim_dt
        self.obs_scales = self.cfg.normalization.obs_scales
        self.reward_scales = class_to_dict(self.cfg.rewards.scales)
        self.command_ranges = class_to_dict(self.cfg.commands.ranges)

        # TODO: Terrain需要单独处理
        if self.cfg.terrain.mesh_type not in ["heightfield", "trimesh"]:
            self.cfg.terrain.curriculum = False

        self.max_episode_length_s = self.cfg.env.episode_length_s
        self.max_episode_length = int(np.ceil(self.max_episode_length_s / self.dt))
        self.cfg.domain_rand.push_interval = int(
            np.ceil(self.cfg.domain_rand.push_interval_s / self.dt)
        )

    def _sync_dt_with_model(self):
        # Use Sire model deltaT as source of truth to avoid cfg/xml mismatch.
        if hasattr(self, "sire_sim_loops") and len(self.sire_sim_loops) > 0:
            # TODO: 这个可能要修正
            sim_dt = float(self.sire_sim_loops[0].deltaT)
            self.dt = self.cfg.control.decimation * sim_dt
            self.max_episode_length = int(np.ceil(self.max_episode_length_s / self.dt))
            self.cfg.domain_rand.push_interval = int(
                np.ceil(self.cfg.domain_rand.push_interval_s / self.dt)
            )

    def _init_height_points(self):
        y = torch.tensor(self.cfg.terrain.measured_points_y, device=self.device)
        x = torch.tensor(self.cfg.terrain.measured_points_x, device=self.device)
        grid_x, grid_y = torch.meshgrid(x, y, indexing="ij")

        self.num_height_points = grid_x.numel()
        points = torch.zeros(
            self.num_envs, self.num_height_points, 3, device=self.device
        )
        points[:, :, 0] = grid_x.flatten()
        points[:, :, 1] = grid_y.flatten()
        return points

    # ------------------------------------------------------------------
    #  Height scanning via terrain heightfield interpolation
    #  (replaces MuJoCo mj_ray — no C++ ray cast needed)
    # ------------------------------------------------------------------
    def _get_heights(self, env_ids=None):
        """Bilinear interpolation on TerrainLayout.global_heightfield."""
        n = self.num_height_points if hasattr(self, "num_height_points") else 1
        if env_ids is None:
            env_ids_tensor = torch.arange(self.num_envs, device=self.device)
        else:
            env_ids_tensor = (
                env_ids
                if isinstance(env_ids, torch.Tensor)
                else torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
            )

        if n == 0 or self._flat_world_plane_only:
            return torch.zeros(
                len(env_ids_tensor), n, dtype=torch.float, device=self.device
            )

        # ── world-space positions of the height-scan points ──
        height_points = self.height_points[env_ids_tensor]  # [E, N, 3]
        base_quat = self.base_quat[env_ids_tensor]           # [E, 4]

        # apply yaw rotation only
        world_points = quat_apply_yaw(
            base_quat.repeat_interleave(self.num_height_points, dim=0),
            height_points.reshape(-1, 3),
        )
        world_points = world_points.reshape(
            len(env_ids_tensor), self.num_height_points, 3
        )
        world_points = world_points + self.root_states[env_ids_tensor, :3].unsqueeze(1)

        # ── terrain grid metadata ──
        terrain = self.terrain
        hf = terrain.global_heightfield  # np.ndarray shape (nrow, ncol)
        slope = terrain.global_slope      # float or None
        if hf is None and slope is None:
            return torch.zeros(
                len(env_ids_tensor), n, dtype=torch.float, device=self.device
            )

        x_start = float(terrain.border)
        x_len = float(terrain.total_length)
        y_start = float(terrain.border)
        y_len = float(terrain.total_width)

        heights = torch.zeros(
            len(env_ids_tensor), self.num_height_points,
            dtype=torch.float, device=self.device,
        )

        # bilinear interpolation (heightfield) or slope formula
        # MuJoCo flips PNG rows → np.flipud(hf) before interpolation.
        hf_for_query = np.flipud(hf) if hf is not None else None
        wp_np = world_points.cpu().numpy()  # [E, N, 3]
        for e in range(len(env_ids_tensor)):
            for p in range(self.num_height_points):
                wx = wp_np[e, p, 0]
                wy = wp_np[e, p, 1]

                if hf_for_query is not None:
                    heights[e, p] = float(bilinear_height(
                        hf_for_query, float(wx), float(wy),
                        x_start, x_len, y_start, y_len))
                elif slope is not None:
                    heights[e, p] = float(slope * max(0.0, wx - float(terrain.border)))
                else:
                    heights[e, p] = 0.0

        return heights

    def _reward_lin_vel_z(self):
        return torch.square(self.base_lin_vel[:, 2])

    def _reward_ang_vel_xy(self):
        return torch.sum(torch.square(self.base_ang_vel[:, :2]), dim=1)

    def _reward_orientation(self):
        return torch.sum(torch.square(self.projected_gravity[:, :2]), dim=1)

    def _reward_base_height(self):
        base_height = torch.mean(
            self.root_states[:, 2].unsqueeze(1) - self.measured_heights, dim=1
        )
        return torch.square(base_height - self.cfg.rewards.base_height_target)

    def _reward_torques(self):
        return torch.sum(torch.square(self.torques), dim=1)

    def _reward_dof_vel(self):
        return torch.sum(torch.square(self.dof_vel), dim=1)

    def _reward_dof_acc(self):
        return torch.sum(
            torch.square((self.last_dof_vel - self.dof_vel) / self.dt), dim=1
        )

    def _reward_action_rate(self):
        return torch.sum(torch.square(self.last_actions - self.actions), dim=1)

    def _reward_collision(self):
        if self.penalised_contact_indices.numel() == 0:
            return torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        return torch.sum(
            (
                torch.norm(
                    self.contact_forces[:, self.penalised_contact_indices, :], dim=-1
                )
                > 0.1
            ).float(),
            dim=1,
        )

    def _reward_termination(self):
        return self.reset_buf * (~self.time_out_buf)

    def _reward_dof_pos_limits(self):
        out_of_limits = -(self.dof_pos - self.dof_pos_limits[:, 0]).clip(max=0.0)
        out_of_limits += (self.dof_pos - self.dof_pos_limits[:, 1]).clip(min=0.0)
        return torch.sum(out_of_limits, dim=1)

    def _reward_dof_vel_limits(self):
        return torch.sum(
            (
                torch.abs(self.dof_vel)
                - self.dof_vel_limits * self.cfg.rewards.soft_dof_vel_limit
            ).clip(min=0.0, max=1.0),
            dim=1,
        )

    def _reward_torque_limits(self):
        return torch.sum(
            (
                torch.abs(self.torques)
                - self.torque_limits * self.cfg.rewards.soft_torque_limit
            ).clip(min=0.0),
            dim=1,
        )

    def _reward_tracking_lin_vel(self):
        lin_vel_error = torch.sum(
            torch.square(self.commands[:, :2] - self.base_lin_vel[:, :2]), dim=1
        )
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
        """惩罚足端侧向打滑——水平接触力 > 5×垂向力时判定为stumble。"""
        return torch.any(
            torch.norm(self.contact_forces[:, self.feet_indices, :2], dim=2)
            > 5 * torch.abs(self.contact_forces[:, self.feet_indices, 2]),
            dim=1,
        )

    def _reward_stand_still(self):
        return torch.sum(torch.abs(self.dof_pos - self.default_dof_pos), dim=1) * (
            torch.norm(self.commands[:, :2], dim=1) < 0.1
        )

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
        """惩罚过大的足端接触力，鼓励轻柔落脚。"""
        return torch.sum(
            (
                torch.norm(self.contact_forces[:, self.feet_indices, :], dim=-1)
                - self.cfg.rewards.max_contact_force
            ).clip(min=0.0),
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

    # _refresh_sim_tensors removed — use _refresh_sim_tensors_sire instead

    # ------------------------------------------------------------------
    #  Post physics step (calls _refresh_sim_tensors_sire)
    # ------------------------------------------------------------------
    def _post_physics_step_sire(self):
        self.extras = {}
        self.episode_length_buf += 1
        self.common_step_counter += 1
        self._refresh_sim_tensors_sire()

        self.base_quat[:] = self.root_states[:, 3:7]
        self.base_lin_vel[:] = quat_rotate_inverse(
            self.base_quat, self.root_states[:, 7:10]
        )
        self.base_ang_vel[:] = quat_rotate_inverse(
            self.base_quat, self.root_states[:, 10:13]
        )
        self.projected_gravity[:] = quat_rotate_inverse(
            self.base_quat, self.gravity_vec
        )

        self._post_physics_step_callback()
        self._update_episode_diagnostics()
        self._post_physics_step_tasks()
        self.check_termination()
        self.compute_reward()
        env_ids = self.reset_buf.nonzero(as_tuple=False).flatten()
        self.reset_idx(env_ids)
        self.compute_observations()

        # Sanitize: zero out NaN/Inf in observations that may leak
        # from envs with undetected physics corruption.
        self.obs_buf = torch.nan_to_num(self.obs_buf, nan=0.0, posinf=0.0, neginf=0.0)
        if self.privileged_obs_buf is not None:
            self.privileged_obs_buf = torch.nan_to_num(
                self.privileged_obs_buf, nan=0.0, posinf=0.0, neginf=0.0)

        self.last_actions[:] = self.actions[:]
        self.last_dof_vel[:] = self.dof_vel[:]
        self.last_root_vel[:] = self.root_states[:, 7:13]
        self.last_feet_pos_world[:] = self.feet_pos_world[:]

        # ---- reward decomposition diagnostic (first 20 steps) ----
        if self.common_step_counter <= 20:
            r_track_lin = self._reward_tracking_lin_vel().mean().item()
            r_track_ang = self._reward_tracking_ang_vel().mean().item()
            r_orient = self._reward_orientation().mean().item()
            r_height = self._reward_base_height().mean().item()
            r_torque = self._reward_torques().mean().item()
            r_air = self._reward_feet_air_time().mean().item()
            r_collision = self._reward_collision().mean().item()
            base_z = self.root_states[:, 2].mean().item()
            tilt = self.projected_gravity[:, :2].norm(dim=1).mean().item()
            foot_contact_frac = self.foot_ground_contact.float().mean().item()
            print(
                f"[Sire diag step {self.common_step_counter}] "
                f"rew_total={self.rew_buf.mean():.4f} "
                f"track_lin={r_track_lin:+.4f} track_ang={r_track_ang:+.4f} "
                f"air={r_air:+.4f} "
                f"orient={r_orient:+.4f} height={r_height:+.4f} "
                f"torque={r_torque:+.4f} collision={r_collision:+.4f} "
                f"base_z={base_z:.3f} tilt={tilt:.4f} foot_ct={foot_contact_frac:.2f}",
                flush=True,
            )

    # ------------------------------------------------------------------
    #  State reading  (the core Sire ↔ MuJoCo compatibility layer)
    #
    #  ╔══════════════════════════════════════════════════════════════════╗
    #  ║  API INCONSISTENCY CHECKLIST (Sire vs MuJoCo)                   ║
    #  ╠══════════════════════════════════════════════════════════════════╣
    #  ║ [1] ROOT STATE — FIXED: generalMotionPool is EMPTY in SRCF.     ║
    #  ║     MuJoCo: d.qpos[root_adr:root_adr+7] / d.qvel[root_adr+6]    ║
    #  ║     Sire:   m.partPool()[1].pq (7d) / .vs (6d)                  ║
    #  ║     See go2.xml: <GeneralMotionPoolElement/> is empty!           ║
    #  ╠══════════════════════════════════════════════════════════════════╣
    #  ║ [2] JOINT POS/VEL — motionPool order must match dof_names.      ║
    #  ║     MuJoCo: d.qpos[qpos_adr[j]] / d.qvel[qvel_adr[j]]           ║
    #  ║     Sire:   m.motionPool()[j].mp / .mv                           ║
    #  ║     NOTE: Sire motion names are "actuator_0".."actuator_11",    ║
    #  ║     NOT "FL_hip_joint" etc. Must verify index alignment.         ║
    #  ╠══════════════════════════════════════════════════════════════════╣
    #  ║ [3] MOTION NAME MAPPING — TODO: build dof_name → motion_idx     ║
    #  ║     MuJoCo: mj_name2id(model, mjOBJ_JOINT, name) → id           ║
    #  ║     Sire:   iterate jointPool, match name → find actuator idx    ║
    #  ║     The joint names in Sire JointPool match MuJoCo (FL_hip_joint║
    #  ║     etc.), but motion names do NOT. Need to map joint → motion.  ║
    #  ╠══════════════════════════════════════════════════════════════════╣
    #  ║ [4] PHYSICS ENGINE — XML uses AnalyticalTangentForceSolver,     ║
    #  ║     not PsVsSolver. Methods like contactForceIdx() /            ║
    #  ║     computePointPairPenetration() may not exist on this solver!  ║
    #  ║     TODO: verify which solver is actually loaded & its API.      ║
    #  ╠══════════════════════════════════════════════════════════════════╣
    #  ║ [5] sire_physics LIST — NOT INITIALIZED in _create_sire_envs!   ║
    #  ║     TODO: store physics engine per env during creation.          ║
    #  ╠══════════════════════════════════════════════════════════════════╣
    #  ║ [6] _sire_geom_to_part — NOT BUILT. Needed for ground contact.  ║
    #  ║     TODO: build geom_id → part_id map from geometryPool.         ║
    #  ╠══════════════════════════════════════════════════════════════════╣
    #  ║ [7] CONTACT FORCE INDEX — formula cf_idx + pid assumes fixed    ║
    #  ║     offset per part in forcePool. Depends on solver layout.      ║
    #  ║     TODO: verify forcePool layout for AnalyticalTangentForceSolver║
    #  ╠══════════════════════════════════════════════════════════════════╣
    #  ║ [8] FOOT POSITION — getPm()[3,7,11] is correct but getPq() is  ║
    #  ║     cleaner. Could also use getPq()[:3] directly.                ║
    #  ╠══════════════════════════════════════════════════════════════════╣
    #  ║ [9] ACTUATOR CONTROL — Sire uses ActuatorSISO with kp=50 kd=2.  ║
    #  ║     Setting .desiredValue triggers internal PD. For direct       ║
    #  ║     torque (MuJoCo 'T' mode), set motion.mf instead.             ║
    #  ║     TODO: match control mode (P/V/T) to Sire actuator mechanism. ║
    #  ╠══════════════════════════════════════════════════════════════════╣
    #  ║ [10] CONTROLLER — Sire XML uses ZeroPosition, not Actuator-based║
    #  ║     control. The SimulationLoop may not call actuator.forward(). ║
    #  ║     TODO: verify step() control flow works with this config.     ║
    #  ╚══════════════════════════════════════════════════════════════════╝
    # ------------------------------------------------------------------
    def _refresh_sim_tensors_sire(self):
        dt = float(self.cfg.sim.dt)

        # Allocate tensors (same as MuJoCo _refresh_sim_tensors)
        self.root_states = torch.zeros(self.num_envs, 13, dtype=torch.float, device=self.device)
        self.dof_pos = torch.zeros(self.num_envs, self.num_dof, dtype=torch.float, device=self.device)
        self.dof_vel = torch.zeros(self.num_envs, self.num_dof, dtype=torch.float, device=self.device)
        self.contact_forces = torch.zeros(self.num_envs, self.num_bodies, 3, dtype=torch.float, device=self.device)
        self.feet_pos_world = torch.zeros(self.num_envs, len(self.feet_indices), 3, dtype=torch.float, device=self.device)
        self.body_ground_contact = torch.zeros(self.num_envs, self.num_bodies, dtype=torch.bool, device=self.device)
        self.foot_ground_contact = torch.zeros(self.num_envs, len(self.feet_indices), dtype=torch.bool, device=self.device)

        for i in range(self.num_envs):
            m = self.sire_models[i]
            s = self.sire_sim_loops[i]

            try:
                # --- root (base link) -------------------------------------------
                # Aris Part.pq returns [x, y, z, qx, qy, qz, qw] (scalar-last).
                # root_states uses the same Aris convention.
                base_part = m.partPool()[1]
                pq = base_part.pq   # [x, y, z, qx, qy, qz, qw]
                vs = base_part.vs   # [vx, vy, vz, wx, wy, wz]

                # MuJoCo-style sanity check: if simulation has blown up
                # (e.g. robot penetrated deep into ground), mark env for
                # reset rather than letting garbage observations through.
                if (abs(pq[0]) > 100.0 or abs(pq[1]) > 100.0
                        or pq[2] < -5.0 or pq[2] > 50.0
                        or any(abs(v) > 200.0 for v in vs)):
                    self._sire_physics_failures.append(i)
                    self._reinit_sire_env(i)
                    continue

                self.root_states[i, :7] = torch.tensor(pq, dtype=torch.float)
                self.root_states[i, 7:13] = torch.tensor(vs, dtype=torch.float)

                # --- joints ------------------------------------------------------
                for j, dof_name in enumerate(self.dof_names):
                    mot_idx = self._sire_dof_to_motion[dof_name]
                    mot = m.motionPool()[mot_idx]
                    mp = float(mot.mp)
                    mv = float(mot.mv)
                    # Joint limit enforcement: clamp position to [min, max]
                    # and write back to simulation (MuJoCo does this internally).
                    lo = self.dof_pos_limits[j, 0].item()
                    hi = self.dof_pos_limits[j, 1].item()
                    if mp < lo:
                        mp = lo
                        mot.mp = lo
                        if mv < 0.0:
                            mot.mv = 0.0
                            mv = 0.0
                    elif mp > hi:
                        mp = hi
                        mot.mp = hi
                        if mv > 0.0:
                            mot.mv = 0.0
                            mv = 0.0
                    self.dof_pos[i, j] = mp
                    self.dof_vel[i, j] = mv

                # --- foot positions ----------------------------------------------
                for f_idx, fname in enumerate(self.foot_names):
                    pid = self._sire_part_name_to_idx.get(f"{fname.upper()}_calf")
                    if pid is not None:
                        pm = m.partPool()[pid].getPm()
                        self.feet_pos_world[i, f_idx, 0] = pm[3]
                        self.feet_pos_world[i, f_idx, 1] = pm[7]
                        self.feet_pos_world[i, f_idx, 2] = pm[11]

                # --- contact forces & ground contact ---------------------------
                dt_act = self._sire_dt_actual.get(i, dt)
                scale = dt_act / dt if dt > 0 else 1.0
                try:
                    # Use part-ID-based results (maps geomId→partId internally,
                    # analogous to MuJoCo's mj_geom2body), so that pa/pb are
                    # valid part pool indices for contact_forces indexing.
                    cr = s.lastContactPairResultsWithPartIds()
                    print(f"[Sire] env {i} contact pair results: {cr}")
                    for (pa, pb, fx, fy, fz, px, py, pz) in cr:
                        sfx, sfy, sfz = fx * scale, fy * scale, fz * scale
                        # Accumulate forces (MuJoCo uses +=; we previously
                        # overwrote with =, losing multi-contact data).
                        self.contact_forces[i, pa, 0] += sfx
                        self.contact_forces[i, pa, 1] += sfy
                        self.contact_forces[i, pa, 2] += sfz
                        self.contact_forces[i, pb, 0] -= sfx
                        self.contact_forces[i, pb, 1] -= sfy
                        self.contact_forces[i, pb, 2] -= sfz
                        if pa == 0 and pb != 0:
                            self.body_ground_contact[i, pb] = True
                        if pb == 0 and pa != 0:
                            self.body_ground_contact[i, pa] = True
                except Exception:
                    pass  # solver doesn't support lastContactPairResults

                if len(self.feet_indices_np) > 0:
                    self.foot_ground_contact[i] = self.body_ground_contact[
                        i, self.feet_indices
                    ]
            except Exception:
                # Reading corrupted state after physics failure — already reset
                pass

    # ------------------------------------------------------------------
    #  Trajectory recording (for comparison / debugging)
    # ------------------------------------------------------------------
    def record_trajectory(
        self,
        actions_seq: np.ndarray,
        env_idx: int = 0,
    ) -> dict:
        """
        Run an action sequence and return raw trajectory data.

        Parameters
        ----------
        actions_seq : (N, num_actions) ndarray
            Sequence of actions (numpy, one per control step).
        env_idx : int
            Which parallel environment to run.

        Returns
        -------
        dict with keys:
            t      : (N+1,)  float64  — simulation time at each step
            q      : (N+1, J) float64 — joint positions
            qd     : (N+1, J) float64 — joint velocities (optional)
            dt     : (N,)    float64  — actual integration dt per step
        """
        actions_seq = np.asarray(actions_seq, dtype=np.float32)
        n_steps = len(actions_seq)

        t_list = [float(self.sire_sim_loops[env_idx].simTime())]
        q_list = [self.dof_pos[env_idx].cpu().numpy().copy()]
        qd_list = [self.dof_vel[env_idx].cpu().numpy().copy()]
        dt_list = []

        for i in range(n_steps):
            act = torch.from_numpy(actions_seq[i]).unsqueeze(0)
            sl = self.sire_sim_loops[env_idx]
            t0 = sl.simTime()
            self.step(act)
            t1 = sl.simTime()
            t_list.append(float(t1))
            q_list.append(self.dof_pos[env_idx].cpu().numpy().copy())
            qd_list.append(self.dof_vel[env_idx].cpu().numpy().copy())
            dt_list.append(t1 - t0)

        return {
            "t": np.array(t_list, dtype=np.float64),
            "q": np.array(q_list, dtype=np.float64),
            "qd": np.array(qd_list, dtype=np.float64),
            "dt": np.array(dt_list, dtype=np.float64),
        }
