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

from collections import deque
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
        """Advance every independent Sire environment through the C++ batch path."""
        return self.stepSireBatch(actions)

    def stepSireBatch(self, actions):
        """Batched Sire step: one Python call, persistent native worker threads."""
        clip_actions = self.cfg.normalization.clip_actions
        self.actions = torch.clip(actions, -clip_actions, clip_actions).to(
            self.device
        ).contiguous()
        outputs = self._sire_batch_stepper.step(self.actions.numpy())
        recovered_env_ids = np.asarray(
            self._sire_batch_stepper.recoveredEnvIds, dtype=np.int64
        )
        recovered_count = int(recovered_env_ids.size)
        window_steps = max(
            1, int(getattr(self.cfg.sim, "sire_recovery_window_steps", 1000))
        )
        self._sire_recovery_counts.append(recovered_count)
        self._sire_recovery_window_total += recovered_count
        while len(self._sire_recovery_counts) > window_steps:
            self._sire_recovery_window_total -= self._sire_recovery_counts.popleft()
        recovery_window_fraction = self._sire_recovery_window_total / (
            max(1, len(self._sire_recovery_counts)) * self.num_envs
        )
        self._sire_physics_failure_buf.zero_()
        if recovered_env_ids.size:
            recovered_messages = list(self._sire_batch_stepper.recoveredErrors)
            recovery_total = int(
                self._sire_batch_stepper.totalRecoveredFailures
            )
            recovered_ids = torch.from_numpy(recovered_env_ids).to(self.device)
            self._sire_physics_failure_buf[recovered_ids] = True
            for message in recovered_messages:
                print(f"[Sire recovered environment] {message}", flush=True)
            max_per_step = int(
                getattr(self.cfg.sim, "sire_max_recoveries_per_step", 8)
            )
            max_total = int(
                getattr(self.cfg.sim, "sire_max_total_recoveries", 0)
            )
            max_fraction = float(
                getattr(self.cfg.sim, "sire_max_recovery_fraction", 1e-4)
            )
            total_exceeded = max_total > 0 and recovery_total > max_total
            window_exceeded = (
                len(self._sire_recovery_counts) >= window_steps
                and max_fraction > 0.0
                and recovery_window_fraction > max_fraction
            )
            if recovered_env_ids.size > max_per_step or total_exceeded or window_exceeded:
                raise RuntimeError(
                    "Sire physics recovery threshold exceeded: "
                    f"this_step={recovered_env_ids.size}/{max_per_step}, "
                    f"total={recovery_total}/{max_total or 'disabled'}, "
                    f"window={self._sire_recovery_window_total}/"
                    f"{len(self._sire_recovery_counts) * self.num_envs} "
                    f"({recovery_window_fraction:.3e}/{max_fraction:.3e})\n"
                    f"latest_recovery={recovered_messages[-1]}"
                )
        (
            root_states,
            dof_pos,
            dof_vel,
            torques,
            contact_forces,
            feet_pos_world,
            body_ground_contact,
            foot_ground_contact,
            _dt_actual,
        ) = outputs
        self.root_states.copy_(torch.from_numpy(root_states))
        self.dof_pos.copy_(torch.from_numpy(dof_pos))
        self.dof_vel.copy_(torch.from_numpy(dof_vel))
        self.torques.copy_(torch.from_numpy(torques))
        self.contact_forces.copy_(torch.from_numpy(contact_forces))
        self.feet_pos_world.copy_(torch.from_numpy(feet_pos_world))
        self.body_ground_contact.copy_(torch.from_numpy(body_ground_contact))
        self.foot_ground_contact.copy_(torch.from_numpy(foot_ground_contact))
        # Native physics runs on a shared generated terrain coordinate system,
        # while every independent Simulator exposes its own local world frame
        # to RL.  Thus every environment starts at local (0, 0, base_height).
        self.root_states[:, :3] -= self._physics_origins
        self.feet_pos_world -= self._physics_origins.unsqueeze(1)

        self._post_physics_step_sire(refresh_from_sire=False)
        if recovered_env_ids.size:
            self.extras["sire_physics_failure_count"] = int(
                recovered_env_ids.size
            )
            self.extras["sire_physics_failure_total"] = int(
                recovery_total
            )
            self.extras["sire_physics_failure_window_fraction"] = float(
                recovery_window_fraction
            )
            self.extras.setdefault("episode", {})[
                "physics_failure_count"
            ] = float(recovered_env_ids.size)
        return self._clip_and_collect_step_result()

    def legacySireStep(self, actions):
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
                do:
                    _update_actuator_torque(i)  # re-PD from current mp/mv
                    handleContact()
                until headerIsCtrl()  # next control interval remains pending

        Key fix: torques are recomputed EVERY event (step AND ctrl),
        just like MuJoCo recomputes torques every decimation substep
        and dog.py recomputes tau every simulator.integrate() call.
        """
        clip_actions = self.cfg.normalization.clip_actions
        self.actions = torch.clip(actions, -clip_actions, clip_actions).to(self.device)
        # ── Capture initial state BEFORE first physics step ──
        if (
            getattr(self.cfg.sim, "sire_diagnostics", False)
            and self.common_step_counter == 0
        ):
            m0 = self.sire_models[0]
            sl0 = self.sire_sim_loops[0]
            base_part = m0.link(1)
            init_pq = list(base_part.pq)
            init_vs = list(base_part.vs)
            init_dof_pos = {}
            for name in self.dof_names:
                mi = self._sire_dof_to_motion[name]
                init_dof_pos[name] = float(m0.motion(mi).mp)
            self._diag_init_state = {
                'base_pq': init_pq,       # [x,y,z, qx,qy,qz,qw]
                'base_vs': init_vs,       # [vx,vy,vz, wx,wy,wz]
                'dof_pos': init_dof_pos,  # {name: angle}
                'seed': getattr(self, '_diag_seed', None),
            }
            self._diag_actions = []  # accumulate per-step actions

            # ── Diagnostic: actual Sire timing params ──
            print(
                f"[Sire timing] deltaT={sl0.deltaT} ctrlT={sl0.ctrlT} "
                f"cfg.sim.dt={self.cfg.sim.dt} decimation={self.cfg.control.decimation} "
                f"simTime={sl0.simTime():.4f}",
                flush=True,
            )

            # ── Print pre-physics initial state ("step 0") ──
            px, py, pz = init_pq[0], init_pq[1], init_pq[2]
            qx, qy, qz, qw = init_pq[3], init_pq[4], init_pq[5], init_pq[6]
            vp_init = sire.vs2vp(init_vs, init_pq[:3])  # body linear velocity
            vx, vy, vz = vp_init[0], vp_init[1], vp_init[2]
            # Compute base_lin_vel = quat_rotate_inverse(quat, world_vel)
            quat_t = torch.tensor([qx, qy, qz, qw], device=self.device)
            world_vel_t = torch.tensor([vx, vy, vz], device=self.device)
            base_vel_t = quat_rotate_inverse(quat_t, world_vel_t)
            dof_str = ' '.join(f'{k}={v:.3f}' for k, v in init_dof_pos.items())
            act = self.actions[0].cpu().tolist()
            act_str = ' '.join(f'{a:.3f}' for a in act)
            print(
                f"[Sire diag step 0 (pre-physics)] "
                f"pos=({px:.3f},{py:.3f},{pz:.3f})\n"
                f"  world_vel: vx={vx:.3f} vy={vy:.3f} vz={vz:.3f}\n"
                f"  base_vel:  vx={base_vel_t[0]:.3f} vy={base_vel_t[1]:.3f} vz={base_vel_t[2]:.3f}\n"
                f"  quat=[{qx:.4f},{qy:.4f},{qz:.4f},{qw:.4f}]\n"
                f"  dof: {dof_str}\n"
                f"  action: [{act_str}]",
                flush=True,
            )

        # ── Record action for this step (from env 0) ──
        if (
            getattr(self.cfg.sim, "sire_diagnostics", False)
            and self.common_step_counter < 20
        ):
            act = self.actions[0].cpu().tolist() if hasattr(self.actions[0], 'cpu') else list(self.actions[0])
            self._diag_actions.append(act)

        # ── Per-substep counter for env 0 diag ──
        substep_idx = 0

        for i in range(self.num_envs):
            sl = self.sire_sim_loops[i]

            try:
                # Process the current interval, starting with init/ctrl.
                # Recompute PD torque from current Sire joint state
                # before EVERY handleContact, matching MuJoCo/dog.py.
                interval_start = sl.simTime()
                step_count = 0
                while step_count == 0 or not sl.headerIsCtrl():
                    if step_count >= 100000:
                        raise RuntimeError('control interval exceeded 100000 events')
                    self._update_actuator_torque(i)
                    t0 = sl.simTime()
                    sl.handleContact()
                    self._sire_dt_actual[i] = sl.simTime() - t0
                    step_count += 1
                    # ── Per-substep diag (env 0 only, first 2 ctrl steps) ──
                    if (
                        getattr(self.cfg.sim, "sire_diagnostics", False)
                        and i == 0
                        and self.common_step_counter <= 1
                    ):
                        pq = self.sire_models[0].link(1).pq
                        vs = self.sire_models[0].link(1).vs
                        as1 = self.sire_models[0].link(1).getAs()
                        vp = sire.vs2vp(vs, pq[:3])
                        ap = sire.as2ap(vs, as1, pq[:3])
                        cr = sl.lastContactPairResultsWithPartIds()
                        nc = len(cr)
                        f0 = f"f=({cr[0][2]:.1f},{cr[0][3]:.1f},{cr[0][4]:.1f})" if nc > 0 else "no_contact"
                        print(
                            f"  [substep {substep_idx}] step  "
                            f"vp=({vp[0]:.3f},{vp[1]:.3f},{vp[2]:.3f})  "
                            f"ap=({ap[0]:.3f},{ap[1]:.3f},{ap[2]:.3f})  "
                            f"z={pq[2]:.4f}  nc={nc}  {f0}",
                            flush=True,
                        )
                        substep_idx += 1

                # The next control event remains pending for the next action.
                self._sire_dt_actual[i] = sl.simTime() - interval_start
                # ── Timing diag: substep count per env (first 3 steps, env 0) ──
                if (
                    getattr(self.cfg.sim, "sire_diagnostics", False)
                    and i == 0
                    and self.common_step_counter <= 2
                ):
                    print(
                        f"[Sire timing] step={self.common_step_counter} "
                        f"events={step_count}, "
                        f"t_start={interval_start:.4f} t_end={sl.simTime():.4f}",
                        flush=True,
                    )
            except Exception as e:
                raise RuntimeError(self._format_sire_exception(i, e)) from e

        # ---- post-step (once per control interval) ----
        self._post_physics_step_sire()
        return self._clip_and_collect_step_result()

    def legacy_sire_step(self, actions):
        """PEP-8 alias retained alongside the requested legacySireStep API."""
        return self.legacySireStep(actions)

    def _clip_and_collect_step_result(self):
        clip_obs = self.cfg.normalization.clip_observations
        self.obs_buf = torch.clip(self.obs_buf, -clip_obs, clip_obs)
        if self.privileged_obs_buf is not None:
            self.privileged_obs_buf = torch.clip(
                self.privileged_obs_buf, -clip_obs, clip_obs
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
        # A native numerical failure is a terminal transition for only that
        # independent environment. The C++ batch step has already restored its
        # Simulator, so the ordinary reset path below can randomize it safely.
        if hasattr(self, "_sire_physics_failure_buf"):
            self.reset_buf |= self._sire_physics_failure_buf
        # A low base is an ordinary fall, not a numerical exception.  This is
        # evaluated per environment and complements contact-based termination
        # when the last plane-contact sample disappears after tunnelling.
        termination_height = float(
            getattr(self.cfg.asset, "termination_height", -float("inf"))
        )
        self.base_height_fall_buf = self.root_states[:, 2] < termination_height
        self.reset_buf |= self.base_height_fall_buf
        # Leaving the finite terrain is a truncation (timeout), not a fall or
        # a native physics failure. This keeps PPO bootstrapping semantics
        # correct and resets only the affected independent environment.
        self.time_out_buf = self.episode_length_buf >= self.max_episode_length
        self.time_out_buf |= self._terrain_out_of_bounds()
        self.reset_buf |= self.time_out_buf
        self._update_task_termination()

        # ── Diagnostic: log WHY reset triggered (first 50 steps only) ──
        if (
            getattr(self.cfg.sim, "sire_diagnostics", False)
            and self.common_step_counter <= 50
        ):
            reset_envs = self.reset_buf.nonzero(as_tuple=False).flatten()
            for eid in reset_envs.tolist():
                if self.time_out_buf[eid]:
                    continue  # timeout is normal, skip
                # Find which termination bodies had contact
                cf = self.contact_forces[eid, self.termination_contact_indices, :]
                force_norms = torch.norm(cf, dim=-1)  # [num_term_bodies]
                triggered = (force_norms > 1.0).nonzero(as_tuple=False).flatten()
                parts_info = []
                for idx in triggered:
                    term_body_id = self.termination_contact_indices[idx].item()
                    # Look up body name via partPool
                    body_name = self.sire_models[eid].partPool()[term_body_id].name
                    f_norm = force_norms[idx].item()
                    parts_info.append(f"{body_name}={f_norm:.1f}N")
                print(
                    f"[Sire termination] step={self.common_step_counter} env={eid} "
                    f"bodies: {parts_info if parts_info else 'unknown'}"
                    f"  base_z={self.root_states[eid, 2].item():.3f}",
                    flush=True,
                )

    def reset_idx(self, env_ids):
        if len(env_ids) == 0:
            return

        if self.cfg.terrain.curriculum:
            self._update_terrain_curriculum(env_ids)
        if self.cfg.commands.curriculum and (
            self.common_step_counter % int(self.max_episode_length) == 0
        ):
            self.update_command_curriculum(env_ids)
        # An episode reset is a full reset of only the completed independent
        # Simulator instances (model, contacts, events, timer, and recorder).
        env_ids_np = env_ids.detach().cpu().numpy().astype(np.int64, copy=False)
        if hasattr(self, "_sire_batch_stepper"):
            self._sire_batch_stepper.reset(env_ids_np)
        else:
            for eid in env_ids.tolist():
                self.sire_simulators[eid].reset()
        self._reset_dofs(env_ids)
        self._reset_root_states(env_ids)
        # Reset changes root state after pre-reset rewards were evaluated.
        # Rebuild body-frame quantities before returning the next observation.
        self.base_quat[env_ids] = self.root_states[env_ids, 3:7]
        self.base_lin_vel[env_ids] = quat_rotate_inverse(
            self.base_quat[env_ids], self.root_states[env_ids, 7:10])
        self.base_ang_vel[env_ids] = quat_rotate_inverse(
            self.base_quat[env_ids], self.root_states[env_ids, 10:13])
        self.projected_gravity[env_ids] = quat_rotate_inverse(
            self.base_quat[env_ids], self.gravity_vec[env_ids])
        if self.cfg.terrain.measure_heights:
            self.measured_heights[env_ids] = self._get_heights(env_ids)

        # ---- settling phase: land robot with zero actions before training ----
        # Without this, random PD torques launch the base upward before feet
        # ever touch ground, causing immediate termination and zero learning.
        # self._settle_sire_envs(env_ids)

        self._resample_commands(env_ids, initialize=True)

        ep_len = self.episode_length_buf[env_ids].float().clamp(min=1.0)
        ep_duration = (ep_len * self.dt).clamp(min=self.dt)

        self.last_actions[env_ids] = 0.0
        self.actions[env_ids] = 0.0
        self.last_dof_vel[env_ids] = 0.0
        self.feet_air_time[env_ids] = 0.0
        self.contact_forces[env_ids] = 0.0
        self.body_ground_contact[env_ids] = False
        self.foot_ground_contact[env_ids] = False
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
        self._update_smoothed_commands()

        if self.cfg.terrain.measure_heights:
            self.measured_heights = self._get_heights()
        if self.cfg.domain_rand.push_robots and (
            self.common_step_counter % int(self.cfg.domain_rand.push_interval) == 0
        ):
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
        self.command_targets[env_ids, 0] = torch_rand_float(
            self.command_ranges["lin_vel_x"][0],
            self.command_ranges["lin_vel_x"][1],
            (len(env_ids), 1),
            device=self.device,
        ).squeeze(1)
        self.command_targets[env_ids, 1] = torch_rand_float(
            self.command_ranges["lin_vel_y"][0],
            self.command_ranges["lin_vel_y"][1],
            (len(env_ids), 1),
            device=self.device,
        ).squeeze(1)
        if self.cfg.commands.heading_command:
            self.command_targets[env_ids, 3] = torch_rand_float(
                self.command_ranges["heading"][0],
                self.command_ranges["heading"][1],
                (len(env_ids), 1),
                device=self.device,
            ).squeeze(1)
        else:
            self.command_targets[env_ids, 2] = torch_rand_float(
                self.command_ranges["ang_vel_yaw"][0],
                self.command_ranges["ang_vel_yaw"][1],
                (len(env_ids), 1),
                device=self.device,
            ).squeeze(1)

        self.command_targets[env_ids, :2] *= (
            torch.norm(self.command_targets[env_ids, :2], dim=1) > 0.2
        ).unsqueeze(1)

        prob_zero = float(getattr(self.cfg.commands, "prob_zero_command", 0.0))
        prob_neg = float(getattr(self.cfg.commands, "prob_negative_command", 0.0))
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
                neg_range = getattr(
                    self.cfg.commands, "negative_lin_vel_x_range", [-0.3, -0.1]
                )
                count = int(neg_mask.sum().item())
                self.command_targets[neg_ids, 0] = torch_rand_float(
                    float(neg_range[0]),
                    float(neg_range[1]),
                    (count, 1),
                    device=self.device,
                ).squeeze(1)
                self.command_targets[neg_ids, 1] = 0.0
                if self.cfg.commands.heading_command:
                    self.command_targets[neg_ids, 3] = self._current_heading(neg_ids)

        if initialize:
            self.commands[env_ids, :3] = self._desired_velocity_commands(env_ids)
            if self.cfg.commands.heading_command:
                self.commands[env_ids, 3] = self.command_targets[env_ids, 3]

    # ------------------------------------------------------------------
    #  Single-environment torque update (reads current Sire mp/mv)
    #  Called EVERY event (step + ctrl), matching MuJoCo/dog.py.
    # ------------------------------------------------------------------
    def _update_actuator_torque(self, env_idx: int):
        """Compute and apply PD torques every simulation substep.

        Reads fresh joint state from C++ each call — necessary for accurate
        position control where joint angles evolve within a control cycle.
        """
        model = self.sire_models[env_idx]
        # ── Read joint state in motionPool order, reorder to dof_names order ──
        mps = np.array(sire.getMotionMps(model), dtype=np.float64)[self._motion_idx]
        mvs = np.array(sire.getMotionMvs(model), dtype=np.float64)[self._motion_idx]
        # ── Vectorized PD control (same formula as MuJoCo's _compute_torques) ──
        actions_scaled = (self.actions[env_idx] * self.cfg.control.action_scale).numpy().astype(np.float64)
        control_type = self.cfg.control.control_type
        if control_type == "P":
            torques = (self._p_gains * (actions_scaled + self._default_pos - mps)
                       - self._d_gains * mvs)
        elif control_type == "V":
            last_dof_vel = self.last_dof_vel[env_idx].numpy().astype(np.float64)
            torques = (self._p_gains * (actions_scaled - mvs)
                       - self._d_gains * (mvs - last_dof_vel) / max(self.dt, 1e-6))
        elif control_type == "T":
            torques = actions_scaled
        else:
            torques = np.zeros(self.num_actions, dtype=np.float64)
        torques = np.clip(torques, -self._torque_limits, self._torque_limits)
        # Write to torch tensor (dof_names order) and C++ (motionPool order)
        self.torques[env_idx] = torch.as_tensor(torques, dtype=torch.float)
        sire.setMotionDesiredValues(model, torques[self._motion_idx_inv].tolist())

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
            # Build full motionPool arrays via numpy advanced indexing (like MuJoCo's qpos_adr_np)
            mps_full = np.zeros(self._num_motions, dtype=np.float64)
            mps_full[self._motion_idx] = self.dof_pos[eid].cpu().numpy()
            sire.setMotionMps(m, mps_full.tolist())
            sire.setMotionMvs(m, [0.0] * self._num_motions)
            m.forwardKinematics()
            m.forwardKinematicsVel()

    def _reset_root_states(self, env_ids):
        """Sire version: write root pose through body pq."""
        # Simulators do not share a world, so actor-grid offsets and random
        # x/y spawn offsets are incorrect here.  State buffers use the local
        # frame; _physics_origins is applied only when writing the generated
        # terrain's physical coordinates into Sire.
        self.root_states[env_ids] = self.base_init_state

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
            physical_position = r[:3] + self._physics_origins[eid]
            # root_states uses body-point velocity (MuJoCo convention);
            # Sire part.vs expects spatial velocity (twist at origin).
            pp = physical_position.cpu().numpy()
            vp = r[7:10].cpu().numpy()
            w  = r[10:13].cpu().numpy()
            vs = np.array(sire.vp2vs(pp, vp, w))   # body-point → spatial twist (with ω)
            physical_pq = r[:7].clone()
            physical_pq[:3] = physical_position
            m.link(1).pq = physical_pq.cpu().numpy()
            m.link(1).vs = vs
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
                    raise RuntimeError(self._format_sire_exception(eid, e)) from e

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
            m = self.sire_models[i]
            pq = m.link(1).pq
            vs = list(m.link(1).vs)
            vp_new = self.root_states[i, 7:9].cpu().numpy().tolist() + [0.0]  # xy push, z=0
            w = vs[3:6]  # preserve current angular velocity
            vs_spatial = sire.vp2vs(pq[:3], vp_new, w)
            m.link(1).vs = list(vs_spatial)

    def _update_terrain_curriculum(self, env_ids):
        if (
            not self.init_done
            or not self.custom_origins
            or not hasattr(self, "terrain_origins")
        ):
            return
        distance = torch.norm(self.root_states[env_ids, :2], dim=1)
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
        self._physics_origins[env_ids] = self.terrain_origins[
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
        # These mappings are required by the initial state refresh below.
        # Keeping them ahead of all reads also ensures initialization errors
        # are reported instead of being hidden by partially initialized state.
        self._dof_limits_lo = self.dof_pos_limits[:, 0].tolist()
        self._dof_limits_hi = self.dof_pos_limits[:, 1].tolist()
        self._num_motions = self.sire_models[0].numMotions()
        self._motion_idx = np.array(
            [self._sire_dof_to_motion[name] for name in self.dof_names],
            dtype=np.int32,
        )
        self._motion_idx_inv = np.zeros(self._num_motions, dtype=np.int32)
        for dof_i, mot_i in enumerate(self._motion_idx):
            self._motion_idx_inv[mot_i] = dof_i

        # ── Pre-allocate tensors (reused in-place by _refresh_sim_tensors_sire) ──
        self.root_states = torch.zeros(self.num_envs, 13, dtype=torch.float, device=self.device)
        self.dof_pos = torch.zeros(self.num_envs, self.num_dof, dtype=torch.float, device=self.device)
        self.dof_vel = torch.zeros(self.num_envs, self.num_dof, dtype=torch.float, device=self.device)
        self.contact_forces = torch.zeros(self.num_envs, self.num_bodies, 3, dtype=torch.float, device=self.device)
        self.feet_pos_world = torch.zeros(self.num_envs, len(self.feet_indices), 3, dtype=torch.float, device=self.device)
        self.body_ground_contact = torch.zeros(self.num_envs, self.num_bodies, dtype=torch.bool, device=self.device)
        self.foot_ground_contact = torch.zeros(self.num_envs, len(self.feet_indices), dtype=torch.bool, device=self.device)
        self._sire_physics_failure_buf = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._sire_recovery_counts = deque()
        self._sire_recovery_window_total = 0
        self._refresh_sim_tensors_sire()   # fill with current Sire state

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
        self.command_targets = torch.zeros_like(self.commands)
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
        # ── Pre-convert to numpy (for vectorized PD math, matching MuJoCo) ──
        self._p_gains = self.p_gains.numpy().astype(np.float64)
        self._d_gains = self.d_gains.numpy().astype(np.float64)
        self._torque_limits = self.torque_limits.numpy().astype(np.float64)
        self._default_pos = self.default_dof_pos.squeeze(0).numpy().astype(np.float64)
        sire_batch_threads = int(getattr(self.cfg.sim, "sire_batch_threads", 0))
        self._sire_batch_stepper = sire.SireRLBatchStepper(
            self.sire_simulators,
            self._motion_idx,
            self.feet_indices_np,
            self.cfg.control.control_type,
            float(self.cfg.control.action_scale),
            self._p_gains,
            self._d_gains,
            self._default_pos,
            self._torque_limits,
            np.asarray(self._dof_limits_lo, dtype=np.float64),
            np.asarray(self._dof_limits_hi, dtype=np.float64),
            float(self.cfg.sim.dt),
            float(self.dt),
            sire_batch_threads,
        )
        if getattr(self.cfg.sim, "sire_diagnostics", False):
            self.setSireHistoryRecording(True)
        # Public spelling requested by the training CLI/config.  It reports
        # the effective count after clamping to num_envs.
        self.sireBatchThread = self._sire_batch_stepper.threadCount
        print(
            f"[Sire batch] envs={self.num_envs} threads={self.sireBatchThread} "
            f"persistent_workers={self._sire_batch_stepper.workerCount}",
            flush=True,
        )
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
        """Reset a single Sire environment after physics crash.

        Uses the existing Simulator (sim.reset()) rather than creating a new one
        from XML, to avoid leaking old C++ Simulator/Model/SimulationLoop objects
        whose pybind11 wrappers won't be freed until Python GC runs.
        """
        # ── Full simulator reset: model state, recorder, contacts, events, timer ──
        self.sire_simulators[env_idx].reset()
        self._sire_dt_actual.pop(env_idx, None)

        # ── Same init flow as a fresh episode ──
        env_ids = torch.tensor([env_idx], device=self.device)
        self._reset_dofs(env_ids)
        self._reset_root_states(env_ids)
        self.episode_length_buf[env_idx] = 0
        self.reset_buf[env_idx] = 1

    def _format_sire_state(self, env_idx: int) -> str:
        """Return a complete compact state snapshot for an error report."""
        model = self.sire_models[env_idx]
        loop = self.sire_sim_loops[env_idx]
        pq, vs = sire.getBasePqVs(model, 1)
        mp = np.asarray(sire.getMotionMps(model), dtype=np.float64)[
            self._motion_idx
        ].tolist()
        mv = np.asarray(sire.getMotionMvs(model), dtype=np.float64)[
            self._motion_idx
        ].tolist()
        actions = self.actions[env_idx].detach().cpu().tolist()
        return (
            f"env_id={env_idx} sim_time={loop.simTime():.17g} "
            f"pq={list(pq)} vs={list(vs)} mp={mp} mv={mv} actions={actions}"
        )

    def _format_sire_exception(self, env_idx: int, error: Exception) -> str:
        try:
            state = self._format_sire_state(env_idx)
        except Exception as snapshot_error:
            state = (
                f"env_id={env_idx} state_snapshot_error="
                f"{type(snapshot_error).__name__}: {snapshot_error}"
            )
        return (
            f"Sire environment failed: {type(error).__name__}: {error}\n{state}"
        )

    def _apply_sire_domain_randomization(self, sim, model):
        """Apply fixed per-environment dynamics randomization before init()."""
        rand_cfg = self.cfg.domain_rand

        nominal_friction = float(getattr(rand_cfg, "sire_nominal_friction", 0.6))
        friction = nominal_friction
        if getattr(rand_cfg, "randomize_friction", False):
            low, high = map(float, rand_cfg.friction_range)
            if low <= 0.0 or high < low:
                raise ValueError(f"Invalid friction_range: {rand_cfg.friction_range}")
            friction = float(np.random.uniform(low, high))

        material_pair = getattr(rand_cfg, "sire_material_pair", ["m1", "m1"])
        if len(material_pair) != 2:
            raise ValueError("domain_rand.sire_material_pair must contain two names")
        material_prop = (
            "{"
            f"k:{float(getattr(rand_cfg, 'sire_contact_k', 2.0e8)):.17g},"
            f"d:{float(getattr(rand_cfg, 'sire_contact_d', 5.0e4)):.17g},"
            f"cr:{float(getattr(rand_cfg, 'sire_contact_cr', 0.3)):.17g},"
            f"cof:{friction:.17g},"
            "threshold_velocity:"
            f"{float(getattr(rand_cfg, 'sire_threshold_velocity', 0.3)):.17g}"
            "}"
        )
        sim.physicsEngine().contactSolver().addMaterialPair(
            str(material_pair[0]), str(material_pair[1]), material_prop
        )

        base_part = model.link(1)
        base_iv = np.asarray(base_part.prtIv, dtype=np.float64).copy()
        nominal_mass = float(base_iv[0])
        mass_offset = 0.0
        if getattr(rand_cfg, "randomize_base_mass", False):
            low, high = map(float, rand_cfg.added_mass_range)
            if high < low:
                raise ValueError(
                    f"Invalid added_mass_range: {rand_cfg.added_mass_range}"
                )
            mass_offset = float(np.random.uniform(low, high))
            randomized_mass = nominal_mass + mass_offset
            if randomized_mass <= 0.0:
                raise ValueError(
                    "Randomized base mass must be positive: "
                    f"nominal={nominal_mass}, offset={mass_offset}"
                )
            # Aris stores first moments and inertia about the part origin.
            # Preserve the original COM and inertia about the COM while changing
            # mass, matching a rigid-body API whose mass is randomized separately.
            com = base_iv[1:4] / nominal_mass
            mass_delta = randomized_mass - nominal_mass
            cx, cy, cz = com
            base_iv[0] = randomized_mass
            base_iv[1:4] = randomized_mass * com
            base_iv[4] += mass_delta * (cy * cy + cz * cz)
            base_iv[5] += mass_delta * (cx * cx + cz * cz)
            base_iv[6] += mass_delta * (cx * cx + cy * cy)
            base_iv[7] -= mass_delta * cx * cy
            base_iv[8] -= mass_delta * cx * cz
            base_iv[9] -= mass_delta * cy * cz
            base_part.prtIv = base_iv.tolist()

        return friction, mass_offset, nominal_mass

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
        friction_coeffs = []
        base_mass_offsets = []
        nominal_base_masses = []
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

            friction, mass_offset, nominal_mass = (
                self._apply_sire_domain_randomization(sim, model)
            )
            friction_coeffs.append(friction)
            base_mass_offsets.append(mass_offset)
            nominal_base_masses.append(nominal_mass)
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

        self.friction_coeffs = torch.tensor(
            friction_coeffs, dtype=torch.float32, device=self.device
        ).unsqueeze(1)
        self.base_mass_offsets = torch.tensor(
            base_mass_offsets, dtype=torch.float32, device=self.device
        ).unsqueeze(1)
        self.nominal_base_masses = torch.tensor(
            nominal_base_masses, dtype=torch.float32, device=self.device
        ).unsqueeze(1)
        if getattr(self.cfg.sim, "sire_diagnostics", False):
            print(
                "[Sire domain randomization] "
                f"friction=[{self.friction_coeffs.min().item():.3f}, "
                f"{self.friction_coeffs.max().item():.3f}], "
                f"base_mass_offset=[{self.base_mass_offsets.min().item():.3f}, "
                f"{self.base_mass_offsets.max().item():.3f}]",
                flush=True,
            )

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
        # These values are from the Go2 MuJoCo model and match the actuator
        # ranges serialized in the Sire XML. The native batch stepper writes
        # them into every cloned actuator so both paths share the same guards.
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
        if getattr(self.cfg.sim, "sire_diagnostics", False):
            self._print_config_diagnostic(
                sire_part_names,
                feet_names_raw,
                penalized_names,
                termination_names,
            )
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
        """Assign terrain patches while exposing a local origin per simulator."""
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
            self._physics_origins = self.terrain_origins[
                self.terrain_levels, self.terrain_types
            ].clone()
            return

        # Sire environments are separate Simulator instances, not actors in a
        # shared world. They therefore all use the same local world origin.
        self.custom_origins = False
        self.env_origins = torch.zeros(self.cfg.env.num_envs, 3, device=self.device)
        self._physics_origins = torch.zeros_like(self.env_origins)
        self.terrain_levels = torch.zeros(
            self.cfg.env.num_envs, dtype=torch.long, device=self.device
        )

    def _terrain_out_of_bounds(self):
        """Return env mask outside its assigned finite terrain patch."""
        outside = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        if self.terrain is None:
            return outside
        margin = float(getattr(self.cfg.terrain, "boundary_margin", 0.0))
        for env_id in range(self.num_envs):
            row = int(self.terrain_levels[env_id].item())
            col = int(self.terrain_types[env_id].item())
            patch = self.terrain.patch_map.get((row, col))
            if patch is None:
                raise RuntimeError(
                    f"missing terrain patch for env_id={env_id}, row={row}, col={col}"
                )
            x = self.root_states[env_id, 0] + self._physics_origins[env_id, 0]
            y = self.root_states[env_id, 1] + self._physics_origins[env_id, 1]
            outside[env_id] = (
                (x < patch.start_x + margin)
                | (x > patch.start_x + self.terrain.patch_length - margin)
                | (y < patch.start_y + margin)
                | (y > patch.start_y + self.terrain.patch_width - margin)
            )
        return outside

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
        world_points = world_points + self._physics_origins[env_ids_tensor].unsqueeze(1)

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

            # Convert queried physical heights back to this simulator's
            # local terrain frame (spawn height is local z=0).
            heights[e] -= self._physics_origins[env_ids_tensor[e], 2]

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

    def _reward_action_magnitude(self):
        """Penalize sustained large policy outputs, including constant saturation."""
        return torch.sum(torch.square(self.actions), dim=1)

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
        # soft_dof_pos_limit scales each joint's half-range around its midpoint.
        # At 0.9, the penalty starts after 90% of the center-to-hard-limit
        # distance, i.e. in the final 10% near either mechanical stop.
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

    def setSireHistoryRecording(self, enabled):
        self._sire_batch_stepper.setHistoryRecording(bool(enabled))

    def resetSireRecorders(self):
        """Clear rollout recordings without changing episode/model state."""
        self._sire_batch_stepper.resetRecorders()

    def reset_sire_recorders(self):
        return self.resetSireRecorders()

    def get_observations(self):
        return self.obs_buf

    def get_privileged_observations(self):
        return self.privileged_obs_buf

    # _refresh_sim_tensors removed — use _refresh_sim_tensors_sire instead

    # ------------------------------------------------------------------
    #  Post physics step (calls _refresh_sim_tensors_sire)
    # ------------------------------------------------------------------
    def _post_physics_step_sire(self, refresh_from_sire=True):
        self.extras = {}
        self.episode_length_buf += 1
        self.common_step_counter += 1
        if refresh_from_sire:
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

        # Never hide numerical failures from PPO.  Native stepping already
        # reports full state; this catches errors introduced by observation or
        # reward code and identifies the affected environments.
        invalid_obs = ~torch.isfinite(self.obs_buf).all(dim=1)
        if self.privileged_obs_buf is not None:
            invalid_obs |= ~torch.isfinite(self.privileged_obs_buf).all(dim=1)
        if invalid_obs.any():
            env_ids = invalid_obs.nonzero(as_tuple=False).flatten().tolist()
            states = [self._format_sire_state(eid) for eid in env_ids]
            raise RuntimeError(
                "non-finite Sire RL observation\n" + "\n".join(states)
            )

        self.last_actions[:] = self.actions[:]
        self.last_actions[env_ids] = 0.0
        self.last_dof_vel[:] = self.dof_vel[:]
        self.last_root_vel[:] = self.root_states[:, 7:13]
        self.last_feet_pos_world[:] = self.feet_pos_world[:]

        # ---- reward decomposition diagnostic (first 20 steps) ----
        if (
            getattr(self.cfg.sim, "sire_diagnostics", False)
            and self.common_step_counter <= 20
        ):
            r_track_lin = self._reward_tracking_lin_vel().mean().item()
            r_track_ang = self._reward_tracking_ang_vel().mean().item()
            r_orient = self._reward_orientation().mean().item()
            r_height = self._reward_base_height().mean().item()
            r_torque = self._reward_torques().mean().item()
            r_air = self._reward_feet_air_time().mean().item()
            r_collision = self._reward_collision().mean().item()
            r_lin_vel_z_raw = self._reward_lin_vel_z().mean().item()
            r_dof_vel_raw = self._reward_dof_vel().mean().item()
            r_dof_acc_raw = self._reward_dof_acc().mean().item()
            r_action_rate_raw = self._reward_action_rate().mean().item()
            r_ang_vel_xy_raw = self._reward_ang_vel_xy().mean().item()
            world_vx = self.root_states[:, 7].mean().item()
            world_vy = self.root_states[:, 8].mean().item()
            world_vz = self.root_states[:, 9].mean().item()
            base_vx = self.base_lin_vel[:, 0].mean().item()
            base_vy = self.base_lin_vel[:, 1].mean().item()
            base_vz = self.base_lin_vel[:, 2].mean().item()
            qx = self.base_quat[0, 0].item()
            qy = self.base_quat[0, 1].item()
            qz = self.base_quat[0, 2].item()
            qw = self.base_quat[0, 3].item()
            dt_val = self.dt
            cum_lin_vel_z = self.episode_sums.get("lin_vel_z",
                torch.zeros(1))[0].item()
            base_z = self.root_states[:, 2].mean().item()
            tilt = self.projected_gravity[:, :2].norm(dim=1).mean().item()
            foot_contact_frac = self.foot_ground_contact.float().mean().item()
            contact_nz = (self.contact_forces.abs().sum(dim=-1).sum(dim=-1) > 0).sum().item()
            force0 = self.contact_forces[0].reshape(-1, 3)
            max_force_index = torch.norm(force0, dim=1).argmax().item()
            first_force = (
                f"fx={force0[max_force_index, 0]:.1f},"
                f"fy={force0[max_force_index, 1]:.1f},"
                f"fz={force0[max_force_index, 2]:.1f}"
            )
            print(
                f"[Sire diag step {self.common_step_counter}] "
                f"rew_total={self.rew_buf.mean():.4f} dt={dt_val:.4f}\n"
                f"  raw_rew: lin_vel_z={r_lin_vel_z_raw:.2f} "
                f"dof_vel={r_dof_vel_raw:.2f} dof_acc={r_dof_acc_raw:.2f} "
                f"act_rate={r_action_rate_raw:.2f} ang_vel_xy={r_ang_vel_xy_raw:.2f}\n"
                f"  world_vel: vx={world_vx:.3f} vy={world_vy:.3f} vz={world_vz:.3f}\n"
                f"  base_vel:  vx={base_vx:.3f} vy={base_vy:.3f} vz={base_vz:.3f}\n"
                f"  quat[0]: qx={qx:.4f} qy={qy:.4f} qz={qz:.4f} qw={qw:.4f}\n"
                f"  base_z={base_z:.3f} tilt={tilt:.4f} foot_ct={foot_contact_frac:.2f} "
                f"nz_envs={contact_nz} force0={first_force}\n"
                f"  scaled: track_lin={r_track_lin:+.4f} track_ang={r_track_ang:+.4f} "
                f"air={r_air:+.4f} orient={r_orient:+.4f} height={r_height:+.4f} "
                f"torque={r_torque:+.4f} collision={r_collision:+.4f} "
                f"cum_lin_vel_z={cum_lin_vel_z:.2f}",
                flush=True,
            )
            # ---- Save recording at step 20 for debugging ----
            if self.common_step_counter == 20:
                import json, os, shutil
                sim0 = self.sire_simulators[0]
                m0 = self.sire_models[0]
                sl0 = self.sire_sim_loops[0]
                result = sl0.recordsToJson()
                display_init = sim0.displayInitJson()
                rec = {
                    "nlinks": int(m0.nbody),
                    "display_init": display_init,
                    "frames": result,
                }
                vis_dir = getattr(self, '_diag_vis_dir', None)
                if vis_dir:
                    out_dir = Path(vis_dir)
                else:
                    out_dir = Path(__file__).resolve().parent
                out_path = out_dir / "sire_first20_debug.json"
                with open(out_path, "w") as f:
                    json.dump(rec, f)
                # Also copy the generated XML for standalone repro
                xml_src = Path(self.generated_model_path)
                xml_dst = out_dir / "sire_first20_scene.xml"
                if xml_src.exists():
                    shutil.copy2(xml_src, xml_dst)
                    print(f"[Sire diag] Copied scene XML → {xml_dst}", flush=True)
                # Save initial state + actions for exact repro
                replay_path = out_dir / "sire_first20_replay.json"
                replay = {
                    'init_state': self._diag_init_state,
                    'actions': self._diag_actions,  # actions for steps 1..20
                    'dof_names': self.dof_names,
                }
                with open(replay_path, "w") as f:
                    json.dump(replay, f)
                print(f"[Sire diag] Saved replay data → {replay_path}  "
                      f"(init_state + {len(self._diag_actions)} actions)", flush=True)
                times = result.get("timeIndex", [])
                t_info = f"{times[0]:.3f}~{times[-1]:.3f}" if times else "empty"
                print(f"[Sire diag] Saved first 20 steps recording → {out_path}  "
                      f"(frames={len(times)}, time={t_info})", flush=True)
                # import sys; sys.exit(0)

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

        # ── Zero out pre-allocated tensors (in-place, no re-allocation) ──
        self.root_states.zero_()
        self.dof_pos.zero_()
        self.dof_vel.zero_()
        self.contact_forces.zero_()
        self.feet_pos_world.zero_()
        self.body_ground_contact.zero_()
        self.foot_ground_contact.zero_()

        for i in range(self.num_envs):
            m = self.sire_models[i]
            s = self.sire_sim_loops[i]

            try:
                # --- root (base link) --- batch read pq + vs in one C++ call ---
                pq, vs = sire.getBasePqVs(m, 1)  # (list[7], list[6])

                # Convert spatial velocity (at origin) to body-point velocity
                # so the bounds check is physically meaningful.
                # vs = [v_O, ω] can be huge from ω×p even when the body is
                # nearly stationary — use vp (body linear velocity) instead.
                vp = sire.vs2vp(vs, pq[:3])  # body linear velocity at pq[:3]

                # MuJoCo-style sanity check: if simulation has blown up
                # (e.g. robot penetrated deep into ground), mark env for
                # reset rather than letting garbage observations through.
                if (abs(pq[0]) > 100.0 or abs(pq[1]) > 100.0
                        or pq[2] < -5.0 or pq[2] > 50.0
                        or any(abs(v) > 100.0 for v in vp)
                        or any(abs(w) > 100.0 for w in vs[3:6])):
                    raise RuntimeError(
                        "physics state exceeded configured safety bounds: "
                        f"pq={pq} vp={vp} vs={vs}"
                    )

                self.root_states[i, :7] = torch.as_tensor(pq, dtype=torch.float)
                self.root_states[i, 7:10] = torch.as_tensor(vp, dtype=torch.float)
                self.root_states[i, 10:13] = torch.as_tensor(vs[3:6], dtype=torch.float)
                self.root_states[i, :3] -= self._physics_origins[i]

                # --- joints --- batch read + numpy reorder (like MuJoCo's qpos_adr_np) ---
                mps_all = np.array(sire.getMotionMps(m), dtype=np.float64)
                mvs_all = np.array(sire.getMotionMvs(m), dtype=np.float64)
                mps = mps_all[self._motion_idx]  # [12] in dof_names order
                mvs = mvs_all[self._motion_idx]
                # Joint limit enforcement: clamp position to [min, max]
                # and write back to simulation (MuJoCo does this internally).
                for j in range(self.num_actions):
                    lo, hi = self._dof_limits_lo[j], self._dof_limits_hi[j]
                    if mps[j] < lo:
                        mps[j] = lo
                        m.motion(self._motion_idx[j]).mp = lo
                        if mvs[j] < 0.0:
                            mvs[j] = 0.0
                            m.motion(self._motion_idx[j]).mv = 0.0
                    elif mps[j] > hi:
                        mps[j] = hi
                        m.motion(self._motion_idx[j]).mp = hi
                        if mvs[j] > 0.0:
                            mvs[j] = 0.0
                            m.motion(self._motion_idx[j]).mv = 0.0
                self.dof_pos[i] = torch.as_tensor(mps, dtype=torch.float)
                self.dof_vel[i] = torch.as_tensor(mvs, dtype=torch.float)

                # --- foot positions ----------------------------------------------
                for f_idx, pid in enumerate(self.feet_indices_np):
                    pm = m.link(int(pid)).getPm()
                    self.feet_pos_world[i, f_idx, 0] = (
                        pm[3] - self._physics_origins[i, 0]
                    )
                    self.feet_pos_world[i, f_idx, 1] = (
                        pm[7] - self._physics_origins[i, 1]
                    )
                    self.feet_pos_world[i, f_idx, 2] = (
                        pm[11] - self._physics_origins[i, 2]
                    )

                # --- contact forces & ground contact ---------------------------
                # Latest physical-substep force in N, matching the native path.
                try:
                    # Use part-ID-based results (maps geomId→partId internally,
                    # analogous to MuJoCo's mj_geom2body), so that pa/pb are
                    # valid part pool indices for contact_forces indexing.
                    cr = s.lastContactPairResultsWithPartIds()
                    for (pa, pb, fx, fy, fz, px, py, pz) in cr:
                        sfx, sfy, sfz = fx, fy, fz
                        # Accumulate forces (MuJoCo uses +=; we previously
                        # overwrote with =, losing multi-contact data).
                        self.contact_forces[i, pa, 0] -= sfx
                        self.contact_forces[i, pa, 1] -= sfy
                        self.contact_forces[i, pa, 2] -= sfz
                        self.contact_forces[i, pb, 0] += sfx
                        self.contact_forces[i, pb, 1] += sfy
                        self.contact_forces[i, pb, 2] += sfz
                        if pa == 0 and pb != 0:
                            self.body_ground_contact[i, pb] = True
                        if pb == 0 and pa != 0:
                            self.body_ground_contact[i, pa] = True
                except Exception as error:
                    raise RuntimeError(
                        "failed to read last contact-pair results"
                    ) from error

                if len(self.feet_indices_np) > 0:
                    self.foot_ground_contact[i] = self.body_ground_contact[
                        i, self.feet_indices
                    ]
            except Exception as error:
                raise RuntimeError(self._format_sire_exception(i, error)) from error

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
