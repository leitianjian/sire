from __future__ import annotations

import torch

from SireRLGym.envs.base.legged_robot import LeggedRobot


class GO2Threshold(LeggedRobot):
    def enable_forced_terrain_level(self, level: int = 0):
        self.local_task_forced_terrain_level = True
        self.set_forced_terrain_level(level)

    def disable_forced_terrain_level(self):
        self.local_task_forced_terrain_level = False

    def set_forced_terrain_level(self, level: int):
        level_int = int(max(0, min(level, self.max_terrain_level - 1)))
        self.local_task_current_forced_terrain_level = level_int
        self.terrain_levels[:] = level_int
        if hasattr(self, 'terrain_origins'):
            self.env_origins[:] = self.terrain_origins[self.terrain_levels, self.terrain_types]
        self._refresh_local_task_targets(torch.arange(self.num_envs, device=self.device))

    def get_available_threshold_heights(self):
        heights = []
        for row in range(self.max_terrain_level):
            patch = self.terrain.patch_map.get((row, 0))
            if patch is None:
                heights.append(float(getattr(self.cfg.terrain, 'threshold_height', 0.0)))
            else:
                heights.append(float(patch.metadata.get('threshold_height', getattr(self.cfg.terrain, 'threshold_height', 0.0))))
        return heights

    def get_current_threshold_height(self) -> float:
        level = int(getattr(self, 'local_task_current_forced_terrain_level', int(self.terrain_levels[0].item())))
        heights = self.get_available_threshold_heights()
        return float(heights[min(level, len(heights) - 1)])

    def _update_terrain_curriculum(self, env_ids):
        if not self.init_done or not self.custom_origins or not hasattr(self, 'terrain_origins') or len(env_ids) == 0:
            return
        if getattr(self, 'local_task_forced_terrain_level', False):
            forced_level = int(getattr(self, 'local_task_current_forced_terrain_level', 0))
            self.terrain_levels[env_ids] = forced_level
            self.env_origins[env_ids] = self.terrain_origins[forced_level, self.terrain_types[env_ids]]
            return

        levels = self.terrain_levels[env_ids]
        move_up = self.local_task_success_buf[env_ids]
        move_down = (self.local_task_failure_buf[env_ids] | self.time_out_buf[env_ids]) & (
            self.local_task_best_rel_x[env_ids] < (self.local_task_threshold_start_rel_x[env_ids] - self.local_task_curriculum_down_margin)
        )

        levels = levels + move_up.long() - move_down.long()
        levels = torch.clamp(levels, min=0, max=self.max_terrain_level - 1)
        self.terrain_levels[env_ids] = levels
        self.env_origins[env_ids] = self.terrain_origins[levels, self.terrain_types[env_ids]]

    def _post_physics_step_tasks(self):
        self._update_local_task_state()
        self.local_task_cmd_vx_max[:] = torch.maximum(self.local_task_cmd_vx_max, self.commands[:, 0])
        self._update_reward_debug_info()

    def _update_task_termination(self):
        self.reset_buf |= self.local_task_success_buf
        self.reset_buf |= self.local_task_failure_buf

    def _update_task_episode_extras(self, env_ids):
        ratio = self.local_task_best_rel_x[env_ids] / torch.clamp(self.local_task_success_rel_x[env_ids], min=1e-6)
        valid_mask = self.local_task_cmd_vx_max[env_ids] >= self.local_task_promotion_vx_min
        valid_ids = env_ids[valid_mask]
        valid_count = int(valid_ids.numel())
        if valid_count > 0:
            self.extras['episode']['episode_count'] = float(valid_count)
            self.extras['episode']['task_success'] = torch.mean(self.local_task_success_buf[valid_ids].float())
            self.extras['episode']['task_failure'] = torch.mean(self.local_task_failure_buf[valid_ids].float())
            self.extras['episode']['task_passed_threshold'] = torch.mean(self.local_task_passed_threshold_buf[valid_ids].float())
        else:
            self.extras['episode']['episode_count'] = 0.0
            self.extras['episode']['task_success'] = torch.zeros((), device=self.device)
            self.extras['episode']['task_failure'] = torch.zeros((), device=self.device)
            self.extras['episode']['task_passed_threshold'] = torch.zeros((), device=self.device)
        self.extras['episode']['task_best_rel_x'] = torch.mean(self.local_task_best_rel_x[env_ids])
        self.extras['episode']['task_best_rel_x_ratio'] = torch.mean(torch.clamp(ratio, min=0.0, max=2.0))
        self.extras['episode']['task_threshold_height'] = torch.mean(self.local_task_threshold_height[env_ids])
        self.extras['episode']['task_clearance'] = torch.mean(self.local_task_max_clearance[env_ids])
        self.extras['episode']['task_front_clearance'] = torch.mean(self.local_task_max_front_clearance[env_ids])
        self.extras['episode']['task_rear_clearance'] = torch.mean(self.local_task_max_rear_clearance[env_ids])
        if self.local_task_rear_feet_passed_mask.numel() > 0:
            self.extras['episode']['task_rear_pass_ratio'] = torch.mean(
                self.local_task_rear_feet_passed_mask[env_ids].float().mean(dim=1)
            )

    def _reset_task_buffers(self, env_ids):
        self._reset_local_task_buffers(env_ids)

    def _init_buffers(self):
        super()._init_buffers()
        self._init_local_task_buffers()

    def _parse_cfg(self, cfg):
        super()._parse_cfg(cfg)
        self.local_task_enabled = bool(getattr(self.cfg.local_task, 'enabled', False))
        self.local_task_forced_terrain_level = bool(getattr(self.cfg.local_task, 'forced_terrain_level', False))
        self.local_task_current_forced_terrain_level = int(getattr(self.cfg.local_task, 'forced_terrain_level_index', 0))
        self.local_task_clearance_margin_before = float(getattr(self.cfg.local_task, 'clearance_margin_before', 0.18))
        self.local_task_clearance_margin_after = float(getattr(self.cfg.local_task, 'clearance_margin_after', 0.05))
        self.local_task_rear_clearance_margin_before = float(getattr(self.cfg.local_task, 'rear_clearance_margin_before', 0.05))
        self.local_task_rear_clearance_margin_after = float(getattr(self.cfg.local_task, 'rear_clearance_margin_after', 0.22))
        self.local_task_clearance_target_margin = float(getattr(self.cfg.local_task, 'clearance_target_margin', 0.02))
        self.local_task_curriculum_down_margin = float(getattr(self.cfg.local_task, 'curriculum_down_margin', 0.05))
        self.local_task_rear_pass_margin = float(getattr(self.cfg.local_task, 'rear_pass_margin', 0.02))
        self.local_task_rear_stuck_height_margin = float(getattr(self.cfg.local_task, 'rear_stuck_height_margin', 0.01))
        self.local_task_stumble_recovery_steps = int(getattr(self.cfg.local_task, 'stumble_recovery_steps', 5))
        self.local_task_success_hold_steps_required = int(getattr(self.cfg.local_task, 'success_hold_steps', 5))
        self.local_task_success_tilt_threshold = float(getattr(self.cfg.local_task, 'success_tilt_threshold', 0.55))
        self.local_task_success_base_contact_force_threshold = float(
            getattr(self.cfg.local_task, 'success_base_contact_force_threshold', 1.0)
        )
        self.local_task_promotion_vx_min = float(
            getattr(self.cfg.local_task, 'promotion_vx_min', 0.2)
        )

    def _reward_task_forward_progress(self):
        return torch.clamp(self.local_task_delta_x, min=0.0) / max(self.dt, 1e-6)

    def _reward_task_front_feet_clearance(self):
        return self.local_task_front_feet_clearance_reward

    def _reward_task_rear_feet_clearance(self):
        return self.local_task_rear_feet_clearance_reward

    def _reward_task_stumble_recovery(self):
        return self.local_task_stumble_recovery_reward

    def _reward_task_all_feet_stumble(self):
        return self.local_task_all_feet_stumble_penalty

    def _reward_task_feet_contact_forces(self):
        return self.local_task_feet_contact_forces_penalty

    def _reward_task_rear_feet_pass_edge(self):
        return self.local_task_rear_feet_pass_edge_reward

    def _reward_task_rear_feet_stuck(self):
        return self.local_task_rear_feet_stuck_penalty

    def _reward_task_stall(self):
        return self.local_task_stall_penalty

    def _reward_task_passed_threshold(self):
        return self.local_task_new_passed_threshold.float()

    def _reward_task_success(self):
        return self.local_task_new_success.float()

    def _init_local_task_buffers(self):
        front_foot_ids = [i for i, name in enumerate(self.foot_names) if name.startswith('fl') or name.startswith('fr')]
        rear_foot_ids = [i for i, name in enumerate(self.foot_names) if name.startswith('rl') or name.startswith('rr')]
        self.local_task_front_foot_indices = torch.tensor(front_foot_ids, dtype=torch.long, device=self.device)
        self.local_task_rear_foot_indices = torch.tensor(rear_foot_ids, dtype=torch.long, device=self.device)

        self.local_task_success_buf = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.local_task_failure_buf = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.local_task_passed_threshold_buf = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.local_task_new_passed_threshold = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.local_task_new_success = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.local_task_prev_rel_x = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_delta_x = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_best_rel_x = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_stall_steps = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self.local_task_success_rel_x = torch.full((self.num_envs,), float('inf'), dtype=torch.float, device=self.device)
        self.local_task_threshold_start_rel_x = torch.full((self.num_envs,), float('inf'), dtype=torch.float, device=self.device)
        self.local_task_threshold_end_rel_x = torch.full((self.num_envs,), float('inf'), dtype=torch.float, device=self.device)
        self.local_task_threshold_height = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_corridor_center_y = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_clearance = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_max_clearance = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_front_clearance = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_max_front_clearance = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_rear_clearance = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_max_rear_clearance = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_front_feet_clearance_reward = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_rear_feet_clearance_reward = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_stumble_recovery_reward = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_all_feet_stumble_penalty = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_feet_contact_forces_penalty = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_rear_feet_pass_edge_reward = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_rear_feet_stuck_penalty = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_stall_penalty = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_obstacle_zone = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.local_task_success_hold_steps = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self.local_task_success_pose_valid = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.local_task_base_contact_force = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_tilt_measure = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_stumble_event = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_feet_contact_force_excess = torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
        self.local_task_foot_stumble_cooldown = torch.zeros(
            self.num_envs, len(self.feet_indices), dtype=torch.long, device=self.device
        )
        self.local_task_rear_feet_passed_mask = torch.zeros(
            self.num_envs, len(self.local_task_rear_foot_indices), dtype=torch.bool, device=self.device
        )
        self.local_task_cmd_vx_max = torch.full((self.num_envs,), -1e9, dtype=torch.float, device=self.device)
        self.reward_debug_info = {}
        self._refresh_local_task_targets(torch.arange(self.num_envs, device=self.device))

    def _refresh_local_task_targets(self, env_ids):
        if not hasattr(self, 'terrain') or self.terrain is None:
            return
        success_margin = float(getattr(self.cfg.local_task, 'success_x_margin', 0.5))
        for env_id in env_ids.tolist():
            row = int(self.terrain_levels[env_id].item())
            col = int(self.terrain_types[env_id].item()) if hasattr(self, 'terrain_types') else 0
            patch = self.terrain.patch_map.get((row, col))
            if patch is None:
                continue
            meta = patch.metadata
            self.local_task_threshold_start_rel_x[env_id] = float(meta.get('threshold_start_x', patch.spawn_x) - patch.spawn_x)
            self.local_task_threshold_end_rel_x[env_id] = float(meta.get('threshold_end_x', patch.spawn_x) - patch.spawn_x)
            self.local_task_success_rel_x[env_id] = float(meta.get('threshold_end_x', patch.spawn_x) + success_margin - patch.spawn_x)
            self.local_task_threshold_height[env_id] = float(meta.get('threshold_height', getattr(self.cfg.terrain, 'threshold_height', 0.0)))
            self.local_task_corridor_center_y[env_id] = float(meta.get('corridor_center_y', patch.spawn_y))

    def _reset_local_task_buffers(self, env_ids):
        self._refresh_local_task_targets(env_ids)
        rel_x = self.root_states[env_ids, 0] - self.env_origins[env_ids, 0]
        self.local_task_success_buf[env_ids] = False
        self.local_task_failure_buf[env_ids] = False
        self.local_task_passed_threshold_buf[env_ids] = False
        self.local_task_new_passed_threshold[env_ids] = False
        self.local_task_new_success[env_ids] = False
        self.local_task_prev_rel_x[env_ids] = rel_x
        self.local_task_delta_x[env_ids] = 0.0
        self.local_task_best_rel_x[env_ids] = rel_x
        self.local_task_stall_steps[env_ids] = 0
        self.local_task_clearance[env_ids] = 0.0
        self.local_task_max_clearance[env_ids] = 0.0
        self.local_task_front_clearance[env_ids] = 0.0
        self.local_task_max_front_clearance[env_ids] = 0.0
        self.local_task_rear_clearance[env_ids] = 0.0
        self.local_task_max_rear_clearance[env_ids] = 0.0
        self.local_task_front_feet_clearance_reward[env_ids] = 0.0
        self.local_task_rear_feet_clearance_reward[env_ids] = 0.0
        self.local_task_stumble_recovery_reward[env_ids] = 0.0
        self.local_task_all_feet_stumble_penalty[env_ids] = 0.0
        self.local_task_feet_contact_forces_penalty[env_ids] = 0.0
        self.local_task_rear_feet_pass_edge_reward[env_ids] = 0.0
        self.local_task_rear_feet_stuck_penalty[env_ids] = 0.0
        self.local_task_stall_penalty[env_ids] = 0.0
        self.local_task_obstacle_zone[env_ids] = False
        self.local_task_success_hold_steps[env_ids] = 0
        self.local_task_success_pose_valid[env_ids] = False
        self.local_task_base_contact_force[env_ids] = 0.0
        self.local_task_tilt_measure[env_ids] = 0.0
        self.local_task_stumble_event[env_ids] = 0.0
        self.local_task_feet_contact_force_excess[env_ids] = 0.0
        self.local_task_foot_stumble_cooldown[env_ids] = 0
        self.local_task_rear_feet_passed_mask[env_ids] = False
        self.local_task_cmd_vx_max[env_ids] = -1e9

    def _update_local_task_state(self):
        rel_x = self.root_states[:, 0] - self.env_origins[:, 0]
        rel_y = self.root_states[:, 1] - self.local_task_corridor_center_y
        self.local_task_delta_x[:] = rel_x - self.local_task_prev_rel_x
        self.local_task_prev_rel_x[:] = rel_x
        self.local_task_new_passed_threshold[:] = False
        self.local_task_new_success[:] = False

        progress_tol = float(getattr(self.cfg.local_task, 'progress_tolerance', 0.01))
        improved = rel_x > (self.local_task_best_rel_x + progress_tol)
        self.local_task_best_rel_x = torch.maximum(self.local_task_best_rel_x, rel_x)
        self.local_task_stall_steps[improved] = 0
        self.local_task_stall_steps[~improved] += 1

        passed_threshold = rel_x > (self.local_task_threshold_end_rel_x + 0.03)
        base_contact_force = torch.norm(self.contact_forces[:, self.termination_contact_indices, :], dim=-1)
        self.local_task_base_contact_force[:] = torch.max(base_contact_force, dim=1).values
        self.local_task_tilt_measure[:] = torch.norm(self.projected_gravity[:, :2], dim=1)
        self.local_task_success_pose_valid[:] = (
            (self.local_task_tilt_measure <= self.local_task_success_tilt_threshold)
            & (self.local_task_base_contact_force <= self.local_task_success_base_contact_force_threshold)
        )
        success_candidate = (rel_x > self.local_task_success_rel_x) & self.local_task_success_pose_valid
        self.local_task_success_hold_steps[success_candidate] += 1
        self.local_task_success_hold_steps[~success_candidate] = 0
        success = self.local_task_success_hold_steps >= self.local_task_success_hold_steps_required

        self.local_task_new_passed_threshold[:] = (~self.local_task_passed_threshold_buf) & passed_threshold
        self.local_task_new_success[:] = (~self.local_task_success_buf) & success
        self.local_task_passed_threshold_buf |= passed_threshold
        self.local_task_success_buf |= success

        clearance_zone_start = torch.clamp(self.local_task_threshold_start_rel_x - self.local_task_clearance_margin_before, min=0.0)
        clearance_zone_end = self.local_task_success_rel_x + self.local_task_rear_clearance_margin_after
        self.local_task_obstacle_zone[:] = (rel_x >= clearance_zone_start) & (rel_x <= clearance_zone_end)
        self._update_foot_task_rewards()

        lateral_limit = float(getattr(self.cfg.local_task, 'lateral_error_threshold', 0.45))
        stall_limit = int(getattr(self.cfg.local_task, 'stall_max_steps', 75))
        self.local_task_failure_buf |= torch.abs(rel_y) > lateral_limit
        stall_failure = (self.local_task_stall_steps > stall_limit) & self.local_task_obstacle_zone
        self.local_task_stall_penalty[:] = stall_failure.float()
        self.local_task_failure_buf |= stall_failure

    def _update_foot_task_rewards(self):
        self.local_task_clearance[:] = 0.0
        self.local_task_front_clearance[:] = 0.0
        self.local_task_rear_clearance[:] = 0.0
        self.local_task_front_feet_clearance_reward[:] = 0.0
        self.local_task_rear_feet_clearance_reward[:] = 0.0
        self.local_task_stumble_recovery_reward[:] = 0.0
        self.local_task_all_feet_stumble_penalty[:] = 0.0
        self.local_task_feet_contact_forces_penalty[:] = 0.0
        self.local_task_rear_feet_pass_edge_reward[:] = 0.0
        self.local_task_rear_feet_stuck_penalty[:] = 0.0
        self.local_task_stumble_event[:] = 0.0
        self.local_task_feet_contact_force_excess[:] = 0.0
        if self.feet_indices.numel() == 0:
            return

        foot_rel_x = self.feet_pos_world[:, :, 0] - self.env_origins[:, 0].unsqueeze(1)
        feet_pos_z = self.feet_pos_world[:, :, 2]
        feet_contact = self.foot_ground_contact
        threshold_height = self.local_task_threshold_height.unsqueeze(1)
        target_height = threshold_height + self.local_task_clearance_target_margin

        above_step = torch.clamp(feet_pos_z - threshold_height, min=0.0)
        self.local_task_clearance[:] = torch.max(above_step, dim=1).values
        self.local_task_max_clearance = torch.maximum(self.local_task_max_clearance, self.local_task_clearance)

        if self.local_task_front_foot_indices.numel() > 0:
            front_x = foot_rel_x[:, self.local_task_front_foot_indices]
            front_z = feet_pos_z[:, self.local_task_front_foot_indices]
            front_contact = feet_contact[:, self.local_task_front_foot_indices]
            front_window = (
                (front_x >= (self.local_task_threshold_start_rel_x - self.local_task_clearance_margin_before).unsqueeze(1))
                & (front_x <= (self.local_task_threshold_end_rel_x + self.local_task_clearance_margin_after).unsqueeze(1))
            )
            front_above = torch.clamp(front_z - threshold_height, min=0.0)
            self.local_task_front_clearance[:] = torch.max(front_above, dim=1).values
            self.local_task_max_front_clearance = torch.maximum(self.local_task_max_front_clearance, self.local_task_front_clearance)
            front_swing = (~front_contact) & front_window
            front_norm = torch.clamp((front_z - target_height) / 0.08, min=0.0, max=1.0) * front_swing.float()
            front_active = torch.sum(front_swing.float(), dim=1)
            self.local_task_front_feet_clearance_reward[:] = torch.where(
                front_active > 0,
                torch.sum(front_norm, dim=1) / torch.clamp(front_active, min=1.0),
                torch.zeros_like(front_active),
            )

        if self.local_task_rear_foot_indices.numel() > 0:
            rear_x = foot_rel_x[:, self.local_task_rear_foot_indices]
            rear_z = feet_pos_z[:, self.local_task_rear_foot_indices]
            rear_contact = feet_contact[:, self.local_task_rear_foot_indices]
            rear_window = (
                (rear_x >= (self.local_task_threshold_start_rel_x - self.local_task_rear_clearance_margin_before).unsqueeze(1))
                & (rear_x <= (self.local_task_success_rel_x + self.local_task_rear_clearance_margin_after).unsqueeze(1))
            )
            rear_above = torch.clamp(rear_z - threshold_height, min=0.0)
            self.local_task_rear_clearance[:] = torch.max(rear_above, dim=1).values
            self.local_task_max_rear_clearance = torch.maximum(self.local_task_max_rear_clearance, self.local_task_rear_clearance)
            rear_swing = (~rear_contact) & rear_window
            rear_norm = torch.clamp((rear_z - target_height) / 0.08, min=0.0, max=1.0) * rear_swing.float()
            rear_active = torch.sum(rear_swing.float(), dim=1)
            self.local_task_rear_feet_clearance_reward[:] = torch.where(
                rear_active > 0,
                torch.sum(rear_norm, dim=1) / torch.clamp(rear_active, min=1.0),
                torch.zeros_like(rear_active),
            )

            rear_passed_now = rear_x > (self.local_task_threshold_end_rel_x + self.local_task_rear_pass_margin).unsqueeze(1)
            new_rear_pass = rear_passed_now & (~self.local_task_rear_feet_passed_mask)
            self.local_task_rear_feet_passed_mask |= rear_passed_now
            self.local_task_rear_feet_pass_edge_reward[:] = (
                torch.sum(new_rear_pass.float(), dim=1) / max(1, self.local_task_rear_foot_indices.numel())
            )

            rear_edge_zone = (
                (rear_x >= (self.local_task_threshold_start_rel_x - self.local_task_rear_clearance_margin_before).unsqueeze(1))
                & (rear_x <= (self.local_task_threshold_end_rel_x + self.local_task_rear_pass_margin).unsqueeze(1))
            )
            rear_low = rear_z < (threshold_height + self.local_task_rear_stuck_height_margin)
            rear_stuck = rear_contact & rear_edge_zone & rear_low
            self.local_task_rear_feet_stuck_penalty[:] = torch.mean(rear_stuck.float(), dim=1)

        contact_force_norm = torch.norm(self.contact_forces[:, self.feet_indices, :], dim=2)
        contact_xy = torch.norm(self.contact_forces[:, self.feet_indices, :2], dim=2)
        contact_z = torch.abs(self.contact_forces[:, self.feet_indices, 2])
        all_window = (
            (foot_rel_x >= (self.local_task_threshold_start_rel_x - self.local_task_clearance_margin_before).unsqueeze(1))
            & (foot_rel_x <= (self.local_task_success_rel_x + self.local_task_rear_clearance_margin_after).unsqueeze(1))
        )
        stumble_mask = contact_xy > 5.0 * torch.clamp(contact_z, min=1e-6)
        self.local_task_foot_stumble_cooldown[:] = torch.clamp(self.local_task_foot_stumble_cooldown - 1, min=0)
        self.local_task_foot_stumble_cooldown[stumble_mask & all_window] = self.local_task_stumble_recovery_steps
        recovery_mask = self.local_task_foot_stumble_cooldown > 0
        recovery_lift = (
            torch.clamp((feet_pos_z - target_height) / 0.08, min=0.0, max=1.0)
            * (~feet_contact).float()
            * recovery_mask.float()
        )
        recovery_active = torch.sum(recovery_mask.float(), dim=1)
        self.local_task_stumble_recovery_reward[:] = torch.where(
            recovery_active > 0,
            torch.sum(recovery_lift, dim=1) / torch.clamp(recovery_active, min=1.0),
            torch.zeros_like(recovery_active),
        )

        stumble = torch.any(stumble_mask & all_window, dim=1).float()
        contact_force_excess = torch.sum(
            torch.clamp(contact_force_norm - self.cfg.rewards.max_contact_force, min=0.0) * all_window.float(),
            dim=1,
        )
        obstacle_mask = self.local_task_obstacle_zone.float()
        self.local_task_stumble_event[:] = stumble
        self.local_task_feet_contact_force_excess[:] = contact_force_excess
        self.local_task_all_feet_stumble_penalty[:] = stumble * obstacle_mask
        self.local_task_feet_contact_forces_penalty[:] = contact_force_excess * obstacle_mask

    def _update_reward_debug_info(self):
        rel_x = self.root_states[:, 0] - self.env_origins[:, 0]
        self.reward_debug_info = {
            'live_rel_x_mean': rel_x.mean().item(),
            'live_best_rel_x_mean': self.local_task_best_rel_x.mean().item(),
            'live_dist_to_step_mean': (self.local_task_threshold_start_rel_x - rel_x).mean().item(),
            'live_clearance_mean': self.local_task_clearance.mean().item(),
            'live_front_clearance_mean': self.local_task_front_clearance.mean().item(),
            'live_rear_clearance_mean': self.local_task_rear_clearance.mean().item(),
            'live_rear_pass_ratio_mean': self.local_task_rear_feet_passed_mask.float().mean().item()
            if self.local_task_rear_feet_passed_mask.numel() > 0
            else 0.0,
            'live_stumble_mean': self.local_task_all_feet_stumble_penalty.mean().item(),
            'live_stumble_event_mean': self.local_task_stumble_event.mean().item(),
            'live_stumble_recovery_mean': self.local_task_stumble_recovery_reward.mean().item(),
            'live_feet_contact_force_excess_mean': self.local_task_feet_contact_force_excess.mean().item(),
            'live_success_base_contact_force_mean': self.local_task_base_contact_force.mean().item(),
            'live_success_tilt_mean': self.local_task_tilt_measure.mean().item(),
            'live_success_rate': self.local_task_success_buf.float().mean().item(),
            'live_pass_rate': self.local_task_passed_threshold_buf.float().mean().item(),
            'live_stall_rate': self.local_task_stall_penalty.mean().item(),
        }
