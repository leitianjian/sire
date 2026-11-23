from SireRLGym.envs.go2.go2_config import GO2RoughCfg, GO2RoughCfgPPO


class GO2ThresholdCfg(GO2RoughCfg):
    class terrain(GO2RoughCfg.terrain):
        mesh_type = 'trimesh'
        terrain_type_mode = 'threshold'
        terrain_length = 5.0
        terrain_width = 2.0
        num_rows = 4
        num_cols = 1
        curriculum = True
        max_init_terrain_level = 1
        border_size = 0.5
        spawn_offset_x = 0.8
        spawn_offset_y = 1.0
        spawn_rand_x_range = [0.0, 0.0]
        spawn_rand_y_range = [0.0, 0.0]
        threshold_height = 0.10
        threshold_height_levels = [0.04, 0.06, 0.08, 0.10]
        threshold_depth = 0.20
        threshold_width = 0.9
        threshold_offset_x = 1.5
        corridor_width = 1.10
        corridor_wall_height = 0.35
        corridor_wall_thickness = 0.08
        corridor_margin = 0.02
        slope_angle_override_deg = None
        heightfield_height_override = None
        # Scene reconstruction npz to drive the curriculum. Specify only the filename;
        # it is resolved from resources/scene_reconstructions/ automatically.
        # Available: "box.npz", "slope_x.npz", "slope_y.npz", "stairs_x.npz", "stairs_y.npz".
        # Archived: "box_old.npz", "slope_y_old.npz", "stairs_y_old.npz".
        # Set to None to disable scene-based curriculum.
        scene_source_npz = "box.npz" # box.npz, slope_x.npz, slope_y.npz, stairs_x.npz, stairs_y.npz

    class init_state(GO2RoughCfg.init_state):
        init_yaw_range = [0.0, 0.0]

    class commands(GO2RoughCfg.commands):
        heading_command = True
        resampling_time = 8.0
        prob_zero_command = 0.05
        prob_negative_command = 0.05
        negative_lin_vel_x_range = [-0.3, -0.1]

        class ranges(GO2RoughCfg.commands.ranges):
            lin_vel_x = [0.2, 0.5]
            lin_vel_y = [0.0, 0.0]
            ang_vel_yaw = [0.0, 0.0]
            heading = [-0.3, 0.3]

    class env(GO2RoughCfg.env):
        num_envs = 256
        episode_length_s = 8.0

    class domain_rand(GO2RoughCfg.domain_rand):
        push_robots = True
        push_interval_s = 4
        max_push_vel_xy = 0.3
        randomize_friction = True
        friction_range = [0.5, 1.5]
        randomize_base_mass = True
        added_mass_range = [-0.5, 0.5]

    class rewards(GO2RoughCfg.rewards):
        base_height_target = 0.38
        only_positive_rewards = False

        class scales(GO2RoughCfg.rewards.scales):
            termination = -6.0
            tracking_lin_vel = 1.0
            tracking_ang_vel = 0.5
            lin_vel_z = -2.0
            ang_vel_xy = -0.05
            orientation = -1.0
            base_height = -1.0
            torques = -5.0e-5
            dof_vel = -2.0e-4
            action_rate = -0.01
            stand_still = -0.1
            feet_air_time = 0.05
            hip_pos = -0.05
            collision = -1.5
            task_forward_progress = 1.5
            task_front_feet_clearance = 0.5
            task_rear_feet_clearance = 1.0
            task_stumble_recovery = 0.5
            task_all_feet_stumble = -0.4
            task_feet_contact_forces = -0.001
            task_rear_feet_pass_edge = 2.5
            task_rear_feet_stuck = -1.0
            task_passed_threshold = 4.0
            task_stall = -2.0
            task_success = 6.0

    class local_task:
        enabled = True
        forced_terrain_level = False
        forced_terrain_level_index = 0
        success_x_margin = 0.25
        lateral_error_threshold = 0.65
        stall_max_steps = 35
        progress_tolerance = 0.005
        clearance_margin_before = 0.18
        clearance_margin_after = 0.05
        clearance_target_margin = 0.02
        rear_clearance_margin_before = 0.05
        rear_clearance_margin_after = 0.22
        rear_pass_margin = 0.02
        rear_stuck_height_margin = 0.01
        stumble_recovery_steps = 5
        curriculum_down_margin = 0.05
        success_hold_steps = 5
        success_tilt_threshold = 0.55
        success_base_contact_force_threshold = 1.0


class GO2ThresholdCfgPPO(GO2RoughCfgPPO):
    class algorithm(GO2RoughCfgPPO.algorithm):
        learning_rate = 1.0e-4
        entropy_coef = 0.003
        num_learning_epochs = 4
        num_mini_batches = 2

    class runner(GO2RoughCfgPPO.runner):
        experiment_name = 'threshold_go2'
        max_iterations = 128
        num_steps_per_env = 80
        save_interval = 16
        infinite_mode = False
        infinite_success_rate_threshold = 0.5
        infinite_min_episodes = 256
        infinite_promotion_window_episodes = 256
        infinite_max_stuck_iterations = 2000
        infinite_min_level_gap = 5.0e-3
        infinite_stats_filename = 'infinite_mode_stats.json'
        log_episode_keys = [
            'episode_count',
            'rew_tracking_lin_vel',
            'rew_tracking_ang_vel',
            'rew_lin_vel_z',
            'rew_stand_still',
            'rew_hip_pos',
            'rew_base_height',
            'rew_task_forward_progress',
            'rew_task_front_feet_clearance',
            'rew_task_rear_feet_clearance',
            'rew_task_stumble_recovery',
            'rew_task_all_feet_stumble',
            'rew_task_feet_contact_forces',
            'rew_task_rear_feet_pass_edge',
            'rew_task_rear_feet_stuck',
            'rew_task_passed_threshold',
            'rew_task_stall',
            'rew_task_success',
            'rew_termination',
            'task_best_rel_x',
            'task_best_rel_x_ratio',
            'task_threshold_height',
            'task_clearance',
            'task_front_clearance',
            'task_rear_clearance',
            'task_rear_pass_ratio',
            'task_success',
            'task_failure',
            'task_passed_threshold',
            'terrain_level',
        ]
