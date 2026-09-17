from RLGym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO


class GO2RoughCfg(LeggedRobotCfg):
    class terrain(LeggedRobotCfg.terrain):
        # Match unitreerobotics/unitree_rl_gym's current Go2 task.  Despite
        # the historical "Rough" class name, the upstream base task defaults
        # to a plane; rough terrain must be selected explicitly.
        mesh_type = 'plane'
        curriculum = False
        measure_heights = False
        num_rows = 1
        num_cols = 1
        max_init_terrain_level = 0
        terrain_length = 10.0
        terrain_width = 10.0
        border_size = 2.0
        terrain_type_mode = 'heightfield'  # 'slope' or 'heightfield'
        slope_range_deg = [3.0, 8.0]
        slope_angle_override_deg = 4.0
        heightfield_height_range = [0.1, 0.3]
        heightfield_height_override = 0.2
        heightfield_nrow = 257
        heightfield_ncol = 257
        heightfield_smooth_steps = 8
        heightfield_spawn_flat_radius = 3
        spawn_offset_x = 2.0
        spawn_offset_y = 5.0

    class init_state(LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 0.42]
        init_yaw_range = [0.0, 0.0]
        default_joint_angles = {
            'FL_hip_joint': 0.1,
            'FR_hip_joint': -0.1,
            'RL_hip_joint': 0.1,
            'RR_hip_joint': -0.1,
            'FL_thigh_joint': 0.8,
            'FR_thigh_joint': 0.8,
            'RL_thigh_joint': 1.0,
            'RR_thigh_joint': 1.0,
            'FL_calf_joint': -1.5,
            'FR_calf_joint': -1.5,
            'RL_calf_joint': -1.5,
            'RR_calf_joint': -1.5,
        }

    class commands(LeggedRobotCfg.commands):
        resampling_time = 5.0
        heading_command = False

    class env(LeggedRobotCfg.env):
        # Keep the policy deployable on the real Go2: base linear velocity is
        # not directly observable without an estimator, so it is excluded.
        num_observations = 45
        # The actor remains deployable at 45 dimensions; only the critic gets
        # simulator-only privileged state during training.
        num_privileged_obs = 235

    class domain_rand(LeggedRobotCfg.domain_rand):
        randomize_friction = False
        friction_range = [0.5, 1.25]
        randomize_base_mass = False
        added_mass_range = [-1.0, 1.0]
        push_robots = False
        push_interval_s = 15
        max_push_vel_xy = 1.0

    class noise(LeggedRobotCfg.noise):
        add_noise = False

    class control(LeggedRobotCfg.control):
        control_type = 'P'
        stiffness = {'joint': 20.0}
        damping = {'joint': 0.5}
        action_scale = 0.25
        # Use a 1 kHz physics loop and keep the policy/control loop at 50 Hz.
        decimation = 20

    class sim(LeggedRobotCfg.sim):
        dt = 0.001
        mujoco_batch_threads = 16

    class asset(LeggedRobotCfg.asset):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/go2/flat.xml'
        name = 'go2'
        foot_name = 'foot'
        penalize_contacts_on = ['thigh', 'calf']
        terminate_after_contacts_on = ['base']
        self_collisions = 1

    class rewards(LeggedRobotCfg.rewards):
        soft_dof_pos_limit = 1.0
        base_height_target = 0.29
        only_positive_rewards = False
        feet_air_time_threshold = 0.08
        feet_air_time_reward_cap = 0.20
        feet_slip_speed_sq_cap = 1.0

        class scales(LeggedRobotCfg.rewards.scales):
            feet_air_time = 3.0
            torques = -0.0001
            dof_pos_limits = -10.0
            action_rate = -0.01
            action_magnitude = -0.001
            orientation = -5.0
            base_height = -5.0
            dof_vel = 0.0
            stand_still = -1.0
            default_pos = -0.1
            hip_pos = -0.2
            feet_slip = -0.2


class GO2RoughCfgPPO(LeggedRobotCfgPPO):
    class algorithm(LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.01

    class runner(LeggedRobotCfgPPO.runner):
        run_name = ''
        experiment_name = 'rough_go2'
        num_steps_per_env = 120
        max_iterations = 1000
        save_interval = 50
