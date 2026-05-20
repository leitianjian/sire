from RLGym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO


class GO2RoughCfg(LeggedRobotCfg):
    class terrain(LeggedRobotCfg.terrain):
        mesh_type = 'trimesh'
        curriculum = False
        measure_heights = True
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
        pos = [0.0, 0.0, 0.34]
        init_yaw_range = [-3.1415926, 3.1415926]
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

    class env(LeggedRobotCfg.env):
        num_observations = 45
        num_privileged_obs = 235

    class domain_rand(LeggedRobotCfg.domain_rand):
        randomize_friction = True
        friction_range = [0.2, 2.5]
        randomize_base_mass = True
        added_mass_range = [-0.5, 1.5]
        push_robots = False
        push_interval_s = 5
        max_push_vel_xy = 0.5

    class control(LeggedRobotCfg.control):
        control_type = 'P'
        stiffness = {'joint': 25.0}
        damping = {'joint': 0.6}
        action_scale = 0.25
        decimation = 4

    class asset(LeggedRobotCfg.asset):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/go2/flat.xml'
        name = 'go2'
        foot_name = 'foot'
        penalize_contacts_on = ['thigh', 'calf']
        terminate_after_contacts_on = ['base']
        self_collisions = 1

    class rewards(LeggedRobotCfg.rewards):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.34

        class scales(LeggedRobotCfg.rewards.scales):
            torques = -0.0001
            dof_pos_limits = -10.0
            action_rate = -0.01
            orientation = -5.0
            base_height = -10.0
            dof_vel = -5.0e-4
            stand_still = -0.05
            hip_pos = -0.4


class GO2RoughCfgPPO(LeggedRobotCfgPPO):
    class algorithm(LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.01

    class runner(LeggedRobotCfgPPO.runner):
        run_name = ''
        experiment_name = 'rough_go2'
