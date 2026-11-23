from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import torch
import torch.nn as nn

ROOT_DIR = Path(__file__).resolve().parents[2]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from SireRLGym.utils.math import quat_apply, quat_rotate_inverse, wrap_to_pi
from SireRLGym.scene_curriculum import apply_scene_curriculum_to_env_cfg, build_scene_curriculum_from_npz, resolve_scene_npz
from SireRLGym.utils.joint_order import JointOrderAdapter, infer_checkpoint_privileged_obs_dim
from SireRLGym.utils.task_registry import make_env_cfg


def _activation_from_name(name: str) -> nn.Module:
    name = str(name).lower()
    if name == "elu":
        return nn.ELU()
    if name == "relu":
        return nn.ReLU()
    if name == "selu":
        return nn.SELU()
    if name == "lrelu":
        return nn.LeakyReLU()
    if name == "tanh":
        return nn.Tanh()
    if name == "sigmoid":
        return nn.Sigmoid()
    raise ValueError(f"Unsupported activation: {name}")


class DeploymentActor(nn.Module):
    def __init__(self, actor_state_dict: dict[str, torch.Tensor], activation: str):
        super().__init__()
        layer_indices = sorted(
            int(key.split(".")[1])
            for key in actor_state_dict.keys()
            if key.startswith("actor.") and key.endswith(".weight")
        )
        if not layer_indices:
            raise ValueError("Checkpoint does not contain actor linear layers.")

        layers: list[nn.Module] = []
        act_name = activation
        for pos, layer_idx in enumerate(layer_indices):
            weight = actor_state_dict[f"actor.{layer_idx}.weight"]
            out_dim, in_dim = weight.shape
            linear = nn.Linear(in_dim, out_dim)
            layers.append(linear)
            if pos != len(layer_indices) - 1:
                layers.append(_activation_from_name(act_name))

        self.actor = nn.Sequential(*layers)
        self.load_state_dict(actor_state_dict, strict=True)

    @property
    def num_actor_obs(self) -> int:
        first_linear = self.actor[0]
        assert isinstance(first_linear, nn.Linear)
        return int(first_linear.in_features)

    @property
    def num_actions(self) -> int:
        last_linear = self.actor[-1]
        assert isinstance(last_linear, nn.Linear)
        return int(last_linear.out_features)

    def act_inference(self, obs: torch.Tensor) -> torch.Tensor:
        return self.actor(obs)


def _load_deployment_actor(checkpoint_path: Path, activation: str) -> tuple[DeploymentActor, list[tuple[str, str]]]:
    ckpt = torch.load(checkpoint_path, map_location="cpu")
    model_state = ckpt["model_state_dict"]
    actor_state = {}
    skipped = []
    for key, value in model_state.items():
        if key.startswith("actor."):
            actor_state[key] = value
        else:
            skipped.append((key, "non-actor parameter ignored"))
    actor = DeploymentActor(actor_state, activation=activation).to("cpu")
    actor.eval()
    return actor, skipped


def _adapt_actor_obs(obs: torch.Tensor, actor_obs_dim: int) -> torch.Tensor:
    if obs.shape[1] < actor_obs_dim:
        raise RuntimeError(
            f"Checkpoint expects actor obs dim {actor_obs_dim}, but env provides {obs.shape[1]}."
        )
    return obs[:, :actor_obs_dim]


def _center_camera_mujoco(viewer, env, distance=3.0, elevation=-20.0, azimuth=135.0):
    base_pos = env.root_states[0, :3].detach().cpu().numpy()
    viewer.cam.lookat[:] = base_pos
    viewer.cam.lookat[2] += 0.2
    viewer.cam.distance = float(distance)
    viewer.cam.elevation = float(elevation)
    viewer.cam.azimuth = float(azimuth)


def _current_heading(env) -> torch.Tensor:
    world_forward = quat_apply(env.root_states[:, 3:7], env.forward_vec)
    return torch.atan2(world_forward[:, 1], world_forward[:, 0])


def _apply_fixed_start_state_mujoco(env):
    import mujoco
    env_ids = torch.arange(env.num_envs, device=env.device)
    env.dof_pos[env_ids] = env.default_dof_pos
    env.dof_vel[env_ids] = 0.0
    env.root_states[env_ids] = env.base_init_state
    env.root_states[env_ids, :3] += env.env_origins[env_ids]
    env.root_states[env_ids, 7:13] = 0.0
    env.actions[env_ids] = 0.0
    env.commands[env_ids] = 0.0
    env.last_actions[env_ids] = 0.0
    env.last_dof_vel[env_ids] = 0.0
    env.last_root_vel[env_ids] = 0.0

    for eid in env_ids.tolist():
        d = env.datas[eid]
        root = env.root_states[eid]
        d.qpos[env.qpos_adr_np] = env.dof_pos[eid].cpu().numpy()
        d.qvel[env.qvel_adr_np] = 0.0
        d.qpos[env.root_qpos_adr_np : env.root_qpos_adr_np + 3] = root[:3].cpu().numpy()
        d.qpos[env.root_qpos_adr_np + 3 : env.root_qpos_adr_np + 7] = root[3:7].cpu().numpy()
        d.qvel[env.root_qvel_adr_np : env.root_qvel_adr_np + 6] = root[7:13].cpu().numpy()
        mujoco.mj_forward(env.models[eid], d)

    env._refresh_sim_tensors()
    env.base_quat[:] = env.root_states[:, 3:7]
    env.base_lin_vel[:] = quat_rotate_inverse(env.base_quat, env.root_states[:, 7:10])
    env.base_ang_vel[:] = quat_rotate_inverse(env.base_quat, env.root_states[:, 10:13])
    env.projected_gravity[:] = quat_rotate_inverse(env.base_quat, env.gravity_vec)
    if env.cfg.terrain.measure_heights:
        env.measured_heights = env._get_heights()
    env.compute_observations()


def _apply_fixed_start_state_sire(env):
    """Set the Sire rollout to the nominal pose with zero velocity."""
    import sire

    env_ids = torch.arange(env.num_envs, device=env.device)
    env.reset_idx(env_ids)

    env.dof_pos[:] = env.default_dof_pos
    env.dof_vel.zero_()
    env.root_states[:] = env.base_init_state
    env.root_states[:, 7:13] = 0.0
    env.actions.zero_()
    env.last_actions.zero_()
    env.last_dof_vel.zero_()
    env.last_root_vel.zero_()

    for eid in env_ids.tolist():
        model = env.sire_models[eid]
        mps = [0.0] * env._num_motions
        for dof_idx, motion_idx in enumerate(env._motion_idx):
            mps[int(motion_idx)] = float(env.default_dof_pos[0, dof_idx])
        sire.setMotionMps(model, mps)
        sire.setMotionMvs(model, [0.0] * env._num_motions)

        physical_pq = env.base_init_state[:7].clone()
        physical_pq[:3] += env._physics_origins[eid]
        position = physical_pq[:3].cpu().tolist()
        model.link(1).pq = physical_pq.cpu().tolist()
        model.link(1).vs = sire.vp2vs(position, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        model.forwardKinematics()
        model.forwardKinematicsVel()

    env._refresh_sim_tensors_sire()
    env.base_quat[:] = env.root_states[:, 3:7]
    env.base_lin_vel[:] = quat_rotate_inverse(env.base_quat, env.root_states[:, 7:10])
    env.base_ang_vel[:] = quat_rotate_inverse(env.base_quat, env.root_states[:, 10:13])
    env.projected_gravity[:] = quat_rotate_inverse(env.base_quat, env.gravity_vec)
    if env.cfg.terrain.measure_heights:
        env.measured_heights = env._get_heights()
    env.episode_length_buf.zero_()
    env.reset_buf.zero_()
    env.time_out_buf.zero_()
    env.compute_observations()


def _configure_commands(env, args):
    command = torch.tensor(
        [float(args.cmd_vx), float(args.cmd_vy), float(args.cmd_yaw)],
        dtype=env.commands.dtype,
        device=env.device,
    )
    env.commands[:, :3] = command
    if hasattr(env, "command_targets"):
        env.command_targets[:, :3] = command
    if env.commands.shape[1] > 3:
        if args.command_mode == "heading":
            if args.target_x is not None and args.target_y is not None:
                dx = float(args.target_x) - env.root_states[:, 0]
                dy = float(args.target_y) - env.root_states[:, 1]
                target_heading = torch.atan2(dy, dx)
            else:
                target_heading = torch.full_like(env.commands[:, 0], float(args.target_heading_deg) * torch.pi / 180.0)
            heading_error = wrap_to_pi(target_heading - _current_heading(env))
            env.commands[:, 2] = torch.clamp(args.heading_kp * heading_error, -1.0, 1.0)
            env.commands[:, 3] = target_heading
            if hasattr(env, "command_targets"):
                env.command_targets[:, 3] = target_heading
        else:
            env.commands[:, 3] = _current_heading(env)
            if hasattr(env, "command_targets"):
                env.command_targets[:, 3] = env.commands[:, 3]


def _should_use_idle_zero_actions(env, args) -> bool:
    if args.idle_mode != "zero":
        return False
    cmd_mag = abs(float(args.cmd_vx)) + abs(float(args.cmd_vy)) + abs(float(args.cmd_yaw))
    if args.command_mode == "heading":
        heading_err = wrap_to_pi(env.commands[:, 3] - _current_heading(env))
        return (cmd_mag < 1e-6) and bool(torch.max(torch.abs(heading_err)).item() < 1e-3)
    return cmd_mag < 1e-6


def _deployment_step_mujoco(env, actions: torch.Tensor):
    import mujoco
    clip_actions = env.cfg.normalization.clip_actions
    env.actions = torch.clip(actions, -clip_actions, clip_actions).to(env.device)

    for _ in range(env.cfg.control.decimation):
        env.torques = env._compute_torques(env.actions).view(env.torques.shape)
        for i in range(env.num_envs):
            env.datas[i].ctrl[env.actuator_ids_np] = env.torques[i].cpu().numpy()
            mujoco.mj_step(env.model, env.datas[i])

    env._refresh_sim_tensors()
    env.base_quat[:] = env.root_states[:, 3:7]
    env.base_lin_vel[:] = quat_rotate_inverse(env.base_quat, env.root_states[:, 7:10])
    env.base_ang_vel[:] = quat_rotate_inverse(env.base_quat, env.root_states[:, 10:13])
    env.projected_gravity[:] = quat_rotate_inverse(env.base_quat, env.gravity_vec)
    if env.cfg.terrain.measure_heights:
        env.measured_heights = env._get_heights()
    env.compute_observations()

    clip_obs = env.cfg.normalization.clip_observations
    env.obs_buf = torch.clip(env.obs_buf, -clip_obs, clip_obs)
    if env.privileged_obs_buf is not None:
        env.privileged_obs_buf = torch.clip(env.privileged_obs_buf, -clip_obs, clip_obs)

    base_contacts = torch.norm(env.contact_forces[:, env.termination_contact_indices, :], dim=-1) > 1.0
    done = torch.any(base_contacts, dim=1)

    env.last_actions[:] = env.actions[:]
    env.last_dof_vel[:] = env.dof_vel[:]
    env.last_root_vel[:] = env.root_states[:, 7:13]
    return env.obs_buf, done


def _deployment_step_sire(env, actions: torch.Tensor):
    """Sire version: delegate to env.step() which handles all physics+obs."""
    obs, priv, rew, reset, extras = env.step(actions)
    done = reset.bool()
    return obs, done


def _resolve_scene_source_npz(scene_source_npz):
    return resolve_scene_npz(scene_source_npz)


def _configure_terrain(env_cfg, args):
    env_cfg.terrain.curriculum = False
    env_cfg.terrain.num_rows = 1
    env_cfg.terrain.num_cols = 1
    env_cfg.terrain.max_init_terrain_level = 0
    env_cfg.terrain.terrain_length = float(args.terrain_length)
    env_cfg.terrain.terrain_width = float(args.terrain_width)
    env_cfg.terrain.border_size = float(args.border_size)
    env_cfg.terrain.spawn_offset_x = float(args.spawn_offset_x)
    env_cfg.terrain.spawn_offset_y = float(args.spawn_offset_y)
    env_cfg.terrain.terrain_seed = int(args.terrain_seed)

    if args.scene_curriculum_dir:
        metadata = apply_scene_curriculum_to_env_cfg(
            env_cfg,
            args.scene_curriculum_dir,
            forced_level=args.scene_level,
            single_level_preview=True,
        )
        if args.scene_level is None:
            env_cfg.local_task.forced_terrain_level_index = 0
        return

    if args.terrain == "plane":
        env_cfg.terrain.mesh_type = "plane"
    else:
        env_cfg.terrain.mesh_type = "trimesh"
        env_cfg.terrain.terrain_type_mode = args.terrain
        if args.terrain == "slope" and args.slope_angle_deg is not None:
            env_cfg.terrain.slope_angle_override_deg = float(args.slope_angle_deg)
        if args.terrain == "heightfield" and args.height_amplitude is not None:
            env_cfg.terrain.heightfield_height_override = float(args.height_amplitude)
        if args.terrain == "threshold":
            env_cfg.terrain.threshold_height = float(args.threshold_height)
            env_cfg.terrain.threshold_height_levels = [float(args.threshold_height)]
            env_cfg.terrain.threshold_depth = float(args.threshold_depth)
            env_cfg.terrain.threshold_width = float(args.threshold_width)
            env_cfg.terrain.threshold_offset_x = float(args.threshold_offset_x)
            env_cfg.terrain.corridor_width = float(args.corridor_width)
            env_cfg.terrain.enable_corridor_walls = False
            env_cfg.terrain.corridor_wall_height = float(args.corridor_wall_height)
            env_cfg.terrain.corridor_wall_thickness = float(args.corridor_wall_thickness)
            env_cfg.terrain.corridor_margin = float(args.corridor_margin)
            env_cfg.terrain.spawn_rand_x_range = [0.0, 0.0]
            env_cfg.terrain.spawn_rand_y_range = [0.0, 0.0]


def _print_play_config(env, args):
    terrain_patch = None
    if getattr(env, "terrain", None) is not None:
        row = int(env.terrain_levels[0].item()) if hasattr(env, "terrain_levels") else 0
        col = int(env.terrain_types[0].item()) if hasattr(env, "terrain_types") else 0
        terrain_patch = env.terrain.patch_map.get((row, col))

    print(
        "play_config "
        f"checkpoint={args.checkpoint} "
        f"terrain={args.terrain} "
        f"mesh_type={env.cfg.terrain.mesh_type} "
        f"scene_curriculum_dir={getattr(env.cfg.terrain, 'scene_curriculum_dir', None)} "
        f"cmd_vx={args.cmd_vx:.3f} "
        f"cmd_vy={args.cmd_vy:.3f} "
        f"cmd_yaw={args.cmd_yaw:.3f} "
        f"command_mode={args.command_mode}"
    )
    print(
        "play_terrain "
        f"length={env.cfg.terrain.terrain_length:.3f} "
        f"width={env.cfg.terrain.terrain_width:.3f} "
        f"seed={env.cfg.terrain.terrain_seed} "
        f"slope_angle_deg={getattr(env.cfg.terrain, 'slope_angle_override_deg', None)} "
        f"height_amplitude={getattr(env.cfg.terrain, 'heightfield_height_override', None)} "
        f"threshold_height={getattr(env.cfg.terrain, 'threshold_height', None)} "
        f"threshold_depth={getattr(env.cfg.terrain, 'threshold_depth', None)} "
        f"threshold_width={getattr(env.cfg.terrain, 'threshold_width', None)} "
        f"corridor_width={getattr(env.cfg.terrain, 'corridor_width', None)}"
    )
    if args.command_mode == "heading":
        if args.target_x is not None and args.target_y is not None:
            print(f"play_heading target_point=({float(args.target_x):.3f},{float(args.target_y):.3f}) kp={float(args.heading_kp):.3f}")
        else:
            print(f"play_heading target_heading_deg={float(args.target_heading_deg):.3f} kp={float(args.heading_kp):.3f}")
    if terrain_patch is not None:
        print(
            "play_patch "
            f"row={terrain_patch.row} col={terrain_patch.col} "
            f"type={terrain_patch.terrain_type} "
            f"origin={env.env_origins[0].tolist()} "
            f"metadata={terrain_patch.metadata}"
        )


def _publish_sire_meshcat(env, args):
    """Publish the completed env-0 rollout and keep MeshCat alive."""
    import meshcat
    import sire

    resource_path = args.resource_path
    if resource_path is None:
        resource_path = ROOT_DIR / "dogRL"
    resource_path = Path(resource_path).expanduser().resolve()
    if not resource_path.is_dir():
        raise FileNotFoundError(
            f"MeshCat resource directory does not exist: {resource_path}"
        )

    loop = env.sire_sim_loops[0]
    model = env.sire_models[0]
    simulator = env.sire_simulators[0]
    records = loop.recordsToJson()
    frame_count = len(records.get("timeIndex", []))
    if frame_count == 0:
        raise RuntimeError("Sire recorder contains no frames for MeshCat")

    visualizer = meshcat.Visualizer()
    url = visualizer.url()
    sire.robotInit(
        model.nbody,
        str(resource_path),
        simulator.displayInitJson(),
        visualizer,
    )
    sire.animateRobotByRecords(
        model.nbody,
        records,
        int(args.meshcat_fps),
        visualizer,
    )
    print(
        f"meshcat_ready url={url} recorded_frames={frame_count} "
        f"playback_fps={int(args.meshcat_fps)}",
        flush=True,
    )
    print("MeshCat will stay alive until this play process is stopped.", flush=True)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


def parse_args():
    parser = argparse.ArgumentParser(description="Deployment-oriented sim2sim play script.")
    parser.add_argument("--play-mode", choices=["classic", "through"], default=None)
    parser.add_argument("--task", type=str, default="go2")
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=3000)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--meshcat",
        action="store_true",
        help="For the Sire engine, publish the rollout to MeshCat and keep it alive.",
    )
    parser.add_argument("--meshcat-fps", type=int, default=50)
    parser.add_argument("--resource-path", type=Path, default=None)
    parser.add_argument("--no-realtime", action="store_true")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--activation", type=str, default="elu")
    parser.add_argument("--terrain", choices=["plane", "slope", "heightfield", "threshold"], default="plane")
    parser.add_argument("--scene-curriculum-dir", type=Path, default=None)
    parser.add_argument("--scene-source-npz", type=str, default=None, help="Override terrain.scene_source_npz filename (e.g. box_old.npz, stairs_x.npz).")
    parser.add_argument("--scene-level", type=int, default=None)
    parser.add_argument("--terrain-seed", type=int, default=1234)
    parser.add_argument("--terrain-length", type=float, default=10.0)
    parser.add_argument("--terrain-width", type=float, default=10.0)
    parser.add_argument("--border-size", type=float, default=2.0)
    parser.add_argument("--spawn-offset-x", type=float, default=2.0)
    parser.add_argument("--spawn-offset-y", type=float, default=5.0)
    parser.add_argument("--slope-angle-deg", type=float, default=4.0)
    parser.add_argument("--height-amplitude", type=float, default=0.2)
    parser.add_argument("--threshold-height", type=float, default=0.10)
    parser.add_argument("--threshold-depth", type=float, default=0.2)
    parser.add_argument("--threshold-width", type=float, default=1.2)
    parser.add_argument("--threshold-offset-x", type=float, default=1.5)
    parser.add_argument("--corridor-width", type=float, default=1.10)
    parser.add_argument("--corridor-wall-height", type=float, default=0.35)
    parser.add_argument("--corridor-wall-thickness", type=float, default=0.08)
    parser.add_argument("--corridor-margin", type=float, default=0.02)
    parser.add_argument("--cmd-vx", type=float, default=0.0)
    parser.add_argument("--cmd-vy", type=float, default=0.0)
    parser.add_argument("--cmd-yaw", type=float, default=0.0)
    parser.add_argument("--command-mode", choices=["yaw_rate", "heading"], default="heading")
    parser.add_argument("--target-heading-deg", type=float, default=0.0)
    parser.add_argument("--target-x", type=float, default=None)
    parser.add_argument("--target-y", type=float, default=None)
    parser.add_argument("--heading-kp", type=float, default=0.5)
    parser.add_argument("--idle-mode", choices=["zero", "policy"], default="zero")
    parser.add_argument("--reset-on-done", action="store_true")
    parser.add_argument("--engine", choices=["mujoco", "sire"], default="sire",
                        help="Physics backend (default: sire)")
    return parser.parse_args()


def _apply_play_mode_defaults(args):
    play_mode = args.play_mode
    if play_mode is None:
        play_mode = "through" if args.task == "go2_threshold" else "classic"
    args.play_mode = play_mode

    if play_mode != "through":
        return

    if args.task == "go2":
        args.task = "go2_threshold"
    if args.terrain == "plane":
        args.terrain = "threshold"

    args.command_mode = "heading"
    args.cmd_vy = 0.0
    args.cmd_yaw = 0.0

    if args.terrain_length == 10.0:
        args.terrain_length = 5.0
    if args.terrain_width == 10.0:
        args.terrain_width = 2.0
    if args.border_size == 2.0:
        args.border_size = 0.5
    if args.spawn_offset_x == 2.0:
        args.spawn_offset_x = 0.8
    if args.spawn_offset_y == 5.0:
        args.spawn_offset_y = 1.0


def main():
    args = parse_args()
    _apply_play_mode_defaults(args)
    checkpoint_privileged_obs_dim = infer_checkpoint_privileged_obs_dim(args.checkpoint)
    env_cfg = make_env_cfg(args.task)
    if checkpoint_privileged_obs_dim is not None:
        env_cfg.env.num_privileged_obs = int(checkpoint_privileged_obs_dim)
    env_cfg.env.num_envs = 1
    env_cfg.commands.heading_command = args.command_mode == "heading"
    env_cfg.commands.curriculum = False
    env_cfg.commands.resampling_time = 1.0e9
    env_cfg.init_state.init_yaw_range = [0.0, 0.0]
    env_cfg.domain_rand.push_robots = False
    env_cfg.domain_rand.randomize_friction = False
    env_cfg.domain_rand.randomize_base_mass = False
    env_cfg.noise.add_noise = False
    if args.scene_source_npz is not None:
        env_cfg.terrain.scene_source_npz = args.scene_source_npz
    scene_curriculum_dir = args.scene_curriculum_dir
    auto_scene_source_npz = _resolve_scene_source_npz(getattr(env_cfg.terrain, "scene_source_npz", None))
    if scene_curriculum_dir is None and args.task == "go2_threshold" and auto_scene_source_npz is not None:
        generated_metadata, artifacts, target_heights = build_scene_curriculum_from_npz(
            auto_scene_source_npz,
            ROOT_DIR / "resources/scene_curricula",
            scene_name=auto_scene_source_npz.stem,
        )
        scene_curriculum_dir = artifacts.output_dir
        print(
            "play_scene_curriculum "
            f"source_npz={auto_scene_source_npz} "
            f"generated_dir={scene_curriculum_dir} "
            f"target_heights={','.join(f'{v:.4f}' for v in target_heights)}",
            flush=True,
        )
        args.scene_curriculum_dir = Path(scene_curriculum_dir)
    _configure_terrain(env_cfg, args)

    use_sire = (args.engine == "sire")
    if use_sire:
        from SireRLGym.envs.base.legged_robot_sire import LeggedRobotSire
        env = LeggedRobotSire(env_cfg, headless=args.headless)
    else:
        from SireRLGym.utils.task_registry import make_env_from_cfg
        env = make_env_from_cfg(args.task, env_cfg, headless=args.headless)
    if args.scene_curriculum_dir and hasattr(env, "enable_forced_terrain_level"):
        forced_level = args.scene_level
        if forced_level is None and getattr(env.cfg.terrain, "num_rows", 0) > 0:
            forced_level = int(env.cfg.terrain.num_rows) - 1
        if forced_level is not None:
            env.enable_forced_terrain_level(int(forced_level))
    if use_sire:
        _apply_fixed_start_state_sire(env)
        if args.meshcat:
            # Record only the policy rollout. The old per-environment recording
            # API was removed when SireRLBatchStepper became the active path.
            env.setSireHistoryRecording(True)
            env.resetSireRecorders()
    else:
        _apply_fixed_start_state_mujoco(env)
    _configure_commands(env, args)
    env.compute_observations()
    _print_play_config(env, args)

    actor, skipped = _load_deployment_actor(args.checkpoint, activation=args.activation)
    if skipped:
        print(f"play_load_info actor_only=true skipped_keys={len(skipped)}")

    joint_order_adapter = JointOrderAdapter.from_privileged_obs_dim(
        env.dof_names,
        device=env.device,
        privileged_obs_dim=checkpoint_privileged_obs_dim,
    )
    obs = joint_order_adapter.actor_obs_env_to_policy(env.get_observations(), env.num_actions)
    obs = _adapt_actor_obs(obs, actor.num_actor_obs)
    realtime = not args.no_realtime
    target_step_time = env.dt / max(args.speed, 1e-6)
    next_t = time.perf_counter()
    episode_step = 0

    def _run_loop(viewer=None):
        nonlocal obs, next_t, episode_step
        for step_idx in range(args.steps):
            _configure_commands(env, args)
            with torch.no_grad():
                if _should_use_idle_zero_actions(env, args):
                    actions = torch.zeros(env.num_envs, env.num_actions, dtype=obs.dtype, device=obs.device)
                else:
                    actions = actor.act_inference(obs)
            env_actions = joint_order_adapter.actions_policy_to_env(actions)
            if use_sire:
                obs, done = _deployment_step_sire(env, env_actions)
            else:
                obs, done = _deployment_step_mujoco(env, env_actions)
            obs = joint_order_adapter.actor_obs_env_to_policy(obs, env.num_actions)
            obs = _adapt_actor_obs(obs, actor.num_actor_obs)
            episode_step += 1

            if viewer is not None:
                if viewer.is_running():
                    viewer.sync()
                else:
                    break

            if step_idx % 200 == 0 or bool(done[0].item()):
                cmd = env.commands[0, :3].tolist()
                base_pos = env.root_states[0, :3].tolist()
                base_lin = env.base_lin_vel[0, :3].tolist()
                current_heading_deg = float(_current_heading(env)[0].item() * 180.0 / torch.pi)
                target_heading_deg = float(env.commands[0, 3].item() * 180.0 / torch.pi) if env.commands.shape[1] > 3 else 0.0
                heading_error_deg = target_heading_deg - current_heading_deg
                print(
                    "play_step "
                    f"step={step_idx} "
                    f"pos=({base_pos[0]:.3f},{base_pos[1]:.3f},{base_pos[2]:.3f}) "
                    f"lin_vel=({base_lin[0]:.3f},{base_lin[1]:.3f},{base_lin[2]:.3f}) "
                    f"cmd=({cmd[0]:.3f},{cmd[1]:.3f},{cmd[2]:.3f}) "
                    f"heading_deg={current_heading_deg:.2f} "
                    f"target_heading_deg={target_heading_deg:.2f} "
                    f"heading_error_deg={heading_error_deg:.2f} "
                    f"done={int(bool(done[0].item()))}"
                )

            if bool(done[0].item()):
                if args.reset_on_done:
                    if use_sire:
                        _apply_fixed_start_state_sire(env)
                    else:
                        _apply_fixed_start_state_mujoco(env)
                    obs = joint_order_adapter.actor_obs_env_to_policy(env.get_observations(), env.num_actions)
                    obs = _adapt_actor_obs(obs, actor.num_actor_obs)
                    episode_step = 0

            if realtime:
                next_t += target_step_time
                sleep_t = next_t - time.perf_counter()
                if sleep_t > 0:
                    time.sleep(sleep_t)

    if args.headless or use_sire:
        _run_loop(viewer=None)
        if args.meshcat:
            if not use_sire:
                raise ValueError("--meshcat currently requires --engine sire")
            _publish_sire_meshcat(env, args)
        return

    import mujoco
    import mujoco.viewer
    with mujoco.viewer.launch_passive(env.models[0], env.datas[0]) as viewer:
        _center_camera_mujoco(viewer, env)
        _run_loop(viewer=viewer)


if __name__ == "__main__":
    main()
