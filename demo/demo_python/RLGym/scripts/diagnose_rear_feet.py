"""Headless diagnostic replay for slope-level checkpoints.

Loads a checkpoint, forces the env to a given scene curriculum level, runs the
policy for N steps, and dumps per-step data (base pose/vel, dof_pos, dof_vel,
torques, per-foot contact forces, feet_pos_world) into an npz.

Usage:
  diagnose_rear_feet.py --checkpoint <path> --scene-level N --out <npz> [--steps 400]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np
import torch

ROOT_DIR = Path(__file__).resolve().parents[2]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from RLGym.scene_curriculum import (
    apply_scene_curriculum_to_env_cfg,
    build_scene_curriculum_from_npz,
    resolve_scene_npz,
)
from RLGym.scripts.play import (
    DeploymentActor,
    _adapt_actor_obs,
    _apply_fixed_start_state,
    _configure_commands,
    _deployment_step,
    _load_deployment_actor,
)
from RLGym.utils.joint_order import JointOrderAdapter, infer_checkpoint_privileged_obs_dim
from RLGym.utils.math import quat_rotate_inverse, wrap_to_pi
from RLGym.utils.task_registry import make_env_cfg, make_env_from_cfg


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", type=Path, required=True)
    p.add_argument("--task", type=str, default="go2_threshold")
    p.add_argument("--scene-npz", type=str, default=None,
                   help="Override scene_source_npz (filename). If None, uses config value.")
    p.add_argument("--scene-level", type=int, required=True)
    p.add_argument("--steps", type=int, default=400)
    p.add_argument("--cmd-vx", type=float, default=0.35)
    p.add_argument("--target-heading-deg", type=float, default=0.0)
    p.add_argument("--heading-kp", type=float, default=0.5)
    p.add_argument("--activation", type=str, default="elu")
    p.add_argument("--out", type=Path, required=True)
    return p.parse_args()


def _build_env(args):
    ckpt_prio_dim = infer_checkpoint_privileged_obs_dim(args.checkpoint)
    env_cfg = make_env_cfg(args.task)
    if ckpt_prio_dim is not None:
        env_cfg.env.num_privileged_obs = int(ckpt_prio_dim)
    env_cfg.env.num_envs = 1
    env_cfg.commands.heading_command = True
    env_cfg.commands.curriculum = False
    env_cfg.commands.resampling_time = 1.0e9
    env_cfg.init_state.init_yaw_range = [0.0, 0.0]
    env_cfg.domain_rand.push_robots = False
    env_cfg.domain_rand.randomize_friction = False
    env_cfg.domain_rand.randomize_base_mass = False
    env_cfg.noise.add_noise = False

    if args.scene_npz is not None:
        env_cfg.terrain.scene_source_npz = args.scene_npz

    scene_npz = resolve_scene_npz(env_cfg.terrain.scene_source_npz)
    if scene_npz is None:
        raise RuntimeError(f"Scene npz not resolvable: {env_cfg.terrain.scene_source_npz}")
    _, artifacts, target_heights = build_scene_curriculum_from_npz(
        scene_npz,
        ROOT_DIR / "resources/scene_curricula",
        scene_name=scene_npz.stem,
    )
    scene_dir = artifacts.output_dir
    print(f"diag_scene source={scene_npz} dir={scene_dir} heights={target_heights}", flush=True)

    # Configure single-patch env at forced level
    env_cfg.terrain.num_rows = 1
    env_cfg.terrain.num_cols = 1
    env_cfg.terrain.max_init_terrain_level = 0
    env_cfg.terrain.curriculum = False

    apply_scene_curriculum_to_env_cfg(
        env_cfg,
        scene_dir,
        forced_level=args.scene_level,
        single_level_preview=True,
    )

    env = make_env_from_cfg(args.task, env_cfg, headless=True)
    if hasattr(env, "enable_forced_terrain_level"):
        env.enable_forced_terrain_level(int(args.scene_level))
    _apply_fixed_start_state(env)

    actual_h = None
    try:
        row = 0
        col = 0
        patch = env.terrain.patch_map.get((row, col))
        if patch is not None:
            actual_h = patch.metadata.get("threshold_height", None)
    except Exception:
        pass
    print(f"diag_env forced_level={args.scene_level} actual_threshold_height={actual_h}", flush=True)
    return env, ckpt_prio_dim


def _fake_args_for_commands(cmd_vx, target_heading_deg, heading_kp):
    """Build a minimal Namespace to drive play._configure_commands."""
    class A:
        pass
    a = A()
    a.cmd_vx = float(cmd_vx)
    a.cmd_vy = 0.0
    a.cmd_yaw = 0.0
    a.command_mode = "heading"
    a.target_heading_deg = float(target_heading_deg)
    a.target_x = None
    a.target_y = None
    a.heading_kp = float(heading_kp)
    return a


def main():
    args = parse_args()
    env, ckpt_prio_dim = _build_env(args)
    actor, _ = _load_deployment_actor(args.checkpoint, activation=args.activation)
    joint_order_adapter = JointOrderAdapter.from_privileged_obs_dim(
        env.dof_names,
        device=env.device,
        privileged_obs_dim=ckpt_prio_dim,
    )
    obs = joint_order_adapter.actor_obs_env_to_policy(env.get_observations(), env.num_actions)
    obs = _adapt_actor_obs(obs, actor.num_actor_obs)

    cmd_args = _fake_args_for_commands(args.cmd_vx, args.target_heading_deg, args.heading_kp)

    num_feet = int(env.feet_indices.numel())
    num_dofs = int(env.num_actions)

    records = {
        "step": [],
        "base_pos": [],          # (x, y, z)
        "base_quat": [],         # (w, x, y, z) actually MuJoCo order
        "base_lin_vel": [],
        "base_ang_vel": [],
        "projected_gravity": [],
        "dof_pos": [],           # (num_dofs,)
        "dof_vel": [],
        "torques": [],
        "actions": [],
        "foot_contact_force_mag": [],  # (num_feet,) world-frame magnitude
        "foot_contact_force_z": [],    # (num_feet,) z-component (normal)
        "feet_pos_world": [],          # (num_feet, 3)
        "done": [],
    }

    dof_names = env.dof_names
    foot_names = env.foot_names if hasattr(env, "foot_names") else [f"foot_{i}" for i in range(num_feet)]
    print(f"diag_names dof={dof_names} feet={foot_names}", flush=True)

    done_count = 0
    for step_idx in range(args.steps):
        _configure_commands(env, cmd_args)
        with torch.no_grad():
            actions = actor.act_inference(obs)
        env_actions = joint_order_adapter.actions_policy_to_env(actions)
        obs, done = _deployment_step(env, env_actions)
        obs = joint_order_adapter.actor_obs_env_to_policy(obs, env.num_actions)
        obs = _adapt_actor_obs(obs, actor.num_actor_obs)

        foot_f = env.contact_forces[0, env.feet_indices, :].detach().cpu().numpy()
        foot_mag = np.linalg.norm(foot_f, axis=-1)
        foot_z = foot_f[:, 2]

        records["step"].append(step_idx)
        records["base_pos"].append(env.root_states[0, :3].detach().cpu().numpy().copy())
        records["base_quat"].append(env.root_states[0, 3:7].detach().cpu().numpy().copy())
        records["base_lin_vel"].append(env.base_lin_vel[0, :3].detach().cpu().numpy().copy())
        records["base_ang_vel"].append(env.base_ang_vel[0, :3].detach().cpu().numpy().copy())
        records["projected_gravity"].append(env.projected_gravity[0, :3].detach().cpu().numpy().copy())
        records["dof_pos"].append(env.dof_pos[0].detach().cpu().numpy().copy())
        records["dof_vel"].append(env.dof_vel[0].detach().cpu().numpy().copy())
        records["torques"].append(env.torques[0].detach().cpu().numpy().copy())
        records["actions"].append(env.actions[0].detach().cpu().numpy().copy())
        records["foot_contact_force_mag"].append(foot_mag)
        records["foot_contact_force_z"].append(foot_z)
        records["feet_pos_world"].append(env.feet_pos_world[0].detach().cpu().numpy().copy())
        records["done"].append(int(bool(done[0].item())))

        if bool(done[0].item()):
            done_count += 1
            # Do not reset — leave to observe dangling behavior
            if done_count >= 3:
                break

    out = {k: np.array(v) for k, v in records.items()}
    out["dof_names"] = np.array(dof_names)
    out["foot_names"] = np.array(foot_names)
    out["checkpoint"] = np.array(str(args.checkpoint))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(args.out, **out)
    print(f"diag_saved out={args.out} steps={len(out['step'])} done_count={done_count}", flush=True)


if __name__ == "__main__":
    main()
