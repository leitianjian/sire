"""Compare one trained policy under several Sire physics time steps.

The policy period stays fixed while ``sim.dt`` and control decimation change.
Evaluation is deterministic: flat ground, fixed initial state and commands,
with observation noise and domain randomization disabled.
"""

from __future__ import annotations

import argparse
import csv
import gc
import json
import math
import re
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np
import torch

ROOT_DIR = Path(__file__).resolve().parents[2]
SCRIPT_DIR = Path(__file__).resolve().parent
for path in (ROOT_DIR, SCRIPT_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from play import (  # noqa: E402
    _adapt_actor_obs,
    _apply_fixed_start_state_sire,
    _load_deployment_actor,
)
from SireRLGym.envs.base.legged_robot_sire import LeggedRobotSire  # noqa: E402
from SireRLGym.utils.joint_order import (  # noqa: E402
    JointOrderAdapter,
    infer_checkpoint_privileged_obs_dim,
)
from SireRLGym.utils.math import quat_rotate_inverse  # noqa: E402
from SireRLGym.utils.task_registry import make_env_cfg  # noqa: E402


@dataclass(frozen=True)
class CommandCase:
    name: str
    vx: float
    vy: float
    yaw: float


def _parse_float_list(text: str) -> list[float]:
    values = [float(item.strip()) for item in text.split(",") if item.strip()]
    if not values:
        raise argparse.ArgumentTypeError("expected at least one comma-separated value")
    if any(not math.isfinite(value) or value <= 0.0 for value in values):
        raise argparse.ArgumentTypeError("simulation time steps must be positive and finite")
    return values


def _parse_commands(text: str) -> list[CommandCase]:
    cases = []
    for case_index, raw_case in enumerate(text.split(";")):
        raw_case = raw_case.strip()
        if not raw_case:
            continue
        if ":" in raw_case:
            name, raw_values = raw_case.split(":", 1)
            name = name.strip()
        else:
            name, raw_values = f"command_{case_index}", raw_case
        values = [float(value.strip()) for value in raw_values.split(",")]
        if len(values) != 3 or any(not math.isfinite(value) for value in values):
            raise argparse.ArgumentTypeError(
                "each command must be name:vx,vy,yaw; entries are separated by ';'"
            )
        cases.append(CommandCase(name or f"command_{case_index}", *values))
    if not cases:
        raise argparse.ArgumentTypeError("expected at least one command")
    if len({case.name for case in cases}) != len(cases):
        raise argparse.ArgumentTypeError("command names must be unique")
    return cases


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run one checkpoint at equal policy periods with different Sire sim_dt values."
    )
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--task", default="go2")
    parser.add_argument("--sim-dts", type=_parse_float_list, default=_parse_float_list("0.001,0.005"))
    parser.add_argument("--control-dt", type=float, default=0.02)
    parser.add_argument("--steps", type=int, default=500, help="Maximum policy steps per case.")
    parser.add_argument(
        "--commands",
        type=_parse_commands,
        default=_parse_commands("stand:0,0,0;walk:0.5,0,0;fast:1.0,0,0"),
        help="Semicolon-separated name:vx,vy,yaw cases.",
    )
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--activation", default="elu")
    parser.add_argument("--sire-batch-threads", type=int, default=1)
    parser.add_argument("--comparison-window-s", type=float, default=1.0)
    parser.add_argument("--continue-after-done", action="store_true")
    parser.add_argument(
        "--native-history",
        action="store_true",
        help="Write full internal Sire event/substep history for focused diagnostics.",
    )
    parser.add_argument("--output-dir", type=Path, default=None)
    return parser.parse_args()


def _configure_command(env, command: CommandCase):
    values = torch.tensor(
        [command.vx, command.vy, command.yaw],
        dtype=env.commands.dtype,
        device=env.device,
    )
    env.commands[:, :3] = values
    if hasattr(env, "command_targets"):
        env.command_targets[:, :3] = values
    if env.commands.shape[1] > 3:
        env.commands[:, 3] = 0.0
        if hasattr(env, "command_targets"):
            env.command_targets[:, 3] = 0.0


def _make_env(task: str, sim_dt: float, control_dt: float, threads: int,
              privileged_obs_dim: int | None):
    ratio = control_dt / sim_dt
    decimation = int(round(ratio))
    if decimation <= 0 or not math.isclose(
        decimation * sim_dt, control_dt, rel_tol=0.0, abs_tol=1e-12
    ):
        raise ValueError(
            f"control_dt={control_dt} is not an integer multiple of sim_dt={sim_dt}"
        )

    cfg = make_env_cfg(task)
    if privileged_obs_dim is not None:
        cfg.env.num_privileged_obs = int(privileged_obs_dim)
    cfg.env.num_envs = 1
    cfg.sim.dt = float(sim_dt)
    cfg.sim.sire_batch_threads = int(threads)
    cfg.sim.sire_diagnostics = False
    cfg.sim.sire_max_recoveries_per_step = 1
    cfg.sim.sire_max_total_recoveries = 0
    cfg.sim.sire_max_recovery_fraction = 0.0
    cfg.control.decimation = decimation
    cfg.terrain.mesh_type = "plane"
    cfg.terrain.curriculum = False
    cfg.terrain.measure_heights = False
    cfg.commands.heading_command = False
    cfg.commands.curriculum = False
    cfg.commands.resampling_time = 1.0e9
    cfg.init_state.init_yaw_range = [0.0, 0.0]
    cfg.domain_rand.push_robots = False
    cfg.domain_rand.randomize_friction = False
    cfg.domain_rand.randomize_base_mass = False
    cfg.noise.add_noise = False

    env = LeggedRobotSire(cfg, headless=True)
    env.setSireHistoryRecording(False)
    env.capture_terminal_state = True
    env.last_terminal_snapshot = None
    _apply_fixed_start_state_sire(env)
    return env, decimation


def _termination_reason(snapshot, row: int) -> str:
    if snapshot is None:
        return "unknown"
    for key, label in (
        ("physics_failure", "physics_failure"),
        ("timeout", "timeout"),
        ("base_height", "base_height"),
        ("termination_contact", "termination_contact"),
    ):
        if bool(snapshot[key][row].item()):
            return label
    return "task_or_other"


def _state_for_row(env, done: bool):
    snapshot = env.last_terminal_snapshot if done else None
    if snapshot is not None:
        matches = (snapshot["env_ids"] == 0).nonzero(as_tuple=False).flatten()
        if matches.numel() > 0:
            row = int(matches[0].item())
            return {
                "root": snapshot["root_states"][row],
                "dof_pos": snapshot["dof_pos"][row],
                "dof_vel": snapshot["dof_vel"][row],
                "actions": snapshot["actions"][row],
                "torques": snapshot["torques"][row],
                "contact_forces": snapshot["contact_forces"][row],
                "feet_pos_world": snapshot["feet_pos_world"][row],
                "body_ground_contact": snapshot["body_ground_contact"][row],
                "foot_ground_contact": snapshot["foot_ground_contact"][row],
                "reason": _termination_reason(snapshot, row),
            }
    return {
        "root": env.root_states[0].detach().cpu(),
        "dof_pos": env.dof_pos[0].detach().cpu(),
        "dof_vel": env.dof_vel[0].detach().cpu(),
        "actions": env.actions[0].detach().cpu(),
        "torques": env.torques[0].detach().cpu(),
        "contact_forces": env.contact_forces[0].detach().cpu(),
        "feet_pos_world": env.feet_pos_world[0].detach().cpu(),
        "body_ground_contact": env.body_ground_contact[0].detach().cpu(),
        "foot_ground_contact": env.foot_ground_contact[0].detach().cpu(),
        "reason": "unknown" if done else "",
    }


def _append_vector(row: dict, prefix: str, values):
    for index, value in enumerate(values):
        row[f"{prefix}_{index}"] = float(value)


def _run_case(args, actor, privileged_obs_dim, sim_dt: float, command: CommandCase):
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    env, decimation = _make_env(
        args.task, sim_dt, args.control_dt, args.sire_batch_threads,
        privileged_obs_dim,
    )
    if args.native_history:
        env.setSireHistoryRecording(True)
    adapter = JointOrderAdapter.from_privileged_obs_dim(
        env.dof_names, device=env.device, privileged_obs_dim=privileged_obs_dim
    )
    _configure_command(env, command)
    env.compute_observations()
    obs = adapter.actor_obs_env_to_policy(env.get_observations(), env.num_actions)
    obs = _adapt_actor_obs(obs, actor.num_actor_obs)
    initial_recoveries = int(env._sire_batch_stepper.totalRecoveredFailures)
    body_names = [
        env.sire_models[0].partPool()[part_id].name
        for part_id in range(env.sire_models[0].nbody)
    ]
    rows = []
    total_reward = 0.0
    termination_reason = "max_steps"

    for step in range(args.steps):
        _configure_command(env, command)
        with torch.inference_mode():
            actions = actor.act_inference(obs)
        env_actions = adapter.actions_policy_to_env(actions)
        obs, _, rewards, dones, _ = env.step(env_actions)
        done = bool(dones[0].item())
        state = _state_for_row(env, done)
        root = state["root"]
        body_lin_vel = quat_rotate_inverse(root[3:7].unsqueeze(0), root[7:10].unsqueeze(0))[0]
        contact_norms = torch.linalg.vector_norm(state["contact_forces"], dim=-1)
        robot_contact_norms = contact_norms[1:]
        max_robot_contact_index = int(torch.argmax(robot_contact_norms).item()) + 1
        max_dof_velocity_index = int(torch.argmax(torch.abs(state["dof_vel"])).item())
        reward = float(rewards[0].item())
        total_reward += reward
        row = {
            "command": command.name,
            "sim_dt": sim_dt,
            "decimation": decimation,
            "control_dt": args.control_dt,
            "step": step + 1,
            "time_s": (step + 1) * args.control_dt,
            "done": int(done),
            "termination_reason": state["reason"],
            "reward": reward,
            "cmd_vx": command.vx,
            "cmd_vy": command.vy,
            "cmd_yaw": command.yaw,
            "body_vx": float(body_lin_vel[0]),
            "body_vy": float(body_lin_vel[1]),
            "body_vz": float(body_lin_vel[2]),
            "ground_resultant_force": float(contact_norms[0].item()),
            "max_contact_force": float(robot_contact_norms.max().item()),
            "max_contact_body": body_names[max_robot_contact_index],
            "contact_body_count": int(state["body_ground_contact"][1:].sum().item()),
            "foot_contact_count": int(state["foot_ground_contact"].sum().item()),
            "action_norm": float(torch.linalg.vector_norm(state["actions"]).item()),
            "max_abs_torque": float(torch.abs(state["torques"]).max().item()),
            "max_abs_dof_vel": float(torch.abs(state["dof_vel"]).max().item()),
            "max_dof_vel_joint": env.dof_names[max_dof_velocity_index],
        }
        foot_contact_norms = contact_norms[env.feet_indices.detach().cpu()]
        for foot_index, foot_name in enumerate(env.foot_names):
            row[f"foot_force_{foot_name}"] = float(foot_contact_norms[foot_index].item())
            row[f"foot_z_{foot_name}"] = float(state["feet_pos_world"][foot_index, 2].item())
            row[f"foot_contact_{foot_name}"] = int(
                state["foot_ground_contact"][foot_index].item()
            )
        _append_vector(row, "root", root.tolist())
        _append_vector(row, "dof_pos", state["dof_pos"].tolist())
        _append_vector(row, "dof_vel", state["dof_vel"].tolist())
        _append_vector(row, "action", state["actions"].tolist())
        rows.append(row)

        obs = adapter.actor_obs_env_to_policy(obs, env.num_actions)
        obs = _adapt_actor_obs(obs, actor.num_actor_obs)
        if done:
            termination_reason = state["reason"]
            if not args.continue_after_done:
                break
            _apply_fixed_start_state_sire(env)
            _configure_command(env, command)
            env.compute_observations()
            obs = adapter.actor_obs_env_to_policy(env.get_observations(), env.num_actions)
            obs = _adapt_actor_obs(obs, actor.num_actor_obs)

    final_recoveries = int(env._sire_batch_stepper.totalRecoveredFailures)
    velocities = np.asarray([[row["body_vx"], row["body_vy"]] for row in rows])
    contact_peaks = np.asarray([row["max_contact_force"] for row in rows])
    dof_velocity_peaks = np.asarray([row["max_abs_dof_vel"] for row in rows])
    peak_contact_row = max(rows, key=lambda row: row["max_contact_force"], default=None)
    peak_dof_velocity_row = max(rows, key=lambda row: row["max_abs_dof_vel"], default=None)
    command_xy = np.asarray([command.vx, command.vy])
    tracking_error = velocities - command_xy if len(rows) else np.empty((0, 2))
    summary = {
        "command": command.name,
        "sim_dt": sim_dt,
        "decimation": decimation,
        "control_dt": args.control_dt,
        "steps": len(rows),
        "simulated_time_s": len(rows) * args.control_dt,
        "terminated": termination_reason != "max_steps",
        "termination_reason": termination_reason,
        "physics_recoveries": final_recoveries - initial_recoveries,
        "return": total_reward,
        "mean_reward": total_reward / max(1, len(rows)),
        "tracking_xy_rmse": float(np.sqrt(np.mean(tracking_error ** 2))) if len(rows) else None,
        "mean_body_vx": float(np.mean(velocities[:, 0])) if len(rows) else None,
        "min_base_height": min((row["root_2"] for row in rows), default=None),
        "p99_contact_force": float(np.quantile(contact_peaks, 0.99)) if len(rows) else None,
        "max_contact_force": max((row["max_contact_force"] for row in rows), default=None),
        "peak_contact_step": peak_contact_row["step"] if peak_contact_row else None,
        "peak_contact_body": peak_contact_row["max_contact_body"] if peak_contact_row else None,
        "p99_abs_dof_vel": float(np.quantile(dof_velocity_peaks, 0.99)) if len(rows) else None,
        "max_abs_dof_vel": max((row["max_abs_dof_vel"] for row in rows), default=None),
        "peak_dof_vel_step": peak_dof_velocity_row["step"] if peak_dof_velocity_row else None,
        "peak_dof_vel_joint": peak_dof_velocity_row["max_dof_vel_joint"] if peak_dof_velocity_row else None,
        "max_abs_torque": max((row["max_abs_torque"] for row in rows), default=None),
    }
    if args.native_history:
        safe_command = re.sub(r"[^A-Za-z0-9_.-]+", "_", command.name)
        dt_label = f"{sim_dt:g}".replace(".", "p")
        history_path = args.output_dir / f"native_{safe_command}_dt_{dt_label}.json"
        history_path.write_text(
            json.dumps(env.sire_sim_loops[0].recordsToJson()), encoding="utf-8"
        )
        summary["native_history"] = str(history_path)
    del env
    gc.collect()
    return rows, summary


def _trajectory_array(rows, keys):
    return np.asarray([[row[key] for key in keys] for row in rows], dtype=np.float64)


def _compare_to_reference(reference_rows, candidate_rows, window_steps: int):
    count = min(len(reference_rows), len(candidate_rows), window_steps)
    if count == 0:
        return {"compared_steps": 0}

    def rmse(keys):
        reference = _trajectory_array(reference_rows[:count], keys)
        candidate = _trajectory_array(candidate_rows[:count], keys)
        return float(np.sqrt(np.mean((candidate - reference) ** 2)))

    return {
        "compared_steps": count,
        "root_position_rmse": rmse(["root_0", "root_1", "root_2"]),
        "body_linear_velocity_rmse": rmse(["body_vx", "body_vy", "body_vz"]),
        "dof_position_rmse": rmse([f"dof_pos_{i}" for i in range(12)]),
        "action_rmse": rmse([f"action_{i}" for i in range(12)]),
    }


def _write_csv(path: Path, rows: list[dict]):
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _write_plots(output_dir: Path, grouped_rows: dict):
    try:
        import matplotlib.pyplot as plt
    except ModuleNotFoundError:
        print("compare_plot skipped: matplotlib is not installed", flush=True)
        return
    for command, dt_rows in grouped_rows.items():
        figure, axes = plt.subplots(3, 2, figsize=(12, 11), sharex=True)
        for sim_dt, rows in sorted(dt_rows.items()):
            if not rows:
                continue
            time_values = [row["time_s"] for row in rows]
            label = f"dt={sim_dt:g}s"
            axes[0, 0].plot(time_values, [row["root_2"] for row in rows], label=label)
            axes[0, 1].plot(time_values, [row["body_vx"] for row in rows], label=label)
            axes[1, 0].plot(time_values, [row["max_contact_force"] for row in rows], label=label)
            axes[1, 1].plot(time_values, [row["reward"] for row in rows], label=label)
            axes[2, 0].plot(time_values, [row["max_abs_dof_vel"] for row in rows], label=label)
            axes[2, 1].plot(time_values, [row["max_abs_torque"] for row in rows], label=label)
        command_vx = next(
            (rows[0]["cmd_vx"] for rows in dt_rows.values() if rows), 0.0
        )
        axes[0, 1].axhline(command_vx, color="black", linestyle="--", alpha=0.5,
                           label="command vx")
        axes[0, 0].set_ylabel("base z [m]")
        axes[0, 1].set_ylabel("body vx [m/s]")
        axes[1, 0].set_ylabel("max contact force [N]")
        axes[1, 1].set_ylabel("reward / policy step")
        axes[2, 0].set_ylabel("max |joint velocity| [rad/s]")
        axes[2, 1].set_ylabel("max |torque| [Nm]")
        axes[1, 0].set_yscale("symlog", linthresh=100.0)
        for axis in axes.flat:
            axis.set_xlabel("time [s]")
            axis.grid(True, alpha=0.3)
            axis.legend()
        figure.suptitle(f"Sire sim-dt comparison: {command}")
        figure.tight_layout()
        figure.savefig(output_dir / f"{command}.png", dpi=160)
        plt.close(figure)


def main():
    args = parse_args()
    if args.steps <= 0 or args.control_dt <= 0.0 or args.comparison_window_s <= 0.0:
        raise ValueError("steps, control_dt and comparison_window_s must be positive")
    checkpoint = args.checkpoint.expanduser().resolve()
    if not checkpoint.is_file():
        raise FileNotFoundError(f"checkpoint not found: {checkpoint}")
    output_dir = args.output_dir
    if output_dir is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = Path("logs") / "sim_dt_compare" / stamp
    output_dir = output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    args.output_dir = output_dir

    actor, skipped = _load_deployment_actor(checkpoint, activation=args.activation)
    actor.eval()
    privileged_obs_dim = infer_checkpoint_privileged_obs_dim(checkpoint)
    print(
        f"compare_start checkpoint={checkpoint} actor_obs={actor.num_actor_obs} "
        f"actions={actor.num_actions} ignored_checkpoint_keys={len(skipped)} "
        f"output_dir={output_dir}",
        flush=True,
    )

    all_rows = []
    summaries = []
    grouped_rows = {}
    for command in args.commands:
        grouped_rows[command.name] = {}
        for sim_dt in args.sim_dts:
            rows, summary = _run_case(
                args, actor, privileged_obs_dim, sim_dt, command
            )
            grouped_rows[command.name][sim_dt] = rows
            all_rows.extend(rows)
            summaries.append(summary)
            print("compare_case " + " ".join(f"{key}={value}" for key, value in summary.items()), flush=True)

    reference_dt = args.sim_dts[0]
    window_steps = max(1, int(round(args.comparison_window_s / args.control_dt)))
    comparisons = []
    for command in args.commands:
        reference_rows = grouped_rows[command.name][reference_dt]
        for sim_dt in args.sim_dts[1:]:
            comparison = {
                "command": command.name,
                "reference_dt": reference_dt,
                "candidate_dt": sim_dt,
                "window_s": args.comparison_window_s,
            }
            comparison.update(
                _compare_to_reference(
                    reference_rows, grouped_rows[command.name][sim_dt], window_steps
                )
            )
            comparisons.append(comparison)
            print("compare_delta " + " ".join(f"{key}={value}" for key, value in comparison.items()), flush=True)

    _write_csv(output_dir / "trajectory.csv", all_rows)
    _write_csv(output_dir / "summary.csv", summaries)
    report = {
        "checkpoint": str(checkpoint),
        "seed": args.seed,
        "control_dt": args.control_dt,
        "max_steps": args.steps,
        "sim_dts": args.sim_dts,
        "summaries": summaries,
        "comparisons": comparisons,
    }
    (output_dir / "report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    _write_plots(output_dir, grouped_rows)
    print(f"compare_complete output_dir={output_dir}", flush=True)


if __name__ == "__main__":
    main()
