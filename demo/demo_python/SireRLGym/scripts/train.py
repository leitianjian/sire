from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import torch

ROOT_DIR = Path(__file__).resolve().parents[2]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from SireRLGym.runners import OnPolicyRunner
from SireRLGym.scene_curriculum import apply_scene_curriculum_to_env_cfg, build_scene_curriculum_from_npz, resolve_scene_npz
from SireRLGym.utils.helpers import class_to_dict, get_load_path, set_seed
from SireRLGym.utils.joint_order import infer_checkpoint_privileged_obs_dim
from SireRLGym.utils.task_registry import make_env_cfg, make_env_from_cfg, make_log_dir, make_train_cfg


def _str2bool(v):
    if isinstance(v, bool):
        return v
    s = str(v).strip().lower()
    if s in ('1', 'true', 't', 'yes', 'y', 'on'):
        return True
    if s in ('0', 'false', 'f', 'no', 'n', 'off'):
        return False
    raise argparse.ArgumentTypeError(f'Invalid boolean value: {v}')


def _parse_float_list(value):
    if value is None:
        return None
    values = []
    for item in str(value).split(','):
        item = item.strip()
        if not item:
            continue
        values.append(float(item))
    return values or None


def _resolve_rl_device(value: str) -> torch.device:
    requested = str(value).strip().lower()
    if requested == 'auto':
        requested = 'cuda' if torch.cuda.is_available() else 'cpu'
    try:
        device = torch.device(requested)
    except (RuntimeError, ValueError) as error:
        raise ValueError(
            f"Invalid --rl_device {value!r}; use auto, cpu, cuda, or cuda:<index>."
        ) from error
    if device.type not in {'cpu', 'cuda'}:
        raise ValueError(
            f"Unsupported --rl_device {value!r}; Sire RL supports CPU or CUDA PPO."
        )
    if device.type == 'cuda':
        if not torch.cuda.is_available():
            raise RuntimeError(
                f"--rl_device {value!r} requested CUDA, but torch.cuda.is_available() is false."
            )
        index = torch.cuda.current_device() if device.index is None else device.index
        if index < 0 or index >= torch.cuda.device_count():
            raise ValueError(
                f"CUDA device index {index} is out of range; "
                f"available device count is {torch.cuda.device_count()}."
            )
        device = torch.device('cuda', index)
    return device


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--task', type=str, default='go2')
    p.add_argument('--num_envs', type=int, default=None)
    p.add_argument(
        '--sire_batch_threads', '--sirebatchthread',
        dest='sire_batch_threads', type=int, default=None,
        help='Persistent Sire batch thread count; 0 selects an automatic value.',
    )
    p.add_argument('--max_iterations', type=int, default=None)
    p.add_argument(
        '--save_interval',
        type=int,
        default=None,
        help='Checkpoint interval in PPO iterations.',
    )
    p.add_argument('--seed', type=int, default=None)
    p.add_argument(
        '--rl_device', type=str, default='auto',
        help='PPO policy/storage device: auto, cpu, cuda, or cuda:<index>.',
    )
    p.add_argument(
        '--flat_terrain',
        action='store_true',
        help='Train with the base scene plane instead of per-env rough heightfields.',
    )
    p.add_argument('--debug_reward', action='store_true')
    p.add_argument('--log_interval', type=int, default=10,
                   help='Write/print training statistics every N iterations; 0 disables.')
    p.add_argument('--memory_interval', type=int, default=0,
                   help='Sample process memory every N iterations to memory.jsonl; 0 disables.')
    p.add_argument('--solver_trace', action='store_true',
                   help='Allow SIRE_SOLVER_TRACE_DIR output (disabled during training by default).')
    p.add_argument('--allow_profile', action='store_true',
                   help='Allow training with a Tracy-instrumented Sire build.')
    p.add_argument('--resume', nargs='?', const='auto', default=None, help='Resume from latest checkpoint, or provide a checkpoint path.')
    p.add_argument('--head', type=_str2bool, nargs='?', const=True, default=False, help='Enable live MuJoCo viewer during training.')
    p.add_argument('--infinite_mode', action='store_true', help='Run threshold training with no iteration cap and global level progression.')
    p.add_argument('--threshold_height_levels', type=str, default=None, help='Comma-separated threshold heights, e.g. 0.04,0.05,...,0.15')
    p.add_argument('--infinite_success_rate_threshold', type=float, default=None)
    p.add_argument('--infinite_min_episodes', type=int, default=None)
    p.add_argument('--infinite_promotion_window_episodes', type=int, default=None)
    p.add_argument('--infinite_max_stuck_iterations', type=int, default=None)
    p.add_argument('--infinite_stats_filename', type=str, default=None)
    p.add_argument('--scene_curriculum_dir', type=str, default=None, help='Directory produced by build_scene_curriculum.py')
    p.add_argument('--scene_source_npz', type=str, default=None, help='Override terrain.scene_source_npz filename (e.g. box_old.npz, stairs_x.npz).')
    p.add_argument('--log_dir', type=str, default=None, help='Directory to save logs and checkpoints (default: logs/<experiment_name>)')
    p.add_argument('--visualize_interval', type=int, default=None,
                   help='Save a meshcat-compatible recording JSON every N iterations (e.g. 50)')
    p.add_argument('--visualize_resource_path', type=str, default=None,
                   help='Resource directory for meshcat visual assets (default: auto-detect from model)')
    return p.parse_args()


def _resolve_resume_path(resume_arg, train_cfg):
    if resume_arg is None:
        return None
    resume_str = str(resume_arg).strip()
    if resume_str == '' or resume_str.lower() in {'auto', 'true', 'latest'}:
        root = Path('logs') / train_cfg.runner.experiment_name
        return get_load_path(str(root), load_run=-1, checkpoint=-1)
    path = Path(resume_str).expanduser()
    if not path.is_absolute():
        path = (Path.cwd() / path).resolve()
    if not path.exists():
        raise FileNotFoundError(f'Resume checkpoint not found: {path}')
    return str(path)


def _resolve_scene_source_npz(scene_source_npz):
    return resolve_scene_npz(scene_source_npz)


def _infer_scene_tag(env_cfg):
    """Pick a subdir name based on the terrain config.

    - 'box' / 'box_old.npz' → 'box' (flat, no x/y variation)
    - 'slope_x.npz' → 'slope/x'
    - 'slope_y.npz' → 'slope/y'
    - 'slope_y_old.npz' → 'slope/y_old'
    - 'stairs_x.npz' / 'stairs_y.npz' / 'stairs_y_old.npz' → 'stairs/x' / 'stairs/y' / 'stairs/y_old'
    - default threshold terrain (no scene) → 'box'
    """
    npz = getattr(env_cfg.terrain, 'scene_source_npz', None)
    if npz:
        stem = Path(str(npz)).stem.lower()
        for tag in ('box', 'slope', 'stairs'):
            if stem == tag or stem.startswith(tag + '_'):
                remainder = stem[len(tag):].lstrip('_')
                if tag == 'box' or not remainder:
                    return tag
                return f"{tag}/{remainder}"
    mode = getattr(env_cfg.terrain, 'terrain_type_mode', None)
    if mode == 'threshold':
        return 'box'
    return None


def main():
    args = parse_args()
    rl_device = _resolve_rl_device(args.rl_device)
    env_cfg = make_env_cfg(args.task)
    train_cfg = make_train_cfg(args.task)

    if args.num_envs is not None:
        env_cfg.env.num_envs = args.num_envs
    if args.sire_batch_threads is not None:
        if args.sire_batch_threads < 0:
            raise ValueError('--sire_batch_threads must be >= 0')
        env_cfg.sim.sire_batch_threads = args.sire_batch_threads
    if args.max_iterations is not None:
        train_cfg.runner.max_iterations = args.max_iterations
    if args.save_interval is not None:
        if args.save_interval <= 0:
            raise ValueError('--save_interval must be > 0')
        train_cfg.runner.save_interval = args.save_interval
    if args.seed is not None:
        train_cfg.seed = args.seed
    if args.flat_terrain:
        env_cfg.terrain.mesh_type = 'plane'
        env_cfg.terrain.measure_heights = False
        env_cfg.terrain.curriculum = False
    if args.scene_source_npz is not None:
        env_cfg.terrain.scene_source_npz = args.scene_source_npz

    threshold_height_levels = _parse_float_list(args.threshold_height_levels)
    scene_curriculum_metadata = None
    auto_scene_source_npz = _resolve_scene_source_npz(getattr(env_cfg.terrain, "scene_source_npz", None))
    scene_curriculum_dir = args.scene_curriculum_dir
    if scene_curriculum_dir is None and auto_scene_source_npz is not None:
        generated_metadata, artifacts, target_heights = build_scene_curriculum_from_npz(
            auto_scene_source_npz,
            ROOT_DIR / "resources/scene_curricula",
            scene_name=auto_scene_source_npz.stem,
        )
        scene_curriculum_dir = str(artifacts.output_dir)
        print(
            "train_scene_curriculum "
            f"source_npz={auto_scene_source_npz} "
            f"generated_dir={scene_curriculum_dir} "
            f"target_heights={','.join(f'{v:.4f}' for v in target_heights)}",
            flush=True,
        )

    if scene_curriculum_dir is not None:
        if args.task != 'go2_threshold':
            raise ValueError('--scene_curriculum_dir is only supported for --task go2_threshold.')
        if threshold_height_levels is not None:
            raise ValueError('--threshold_height_levels cannot be used together with --scene_curriculum_dir.')
        scene_curriculum_metadata = apply_scene_curriculum_to_env_cfg(env_cfg, scene_curriculum_dir)
    if args.infinite_mode:
        if args.task != 'go2_threshold':
            raise ValueError('Infinite mode is only supported for --task go2_threshold.')
        if threshold_height_levels is None and scene_curriculum_metadata is None:
            threshold_height_levels = [round(v, 3) for v in [0.15 + 0.02 * i for i in range(1)]]
        if threshold_height_levels is not None:
            env_cfg.terrain.threshold_height_levels = [float(v) for v in threshold_height_levels]
            env_cfg.terrain.threshold_height = float(env_cfg.terrain.threshold_height_levels[0])
            env_cfg.terrain.num_rows = len(env_cfg.terrain.threshold_height_levels)
        env_cfg.terrain.max_init_terrain_level = 0
        env_cfg.local_task.forced_terrain_level = True
        env_cfg.local_task.forced_terrain_level_index = 0
        train_cfg.runner.infinite_mode = True
        if args.infinite_success_rate_threshold is not None:
            train_cfg.runner.infinite_success_rate_threshold = args.infinite_success_rate_threshold
        if args.infinite_min_episodes is not None:
            train_cfg.runner.infinite_min_episodes = args.infinite_min_episodes
        if args.infinite_promotion_window_episodes is not None:
            train_cfg.runner.infinite_promotion_window_episodes = args.infinite_promotion_window_episodes
        if args.infinite_max_stuck_iterations is not None:
            train_cfg.runner.infinite_max_stuck_iterations = args.infinite_max_stuck_iterations
        if args.infinite_stats_filename is not None:
            train_cfg.runner.infinite_stats_filename = args.infinite_stats_filename
    elif threshold_height_levels is not None:
        env_cfg.terrain.threshold_height_levels = [float(v) for v in threshold_height_levels]
        env_cfg.terrain.threshold_height = float(env_cfg.terrain.threshold_height_levels[0])
        env_cfg.terrain.num_rows = len(env_cfg.terrain.threshold_height_levels)

    resume_path = _resolve_resume_path(args.resume, train_cfg)
    if resume_path is not None:
        checkpoint_privileged_obs_dim = infer_checkpoint_privileged_obs_dim(resume_path)
        if checkpoint_privileged_obs_dim is not None:
            env_cfg.env.num_privileged_obs = int(checkpoint_privileged_obs_dim)

    mesh_type = str(env_cfg.terrain.mesh_type)
    active_terrain_type = (
        str(env_cfg.terrain.terrain_type_mode)
        if mesh_type in {'heightfield', 'trimesh'}
        else mesh_type
    )
    print(
        "train_terrain "
        f"type={active_terrain_type} "
        f"mesh_type={mesh_type} "
        f"scene_curriculum_dir={getattr(env_cfg.terrain, 'scene_curriculum_dir', None)} "
        f"slope_angle_override_deg={getattr(env_cfg.terrain, 'slope_angle_override_deg', None)} "
        f"heightfield_height_override={getattr(env_cfg.terrain, 'heightfield_height_override', None)} "
        f"terrain_size=({env_cfg.terrain.terrain_length}, {env_cfg.terrain.terrain_width}) "
        f"grid=({env_cfg.terrain.num_rows}, {env_cfg.terrain.num_cols})"
    , flush=True)

    set_seed(train_cfg.seed)

    if args.log_interval < 0 or args.memory_interval < 0:
        raise ValueError('log_interval and memory_interval must be nonnegative')
    if not args.solver_trace:
        os.environ.pop('SIRE_SOLVER_TRACE_DIR', None)
    import sire
    if sire.tracyEnabled and not args.allow_profile:
        raise RuntimeError(
            'Training requires SIRE_ENABLE_TRACY=OFF. Rebuild/install Sire with '
            'profiling disabled, or pass --allow_profile for diagnostic runs.')
    env_cfg.sim.sire_diagnostics = False
    env = make_env_from_cfg(args.task, env_cfg, headless=not args.head)
    scene_tag = _infer_scene_tag(env_cfg)
    log_root = str(args.log_dir) if args.log_dir else 'logs'
    log_dir = make_log_dir(train_cfg, root=log_root, scene_tag=scene_tag)
    print(
        "train_start "
        f"task={args.task} "
        f"num_envs={env_cfg.env.num_envs} "
        f"sire_batch_threads={env_cfg.sim.sire_batch_threads} "
        f"max_iterations={'inf' if getattr(train_cfg.runner, 'infinite_mode', False) else train_cfg.runner.max_iterations} "
        f"save_interval={train_cfg.runner.save_interval} "
        f"visualize_interval={args.visualize_interval} "
        f"rl_device={rl_device} "
        f"seed={train_cfg.seed} "
        f"log_dir={log_dir}",
        flush=True,
    )
    train_cfg_dict = class_to_dict(train_cfg)
    train_cfg_dict['runner']['debug_reward'] = bool(args.debug_reward)
    train_cfg_dict['runner']['log_interval'] = args.log_interval
    train_cfg_dict['runner']['memory_interval'] = args.memory_interval
    if args.visualize_interval is not None:
        train_cfg_dict['runner']['visualize_interval'] = args.visualize_interval
    if args.visualize_resource_path is not None:
        train_cfg_dict['runner']['visualize_resource_path'] = args.visualize_resource_path
    train_cfg_dict['runner']['log_root'] = log_root
    runner = OnPolicyRunner(env, train_cfg_dict, log_dir=log_dir, device=rl_device)
    if resume_path is not None:
        checkpoint = runner.load(resume_path)
        print(f"train_resume path={resume_path} iter={runner.current_learning_iteration} log_dir={log_dir}", flush=True)
    target_iteration = int(train_cfg.runner.max_iterations)
    remaining_iterations = max(
        0, target_iteration - int(runner.current_learning_iteration)
    )
    print(
        f"train_iterations current={runner.current_learning_iteration} "
        f"target={target_iteration} remaining={remaining_iterations}",
        flush=True,
    )
    runner.learn(remaining_iterations)


if __name__ == '__main__':
    main()
