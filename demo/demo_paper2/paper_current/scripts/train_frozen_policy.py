"""Inspect archived paper configuration, or explicitly launch a new PPO training."""
import argparse
import copy
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]


def hydrate(obj, data):
    for key, value in data.items():
        current = getattr(obj, key, None)
        if isinstance(value, dict) and current is not None and not isinstance(current, dict):
            hydrate(current, value)
        elif isinstance(value, dict) and isinstance(current, dict):
            # Preserve live actor joint-block ordering; YAML serialization sorts keys.
            keys = [k for k in current if k in value] + [k for k in value if k not in current]
            setattr(obj, key, {k: copy.deepcopy(value[k]) for k in keys})
        else:
            setattr(obj, key, copy.deepcopy(value))
    return obj


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', choices=(2, 3, 4), type=int, default=2)
    parser.add_argument('--method', choices=('shifted', 'smooth'), default='shifted')
    parser.add_argument('--repo-root', type=Path)
    parser.add_argument('--log-dir', type=Path)
    parser.add_argument('--rl-device', default='cuda:0')
    parser.add_argument('--train', action='store_true', help='Actually start 1200 iterations; default is read-only review')
    args = parser.parse_args()
    import yaml
    if args.method == 'smooth' and args.seed != 2:
        parser.error('The smooth comparator is archived for seed 2 only.')
    config = (ROOT / f'data/training/seed_{args.seed}/flat_go2/exp1/config.yaml'
              if args.method == 'shifted' else
              ROOT / 'sources/smooth_training/logs/smooth_linear_shared_flow_seed2/flat_go2/exp1/config.yaml')
    frozen = yaml.safe_load(config.read_text(encoding='utf-8'))
    log_dir = (args.log_dir or ROOT / f'outputs/training/{args.method}/seed_{args.seed}').resolve()
    domain = frozen['env_cfg']['domain_rand']
    print(json.dumps({'config': str(config), 'seed': frozen['train_cfg']['seed'],
                      'method': args.method, 'num_envs': frozen['env_cfg']['env']['num_envs'],
                      'max_iterations': frozen['train_cfg']['runner']['max_iterations'],
                      'contact_parameters': {k: v for k, v in domain.items() if k.startswith('sire_')},
                      'new_log_dir': str(log_dir), 'will_train': args.train}, indent=2))
    if not args.train:
        return
    if log_dir.exists() and any(log_dir.iterdir()):
        raise RuntimeError('Choose an empty log directory for a new training run.')
    repo = args.repo_root
    if repo is None:
        repo = next((p for p in ROOT.parents if (p / 'demo/demo_python/SireRLGym').is_dir()), None)
    if repo is None:
        parser.error('--repo-root must identify a Sire checkout with SireRLGym.')
    sys.path.insert(0, str(repo / 'demo/demo_python'))
    from SireRLGym.scripts import train
    task = 'go2' if args.method == 'shifted' else 'go2_smooth_linear'
    env = hydrate(train.make_env_cfg(task), frozen['env_cfg'])
    ppo = hydrate(train.make_train_cfg(task), frozen['train_cfg'])
    ppo.runner.resume = False
    ppo.runner.resume_path = None
    if args.method == 'smooth':
        env.sim.sire_contact_solver = 'smooth_linear_shared_v5'
    # Use the saved training scene, including initial_penetration_policy=eliminate.
    env.sim.sire_model_file = str(ROOT / 'sources/smooth_training/demo/demo_python/sirePaperDogRL/go2_rai_foot.xml')
    import os
    old_cwd = Path.cwd()
    original_env, original_train = train.make_env_cfg, train.make_train_cfg
    train.make_env_cfg = lambda _: copy.deepcopy(env)
    train.make_train_cfg = lambda _: copy.deepcopy(ppo)
    sys.argv = [str(repo / 'demo/demo_python/SireRLGym/scripts/train.py'),
                '--task', task, '--num_envs', '1024', '--sire_batch_threads', '16',
                '--max_iterations', '1200', '--save_interval', '50', '--log_interval', '1',
                '--seed', str(args.seed), '--rl_device', args.rl_device,
                '--flat_terrain', '--log_dir', str(log_dir)]
    try:
        os.chdir(repo)
        train.main()
    finally:
        train.make_env_cfg, train.make_train_cfg = original_env, original_train
        os.chdir(old_cwd)


if __name__ == '__main__':
    main()
