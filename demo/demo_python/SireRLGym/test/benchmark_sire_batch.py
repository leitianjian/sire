"""Small repeatable CPU/RSS benchmark for SireRLBatchStepper."""

from __future__ import annotations

import argparse
import json
import resource
import time

import torch

from SireRLGym.utils.task_registry import make_env_cfg, make_env_from_cfg


def _current_rss_mib() -> float:
    with open("/proc/self/status", encoding="utf-8") as status_file:
        for line in status_file:
            if line.startswith("VmRSS:"):
                return float(line.split()[1]) / 1024.0
    raise RuntimeError("VmRSS is missing from /proc/self/status")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--num-envs", type=int, default=16)
    parser.add_argument("--threads", type=int, default=16)
    parser.add_argument("--steps", type=int, default=100)
    parser.add_argument("--warmup-steps", type=int, default=5)
    parser.add_argument(
        "--flat-terrain",
        action="store_true",
        help="Use the base scene's plane to isolate batch-stepper scaling.",
    )
    args = parser.parse_args()

    cfg = make_env_cfg("go2")
    cfg.env.num_envs = args.num_envs
    cfg.sim.sire_batch_threads = args.threads
    cfg.sim.sire_diagnostics = False
    cfg.noise.add_noise = False
    if args.flat_terrain:
        cfg.terrain.mesh_type = "plane"
        cfg.terrain.measure_heights = False
        cfg.terrain.curriculum = False
    torch.manual_seed(12345)

    rss_before_mib = _current_rss_mib()
    env = make_env_from_cfg("go2", cfg, headless=True)
    rss_after_init_mib = _current_rss_mib()
    actions = torch.zeros(args.num_envs, env.num_actions)

    for _ in range(args.warmup_steps):
        env.step(actions)
    dispatch_before = env._sire_batch_stepper.dispatchCount
    wall_start = time.perf_counter()
    cpu_start = time.process_time()
    for _ in range(args.steps):
        env.step(actions)
    cpu_seconds = time.process_time() - cpu_start
    wall_seconds = time.perf_counter() - wall_start

    result = {
        "num_envs": args.num_envs,
        "requested_threads": args.threads,
        "effective_threads": env._sire_batch_stepper.threadCount,
        "persistent_workers": env._sire_batch_stepper.workerCount,
        "steps": args.steps,
        "dispatches": env._sire_batch_stepper.dispatchCount - dispatch_before,
        "wall_seconds": wall_seconds,
        "cpu_seconds": cpu_seconds,
        "aggregate_cpu_percent": 100.0 * cpu_seconds / max(wall_seconds, 1e-12),
        "control_steps_per_second": args.steps / max(wall_seconds, 1e-12),
        "env_steps_per_second": args.num_envs * args.steps / max(wall_seconds, 1e-12),
        "rss_before_mib": rss_before_mib,
        "rss_after_init_mib": rss_after_init_mib,
        "rss_environment_delta_mib": rss_after_init_mib - rss_before_mib,
        "rss_after_run_mib": _current_rss_mib(),
        "max_rss_mib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0,
    }
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
