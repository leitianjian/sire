"""Measure Go2 control-step throughput for native CPU and CUDA Sire."""

from __future__ import annotations

import argparse
import json
import resource
import time
from functools import partial

import torch

from SireRLGym.utils.helpers import set_seed
from SireRLGym.utils.task_registry import make_env_cfg, make_env_from_cfg


def _current_rss_mib() -> float:
    with open("/proc/self/status", encoding="utf-8") as status_file:
        for line in status_file:
            if line.startswith("VmRSS:"):
                return float(line.split()[1]) / 1024.0
    raise RuntimeError("VmRSS is missing from /proc/self/status")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=("cpu", "cuda"), default="cpu")
    parser.add_argument("--device", default=None, help="PyTorch device, e.g. cpu or cuda:0.")
    parser.add_argument("--native-only", action="store_true",
                        help="Time native simulator stepping without observations or rewards.")
    parser.add_argument("--num-envs", type=int, default=16)
    parser.add_argument("--threads", type=int, default=16)
    parser.add_argument("--streams", type=int, default=None,
                        help="CUDA simulation streams (default: 4, capped at the environment count).")
    parser.add_argument("--steps", type=int, default=100)
    parser.add_argument("--warmup-steps", type=int, default=5)
    parser.add_argument(
        "--flat-terrain",
        action="store_true",
        help="Use the base scene's plane to isolate batch-stepper scaling.",
    )
    args = parser.parse_args()
    try:
        device = torch.device(args.device or args.backend)
    except (ValueError, RuntimeError) as error:
        parser.error(str(error))
    if device.type != args.backend:
        parser.error("--device must match --backend")
    if args.backend == "cuda":
        if not args.flat_terrain:
            parser.error("the CUDA backend requires --flat-terrain")
        if not torch.cuda.is_available():
            parser.error("CUDA is unavailable in this PyTorch environment")
        index = device.index if device.index is not None else torch.cuda.current_device()
        if not 0 <= index < torch.cuda.device_count():
            parser.error(f"CUDA device {index} is unavailable")
        device = torch.device("cuda", index)
    if args.num_envs <= 0 or args.steps <= 0 or args.warmup_steps < 0 or args.threads < 0:
        parser.error("--num-envs and --steps must be positive; --warmup-steps and --threads must be nonnegative")
    if args.streams is not None and args.streams < 1:
        parser.error("--streams must be positive")

    cfg = make_env_cfg("go2")
    cfg.env.num_envs = args.num_envs
    cfg.sim.sire_batch_threads = args.threads
    if args.streams is not None:
        cfg.sim.sire_cuda_streams = args.streams
    cfg.sim.sire_diagnostics = False
    cfg.noise.add_noise = False
    if args.flat_terrain:
        cfg.terrain.mesh_type = "plane"
        cfg.terrain.measure_heights = False
        cfg.terrain.curriculum = False
    set_seed(12345)

    rss_before_mib = _current_rss_mib()
    env = make_env_from_cfg("go2", cfg, headless=True, sire_backend=args.backend, sim_device=device)
    rss_after_init_mib = _current_rss_mib()
    actions = torch.zeros(args.num_envs, env.num_actions, dtype=torch.float32, device=device)
    stepper = env._sire_batch_stepper
    if not args.native_only:
        step = partial(env.step, actions)
    elif device.type == "cuda":
        stream = torch.cuda.current_stream(device).cuda_stream
        step = partial(stepper.step, actions, stream=stream)
    else:
        step = partial(stepper.step, actions.numpy())

    if device.type == "cuda":
        torch.cuda.synchronize(device)
    for _ in range(args.warmup_steps):
        step()
    if device.type == "cuda":
        torch.cuda.synchronize(device)
    dispatch_before = getattr(stepper, "dispatchCount", None)
    recovered_before = getattr(stepper, "totalRecoveredFailures", None)
    wall_start = time.perf_counter()
    cpu_start = time.process_time()
    for _ in range(args.steps):
        step()
    if device.type == "cuda":
        torch.cuda.synchronize(device)
    cpu_seconds = time.process_time() - cpu_start
    wall_seconds = time.perf_counter() - wall_start

    result = {
        "backend": args.backend,
        "device": str(device),
        "native_only": args.native_only,
        "flat_terrain": args.flat_terrain,
        "num_envs": args.num_envs,
        "requested_threads": args.threads,
        "effective_threads": getattr(stepper, "threadCount", None),
        "requested_streams": cfg.sim.sire_cuda_streams if device.type == "cuda" else None,
        "effective_streams": getattr(env, "sire_cuda_streams", None),
        "persistent_workers": getattr(stepper, "workerCount", None),
        "steps": args.steps,
        "warmup_steps": args.warmup_steps,
        "dispatches": stepper.dispatchCount - dispatch_before if dispatch_before is not None else None,
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
    if recovered_before is not None:
        result["recovered_failures"] = stepper.totalRecoveredFailures - recovered_before
    if device.type == "cuda":
        if args.native_only:
            result["failed_envs"] = torch.count_nonzero(env._cuda_state["status"]).item()
        else:
            result["last_step_failed_envs"] = torch.count_nonzero(env._sire_physics_failure_buf).item()
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
