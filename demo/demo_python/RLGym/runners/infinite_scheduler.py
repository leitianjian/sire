from __future__ import annotations

import json
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Optional

import torch


def _extract_scalar(value, default=0.0):
    if value is None:
        return float(default)
    if isinstance(value, torch.Tensor):
        if value.numel() == 0:
            return float(default)
        return float(value.detach().float().mean().item())
    return float(value)


class InfiniteLevelScheduler:
    def __init__(self, env, cfg, log_dir: Optional[str] = None):
        self.env = env
        self.log_dir = log_dir
        self.success_rate_threshold = float(cfg.get('infinite_success_rate_threshold', 0.8))
        self.min_episodes = int(cfg.get('infinite_min_episodes', max(1, env.num_envs)))
        self.promotion_window_episodes = int(
            cfg.get('infinite_promotion_window_episodes', self.min_episodes)
        )
        self.max_stuck_iterations = int(cfg.get('infinite_max_stuck_iterations', 2000))
        self.stats_filename = str(cfg.get('infinite_stats_filename', 'infinite_mode_stats.json'))
        self.min_level_gap = float(cfg.get('infinite_min_level_gap', 5.0e-3))

        self._heights: list[float] = []
        self._level_index = 0
        self._level_start_iteration = 0
        self._level_start_total_time = 0.0
        self._level_start_timesteps = 0
        self._level_iterations = 0
        self._level_episode_count = 0
        self._level_success_episodes = 0.0
        self._level_pass_episodes = 0.0
        self._level_failure_episodes = 0.0
        self._level_best_rel_x_sum = 0.0
        self._level_clearance_sum = 0.0
        self._recent_successes: deque = deque(maxlen=max(1, self.promotion_window_episodes))
        self._recent_passes: deque = deque(maxlen=max(1, self.promotion_window_episodes))
        self._recent_failures: deque = deque(maxlen=max(1, self.promotion_window_episodes))
        self._records: list[dict] = []
        self._final_status: Optional[dict] = None
        self._stats_path: Optional[Path] = None
        self._run_started_at: Optional[str] = None
        self._initialized = False

    @property
    def initialized(self) -> bool:
        return self._initialized

    def initialize(self, current_iteration: int, tot_time: float, tot_timesteps: int) -> None:
        if self._initialized:
            return
        if not hasattr(self.env, 'enable_forced_terrain_level'):
            raise RuntimeError('Infinite mode requires env.enable_forced_terrain_level().')

        self._heights = list(getattr(self.env, 'get_available_threshold_heights')())
        if len(self._heights) == 0:
            raise RuntimeError('Infinite mode requires at least one threshold height level.')
        trimmed = [float(self._heights[0])]
        for h in self._heights[1:]:
            if float(h) > trimmed[-1] + self.min_level_gap:
                trimmed.append(float(h))
            else:
                break
        if len(trimmed) < len(self._heights):
            print(
                "infinite_mode heights_trimmed "
                f"original={len(self._heights)} effective={len(trimmed)} "
                f"last_height={trimmed[-1]:.4f} "
                f"dropped={[f'{float(h):.4f}' for h in self._heights[len(trimmed):]]}",
                flush=True,
            )
        self._heights = trimmed
        self._level_start_iteration = current_iteration
        self._level_start_total_time = tot_time
        self._level_start_timesteps = tot_timesteps
        self._run_started_at = datetime.now(timezone.utc).isoformat()
        if self.log_dir is not None:
            self._stats_path = Path(self.log_dir) / self.stats_filename
        with torch.inference_mode():
            self.env.enable_forced_terrain_level(0)
            self.env.reset()
        self._initialized = True
        self._write_stats(current_iteration, tot_time, tot_timesteps)

    def level_height(self, level_index: Optional[int] = None) -> float:
        if level_index is None:
            level_index = self._level_index
        idx = int(max(0, min(level_index, len(self._heights) - 1)))
        return float(self._heights[idx])

    def level_summary(self, current_iteration: int, tot_time: float, tot_timesteps: int) -> dict:
        episode_count = max(0, int(self._level_episode_count))
        if episode_count > 0:
            success_rate = self._level_success_episodes / episode_count
            pass_rate = self._level_pass_episodes / episode_count
            failure_rate = self._level_failure_episodes / episode_count
            mean_best_rel_x = self._level_best_rel_x_sum / episode_count
            mean_clearance = self._level_clearance_sum / episode_count
        else:
            success_rate = pass_rate = failure_rate = mean_best_rel_x = mean_clearance = 0.0

        recent_count = len(self._recent_successes)
        if recent_count > 0:
            recent_success_rate = float(sum(self._recent_successes)) / recent_count
            recent_pass_rate = float(sum(self._recent_passes)) / recent_count
            recent_failure_rate = float(sum(self._recent_failures)) / recent_count
        else:
            recent_success_rate = recent_pass_rate = recent_failure_rate = 0.0

        return {
            'level_index': int(self._level_index),
            'threshold_height': self.level_height(),
            'iteration_start': int(self._level_start_iteration),
            'iteration_current': int(current_iteration),
            'iterations_spent': int(self._level_iterations),
            'episodes': episode_count,
            'success_rate': float(success_rate),
            'pass_rate': float(pass_rate),
            'failure_rate': float(failure_rate),
            'promotion_window_episodes': int(self.promotion_window_episodes),
            'recent_episodes': int(recent_count),
            'recent_success_rate': float(recent_success_rate),
            'recent_pass_rate': float(recent_pass_rate),
            'recent_failure_rate': float(recent_failure_rate),
            'mean_best_rel_x': float(mean_best_rel_x),
            'mean_clearance': float(mean_clearance),
            'elapsed_seconds': float(max(0.0, tot_time - self._level_start_total_time)),
            'elapsed_timesteps': int(max(0, tot_timesteps - self._level_start_timesteps)),
        }

    def _write_stats(self, current_iteration: int, tot_time: float, tot_timesteps: int) -> None:
        if self._stats_path is None:
            return
        payload = {
            'mode': 'infinite_threshold',
            'run_started_at_utc': self._run_started_at,
            'current_learning_iteration': int(current_iteration),
            'success_rate_threshold': float(self.success_rate_threshold),
            'min_episodes': int(self.min_episodes),
            'promotion_window_episodes': int(self.promotion_window_episodes),
            'max_stuck_iterations': int(self.max_stuck_iterations),
            'height_levels': [float(v) for v in self._heights],
            'current_level': self.level_summary(current_iteration, tot_time, tot_timesteps),
            'completed_levels': list(self._records),
            'final_status': self._final_status,
        }
        self._stats_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding='utf-8')

    def _reset_counters(self, next_level: int, next_iteration: int, tot_time: float, tot_timesteps: int) -> None:
        self._level_index = int(next_level)
        self._level_start_iteration = int(next_iteration)
        self._level_start_total_time = float(tot_time)
        self._level_start_timesteps = int(tot_timesteps)
        self._level_iterations = 0
        self._level_episode_count = 0
        self._level_success_episodes = 0.0
        self._level_pass_episodes = 0.0
        self._level_failure_episodes = 0.0
        self._level_best_rel_x_sum = 0.0
        self._level_clearance_sum = 0.0
        self._recent_successes.clear()
        self._recent_passes.clear()
        self._recent_failures.clear()

    def _finalize(self, status: str, iteration: int, tot_time: float, tot_timesteps: int) -> None:
        self._final_status = {
            'status': str(status),
            'iteration': int(iteration),
            'total_timesteps': int(tot_timesteps),
            'total_time_seconds': float(tot_time),
            'completed_levels': int(len(self._records)),
            'last_level': self.level_summary(iteration, tot_time, tot_timesteps),
        }
        self._write_stats(iteration, tot_time, tot_timesteps)

    def handle_iteration(
        self,
        iteration: int,
        ep_infos,
        tot_time: float,
        tot_timesteps: int,
        save_fn: Callable[[str, int], None],
    ) -> tuple[bool, bool]:
        """Consume this iteration's episode infos. Returns (stop_training, transitioned_level)."""
        self._level_iterations += 1
        for ep_info in ep_infos:
            episode_count = int(round(_extract_scalar(ep_info.get('episode_count', 0.0), default=0.0)))
            if episode_count <= 0:
                continue
            success_rate = max(0.0, min(1.0, _extract_scalar(ep_info.get('task_success'))))
            pass_rate = max(0.0, min(1.0, _extract_scalar(ep_info.get('task_passed_threshold'))))
            failure_rate = max(0.0, min(1.0, _extract_scalar(ep_info.get('task_failure'))))
            success_count = max(0, min(int(round(success_rate * episode_count)), episode_count))
            pass_count = max(0, min(int(round(pass_rate * episode_count)), episode_count))
            failure_count = max(0, min(int(round(failure_rate * episode_count)), episode_count))

            self._level_episode_count += episode_count
            self._level_success_episodes += success_count
            self._level_pass_episodes += pass_count
            self._level_failure_episodes += failure_count
            self._level_best_rel_x_sum += _extract_scalar(ep_info.get('task_best_rel_x')) * episode_count
            self._level_clearance_sum += _extract_scalar(ep_info.get('task_clearance')) * episode_count
            self._recent_successes.extend([1] * success_count)
            self._recent_successes.extend([0] * max(0, episode_count - success_count))
            self._recent_passes.extend([1] * pass_count)
            self._recent_passes.extend([0] * max(0, episode_count - pass_count))
            self._recent_failures.extend([1] * failure_count)
            self._recent_failures.extend([0] * max(0, episode_count - failure_count))

        level_summary = self.level_summary(iteration, tot_time, tot_timesteps)
        self._write_stats(iteration, tot_time, tot_timesteps)

        enough_recent = level_summary['recent_episodes'] >= self.min_episodes
        passed_level = enough_recent and (level_summary['recent_success_rate'] >= self.success_rate_threshold)
        if passed_level:
            milestone_path = Path(self.log_dir) / (
                f"model_level_{level_summary['level_index']:02d}_h_{level_summary['threshold_height']:.3f}_it_{iteration}.pt"
            )
            save_fn(str(milestone_path), iteration)
            record = dict(level_summary)
            record['status'] = 'passed'
            record['model_path'] = str(milestone_path)
            record['passed_at_iteration'] = int(iteration)
            record['passed_at_total_time_seconds'] = float(tot_time)
            self._records.append(record)
            if level_summary['level_index'] >= len(self._heights) - 1:
                self._finalize('passed_max_level', iteration, tot_time, tot_timesteps)
                return True, False

            next_level = level_summary['level_index'] + 1
            with torch.inference_mode():
                self.env.set_forced_terrain_level(next_level)
                self.env.reset()
            self._reset_counters(next_level=next_level, next_iteration=iteration + 1, tot_time=tot_time, tot_timesteps=tot_timesteps)
            self._write_stats(iteration, tot_time, tot_timesteps)
            return False, True

        if self._level_iterations >= self.max_stuck_iterations:
            record = dict(level_summary)
            record['status'] = 'stuck'
            record['stopped_at_iteration'] = int(iteration)
            record['stopped_at_total_time_seconds'] = float(tot_time)
            self._records.append(record)
            self._finalize('stuck_max_iterations', iteration, tot_time, tot_timesteps)
            return True, False

        return False, False
