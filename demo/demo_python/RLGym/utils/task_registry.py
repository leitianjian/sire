from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Type
import re

from RLGym.envs import GO2RoughCfg, GO2RoughCfgPPO, GO2Threshold, GO2ThresholdCfg, GO2ThresholdCfgPPO, LeggedRobot
from RLGym import ROOT_DIR


@dataclass
class TaskSpec:
    env_class: Type
    env_cfg_class: Type
    train_cfg_class: Type


_TASKS = {
    'go2': TaskSpec(env_class=LeggedRobot, env_cfg_class=GO2RoughCfg, train_cfg_class=GO2RoughCfgPPO),
    'go2_threshold': TaskSpec(env_class=GO2Threshold, env_cfg_class=GO2ThresholdCfg, train_cfg_class=GO2ThresholdCfgPPO),
}


def make_env(task: str, headless: bool = True):
    env_cfg = make_env_cfg(task)
    env = make_env_from_cfg(task, env_cfg, headless=headless)
    return env, env_cfg


def make_train_cfg(task: str):
    spec = _TASKS[task]
    return spec.train_cfg_class()


def make_env_cfg(task: str):
    spec = _TASKS[task]
    env_cfg = spec.env_cfg_class()
    return env_cfg


def make_env_from_cfg(task: str, env_cfg, headless: bool = True):
    spec = _TASKS[task]
    env = spec.env_class(env_cfg, sim_params=env_cfg.sim, physics_engine='mujoco', sim_device='cpu', headless=headless)
    return env


def make_log_dir(train_cfg, root='logs', scene_tag=None):
    base = Path(root) / train_cfg.runner.experiment_name
    if scene_tag:
        base = base / scene_tag
    base.mkdir(parents=True, exist_ok=True)

    exp_pat = re.compile(r"^exp(\d+)$")
    exp_dirs = [p for p in base.iterdir() if p.is_dir() and exp_pat.match(p.name)]

    max_id = 0
    for d in exp_dirs:
        m = exp_pat.match(d.name)
        if m:
            max_id = max(max_id, int(m.group(1)))
    next_dir = base / f"exp{max_id + 1}"
    next_dir.mkdir(parents=True, exist_ok=True)
    return str(next_dir)
