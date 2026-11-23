import os
import random
import re
from pathlib import Path

import numpy as np
import torch

"""
强化学习的辅助工具函数 (Helper functions for RL)。
本文件提供了一些通用的操作，包括类的字典化转换、随机种子设置，
以及模型权重的加载路径解析。
"""


def class_to_dict(obj) -> dict:
    """
    将包含配置参数的类转换为字典格式 (dict)。
    这在保存实验参数 (Hyperparameters) 或向外部日志库上传数据时非常实用。
    """
    if not hasattr(obj, "__dict__"):
        return obj
    result = {}
    for key in dir(obj):
        if key.startswith("_"):
            continue
        val = getattr(obj, key)
        if callable(val):
            continue
        if isinstance(val, list):
            result[key] = [class_to_dict(i) for i in val]
        else:
            result[key] = class_to_dict(val)
    return result


def set_seed(seed: int) -> None:
    if seed == -1:
        seed = np.random.randint(0, 10000)
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    os.environ["PYTHONHASHSEED"] = str(seed)


def get_load_path(root: str, load_run=-1, checkpoint=-1):
    root_path = Path(root)
    runs = [d for d in root_path.iterdir() if d.is_dir()]
    runs = [d for d in runs if len(list(d.glob('model_*.pt'))) > 0]
    if not runs:
        raise ValueError(f"No runs in this directory: {root}")
    exp_pat = re.compile(r"^exp(\d+)$")
    runs = sorted(runs, key=lambda p: int(exp_pat.match(p.name).group(1)) if exp_pat.match(p.name) else p.name)
    run = runs[-1] if load_run == -1 else root_path / load_run
    if checkpoint == -1:
        model_pat = re.compile(r"^model_(\d+)\.pt$")
        models = sorted(
            run.glob('model_*.pt'),
            key=lambda p: int(model_pat.match(p.name).group(1)) if model_pat.match(p.name) else p.name,
        )
        model = models[-1]
    else:
        model = run / f"model_{checkpoint}.pt"
    return str(model)
