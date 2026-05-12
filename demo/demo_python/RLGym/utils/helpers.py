import os
import random
import re
from pathlib import Path

import numpy as np
import torch


def class_to_dict(obj) -> dict:
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
