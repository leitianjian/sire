"""
Shared heightfield height query — used by both RL training (_get_heights)
and standalone tests.  Only depends on numpy (no torch, no RLGym).

Implements the same triangulated heightfield interpolation as MuJoCo's
mj_rayHfield (engine_ray.c). Each hfield cell is split into two triangles
with a diagonal from (c,r) to (c+1,r+1), and height is computed via
barycentric interpolation within the containing triangle.
"""

from __future__ import annotations

import numpy as np


def heightfield_height(
    hf: np.ndarray,
    wx: float,
    wy: float,
    border: float,
    total_len: float,
    total_wid: float,
) -> float:
    """Return terrain height at world point (wx, wy).

    Uses MuJoCo-compatible triangulated interpolation:
    - Cell diagonal from (c,r) to (c+1,r+1)
    - Triangle 1 (right): fx >= fy, vertices (c,r), (c+1,r), (c+1,r+1)
    - Triangle 2 (left):  fx <  fy, vertices (c,r), (c+1,r+1), (c,r+1)
    """
    nrow, ncol = hf.shape
    col_f = (wx - border) / max(total_len, 1e-6) * (ncol - 1)
    row_f = (wy - border) / max(total_wid, 1e-6) * (nrow - 1)

    c0 = int(np.floor(col_f))
    r0 = int(np.floor(row_f))
    if c0 < 0 or r0 < 0 or c0 >= ncol or r0 >= nrow:
        return 0.0
    c1 = min(c0 + 1, ncol - 1)
    r1 = min(r0 + 1, nrow - 1)

    fx = min(max(col_f - c0, 0.0), 1.0)
    fy = min(max(row_f - r0, 0.0), 1.0)

    h00 = hf[r0, c0]  # bottom-left
    h10 = hf[r0, c1]  # bottom-right
    h01 = hf[r1, c0]  # top-left
    h11 = hf[r1, c1]  # top-right

    if fx >= fy:
        # Triangle 1: (c,r), (c+1,r), (c+1,r+1)
        # Barycentric: α = 1-fx, β = fx-fy, γ = fy
        return float((1.0 - fx) * h00 + (fx - fy) * h10 + fy * h11)
    else:
        # Triangle 2: (c,r), (c+1,r+1), (c,r+1)
        # Barycentric: α = 1-fy, β = fy-fx, γ = fx
        # (using convention: α*h00 + β*h01 + γ*h11)
        return float((1.0 - fy) * h00 + (fy - fx) * h01 + fx * h11)


# Keep old name as alias for backward compat
bilinear_height = heightfield_height
