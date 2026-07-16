import math
import torch

"""
强化学习中的数学工具库 (Math utilities for RL) — Aris convention.
Sire/Aris uses scalar-last quaternion: [qx, qy, qz, qw] = (x, y, z, w).
This differs from MuJoCo which uses scalar-first: [qw, qx, qy, qz] = (w, x, y, z).

All functions below assume Aris scalar-last convention.
"""


def wrap_to_pi(angles: torch.Tensor) -> torch.Tensor:
    return (angles + math.pi) % (2 * math.pi) - math.pi


def quat_conjugate(q: torch.Tensor) -> torch.Tensor:
    """Conjugate for scalar-last (x,y,z,w): negate x,y,z, keep w."""
    out = q.clone()
    out[..., :3] *= -1.0
    return out


def quat_mul(q: torch.Tensor, r: torch.Tensor) -> torch.Tensor:
    """Quaternion multiplication for scalar-last (x,y,z,w)."""
    x0, y0, z0, w0 = q.unbind(dim=-1)
    x1, y1, z1, w1 = r.unbind(dim=-1)
    return torch.stack(
        [
            w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
            w0 * y1 + y0 * w1 + z0 * x1 - x0 * z1,
            w0 * z1 + z0 * w1 + x0 * y1 - y0 * x1,
            w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
        ],
        dim=-1,
    )


def quat_apply(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    """Rotate vector v by quaternion q (scalar-last)."""
    zeros = torch.zeros_like(v[..., :1])
    v_as_quat = torch.cat([v, zeros], dim=-1)  # [vx, vy, vz, 0]
    return quat_mul(quat_mul(q, v_as_quat), quat_conjugate(q))[..., :3]


def quat_rotate_inverse(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    """Inverse-rotate vector v by quaternion q (scalar-last)."""
    return quat_apply(quat_conjugate(q), v)


def quat_apply_yaw(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    """Apply yaw-only rotation for scalar-last (x,y,z,w)."""
    # yaw = atan2(2(qw*qz + qx*qy), 1 - 2(qy² + qz²))
    yaw = torch.atan2(
        2 * (q[..., 3] * q[..., 2] + q[..., 0] * q[..., 1]),
        1 - 2 * (q[..., 1] * q[..., 1] + q[..., 2] * q[..., 2]),
    )
    cy = torch.cos(yaw)
    sy = torch.sin(yaw)
    x = v[..., 0] * cy - v[..., 1] * sy
    y = v[..., 0] * sy + v[..., 1] * cy
    z = v[..., 2]
    return torch.stack([x, y, z], dim=-1)


def torch_rand_float(low: float, high: float, shape, device='cpu') -> torch.Tensor:
    return (high - low) * torch.rand(*shape, device=device) + low
