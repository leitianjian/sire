import math
import torch


def wrap_to_pi(angles: torch.Tensor) -> torch.Tensor:
    return (angles + math.pi) % (2 * math.pi) - math.pi


def quat_conjugate(q: torch.Tensor) -> torch.Tensor:
    out = q.clone()
    out[..., 1:] *= -1.0
    return out


def quat_mul(q: torch.Tensor, r: torch.Tensor) -> torch.Tensor:
    w0, x0, y0, z0 = q.unbind(dim=-1)
    w1, x1, y1, z1 = r.unbind(dim=-1)
    return torch.stack(
        [
            w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
            w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
            w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
            w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
        ],
        dim=-1,
    )


def quat_apply(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    zeros = torch.zeros_like(v[..., :1])
    v_as_quat = torch.cat([zeros, v], dim=-1)
    return quat_mul(quat_mul(q, v_as_quat), quat_conjugate(q))[..., 1:]


def quat_rotate_inverse(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    return quat_apply(quat_conjugate(q), v)


def quat_apply_yaw(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    yaw = torch.atan2(
        2 * (q[..., 0] * q[..., 3] + q[..., 1] * q[..., 2]),
        1 - 2 * (q[..., 2] * q[..., 2] + q[..., 3] * q[..., 3]),
    )
    cy = torch.cos(yaw)
    sy = torch.sin(yaw)
    x = v[..., 0] * cy - v[..., 1] * sy
    y = v[..., 0] * sy + v[..., 1] * cy
    z = v[..., 2]
    return torch.stack([x, y, z], dim=-1)


def torch_rand_float(low: float, high: float, shape, device='cpu') -> torch.Tensor:
    return (high - low) * torch.rand(*shape, device=device) + low
