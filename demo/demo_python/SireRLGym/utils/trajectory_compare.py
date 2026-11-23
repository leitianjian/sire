"""
Joint trajectory comparison between two time series with different time grids.

Pure numpy — no torch, no RL dependency.

Handles different time grids by resampling both to a common uniform grid
via linear interpolation, then computes similarity metrics.

Usage:
    from SireRLGym.utils.trajectory_compare import compare_trajectories

    result = compare_trajectories(t_a, q_a, t_b, q_b, joint_names=names)
    print(result.summary())
    result.plot()
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


# ═══════════════════════════════════════════════════════════════════════
#  Data structures
# ═══════════════════════════════════════════════════════════════════════
@dataclass
class JointMetrics:
    """Per-joint comparison metrics."""
    joint_name: str = ""
    rmse: float = 0.0
    max_err: float = 0.0
    max_err_time: float = 0.0
    r2: float = 0.0
    mean_a: float = 0.0
    mean_b: float = 0.0
    range_a: float = 0.0
    range_b: float = 0.0
    n_points: int = 0

    @property
    def nrmse_pct(self) -> float:
        """RMSE as percentage of reference (A) range."""
        denom = self.range_a
        return (self.rmse / denom * 100.0) if denom > 1e-8 else float("nan")


@dataclass
class TrajectoryResult:
    """Comparison result for two multi-joint trajectories A and B."""
    joint_metrics: list[JointMetrics] = field(default_factory=list)
    common_time: np.ndarray = field(default_factory=lambda: np.array([]))
    q_a_resampled: np.ndarray = field(default_factory=lambda: np.array([]))
    q_b_resampled: np.ndarray = field(default_factory=lambda: np.array([]))
    t_a_original: np.ndarray = field(default_factory=lambda: np.array([]))
    t_b_original: np.ndarray = field(default_factory=lambda: np.array([]))
    dt_ref: float = 0.0
    dt_common: float = 0.0
    n_steps_a: int = 0
    n_steps_b: int = 0
    n_common: int = 0

    # raw data attached after construction (for optional plotting)
    q_a_raw: np.ndarray = field(default_factory=lambda: np.array([]))
    q_b_raw: np.ndarray = field(default_factory=lambda: np.array([]))

    @property
    def overall_rmse(self) -> float:
        if self.q_a_resampled.size == 0:
            return float("nan")
        return float(np.sqrt(np.mean((self.q_a_resampled - self.q_b_resampled) ** 2)))

    @property
    def overall_max_err(self) -> float:
        if self.q_a_resampled.size == 0:
            return float("nan")
        return float(np.max(np.abs(self.q_a_resampled - self.q_b_resampled)))

    def summary(self) -> str:
        """Human-readable per-joint summary."""
        lines = [
            f"Trajectory comparison: {self.n_steps_a} pts (A) vs "
            f"{self.n_steps_b} pts (B) → {self.n_common} common pts "
            f"(dt={self.dt_common*1000:.2f} ms)",
            f"{'Joint':<18s} {'RMSE':>8s} {'NRMSE%':>8s} {'MaxErr':>8s} {'R²':>8s}",
            "-" * 55,
        ]
        for m in self.joint_metrics:
            lines.append(
                f"{m.joint_name:<18s} {m.rmse:8.4f} {m.nrmse_pct:7.1f}% "
                f"{m.max_err:8.4f} {m.r2:8.4f}"
            )
        lines.append("-" * 55)
        lines.append(f"{'OVERALL':<18s} {self.overall_rmse:8.4f} {'':>8s} "
                     f"{self.overall_max_err:8.4f}")
        return "\n".join(lines)

    def to_dict(self) -> dict:
        """JSON-serialisable summary."""
        return {
            "n_steps_a": self.n_steps_a,
            "n_steps_b": self.n_steps_b,
            "n_common": self.n_common,
            "dt_ref": self.dt_ref,
            "dt_common": self.dt_common,
            "overall_rmse": self.overall_rmse,
            "overall_max_err": self.overall_max_err,
            "joints": [
                {"name": m.joint_name, "rmse": m.rmse,
                 "nrmse_pct": m.nrmse_pct, "max_err": m.max_err, "r2": m.r2}
                for m in self.joint_metrics
            ],
        }

    def plot(self, title: str = "Trajectory Comparison A vs B",
             max_joints: int = 6, figsize: tuple = (14, 10)):
        """Quick matplotlib plot.  Call matplotlib.use('Agg') for headless."""
        import matplotlib.pyplot as plt

        n = min(len(self.joint_metrics), max_joints)
        cols = min(3, n)
        rows = max(1, int(np.ceil(n / cols)))
        fig, axes = plt.subplots(rows, cols, figsize=figsize, squeeze=False)
        fig.suptitle(title, fontsize=13)

        for idx in range(n):
            ax = axes[idx // cols][idx % cols]
            m = self.joint_metrics[idx]
            t = self.common_time
            qa = self.q_a_raw if self.q_a_raw.size else self.q_a_resampled
            qb = self.q_b_raw if self.q_b_raw.size else self.q_b_resampled
            ax.plot(self.t_a_original, qa[:, idx],
                    '.', alpha=0.3, markersize=2, label="A raw")
            ax.plot(self.t_b_original, qb[:, idx],
                    '.', alpha=0.3, markersize=2, label="B raw")
            ax.plot(t, self.q_a_resampled[:, idx], lw=1.5, label="A")
            ax.plot(t, self.q_b_resampled[:, idx], lw=1.5, label="B")
            ax.set_title(f"{m.joint_name}  RMSE={m.rmse:.4f}")
            ax.legend(fontsize=7)
            ax.grid(True, alpha=0.3)

        for idx in range(n, rows * cols):
            axes[idx // cols][idx % cols].set_visible(False)
        fig.tight_layout()
        return fig


# ═══════════════════════════════════════════════════════════════════════
#  Resampling helpers
# ═══════════════════════════════════════════════════════════════════════
def _resample(t_src: np.ndarray, q_src: np.ndarray,
              t_target: np.ndarray) -> np.ndarray:
    """Linear interpolation of (t_src, q_src) onto t_target."""
    if q_src.ndim == 1:
        q_src = q_src[:, None]
    n_joints = q_src.shape[1]
    out = np.empty((len(t_target), n_joints))
    for j in range(n_joints):
        out[:, j] = np.interp(t_target, t_src, q_src[:, j],
                              left=q_src[0, j], right=q_src[-1, j])
    return out


def _common_grid(t_a: np.ndarray, t_b: np.ndarray,
                 dt: float) -> tuple[np.ndarray, float]:
    """Uniform grid over the overlapping time range."""
    t0 = max(t_a[0], t_b[0])
    t1 = min(t_a[-1], t_b[-1])
    if t1 <= t0:
        raise ValueError(
            f"No overlap: A=[{t_a[0]:.4f},{t_a[-1]:.4f}] "
            f"B=[{t_b[0]:.4f},{t_b[-1]:.4f}]"
        )
    n = max(2, int(np.ceil((t1 - t0) / dt)) + 1)
    return np.linspace(t0, t1, n), dt


# ═══════════════════════════════════════════════════════════════════════
#  Core comparison — pure numpy, no external dependencies
# ═══════════════════════════════════════════════════════════════════════
def compare_trajectories(
    t_a: np.ndarray,
    q_a: np.ndarray,
    t_b: np.ndarray,
    q_b: np.ndarray,
    joint_names: Optional[list[str]] = None,
    dt: Optional[float] = None,
    label_a: str = "A",
    label_b: str = "B",
) -> TrajectoryResult:
    """
    Compare two joint trajectories with (possibly) different time grids.

    Both are resampled to a common uniform grid via linear interpolation,
    then RMSE, max error, R², and NRMSE% are computed per joint.

    Parameters
    ----------
    t_a : (N_a,) array
        Time stamps of trajectory A.
    q_a : (N_a, J) array
        Joint positions of trajectory A.
    t_b : (N_b,) array
        Time stamps of trajectory B (may be non-uniform).
    q_b : (N_b, J) array
        Joint positions of trajectory B.
    joint_names : list[str], optional
        Labels for each joint column.
    dt : float, optional
        Resolution of the common grid.  Default: inferred from median
        step of the trajectory with more points.
    label_a, label_b : str
        Used only in plot legend.

    Returns
    -------
    TrajectoryResult
    """
    t_a = np.asarray(t_a, dtype=np.float64).ravel()
    t_b = np.asarray(t_b, dtype=np.float64).ravel()
    q_a = np.atleast_2d(np.asarray(q_a, dtype=np.float64))
    q_b = np.atleast_2d(np.asarray(q_b, dtype=np.float64))

    assert len(t_a) == q_a.shape[0], f"t_a({len(t_a)}) != q_a rows({q_a.shape[0]})"
    assert len(t_b) == q_b.shape[0], f"t_b({len(t_b)}) != q_b rows({q_b.shape[0]})"
    n_joints = q_a.shape[1]
    assert q_b.shape[1] == n_joints, f"Joint count: A={n_joints} B={q_b.shape[1]}"

    if dt is None:
        # pick the finer of the two median steps
        dt = min(
            float(np.median(np.diff(t_a))) if len(t_a) > 1 else 0.001,
            float(np.median(np.diff(t_b))) if len(t_b) > 1 else 0.001,
        )
    dt = max(dt, 1e-8)

    if joint_names is None:
        joint_names = [f"joint_{j}" for j in range(n_joints)]

    t_common, dt_common = _common_grid(t_a, t_b, dt)
    q_ar = _resample(t_a, q_a, t_common)
    q_br = _resample(t_b, q_b, t_common)

    result = TrajectoryResult(
        common_time=t_common,
        q_a_resampled=q_ar, q_b_resampled=q_br,
        t_a_original=t_a,   t_b_original=t_b,
        dt_ref=dt, dt_common=dt_common,
        n_steps_a=len(t_a), n_steps_b=len(t_b), n_common=len(t_common),
        q_a_raw=q_a, q_b_raw=q_b,
    )

    for j in range(n_joints):
        diff = q_ar[:, j] - q_br[:, j]
        ss_res = float(np.sum(diff ** 2))
        mean_a = float(np.mean(q_ar[:, j]))
        ss_tot = float(np.sum((q_ar[:, j] - mean_a) ** 2))
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else float("nan")
        idx = int(np.argmax(np.abs(diff)))

        result.joint_metrics.append(JointMetrics(
            joint_name=joint_names[j],
            rmse=float(np.sqrt(ss_res / len(diff))),
            max_err=float(np.abs(diff[idx])),
            max_err_time=float(t_common[idx]),
            r2=r2,
            mean_a=mean_a,
            mean_b=float(np.mean(q_br[:, j])),
            range_a=float(np.ptp(q_ar[:, j])),
            range_b=float(np.ptp(q_br[:, j])),
            n_points=len(diff),
        ))

    return result


# ═══════════════════════════════════════════════════════════════════════
#  Self-test with synthetic data
# ═══════════════════════════════════════════════════════════════════════
def _demo():
    """Standalone demo — no engines, no torch, just numpy."""
    np.random.seed(42)
    n_joints = 12
    dt_fixed = 0.005

    # A: uniform grid (like MuJoCo)
    t_a = np.arange(0, 2.0, dt_fixed)
    omega = 2 * np.pi * 2.0
    q_a = 0.5 * np.sin(omega * t_a[:, None] + np.arange(n_joints) * 0.3)
    q_a *= np.exp(-0.3 * t_a[:, None])

    # B: variable grid (like Sire) + small noise
    np.random.seed(7)
    dt_b = dt_fixed * (0.5 + 0.5 * np.abs(np.sin(np.linspace(0, 4*np.pi, 500))))
    t_b = np.cumsum(dt_b); t_b = t_b[t_b <= t_a[-1]]
    q_b = 0.5 * np.sin(omega * t_b[:, None] + np.arange(n_joints) * 0.3)
    q_b *= np.exp(-0.3 * t_b[:, None])
    q_b += np.random.randn(*q_b.shape) * 0.02

    result = compare_trajectories(t_a, q_a, t_b, q_b,
                                  label_a="Fixed-step", label_b="Variable-step")
    print(result.summary())
    print(f"\nOverall RMSE={result.overall_rmse:.4f}  "
          f"MaxErr={result.overall_max_err:.4f}")
    print(f"A: {result.n_steps_a} pts  B: {result.n_steps_b} pts  "
          f"common: {result.n_common} pts")


if __name__ == "__main__":
    _demo()
