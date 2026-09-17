# Paper 2 — Experiment 1: single-point contact law

This demo validates the event-targeted single-point contact path without requiring a physical impact rig.

It performs two distinct comparisons:

1. Full simulation trajectories: shifted spectral ADMM versus the unshifted spectral-ADMM ablation, with the same collision, contact-time, integration, and warm-start configuration.
2. Frozen contact systems: unshifted spectral ADMM, shifted spectral ADMM, and the high-accuracy Exact-Coulomb reference on identical recorded matrices and targets.

The scene is a sphere with independently specified normal and tangential impact velocities hitting a fixed horizontal plane.
The sphere/plane pair produces one contact point and therefore matches the paper's current single-point scope.

## Run

Use the project virtual environment and the locally installed Sire module.

Quick smoke run:

```powershell
D:\code\sire\.venv\Scripts\python.exe `
  D:\code\sire\demo\demo_paper2\run_contact_law_experiment.py `
  --preset smoke
```

Full one-factor-at-a-time sweep:

```powershell
D:\code\sire\.venv\Scripts\python.exe `
  D:\code\sire\demo\demo_paper2\run_contact_law_experiment.py `
  --preset full `
  --timing-runs 20
```

Pure-normal time-step convergence sweep (0.125--2 ms):

```powershell
D:\code\sire\.venv\Scripts\python.exe `
  D:\code\sire\demo\demo_paper2\run_contact_law_experiment.py `
  --preset convergence `
  --skip-replay
```

Fast trajectory-only check, without the offline Exact replay:

```powershell
D:\code\sire\.venv\Scripts\python.exe `
  D:\code\sire\demo\demo_paper2\run_contact_law_experiment.py `
  --preset smoke `
  --skip-replay
```

Results are written to `demo/demo_paper2/results/<timestamp>` by default.

## Outputs

- `cases.json`: exact sweep parameters.
- `summaries.csv`: first-contact duration, terminal velocity, penetration, impulse, and sliding metrics.
- `aggregate_contact_law.png`: analytical-versus-simulated contact duration and terminal normal velocity.
- `timestep_convergence.csv`: pure-normal discretization errors and fitted convergence orders (convergence preset).
- `timestep_convergence.png`: log--log time-step convergence plot (convergence preset).
- `runs/<case>/<variant>/trajectory.csv`: complete recorded trajectory.
- `runs/<case>/trajectory_comparison.png`: depth, normal velocity, force, and tangential-speed overlay.
- `runs/<case>/first_contact_comparison.png`: zoomed view of the first impact and separation.
- `runs/<case>/<variant>/traces/trajectory_*`: frozen contact systems recorded by Sire.
- `solver_comparison/<case>/metrics.csv`: offline comparison against Exact-Coulomb.
- `solver_comparison/<case>/trajectory_comparison.png`: force and induced-velocity differences from the Exact reference.

## Interpretation

The Exact solver is a numerical reference for the shifted NCP, not physical ground truth.
The comparison has two intended conclusions:

- Shifted spectral ADMM should closely match Exact-Coulomb on the same shifted contact systems.
- The unshifted formulation can converge for its own problem but should fail to reproduce a nonzero event-derived terminal normal velocity.

The analytical Kelvin–Voigt curve is included as a transparent single-body check.
It includes gravity and uses the first detected modified depth and pre-contact normal speed.
Small discrepancies are expected because collision detection begins at a discrete configuration and the full Sire pipeline recomputes the coupled contact state at accepted event intervals.
If a contact has not separated before the configured simulation horizon, `contact_time_is_lower_bound=1` and terminal-velocity error is intentionally left undefined rather than treating the last sample as a separation event.

Do not interpret this demo alone as real-world contact-model validation; the paper's hardware evidence comes from the robot-policy experiment.
