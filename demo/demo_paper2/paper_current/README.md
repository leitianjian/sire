# Trajectory-referenced frictional contact: paper reproduction

This is the maintained experiment and plotting entry point for
**A Velocity-Shifted Frictional Contact Solver for High-Stiffness Contact with Explicit Separation Time**.
The implementation is in the Sire engine, not in a standalone plotting model.
The manuscript version used for the figure mapping below is v7.

## Environment and checks

Build and run Sire with the same project Python interpreter. On this checkout:

```powershell
$py = 'D:/code/sire/.venv/Scripts/python.exe'
$paper = 'D:/code/sire/demo/demo_paper2/paper_current'
& $py "$paper/run.py" check
& $py "$paper/run.py" plot --dry-run
```

On another machine, substitute the checkout and virtual-environment paths.
See [dependency patches](../../../docs/contact_implementation.md) before building,
and [the native build workflow](../../../python/BUILDING.md).
The GitHub release contains code and configurations only; plotting requires
restoring the corresponding research inputs from the local archive.
Install plotting dependencies from `requirements-plot.txt`; analysis additionally
uses `requirements-analysis.txt`. SciencePlots uses `science`/`ieee`, a LaTeX
installation is needed for curve text, and scene cropping needs Poppler's
`pdftoppm`. The plotting style is documented in `FIGURE_STYLE.md`.

`check` verifies SHA256 for the code/configurations distributed in Git and
checks Python syntax. Cohort/metric checks also run when the optional research
inputs are present. The local-only `archive_inventory.json` identifies the
complete archive, including raw data and checkpoints; `check --full-archive`
requires that local manifest and its files. Neither check imports
Sire, runs an experiment, trains a policy, nor operates a robot.

## Recreate the current figures and tables (requires local research inputs)

```powershell
& $py "$paper/run.py" plot --output-dir "$paper/outputs/figures"
```

The CLI's historical figure keys are retained for compatibility. They are
**not manuscript figure numbers**. Use this mapping instead:

| Current manuscript item | CLI `plot --figure` key | Script/input |
| --- | --- | --- |
| Fig. 1(a), sphere--plane scene | `contact-setup` | `plot_contact_setup.py` |
| Fig. 1(b), inclined-box scene | `fig2` | Vector crop of `res/sliding_box.pdf` |
| Fig. 2(a,b), normal-reference ablation and tangential violation | `fig1` | `plot_contact_law_two_panel.py`; `res/source_data/experiment1/dissipation_20261029` |
| Fig. 3, incline reversal and holding | `fig3` | `plot_box_sliding_combined_paper_figure.py`; `res/incline_audited_20261029` |
| Fig. 4(a), drop scene | copied by `all` | `res/sphere_free_fall_test.pdf` |
| Fig. 4(b), high-stiffness normal impact | `fig4` | `plot_high_stiffness_regression.py`; `res/high_stiffness_release4m_20261020/normal` |
| Fig. 5, four PPO runs; Table III, reward components | `fig6` | `plot_ppo_reward_seeds.py`; `res/source_data/rl_training_comparison` |
| Fig. 6, onboard forward/turning response | `fig7` | `plot_seed2_velocity_response.py`; `analysis/seed2_velocity_comparison` |
| Fig. 7, real-robot motion sequences | `fig5` | `plot_motion_montage.py`; twelve archived `data/montage` image tiles |
| Table IV, command-matched hardware metrics | `table4` | `build_seed2_prefall_table.py`; `analysis/hardware_normal_motion_20261031` |

Fig. 2 exports individual panel PDFs for LaTeX subfigures as well as a combined
historical output. Fig. 3 uses a 2 s display window. Fig. 4(b) uses first rebound
bottom clearance divided by the initial 4 m bottom clearance; it does not use
an incident-speed-derived ballistic denominator. Fig. 5 displays individual
runs with the original 25-iteration trailing mean, not a cross-seed average.
Saved data, units, normalization and source recordings are not changed by plotting.
All new output goes to ignored `outputs/`; scene PDFs remain vector inputs.

## Run new Sire contact experiments

```powershell
& $py "$paper/run.py" simulate contact-smoke --output-dir "$paper/outputs/smoke"
& $py "$paper/run.py" simulate impact-audit --output-dir "$paper/outputs/impact"
& $py "$paper/run.py" simulate incline-audit --output-dir "$paper/outputs/incline"
& $py "$paper/run.py" simulate normal --output-dir "$paper/outputs/normal"
```

The impact sweep runs the two solvers at 46 initial tangential speeds and an
otherwise identical zero-normal-reference ablation. These are complete Sire
sphere--plane simulations, with translation and rotation. The incline command
runs both solvers at friction coefficients 0.2 and 0.3 using four supports.
The normal command reproduces the previous 4 m release configuration and its
stiffness sweep; the current figure selects eight stiffnesses through 1e30 N/m.

Recompute Fig. 2(b)'s interval-level vector projection violation from a new run:

```powershell
& $py "$paper/scripts/analyze_impact_dissipation.py" --input "$paper/outputs/impact" --output-dir "$paper/outputs/dissipation"
```

The two methods use the same frozen impulse--velocity relation, each with its
own recorded interval-start state and geometry. Normalizing requires positive
normal impulse. Historical frozen-load tests and the old c/d panels remain
available in scripts for traceability; they are not current Sire-scene evidence.

## Training and hardware data

The Git distribution includes the implementation, experiment/plotting/analysis
scripts, deployment tools, saved training configurations and training-scene XML.
Extracted curves, reward summaries, hardware metrics and images, raw logs,
weights, videos, manuscript snapshots and generated figures remain local.
Checksums for the full local archive are in the local-only
`archive_inventory.json`; no public data-download URL is asserted.
A clone can inspect configurations and run new simulations after building
Sire. Plotting, reanalyzing original records, replaying archived traces or
deploying saved policies requires restoring the corresponding research inputs.
A new simulation does not require old logs.

Inspect configurations without starting training:

```powershell
& $py "$paper/scripts/train_frozen_policy.py" --method shifted --seed 2
& $py "$paper/scripts/train_frozen_policy.py" --method smooth --seed 2
```

Explicit `--train` launches a new run and requires the SireRLGym/PyTorch/CUDA
environment. Current engine code and the historical training checkout are not
claimed to be bitwise identical; frozen configurations record the original
settings. Recorded runs are Proposed seeds 2/3/4 and Velocity-regularized seed 2.

The current Table IV uses the September 28 Proposed seed-2 session shown in
Figs. 6/7 and the October 15 velocity-regularized seed-2 session. It excludes
tilt events of at least 20 degrees with a fixed 2 s margin on both sides, then
matches commands within the first 40 s. The resulting shared exposure is
15.30 s in four bins. The 1 s and 3 s margin analyses are retained in the report.
The older October 15 Proposed comparison is historical, not the current table.

`run.py analyze training`, `training-comparison`, `hardware` and `prefall`
require the optional raw archives; each requires an explicit `--output-dir`.
Deployment tools live in `hardware/` and are never invoked by `check`, `plot`,
or the simulation commands.
