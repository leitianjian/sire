"""Current-paper entry point. Plot archives by default; simulations are explicit."""
from pathlib import Path
import argparse
import ast
import csv
import hashlib
import json
import os
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
HIGH = '1e7,1e9,1e12,1e15,1e18,1e21,1e24,1e30'


def execute(script, *args, dry=False):
    command = [sys.executable, str(ROOT / script), *map(str, args)]
    print(shlex.join(command), flush=True)
    if not dry:
        env = dict(os.environ, MPLBACKEND='Agg')
        env.setdefault('MPLCONFIGDIR', str(ROOT / 'outputs/.mplconfig'))
        subprocess.run(command, cwd=ROOT, env=env, check=True)


def check(full_archive=False):
    manifest = 'archive_inventory.json' if full_archive else 'inventory.json'
    records = json.loads((ROOT / manifest).read_text(encoding='utf-8'))
    errors = []
    for row in records:
        path = ROOT / row['path']
        if not path.is_file():
            errors.append(f"Missing: {row['path']}")
        elif hashlib.sha256(path.read_bytes()).hexdigest() != row['sha256']:
            errors.append(f"Hash differs: {row['path']}")
    for p in ROOT.rglob('*.py'):
        if not any(x in p.parts for x in ('outputs', '__pycache__')):
            ast.parse(p.read_text(encoding='utf-8-sig'), filename=str(p))
    if (ROOT / "res/source_data/rl_training_comparison/training_curves.csv").is_file():
        table = list(csv.DictReader((ROOT / 'res/source_data/seed2_prefall_comparison/table_values.csv').open(encoding='utf-8')))
        assert len(table) == 5
        assert {int(r['seed']) for r in csv.DictReader((ROOT / 'res/source_data/rl_seed234/multiseed_training_curves.csv').open())} == {2, 3, 4}
        curves = list(csv.DictReader((ROOT / 'res/source_data/rl_training_comparison/training_curves.csv').open()))
        expected = {('proposed', 2), ('proposed', 3), ('proposed', 4), ('velocity_regularized', 2)}
        assert {(r['method'], int(r['seed'])) for r in curves} == expected
        for method, seed in expected:
            run = [r for r in curves if r['method'] == method and int(r['seed']) == seed]
            assert [int(r['iteration']) for r in run] == list(range(1, 1201))
            assert all(r['iteration'] == r['raw_step'] for r in run)
    else:
        print("Optional research inputs absent: cohort/metric checks skipped (code-only checkout).")
    if errors:
        raise RuntimeError('\n'.join(errors))
    print(f'PASS: {len(records)} manifest files and Python syntax verified.')


def plot(selection, out, dry):
    out.mkdir(parents=True, exist_ok=True)
    jobs = {
        'fig1': ('scripts/plot_contact_law_two_panel.py',
                 '--source-dir', ROOT / 'res/source_data/experiment1/dissipation_20261029',
                 '--output', out / 'contact_law_validation_two_panel'),
        'contact-setup': ('scripts/plot_contact_setup.py',
                          '--output', out / 'sphere_oblique_contact_setup'),
        'fig2': ('scripts/crop_scene_pdf.py', '--input', ROOT / 'res/sliding_box.pdf',
                 '--output', out / 'sliding_box_cropped.pdf', '--margin', 2),
        'fig3': ('scripts/plot_box_sliding_combined_paper_figure.py',
                 '--sliding-dir', ROOT / 'res/incline_audited_20261029/mu_0.2',
                 '--static-dir', ROOT / 'res/incline_audited_20261029/mu_0.3',
                 '--output', out / 'box_sliding_validation_final_layout'),
        'fig4': ('scripts/plot_high_stiffness_regression.py',
                 '--input', ROOT / 'res/high_stiffness_release4m_20261020/normal/high_stiffness_regression.csv',
                 '--stiffness', HIGH, '--reference', 'release', '--width', '2.4', '--height', '2.25',
                 '--output', out / 'high_stiffness_normal_impact_panel_b'),
        'fig5': ('scripts/plot_motion_montage.py', '--output', out / 'seed2_smooth_hardware_montage'),
        'fig6': ('scripts/plot_ppo_reward_seeds.py',
                 '--source-dir', ROOT / 'res/source_data/rl_multiseed',
                 '--comparison-dir', ROOT / 'res/source_data/rl_training_comparison',
                 '--cohort-dir', out / 'rl_training_comparison', '--output', out / 'ppo_reward_training_comparison'),
        'fig7': ('scripts/plot_seed2_velocity_response.py',
                 '--input', ROOT / 'analysis/seed2_velocity_comparison/seed2_velocity_comparison_full.csv',
                 '--output', out / 'seed2_forward_turn_response'),
        'table4': ('scripts/build_seed2_prefall_table.py',
                   '--input', ROOT / 'analysis/hardware_normal_motion_20261031/normal_motion_reanalysis.json',
                   '--output', out / 'seed2_prefall_comparison_table.tex',
                   '--source-data', out / 'seed2_prefall_source_data'),
    }
    for key in jobs if selection == 'all' else [selection]:
        execute(*jobs[key], dry=dry)
    if selection == 'all' and not dry:
        import shutil
        for name in ('sphere_free_fall_test.pdf',):
            shutil.copy2(ROOT / 'res' / name, out / name)
        print('Cropped Fig. 2 and copied Fig. 4(a); no simulation, training or robot operation performed.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest='command', required=True)
    p = subs.add_parser('check', help='Verify versioned inputs and syntax without importing Sire')
    p.add_argument('--full-archive', action='store_true', help='Also require the optional local raw logs and weights')
    p = subs.add_parser('plot', help='Replot the exact archived manuscript inputs')
    p.add_argument('--figure', choices=['all', 'fig1', 'contact-setup', 'fig2', 'fig3', 'fig4', 'fig5', 'fig6', 'fig7', 'table4'], default='all')
    p.add_argument('--output-dir', type=Path, default=ROOT / 'outputs/figures')
    p.add_argument('--dry-run', action='store_true')
    p = subs.add_parser('simulate', help='Explicitly run new simulations with the installed Sire binary')
    p.add_argument('experiment', choices=['contact-smoke', 'contact-paper', 'contact-corner', 'contact-friction-comparison', 'impact-audit', 'incline-audit', 'incline', 'normal', 'oblique', 'refinement', 'replay'])
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--dry-run', action='store_true')
    p = subs.add_parser('analyze', help='Reanalyze bundled raw training and hardware records')
    p.add_argument('experiment', choices=['prefall', 'training', 'training-comparison', 'hardware'])
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    if args.command == 'check':
        return check(args.full_archive)
    out = args.output_dir.resolve()
    if args.command == 'plot':
        return plot(args.figure, out, args.dry_run)
    if args.command == 'simulate':
        kind = args.experiment
        if kind == 'impact-audit':
            execute('scripts/audit_impact_experiments.py', '--output-dir', out, dry=args.dry_run)
        elif kind == 'incline-audit':
            execute('scripts/audit_incline_experiments.py', '--output-dir', out, dry=args.dry_run)
        elif kind == 'contact-corner':
            execute('scripts/refine_contact_impulse.py', '--output-dir', out, dry=args.dry_run)
        elif kind == 'contact-friction-comparison':
            execute('scripts/compare_contact_impulse.py', '--output-dir', out, dry=args.dry_run)
        elif kind.startswith('contact-'):
            execute('simulation/run_contact_law_experiment.py', '--preset', kind.split('-')[1],
                    '--output-dir', out, '--timing-runs', 1, dry=args.dry_run)
        elif kind == 'incline':
            for mu in (0.2, 0.3):
                execute('simulation/run_box_sliding_experiment.py', '--friction', mu,
                        '--comparison-time', 2, '--output-dir', out / f'mu_{mu}', dry=args.dry_run)
        elif kind in ('normal', 'oblique', 'refinement'):
            options = ['--previous-paper'] if kind == 'normal' else ['--refinement-only'] if kind == 'refinement' else []
            execute('scripts/run_normal_stiffness_regression.py', '--sire-demo', ROOT / 'simulation',
                    '--output-dir', out, *options, dry=args.dry_run)
        else:
            trace = next((ROOT / 'data/contact_law/runs/nominal/shifted/traces').iterdir())
            execute('tools/compare_solvers.py', '--load', trace,
                    '--solvers', 'Shifted-Spectral-ADMM', 'Exact-Coulomb', '--reference', 'Exact-Coulomb',
                    '--max-iters', 200, '--max-err', 1e-8, '--runs', 1, '--output-dir', out, dry=args.dry_run)
        return
    if args.experiment == 'prefall':
        execute('scripts/analyze_hardware_normal_motion.py', '--proposed', ROOT / 'data/hardware_raw/seed_2',
                '--velocity-regularized', ROOT / 'data/prefall_raw/smooth', '--output-dir', out, dry=args.dry_run)
    elif args.experiment == 'training':
        execute('scripts/analyze_multiseed_training.py', ROOT / 'data/training', '--output-dir', out, dry=args.dry_run)
    elif args.experiment == 'training-comparison':
        execute('scripts/analyze_training_comparison.py', '--proposed-root', ROOT / 'data/training',
                '--velocity-root', ROOT / 'data/training_velocity_regularized/seed_2', '--output-dir', out, dry=args.dry_run)
    else:
        options = [v for seed in range(1, 5) for v in ('--run', f'{seed}={ROOT / "data/hardware_raw" / f"seed_{seed}"}')]
        execute('scripts/analyze_multiseed_hardware.py', *options, '--output-dir', out, dry=args.dry_run)


if __name__ == '__main__':
    main()
