"""Map new runner CSV names to the final three-panel plot without changing values."""
import argparse
from pathlib import Path
import shutil
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    mapped = args.output_dir / 'source_data'
    mapped.mkdir(parents=True, exist_ok=True)
    mapping = {
        'panel_a_trajectory.csv': 'panel_a_trajectory_v5.csv',
        'panel_c_impulse.csv': 'panel_c_impulse_v5.csv',
        'panel_d_static_friction.csv': 'panel_d_static_friction_v5.csv',
    }
    for original, target in mapping.items():
        candidates = [args.input_dir / 'paper_figure' / original, args.input_dir / original]
        if original.startswith('panel_d'):
            candidates += [args.input_dir / 'static_friction_sweep.csv']
        source = next((p for p in candidates if p.is_file()), None)
        if source is None:
            raise FileNotFoundError(f'Cannot find {original}; checked {candidates}')
        shutil.copy2(source, mapped / target)
    subprocess.run([sys.executable, str(Path(__file__).with_name('plot_contact_law_three_panel.py')),
                    '--source-dir', str(mapped), '--output', str(args.output_dir / 'contact_law_validation_three_panel')], check=True)


if __name__ == '__main__':
    main()
