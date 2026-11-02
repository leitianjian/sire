"""Re-export Fig. 5 from archived real-image composites without decoding videos."""
import argparse
import json
from pathlib import Path
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib import font_manager
import scienceplots  # noqa: F401
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source = ROOT / 'data/montage'
    manifest = json.loads((source / 'manifest.json').read_text(encoding='utf-8'))
    seed = json.loads((source / 'seed_manifest.json').read_text(encoding='utf-8'))
    tiles = [Image.open(source / f'seed_{i:02d}.png').convert('RGB') for i in range(1, 9)]
    tiles += [Image.open(source / f'smooth_{i:02d}.png').convert('RGB') for i in range(1, 5)]
    units = seed['units'][:8] + manifest['smooth_units']
    width = 183.0
    height = manifest['figure_size_mm'][1]
    margin, col_gap, row_gap, heading = 1.0, 1.1, 1.5, 4.0
    tile_w = (width - 2 * margin - 3 * col_gap) / 4
    tile_h = tile_w * 920 / 1660
    with plt.style.context(['science', 'ieee']):
        for filename in ('times.ttf', 'timesbd.ttf', 'timesi.ttf', 'timesbi.ttf'):
            font = Path('C:/Windows/Fonts') / filename
            if font.is_file():
                font_manager.fontManager.addfont(str(font))
        mpl.rcParams.update({'pdf.fonttype': 42, 'svg.fonttype': 'none', 'font.size': 8,
                             'text.usetex': False, 'font.family': 'serif',
                             'font.serif': ['Times New Roman', 'Times', 'DejaVu Serif']})
        fig = plt.figure(figsize=(width / 25.4, height / 25.4), facecolor='white')
        for index, (tile, unit) in enumerate(zip(tiles, units)):
            row, col = divmod(index, 4)
            bottom = margin + (2 - row) * (tile_h + row_gap) + (heading if row < 2 else 0)
            ax = fig.add_axes([(margin + col * (tile_w + col_gap)) / width,
                               bottom / height, tile_w / width, tile_h / height])
            ax.imshow(tile, interpolation='none')
            ax.axis('off')
            ax.text(.025, .965, f"{unit['start']:.1f}–{unit['end']:.1f} s",
                    transform=ax.transAxes, ha='left', va='top', fontsize=6.5,
                    bbox=dict(facecolor='white', edgecolor='none', alpha=.9, pad=1.2))
            if index in (2, 11):
                ax.text(.025, .045, 'Fall and recovery' if index == 2 else 'Fall',
                        transform=ax.transAxes, ha='left', va='bottom', fontsize=6.5,
                        bbox=dict(facecolor='white', edgecolor='none', alpha=.9, pad=1.2))
        fig.text(margin / width, (height - margin - 1) / height, 'Proposed (seed 2)', va='top', fontsize=7)
        fig.text(margin / width, (margin + tile_h + heading - .5) / height, 'Velocity-regularized (seed 2)', va='top', fontsize=7)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        for extension in ('.pdf', '.png', '.svg'):
            fig.savefig(args.output.with_suffix(extension), dpi=400 if extension == '.png' else 600, facecolor='white')
        plt.close(fig)


if __name__ == '__main__':
    main()
