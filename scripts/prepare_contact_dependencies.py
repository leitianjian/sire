"""Verify published dependencies; retain patch support for their historical bases."""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess

ROOT = Path(__file__).resolve().parents[1]

def digest(path, normalize=True):
    data = path.read_bytes()
    if normalize:
        data = data.replace(b'\r\n', b'\n')
    return hashlib.sha256(data).hexdigest()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true', help='Apply only pristine, matching dependency patches')
    args = parser.parse_args()
    rows = json.loads((ROOT / 'third_party/patches/manifest.json').read_text(encoding='utf-8'))
    pending = []
    for row in rows:
        sub = ROOT / row['path']
        head = subprocess.check_output(['git','-C',str(sub),'rev-parse','HEAD'],text=True).strip()
        published = row.get('patched_commit')
        if head not in (row['base_commit'], published):
            raise RuntimeError(f"{row['path']}: expected published {published} or legacy base {row['base_commit']}, got {head}")
        patch = ROOT / row['patch']
        if digest(patch, normalize=False) != row['patch_sha256']:
            raise RuntimeError(f'Patch checksum differs: {patch}')
        states = []
        for f in row['files']:
            value = digest(sub / f['path'])
            states.append('patched' if value == f['after_sha256'] else
                          'pristine' if value == f['before_sha256'] else 'modified')
        if head == published:
            if set(states) != {'patched'}:
                raise RuntimeError(f"{row['path']}: published revision has unrecognized source edits")
            print(f"{row['path']}: published revision and source checksums verified")
        elif set(states) == {'patched'}:
            print(f"{row['path']}: historical base with patch already present")
        elif set(states) == {'pristine'}:
            subprocess.run(['git','-C',str(sub),'apply','--check',str(patch)],check=True)
            pending.append((sub,patch))
            print(f"{row['path']}: matching patch ready")
        else:
            raise RuntimeError(f"{row['path']}: mixed/unrecognized edits; preserve and inspect them manually")
    if pending and not args.apply:
        parser.exit(1, 'Run again with --apply before building. No files changed.\n')
    for sub,patch in pending:
        subprocess.run(['git','-C',str(sub),'apply',str(patch)],check=True)
        print(f'{sub.name}: patch applied')
    for row in rows:
        assert all(digest(ROOT/row['path']/f['path']) == f['after_sha256'] for f in row['files'])
    print('PASS: dependency commits, patch checksums and patched sources verified')

if __name__ == '__main__':
    main()
