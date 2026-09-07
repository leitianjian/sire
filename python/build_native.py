"""Fast development build, without pip metadata or a full project install."""
import argparse
from native_build import build_native


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=('release', 'debug', 'profile'))
    parser.add_argument('--jobs', type=int)
    parser.add_argument('--dry-run', action='store_true',
                        help='Print paths and settings without running CMake or copying files')
    args = parser.parse_args()
    build_native(args.profile, args.jobs, args.dry_run)


if __name__ == '__main__':
    main()
