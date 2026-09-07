"""Shared native build path for pip and the fast development command."""
from __future__ import annotations

import filecmp
import hashlib
import importlib.machinery
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import sysconfig
try:
    import tomllib
except ModuleNotFoundError:  # Python 3.8-3.10
    import tomli as tomllib

PYTHON_ROOT = Path(__file__).resolve().parent
REPO_ROOT = PYTHON_ROOT.parent


def require_project_python():
    expected = REPO_ROOT / '.venv'
    if not expected.is_dir() or Path(sys.prefix).resolve() != expected.resolve():
        executable = expected / ('Scripts/python.exe' if os.name == 'nt' else 'bin/python')
        raise RuntimeError(
            f'Use the project venv: {executable}\n'
            f'Current interpreter: {sys.executable}\n'
            'Building with another Python can overwrite incompatible native artifacts.')


def load_config():
    config = dict(cpp_base_path=str(REPO_ROOT), cpp_build_type='release',
                  py_install_type='release', build_python=True, build_demo=False,
                  build_test=False, sire_enable_tracy=False, rerun_config=False,
                  sire_clarabel_force_source_build=False,
                  rm_cmake_cache=False, cmake_only=False)
    for path in (PYTHON_ROOT / 'pyproject.toml', PYTHON_ROOT / 'local.toml'):
        if path.exists():
            with path.open('rb') as stream:
                config.update(tomllib.load(stream).get('tool', {}).get('sire', {}).get('build', {}))
    for key, value in list(config.items()):
        override = os.environ.get('BUILD_' + key.upper())
        if override is not None:
            if isinstance(value, bool):
                if override.lower() not in ('1', '0', 'true', 'false', 'on', 'off', 'yes', 'no'):
                    raise ValueError(f'Invalid boolean for BUILD_{key.upper()}: {override}')
                value = override.lower() in ('1', 'true', 'on', 'yes')
            elif isinstance(value, int):
                value = int(override)
            else:
                value = override
        config[key] = os.path.expandvars(value) if isinstance(value, str) else value
    return config


def runtime_tag():
    return f'{sys.implementation.cache_tag}-{sysconfig.get_platform()}'


def build_plan(config, profile=None):
    profile = profile or ('profile' if config['sire_enable_tracy'] else config['py_install_type'])
    if profile not in ('release', 'debug', 'profile'):
        raise ValueError('profile must be release, debug or profile')
    mode = 'Debug' if profile == 'debug' else 'Release'
    identity = hashlib.sha256(str(Path(sys.prefix).resolve()).lower().encode()).hexdigest()[:10]
    tag = f'{runtime_tag()}-{identity}-{profile}'
    base = Path(config.get('build_dir') or REPO_ROOT / 'build' / 'python') / tag
    stage = Path(config.get('install_dir') or REPO_ROOT / 'build' / 'python-install') / tag
    runtime = PYTHON_ROOT / 'src' / 'sire' / 'native' / '_runtime' / runtime_tag()
    def dependency_prefix(value):
        if not value:
            return ''
        path = Path(value)
        # Windows dependency installs are split into Debug/Release folders.
        # Unix installations use a single prefix such as /usr or /usr/local.
        return str(path / mode) if os.name == 'nt' else str(path)

    defines = {
        'BUILD_PYTHON': True, 'BUILD_DEMO': bool(config['build_demo']),
        'BUILD_TEST': bool(config['build_test']), 'SIRE_ENABLE_TRACY': profile == 'profile',
        'SIRE_CLARABEL_FORCE_SOURCE_BUILD':
            bool(config['sire_clarabel_force_source_build']),
        'CMAKE_BUILD_TYPE': mode, 'PYTHON_EXECUTABLE': sys.executable,
        'TARGET_ARIS_PATH': config.get('aris_path', ''),
        'TARGET_HPP_FCL_PATH': dependency_prefix(config.get('fcl_path', '')),
        'TARGET_STDUUID_PATH': dependency_prefix(config.get('uuid_path', '')),
    }
    if config.get('toolchain_path'):
        defines['CMAKE_TOOLCHAIN_FILE'] = config['toolchain_path']
    return dict(profile=profile, mode=mode, base=base, build=base / mode,
                stage=stage, runtime=runtime, defines=defines)


def sync_runtime(files, destination):
    """Sync only manifest-owned files; do not sweep an arbitrary DLL directory."""
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    manifest = destination / '.sire-files.json'
    previous = json.loads(manifest.read_text()) if manifest.exists() else []
    names = []
    changed = 0
    for source in files:
        source = Path(source)
        target = destination / source.name
        names.append(source.name)
        if not target.exists() or not filecmp.cmp(source, target, shallow=False):
            shutil.copy2(source, target)
            changed += 1
    for name in set(previous) - set(names):
        target = destination / name
        if Path(name).name != name or target.resolve().parent != destination.resolve():
            raise RuntimeError(f'Unsafe runtime manifest entry: {name}')
        target.unlink(missing_ok=True)
    manifest.write_text(json.dumps(sorted(names)), encoding='utf-8')
    print(f'[sire] runtime: {destination} ({changed} files updated)', flush=True)
    return [destination / name for name in names]


def verify_runtime(runtime):
    # Import the extension directly: avoids optional visualization dependencies
    # while checking the ABI, dependent DLLs and the current RL interface.
    code = """
import importlib.util, os, pathlib, sys
root = pathlib.Path(sys.argv[1])
handle = os.add_dll_directory(str(root)) if os.name == 'nt' else None
path = root / sys.argv[2]
spec = importlib.util.spec_from_file_location('sire', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
assert hasattr(module, 'SireRLBatchStepper'), 'native extension lacks SireRLBatchStepper'
assert hasattr(module.SireRLBatchStepper, 'setHistoryRecording'), 'native extension lacks setHistoryRecording'
print('[sire] verified:', sys.executable, path, 'Tracy:', module.tracyEnabled)
"""
    extension = next((runtime / ('sire' + suffix)
                      for suffix in importlib.machinery.EXTENSION_SUFFIXES
                      if (runtime / ('sire' + suffix)).is_file()), None)
    if extension is None:
        raise RuntimeError(f'No extension for {runtime_tag()} in {runtime}')
    subprocess.run([sys.executable, '-B', '-c', code, str(runtime), extension.name], check=True)


def build_native(profile=None, jobs=None, dry_run=False):
    require_project_python()
    config = load_config()
    if not config['build_python']:
        raise ValueError('Python packaging requires build_python=true')
    if config['cpp_build_type'] not in ('release', 'debug', 'all'):
        raise ValueError('cpp_build_type must be release, debug or all')
    if config['cpp_build_type'] != 'all' and config['py_install_type'] != config['cpp_build_type'] and profile is None:
        raise ValueError('py_install_type must match cpp_build_type')
    plan = build_plan(config, profile)
    if jobs is None:
        jobs = int(os.environ.get('MAX_JOBS', min(8, os.cpu_count() or 1)))
    if jobs < 1:
        raise ValueError('jobs must be positive')
    print(f'[sire] Python: {sys.executable}\n[sire] build: {plan["build"]}\n'
          f'[sire] profile: {plan["profile"]}; target: sire; jobs: {jobs}', flush=True)
    if dry_run:
        print('[sire] dry run: no configure, build, install or copy')
        print(json.dumps({**plan, 'defines': plan['defines']}, default=str, indent=2))
        return plan['runtime']

    from cmake_py import _create_build_env
    from cmake_py.cmake import CMake
    env = _create_build_env()
    # pip does not necessarily activate the venv before invoking build_ext.
    # Make its cmake/ninja executables discoverable on every platform.
    executable_dir = str(Path(sys.executable).resolve().parent)
    env['PATH'] = executable_dir + os.pathsep + env.get('PATH', os.environ.get('PATH', ''))
    env['USE_NINJA'] = '1'
    env['CMAKE_GENERATOR'] = 'Ninja'
    env['CMAKE_BUILD_TYPE'] = plan['mode']
    for key in ('http_proxy', 'https_proxy'):
        if config.get('cmake_' + key):
            env[key.upper()] = config['cmake_' + key]
    cmake = CMake(config['cpp_base_path'], str(plan['base']), str(plan['stage']), env=env)
    cmake.defines(**plan['defines'])
    cmake.configure(env, config['rerun_config'], config['rm_cmake_cache'])
    if config['cmake_only']:
        raise RuntimeError('cmake_only cannot produce an installable Python extension')
    cmake.run(['--build', cmake.build_dir, '--config', plan['mode'],
               '--target', 'sire', '--parallel', str(jobs)], env)
    cmake.run(['--install', cmake.build_dir, '--config', plan['mode'],
               '--prefix', str(plan['stage']), '--component', 'PythonRuntime'], env)
    manifest = Path(cmake.build_dir) / 'install_manifest_PythonRuntime.txt'
    files = [Path(line) for line in manifest.read_text().splitlines() if line.strip()]
    if not files:
        raise RuntimeError('PythonRuntime install manifest is empty')
    expected = (plan['stage'] / 'python' / plan['mode'].lower()).resolve()
    for file in files:
        if file.resolve().parent != expected or not file.is_file():
            raise RuntimeError(f'Unexpected/missing installed runtime file: {file}')
    # Verify before updating the active runtime, then verify the copied result.
    verify_runtime(expected)
    sync_runtime(files, plan['runtime'])
    verify_runtime(plan['runtime'])
    return plan['runtime']
