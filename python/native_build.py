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

_CMAKE_OPTIONS = {
    'sire_enable_cuda': 'SIRE_ENABLE_CUDA',
    'cmake_cuda_compiler': 'CMAKE_CUDA_COMPILER',
    'cmake_cuda_architectures': 'CMAKE_CUDA_ARCHITECTURES',
    'cuda_toolkit_root': 'CUDAToolkit_ROOT',
    'cmake_prefix_path': 'CMAKE_PREFIX_PATH',
    'pybind11_dir': 'pybind11_DIR',
}


def load_config():
    config = dict(cpp_base_path=str(REPO_ROOT), cpp_build_type='release',
                  py_install_type='release', build_python=True, build_demo=False,
                  build_test=False, sire_enable_tracy=False, rerun_config=False,
                  sire_clarabel_force_source_build=False,
                  rm_cmake_cache=False, cmake_only=False, sire_enable_cuda=False,
                  cmake_cuda_compiler='', cmake_cuda_architectures='', cuda_toolkit_root='',
                  cmake_prefix_path='', pybind11_dir='', aris_path='', fcl_path='', uuid_path='',
                  build_dir='', install_dir='', toolchain_path='')
    for path in (PYTHON_ROOT / 'pyproject.toml', PYTHON_ROOT / 'local.toml'):
        if path.exists():
            with path.open('rb') as stream:
                config.update(tomllib.load(stream).get('tool', {}).get('sire', {}).get('build', {}))
    for key, value in list(config.items()):
        setting = 'BUILD_' + key.upper()
        if setting not in os.environ:
            setting = _CMAKE_OPTIONS.get(key, setting)
        override = os.environ.get(setting)
        if override is not None:
            if setting == 'CMAKE_PREFIX_PATH':
                override = override.replace(os.pathsep, ';')
            if isinstance(value, bool):
                if override.lower() not in ('1', '0', 'true', 'false', 'on', 'off', 'yes', 'no'):
                    raise ValueError(f'Invalid boolean for {setting}: {override}')
                value = override.lower() in ('1', 'true', 'on', 'yes')
            elif isinstance(value, int):
                value = int(override)
            else:
                value = override
        config[key] = os.path.expandvars(value) if isinstance(value, str) else value
    return config


def runtime_tag():
    # Keep this selection in sync with src/sire/native/__init__.py.
    parts = (os.path.normcase(str(Path(sys.prefix).resolve())),
             os.path.normcase(str(Path(sys.executable).resolve())),
             sysconfig.get_config_var('EXT_SUFFIX'))
    identity = hashlib.sha256(repr(parts).encode()).hexdigest()[:16]
    return f'{sys.implementation.cache_tag}-{sysconfig.get_platform()}-{identity}'


def build_plan(config, profile=None):
    profile = profile or ('profile' if config['sire_enable_tracy'] else config['py_install_type'])
    if profile not in ('release', 'debug', 'profile'):
        raise ValueError('profile must be release, debug or profile')
    mode = 'Debug' if profile == 'debug' else 'Release'
    tag = f'{runtime_tag()}-{profile}'
    if config['sire_enable_cuda']:
        cuda_options = (config['cmake_cuda_compiler'] or os.environ.get('CUDACXX', ''),
                        config['cmake_cuda_architectures'] or os.environ.get('CUDAARCHS', ''),
                        config['cuda_toolkit_root'])
        cuda_identity = hashlib.sha256(repr(cuda_options).encode()).hexdigest()[:12]
        tag += f'-cuda-{cuda_identity}'
    source = Path(config['cpp_base_path']).resolve()
    base = source / Path(config['build_dir'] or REPO_ROOT / 'build' / 'python') / tag
    stage = source / Path(config['install_dir'] or REPO_ROOT / 'build' / 'python-install') / tag
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
        'Python_EXECUTABLE': sys.executable,
        'TARGET_ARIS_PATH': config.get('aris_path', ''),
        'TARGET_HPP_FCL_PATH': dependency_prefix(config.get('fcl_path', '')),
        'TARGET_STDUUID_PATH': dependency_prefix(config.get('uuid_path', '')),
    }
    if config.get('toolchain_path'):
        defines['CMAKE_TOOLCHAIN_FILE'] = config['toolchain_path']
    defines['SIRE_ENABLE_CUDA'] = config['sire_enable_cuda']
    prefixes = [config['cmake_prefix_path']] if config['cmake_prefix_path'] else []
    if config['pybind11_dir']:
        defines['pybind11_DIR'] = config['pybind11_dir']
    else:
        # Prefer explicit prefixes and also search the isolated pip build environment.
        try:
            from pybind11 import get_cmake_dir
        except ImportError:
            pass
        else:
            prefixes.append(get_cmake_dir())
    if prefixes:
        defines['CMAKE_PREFIX_PATH'] = ';'.join(prefixes)
    if config['sire_enable_cuda']:
        for key in ('cmake_cuda_compiler', 'cmake_cuda_architectures', 'cuda_toolkit_root'):
            if config[key]:
                defines[_CMAKE_OPTIONS[key]] = config[key]
    return dict(profile=profile, mode=mode, source=source, base=base, build=base / mode,
                stage=stage, runtime=runtime, defines=defines)


def runtime_files(runtime):
    manifest = Path(runtime) / '.sire-files.json'
    return json.loads(manifest.read_text(encoding='utf-8')) if manifest.exists() else []


def sync_runtime(files, destination):
    """Sync only manifest-owned files; do not sweep an arbitrary DLL directory."""
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    manifest = destination / '.sire-files.json'
    previous = runtime_files(destination)
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


def verify_runtime(runtime, cuda_enabled=False):
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
assert hasattr(module, 'SireCudaBatchStepper') == (sys.argv[3] == 'True'), 'native extension has the wrong CUDA configuration'
print('[sire] verified:', sys.executable, path, 'Tracy:', module.tracyEnabled)
"""
    extension = next((runtime / ('sire' + suffix)
                      for suffix in importlib.machinery.EXTENSION_SUFFIXES
                      if (runtime / ('sire' + suffix)).is_file()), None)
    if extension is None:
        raise RuntimeError(f'No extension for {runtime_tag()} in {runtime}')
    subprocess.run([sys.executable, '-B', '-c', code, str(runtime), extension.name,
                    str(cuda_enabled)], check=True)


def build_native(profile=None, jobs=None, dry_run=False, *, activate=True):
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
    executable_dir = str(Path(sys.executable).parent)
    env['PATH'] = executable_dir + os.pathsep + env.get('PATH', os.environ.get('PATH', ''))
    if os.name == 'nt':
        env['USE_NINJA'] = '1'
        env['CMAKE_GENERATOR'] = 'Ninja'
    else:
        # Unix Makefiles are available with the system toolchain and match the
        # conventional single-config install layout used by Linux packages.
        # cmake_py treats a missing USE_NINJA as enabled, so disable it
        # explicitly instead of removing the variable.
        env['USE_NINJA'] = '0'
        env['CMAKE_GENERATOR'] = 'Unix Makefiles'
    env['CMAKE_BUILD_TYPE'] = plan['mode']
    for key in ('http_proxy', 'https_proxy'):
        if config.get('cmake_' + key):
            env[key.upper()] = config['cmake_' + key]
    cmake = CMake(str(plan['source']), str(plan['base']), str(plan['stage']), env=env)
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
    verify_runtime(expected, config['sire_enable_cuda'])
    if not activate:
        return files
    sync_runtime(files, plan['runtime'])
    verify_runtime(plan['runtime'], config['sire_enable_cuda'])
    return plan['runtime']
