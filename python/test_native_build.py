"""Build-tool regression tests. No CMake, compiler, pip install or Rust build."""
from pathlib import Path
import runpy
import sys
import sysconfig
import tempfile
import unittest
from unittest.mock import patch

from setuptools import Distribution
import native_build as build


class NativeBuildTest(unittest.TestCase):
    def test_rejects_foreign_interpreter(self):
        build.require_project_python()
        with patch.object(sys, 'prefix', str(build.REPO_ROOT / 'not-the-project-venv')):
            with self.assertRaisesRegex(RuntimeError, 'Use the project venv'):
                build.require_project_python()

    def test_environment_boolean_and_profile_isolation(self):
        with patch.dict('os.environ', {'BUILD_SIRE_ENABLE_TRACY': 'false',
                                      'BUILD_BUILD_DEMO': '0'}):
            config = build.load_config()
        self.assertFalse(config['sire_enable_tracy'])
        self.assertFalse(config['build_demo'])
        release = build.build_plan(config, 'release')
        profile = build.build_plan(config, 'profile')
        debug = build.build_plan(config, 'debug')
        self.assertEqual(len({release['build'], profile['build'], debug['build']}), 3)
        self.assertEqual(release['defines']['PYTHON_EXECUTABLE'], sys.executable)
        self.assertFalse(release['defines']['SIRE_ENABLE_TRACY'])
        self.assertFalse(release['defines']['SIRE_CLARABEL_FORCE_SOURCE_BUILD'])
        self.assertTrue(profile['defines']['SIRE_ENABLE_TRACY'])
        with patch.dict('os.environ', {'BUILD_SIRE_ENABLE_TRACY': 'nonsense'}):
            with self.assertRaises(ValueError):
                build.load_config()

    def test_dry_run_never_executes_tools(self):
        with patch.object(build.subprocess, 'run', side_effect=AssertionError('ran a tool')):
            with patch.object(build, 'sync_runtime', side_effect=AssertionError('copied artifacts')):
                build.build_native('release', jobs=2, dry_run=True)

    def test_sync_skips_unchanged_and_only_removes_owned_files(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source, destination = root / 'source', root / 'runtime'
            source.mkdir()
            (source / 'a.dll').write_bytes(b'one')
            (source / 'b.dll').write_bytes(b'two')
            build.sync_runtime(list(source.iterdir()), destination)
            before = (destination / 'a.dll').stat().st_mtime_ns
            (destination / 'user.txt').write_text('keep')
            build.sync_runtime([source / 'a.dll'], destination)
            self.assertEqual((destination / 'a.dll').stat().st_mtime_ns, before)
            self.assertFalse((destination / 'b.dll').exists())
            self.assertTrue((destination / 'user.txt').exists())
            (source / 'a.dll').write_bytes(b'changed')
            build.sync_runtime([source / 'a.dll'], destination)
            self.assertEqual((destination / 'a.dll').read_bytes(), b'changed')

    def test_setuptools_declares_extension_and_copies_wheel_dependencies(self):
        with patch('setuptools.setup') as setup:
            namespace = runpy.run_path(str(build.PYTHON_ROOT / 'setup.py'))
        options = setup.call_args.kwargs
        distribution = Distribution(options)
        self.assertTrue(distribution.has_ext_modules())
        command_type = namespace['CMakeBuildExt']
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            runtime = root / 'runtime'
            runtime.mkdir()
            extension = runtime / ('sire' + sysconfig.get_config_var('EXT_SUFFIX'))
            extension.write_bytes(b'fake extension for packaging test')
            (runtime / 'dependency.dll').write_bytes(b'fake dependency')
            command = command_type(distribution)
            command.ensure_finalized()
            command.build_lib = str(root / 'wheel')
            with patch.dict(command_type.run.__globals__, {'build_native': lambda: runtime}):
                command.run()
            outputs = [Path(name) for name in command.get_outputs()]
            self.assertEqual({p.name for p in outputs}, {extension.name, 'dependency.dll'})
            self.assertTrue(all(p.is_file() for p in outputs))
            self.assertTrue(all(p.parent == root / 'wheel' / 'sire' / 'native' for p in outputs))
            command.editable_mode = True
            with patch.dict(command_type.run.__globals__, {'build_native': lambda: runtime}):
                command.run()
            self.assertTrue(all(Path(p).parent == runtime for p in command.get_outputs()))


if __name__ == '__main__':
    unittest.main(verbosity=2)
