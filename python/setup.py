"""Setuptools integration; native compilation is shared with build_native.py."""
from pathlib import Path
import sys

from setuptools import Extension, find_packages, setup
from setuptools.command.build_ext import build_ext

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from native_build import build_native, runtime_files, sync_runtime


class CMakeBuildExt(build_ext):
    def run(self):
        if self.inplace or getattr(self, 'editable_mode', False):
            runtime = build_native()
            files = [runtime / name for name in runtime_files(runtime)]
        else:
            destination = Path(self.get_ext_fullpath('sire.native.sire')).parent
            files = sync_runtime(build_native(activate=False), destination)
        self._native_outputs = [str(path) for path in files]

    def get_outputs(self):
        return getattr(self, '_native_outputs', [])

    def get_output_mapping(self):
        return {}


setup(
    package_dir={'': 'src'},
    packages=find_packages(where='src'),
    # Declare a real extension for ABI/platform wheel tags and one build_ext run.
    ext_modules=[Extension('sire.native.sire', sources=[])],
    include_package_data=False,
    zip_safe=False,
    cmdclass={'build_ext': CMakeBuildExt},
)
