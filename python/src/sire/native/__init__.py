"""Select editable native artifacts for the running Python interpreter."""
import hashlib
import os
from pathlib import Path
import sys
import sysconfig

_root = Path(__file__).resolve().parent
# Keep this selection in sync with native_build.py.
_parts = (os.path.normcase(str(Path(sys.prefix).resolve())),
          os.path.normcase(str(Path(sys.executable).resolve())),
          sysconfig.get_config_var('EXT_SUFFIX'))
_identity = hashlib.sha256(repr(_parts).encode()).hexdigest()[:16]
_runtime_tag = f'{sys.implementation.cache_tag}-{sysconfig.get_platform()}-{_identity}'
_runtime = _root / '_runtime' / _runtime_tag
_editable_source = (_root.parents[2] / 'pyproject.toml').is_file()
if _runtime.is_dir() or _editable_source:
    # Do not silently fall through to a stale, untagged sire.pyd in _root.
    __path__ = [str(_runtime)]
    _dll_root = _runtime
else:
    # Standard wheels place their ABI-tagged extension alongside this file.
    _dll_root = _root
_dll_directory = os.add_dll_directory(str(_dll_root)) if os.name == 'nt' and _dll_root.is_dir() else None


def __getattr__(name):
    if name == 'sire' and _editable_source and not _runtime.is_dir():
        raise ImportError(
            f'No native runtime for {_runtime_tag}. From the repository root run '
            f'`{sys.executable} python/build_native.py` with your native dependency prefixes.'
        )
    raise AttributeError(name)
