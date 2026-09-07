"""Select editable native artifacts for the running Python ABI."""
import os
from pathlib import Path
import sys
import sysconfig

_root = Path(__file__).resolve().parent
_runtime = _root / '_runtime' / f'{sys.implementation.cache_tag}-{sysconfig.get_platform()}'
_editable_source = (_root.parents[2] / 'pyproject.toml').is_file()
if _runtime.is_dir() or _editable_source:
    # Do not silently fall through to a stale, untagged sire.pyd in _root.
    __path__ = [str(_runtime)]
    _dll_root = _runtime
else:
    # Standard wheels place their ABI-tagged extension alongside this file.
    _dll_root = _root
_dll_directory = os.add_dll_directory(str(_dll_root)) if os.name == 'nt' else None
