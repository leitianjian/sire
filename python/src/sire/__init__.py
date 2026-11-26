"""Python interface to the Sire simulator."""

from importlib import import_module as _import_module
from importlib.metadata import PackageNotFoundError as _PackageNotFoundError
from importlib.metadata import version as _package_version
import sys as _sys

try:
    from .native import sire
except ImportError as error:
    raise ImportError(
        f"Sire native extension could not be loaded by {_sys.executable}. "
        "Install Sire for this Python interpreter. See python/BUILDING.md. "
        f"Original error: {error}"
    ) from error

try:
    __version__ = _package_version("sire")
except _PackageNotFoundError:
    __version__ = "0.1.0"

_native_exports = [name for name in dir(sire) if not name.startswith("_")]
globals().update({name: getattr(sire, name) for name in _native_exports})

# Recording playback historically exported these functions at package level.
# Load its dependencies only when callers use playback.
_playback_exports = (
    "animateRobotByRecords", "animateRobotByRecordsRealtime", "binarySearch",
    "buildHFieldMeshFull", "buildHFieldMeshHeights", "buildHFieldMeshPNG",
    "create_scale_matrix", "draw_arrow_realtime", "pe3132tfmatrix", "pq2tfmatrix",
    "robotInit", "rotation_matrix_from_vectors", "s_pe_dot_pe", "setRobotPq",
    "setRobotPqRealtime", "visualize",
)
__all__ = _native_exports + list(_playback_exports)


def __getattr__(name):
    if name in _playback_exports:
        visualize = _import_module(".py_module.visualize", __name__)
        value = visualize if name == "visualize" else getattr(visualize, name)
        globals()[name] = value
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return sorted(set(globals()).union(_playback_exports))
