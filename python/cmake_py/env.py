import os
import platform
import struct
from itertools import chain
from typing import Iterable, List, Dict

from .msvc import get_vc_env  # type: ignore[import]
from .cmake_utils import which

IS_WINDOWS = platform.system() == "Windows"
IS_DARWIN = platform.system() == "Darwin"
IS_LINUX = platform.system() == "Linux"
CMAKE_EXE_PATH = ""

# IS_CONDA = (
#     "conda" in sys.version
#     or "Continuum" in sys.version
#     or any(x.startswith("CONDA") for x in os.environ)
# )
# CONDA_DIR = os.path.join(os.path.dirname(sys.executable), "..")

IS_64BIT = struct.calcsize("P") == 8

BUILD_DIR = "build"
INSTALL_DIR = "install"

def check_env_flag(name: str, default: str = "", env: Dict[str, str] = os.environ) -> bool:
    return env.get(name, default).upper() in ["ON", "1", "YES", "TRUE", "Y"]


def check_negative_env_flag(name: str, default: str = "", env: Dict[str, str] = os.environ) -> bool:
    return env.get(name, default).upper() in ["OFF", "0", "NO", "FALSE", "N"]


def gather_paths(env_vars: Iterable[str], env: Dict[str, str] = os.environ) -> List[str]:
    return list(chain(*(env.get(v, "").split(os.pathsep) for v in env_vars)))


def lib_paths_from_base(base_path: str) -> List[str]:
    return [os.path.join(base_path, s) for s in ["lib/x64", "lib", "lib64"]]

# We promised that CXXFLAGS should also be affected by CFLAGS
if "CFLAGS" in os.environ and "CXXFLAGS" not in os.environ:
    os.environ["CXXFLAGS"] = os.environ["CFLAGS"]

def _overlay_windows_vcvars(env: Dict[str, str]) -> Dict[str, str]:
    vc_arch = "x64" if IS_64BIT else "x86"

    if platform.machine() == "ARM64":
        vc_arch = "x64_arm64"

        # First Win11 Windows on Arm build version that supports x64 emulation
        # is 10.0.22000.
        win11_1st_version = (10, 0, 22000)
        current_win_version = tuple(
            int(version_part) for version_part in platform.version().split(".")
        )
        if current_win_version < win11_1st_version:
            vc_arch = "x86_arm64"
            print(
                "Warning: 32-bit toolchain will be used, but 64-bit linker "
                "is recommended to avoid out-of-memory linker error!"
            )
            print(
                "Warning: Please consider upgrading to Win11, where x64 "
                "emulation is enabled!"
            )
    
    vc_env: Dict[str, str] = get_vc_env(vc_arch)
    # Keys in `_get_vc_env` are always lowercase.
    # We turn them into uppercase before overlaying vcvars
    # because OS environ keys are always uppercase on Windows.
    # https://stackoverflow.com/a/7797329
    vc_env = {k.upper(): v for k, v in vc_env.items()}
    for k, v in env.items():
        uk = k.upper()
        if uk not in vc_env:
            vc_env[uk] = v
    return vc_env
 
USE_NINJA = not check_negative_env_flag("USE_NINJA") and which("ninja") is not None
if "CMAKE_GENERATOR" in os.environ:
    USE_NINJA = os.environ["CMAKE_GENERATOR"].lower() == "ninja"

def _create_build_env() -> Dict[str, str]:
    # XXX - our cmake file sometimes looks at the system environment
    # and not cmake flags!
    # you should NEVER add something to this list. It is bad practice to
    # have cmake read the environment
    my_env = os.environ.copy()
    if IS_WINDOWS:
        # When using Ninja under Windows, the gcc toolchain will be chosen as
        # default. But it should be set to MSVC as the user's first choice.
        my_env = _overlay_windows_vcvars(my_env)
        my_env.setdefault("CC", "cl")
        my_env.setdefault("CXX", "cl")
        if USE_NINJA:
            my_env['use_ninja'] = "True"

    return my_env