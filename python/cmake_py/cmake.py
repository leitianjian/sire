"Manages CMake."


import multiprocessing
import os
import platform
import sys
import shutil
import hashlib
from packaging.version import parse as LooseVersion
from subprocess import CalledProcessError, check_call, check_output
from typing import Any, cast, Dict, List, Optional

from .cmake_utils import which, CMakeValue, get_cmake_cache_variables_from_file
from .env import BUILD_DIR, INSTALL_DIR, check_negative_env_flag, check_env_flag, IS_64BIT, IS_DARWIN, IS_WINDOWS, CMAKE_EXE_PATH
from pathlib import PurePath
# from .numpy_ import NUMPY_INCLUDE_DIR, USE_NUMPY

def _mkdir_p(d: str) -> None:
    try:
        os.makedirs(d, exist_ok=True)
    except OSError as e:
        raise RuntimeError(
            f"Failed to create folder {os.path.abspath(d)}: {e.strerror}"
        ) from e

class CMake:
    "Manages cmake."

    def __init__(self, base_dir: str, build_dir: Optional[str], install_dir: Optional[str], env: os._Environ[str] = os.environ, splitReleaseDebug: bool = False) -> None:
        self._cmake_command = CMake._get_cmake_command(env=env)
        self.build_type = self.BuildType(env.get("CMAKE_BUILD_TYPE", "Release"))
        self.base_dir = base_dir
        if not (os.path.isdir(base_dir) and os.path.isfile(os.path.join(base_dir, 'CMakeLists.txt'))):
            raise RuntimeError("no CMakeLists.txt found in path base_dir")
        if build_dir is None:
            build_dir = BUILD_DIR
        if install_dir is None:
            install_dir = INSTALL_DIR

        if PurePath(build_dir).is_absolute():
            self.build_dir = build_dir
        else:
            self.build_dir = os.path.join(self.base_dir, build_dir)

        if PurePath(install_dir).is_absolute():
            self.install_dir = install_dir
        else:
            self.install_dir = os.path.join(self.base_dir, install_dir)
        self.build_dir = os.path.join(self.build_dir, self.build_type.build_type_string)
        if splitReleaseDebug:
            self.install_dir = os.path.join(self.install_dir, self.build_type.build_type_string)
        self.config_args = []
        self.build_args = []

    @property
    def _cmake_cache_file(self) -> str:
        r"""Returns the path to CMakeCache.txt.

        Returns:
          string: The path to CMakeCache.txt.
        """
        return os.path.join(self.build_dir, "CMakeCache.txt")
    
    @property
    def _cmake_cache_dir(self) -> str:
        r"""Returns the path to CMakeFiles directory

        Returns:
          string: The path to CMakeFiles.
        """
        return os.path.join(self.build_dir, "CMakeFiles");

    @property
    def _cmake_py_signature_file(self) -> str:
        return os.path.join(self.build_dir, ".cmake_py_config_signature")

    def _expected_config_signature(self, my_env: Dict[str, str]) -> str:
        """Return a stable signature of inputs that should trigger reconfigure.

        We use this to decide whether it is safe to skip running CMake
        configuration. The previous logic only checked build type / generator
        artifacts and could miss changed -D options (e.g. SIRE_ENABLE_TRACY).
        """

        signature_parts: List[str] = []
        signature_parts.append(f"base_dir={self.base_dir}")
        signature_parts.append(f"build_dir={self.build_dir}")
        signature_parts.append(f"install_dir={self.install_dir}")
        signature_parts.append(f"build_type={self.build_type.build_type_string}")

        use_ninja = not check_negative_env_flag("USE_NINJA", env=my_env)
        signature_parts.append(f"use_ninja={int(use_ninja)}")
        if use_ninja:
            signature_parts.append("generator=Ninja")
        elif IS_WINDOWS:
            generator = os.getenv("CMAKE_GENERATOR", "Visual Studio 16 2019")
            signature_parts.append(f"generator={generator}")
            signature_parts.append(f"arch={'ARM64' if platform.machine() == 'ARM64' else 'x64' if IS_64BIT else 'x86'}")
            toolset_version = os.getenv("CMAKE_GENERATOR_TOOLSET_VERSION")
            if toolset_version is not None:
                signature_parts.append(f"toolset_version={toolset_version}")
            curr_toolset = os.getenv("VCToolsVersion")
            if curr_toolset is not None:
                signature_parts.append(f"vctools_version={curr_toolset}")

        # Capture the -D defines already collected via `defines()`.
        # Sort to avoid order-based churn.
        # NOTE: `configure()` may append some auto-derived -D options (e.g.
        # CMAKE_INSTALL_PREFIX) before running CMake. We intentionally filter
        # those out here and instead add their expected values deterministically
        # below, so the signature remains stable across runs.
        filtered_define_args: List[str] = []
        for arg in self.config_args:
            if not arg.startswith("-D"):
                continue
            # Parse "-DKEY=VALUE".
            key = arg[2:].split("=", 1)[0]
            if key in {"CMAKE_INSTALL_PREFIX", "CMAKE_C_COMPILER", "CMAKE_CXX_COMPILER"}:
                continue
            filtered_define_args.append(arg)
        for arg in sorted(filtered_define_args):
            signature_parts.append(arg)

        # Auto-derived options.
        signature_parts.append(f"-DCMAKE_INSTALL_PREFIX={self.install_dir}")

        expected_wrapper = "/usr/local/opt/ccache/libexec"
        if IS_DARWIN and os.path.exists(expected_wrapper):
            if "CC" not in os.environ:
                signature_parts.append(f"-DCMAKE_C_COMPILER={expected_wrapper}/gcc")
            if "CXX" not in os.environ:
                signature_parts.append(f"-DCMAKE_CXX_COMPILER={expected_wrapper}/g++")

        signature_blob = "\n".join(signature_parts).encode("utf-8")
        return hashlib.sha256(signature_blob).hexdigest()

    @staticmethod
    def _get_cmake_command(env: os._Environ[str] = os.environ) -> str:
        "Returns cmake command."

        cmake_command = which("cmake", env=env)
        if IS_WINDOWS:
            return cmake_command
        cmake3_version = CMake._get_version(which("cmake3", env=env))
        cmake_version = CMake._get_version(which("cmake", env=env))

        _cmake_min_version = LooseVersion("3.18.0")
        if all(
            ver is None or ver < _cmake_min_version
            for ver in [cmake_version, cmake3_version]
        ):
            raise RuntimeError("no cmake or cmake3 with version >= 3.18.0 found")
        if cmake3_version is None:
            cmake_command = which("cmake", env=env)
        elif cmake_version is None:
            cmake_command = which("cmake3", env=env)
        else:
            if cmake3_version >= cmake_version:
                cmake_command = which("cmake3", env=env)
            else:
                cmake_command = which("cmake", env=env)
        return cmake_command

    @staticmethod
    def _get_version(cmd: Optional[str]) -> Any:
        "Returns cmake version."

        if cmd is None:
            return None
        for line in check_output([cmd, "--version"]).decode("utf-8").split("\n"):
            if "version" in line:
                return LooseVersion(line.strip().split(" ")[2])
        raise RuntimeError("no version found")

    def run(self, args: List[str], env: Dict[str, str]) -> None:
        "Executes cmake with arguments and an environment."

        command = [self._cmake_command] + args
        print(" ".join(command))
        try:
            check_call(command, cwd=self.build_dir, env=env)
        except (CalledProcessError, KeyboardInterrupt) as e:
            # This error indicates that there was a problem with cmake, the
            # Python backtrace adds no signal here so skip over it by catching
            # the error and exiting manually
            sys.exit(1)

    def defines(self, **kwargs: CMakeValue) -> None:
        "Adds definitions to a cmake argument list."
        for key, value in sorted(kwargs.items()):
            if value is not None:
                self.config_args.append(f"-D{key}={value}")

    def get_cmake_cache_variables(self) -> Dict[str, CMakeValue]:
        r"""Gets values in CMakeCache.txt into a dictionary.
        Returns:
          dict: A ``dict`` containing the value of cached CMake variables.
        """
        with open(self._cmake_cache_file) as f:
            return get_cmake_cache_variables_from_file(f)

    def configure(
        self,
        # build_test: bool,
        my_env: Dict[str, str],
        reconfig: bool,
        rm_cache: bool,
    ) -> None:
        "Runs cmake to generate native build files."

        if reconfig and os.path.isfile(self._cmake_cache_file):
            os.remove(self._cmake_cache_file)
        
        if rm_cache and os.path.isdir(self._cmake_cache_dir):
            shutil.rmtree(self._cmake_cache_dir)

        ninja_build_file = os.path.join(self.build_dir, "build.ninja")
        if os.path.exists(self._cmake_cache_file) and not (
            not check_negative_env_flag("USE_NINJA", env=my_env) and not os.path.exists(ninja_build_file)) and self.build_type.build_type_string == self.BuildType(cmake_cache_path=self.build_dir).build_type_string:
            expected_sig = self._expected_config_signature(my_env)
            try:
                with open(self._cmake_py_signature_file, "r", encoding="utf-8") as f:
                    cached_sig = f.read().strip()
            except FileNotFoundError:
                cached_sig = ""

            # Everything's in place AND inputs are unchanged: skip configure.
            if cached_sig == expected_sig:
                return

        if not check_negative_env_flag("USE_NINJA", env=my_env):
            # Avoid conflicts in '-G' and the `CMAKE_GENERATOR`
            os.environ["CMAKE_GENERATOR"] = "Ninja"
            self.config_args.append("-GNinja")
        elif IS_WINDOWS:
            generator = os.getenv("CMAKE_GENERATOR", "Visual Studio 16 2019")
            supported = ["Visual Studio 16 2019", "Visual Studio 17 2022"]
            if generator not in supported:
                print("Unsupported `CMAKE_GENERATOR`: " + generator)
                print("Please set it to one of the following values: ")
                print("\n".join(supported))
                sys.exit(1)
            self.config_args.append("-G" + generator)
            toolset_dict = {}
            toolset_version = os.getenv("CMAKE_GENERATOR_TOOLSET_VERSION")
            if toolset_version is not None:
                toolset_dict["version"] = toolset_version
                curr_toolset = os.getenv("VCToolsVersion")
                if curr_toolset is None:
                    print(
                        "When you specify `CMAKE_GENERATOR_TOOLSET_VERSION`, you must also "
                        "activate the vs environment of this version. Please read the notes "
                        "in the build steps carefully."
                    )
                    sys.exit(1)
            if IS_64BIT:
                if platform.machine() == "ARM64":
                    self.config_args.append("-A ARM64")
                else:
                    self.config_args.append("-Ax64")
                    toolset_dict["host"] = "x64"
            if toolset_dict:
                toolset_expr = ",".join([f"{k}={v}" for k, v in toolset_dict.items()])
                self.config_args.append("-T" + toolset_expr)

        _mkdir_p(self.install_dir)
        _mkdir_p(self.build_dir)

        # Options starting with CMAKE_
        cmake__options = {
            "CMAKE_INSTALL_PREFIX": self.install_dir,
        }

        self.defines(
            **cmake__options,
        )

        expected_wrapper = "/usr/local/opt/ccache/libexec"
        if IS_DARWIN and os.path.exists(expected_wrapper):
            if "CMAKE_C_COMPILER" not in self.config_args and "CC" not in os.environ:
                self.defines(CMAKE_C_COMPILER=f"{expected_wrapper}/gcc")
            if "CMAKE_CXX_COMPILER" not in self.config_args and "CXX" not in os.environ:
                self.defines(CMAKE_CXX_COMPILER=f"{expected_wrapper}/g++")

        for env_var_name in my_env:
            if env_var_name.startswith("gh"):
                # github env vars use utf-8, on windows, non-ascii code may
                # cause problem, so encode first
                try:
                    my_env[env_var_name] = str(my_env[env_var_name].encode("utf-8"))
                except UnicodeDecodeError as e:
                    shex = ":".join(f"{ord(c):02x}" for c in my_env[env_var_name])
                    print(
                        f"Invalid ENV[{env_var_name}] = {shex}",
                        file=sys.stderr,
                    )
                    print(e, file=sys.stderr)
        # According to the CMake manual, we should pass the arguments first,
        # and put the directory as the last element. Otherwise, these flags
        # may not be passed correctly.
        # Reference:
        # 1. https://cmake.org/cmake/help/latest/manual/cmake.1.html#synopsis
        # 2. https://stackoverflow.com/a/27169347
        self.config_args.append(self.base_dir)
        self.run(args=self.config_args, env=my_env)

        # Persist the signature so subsequent builds can detect changed -D args.
        try:
            with open(self._cmake_py_signature_file, "w", encoding="utf-8") as f:
                f.write(self._expected_config_signature(my_env))
        except OSError:
            # Best-effort: inability to write this file should not break builds.
            pass
        # hotpatch environment variable 'CMAKE_BUILD_TYPE'. 'CMAKE_BUILD_TYPE' always prevails over DEBUG or REL_WITH_DEB_INFO.
        if "CMAKE_BUILD_TYPE" not in os.environ:
            if check_env_flag("DEBUG"):
                os.environ["CMAKE_BUILD_TYPE"] = "Debug"
            elif check_env_flag("REL_WITH_DEB_INFO"):
                os.environ["CMAKE_BUILD_TYPE"] = "RelWithDebInfo"
            else:
                os.environ["CMAKE_BUILD_TYPE"] = "Release"
        self.build_type = self.BuildType(cmake_cache_path=self.build_dir)
    
    class BuildType:
        """Checks build type. The build type will be given in :attr:`cmake_build_type_env`. If :attr:`cmake_build_type_env`
        is ``None``, then the build type will be inferred from ``CMakeCache.txt``. If ``CMakeCache.txt`` does not exist,
        os.environ['CMAKE_BUILD_TYPE'] will be used.

        Args:
          cmake_build_type_env (str): The value of os.environ['CMAKE_BUILD_TYPE']. If None, the actual build type will be
            inferred.

        """

        def __init__(self, cmake_build_type_env: Optional[str] = None, cmake_cache_path: Optional[str] = None) -> None:
            if cmake_build_type_env is not None:
                self.build_type_string = cmake_build_type_env
                return

            if (cmake_cache_path is not None) and os.path.isdir(cmake_cache_path):
                cmake_cache_txt = os.path.join(cmake_cache_path, "CMakeCache.txt")
                if os.path.isfile(cmake_cache_txt):
                    # Found CMakeCache.txt. Use the build type specified in it.
                    from .cmake_utils import get_cmake_cache_variables_from_file

                    with open(cmake_cache_txt) as f:
                        cmake_cache_vars = get_cmake_cache_variables_from_file(f)
                    # Normally it is anti-pattern to determine build type from CMAKE_BUILD_TYPE because it is not used for
                    # multi-configuration build tools, such as Visual Studio and XCode. But since we always communicate with
                    # CMake using CMAKE_BUILD_TYPE from our Python scripts, this is OK here.
                    self.build_type_string = cast(str, cmake_cache_vars["CMAKE_BUILD_TYPE"])
                else:
                    self.build_type_string = os.environ.get("CMAKE_BUILD_TYPE", "Release")
            else:
                self.build_type_string = os.environ.get("CMAKE_BUILD_TYPE", "Release")

        def is_debug(self) -> bool:
            "Checks Debug build."
            return self.build_type_string == "Debug"

        def is_rel_with_deb_info(self) -> bool:
            "Checks RelWithDebInfo build."
            return self.build_type_string == "RelWithDebInfo"

        def is_release(self) -> bool:
            "Checks Release build."
            return self.build_type_string == "Release"

    def build(self, my_env: Dict[str, str]) -> None:
        "Runs cmake to build binaries."
        build_args = [
            "--build",
            self.build_dir,
            "--config",
            self.build_type.build_type_string,
        ]

        # Determine the parallelism according to the following
        # priorities:
        # 1) MAX_JOBS environment variable
        # 2) If using the Ninja build system, delegate decision to it.
        # 3) Otherwise, fall back to the number of processors.

        # Allow the user to set parallelism explicitly. If unset,
        # we'll try to figure it out.
        max_jobs = os.getenv("MAX_JOBS")

        if max_jobs is not None or check_negative_env_flag("USE_NINJA", env=my_env):
            # Ninja is capable of figuring out the parallelism on its
            # own: only specify it explicitly if we are not using
            # Ninja.

            # This lists the number of processors available on the
            # machine. This may be an overestimate of the usable
            # processors if CPU scheduling affinity limits it
            # further. In the future, we should check for that with
            # os.sched_getaffinity(0) on platforms that support it.
            max_jobs = max_jobs or str(multiprocessing.cpu_count())

            # This ``if-else'' clause would be unnecessary when cmake
            # 3.12 becomes minimum, which provides a '-j' option:
            # build_args += ['-j', max_jobs] would be sufficient by
            # then. Until then, we use "--" to pass parameters to the
            # underlying build system.
            build_args += ["--"]
            if IS_WINDOWS and check_negative_env_flag("USE_NINJA", env=my_env):
                # We are likely using msbuild here
                build_args += [f"/p:CL_MPCount={max_jobs}"]
            else:
                build_args += ["-j", max_jobs]
        self.run(args=build_args, env=my_env)
    
    def install(self, my_env: Dict[str, str]) -> None:
        "Run cmake to install target"
        install_args = [
            "--install",
            self.build_dir,
            "--prefix",
            self.install_dir,
        ]
        self.run(args=install_args, env=my_env)


def build_project(
    project_path: str,
    build_dir: Optional[str],
    version: Optional[str],
    rerun_config: bool,
    rm_cache: bool,
    cmake_only: bool = False,
    build_only: bool = True,
    install_dir: Optional[str] = None,
    env: os._Environ[str] = os.environ,
    **kwargs: CMakeValue,
):
    splitReleaseDebug = kwargs.get("splitReleaseDebug", False)
    cmake = CMake(base_dir=project_path, build_dir=build_dir, install_dir=install_dir, env=env, splitReleaseDebug=splitReleaseDebug)
    cmake.defines(
            # PYTHON_EXECUTABLE=sys.executable,
            Python_EXECUTABLE=sys.executable,
            PYTHON_EXECUTABLE=sys.executable,
            #   SIRE_BUILD_VERSION=version,
            #   BUILD_DEMO=check_env_flag("BUILD_DEMO", env=env),
            #   BUILD_TEST=check_env_flag("BUILD_TEST", env=env),
            #   CMAKE_TOOLCHAIN_FILE=cmake_toolchain_file,
              **kwargs,
              )
    cmake.configure(
        env, rerun_config, rm_cache
    )
    if cmake_only:
        return cmake.install_dir
    cmake.build(env)

    if build_only:
        return cmake.install_dir
    cmake.install(env)
    return cmake.install_dir
