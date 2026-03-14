# hpp-fcl github link https://github.com/humanoid-path-planner/hpp-fcl
import argparse
from typing import Optional
import sys
from os.path import abspath, dirname
import pathlib

build_root = dirname(dirname(abspath(__file__)))
sys.path.append(build_root)
from build_scripts.cmake.cmake import CMakeValue
from build_scripts.cmake import _create_build_env, which, build_project

def __create_clarabel_env():
    clarabel_env = _create_build_env()
    
    USE_NINJA = which("ninja", env=clarabel_env) is not None
    if "CMAKE_GENERATOR" in clarabel_env:
        USE_NINJA = clarabel_env["CMAKE_GENERATOR"].lower() == "ninja"

    if USE_NINJA:
        clarabel_env["CMAKE_GENERATOR"] = "ninja"

    return clarabel_env

def build_clarabel(
    clarabel_path: str,
    build_dir: Optional[str],
    version: Optional[str],
    # cmake_python_library: Optional[str],
    # build_python: bool,
    rerun_config: bool,
    rm_cache: bool,
    cmake_only: bool = False,
    build_only: bool = False,
    install_dir: Optional[str] = None,
    build_test: bool = False,
    build_release: bool = False,
    build_debug: bool = False,
    **kwargs: CMakeValue,
) -> None:
    clarabel_env = __create_clarabel_env()
    cmake_args = { 
        "BUILD_PYTHON_INTERFACE": False, 
        "BUILD_TESTING": build_test 
    }

    for build_type in ["Debug", "Release"]:
        not_debug, not_release = False, False
        if not (build_type == "Debug" and build_debug):
            not_debug = True
        if not (build_type == "Release" and build_release):
            not_release = True
        if not_debug and not_release:
            continue
        clarabel_env["CMAKE_BUILD_TYPE"] = build_type
        cmake_args["CMAKE_BUILD_TYPE"] = build_type
        build_project(
            project_path=str(clarabel_path),
            build_dir=build_dir,
            version=version,
            rerun_config=rerun_config,
            rm_cache=rm_cache,
            cmake_only=cmake_only,
            build_only=build_only,
            install_dir=install_dir,
            env=clarabel_env,
            **cmake_args,
            **kwargs
        )

if __name__ == "__main__":
    # Placeholder for future interface. For now just gives a nice -h.
    parser = argparse.ArgumentParser(description="Build clarabel", allow_abbrev=True)
    parser.add_argument(
        '-b',
        "--base-path",
        type=pathlib.Path,
        required=True,
        help="clarabel base path (CMakeList.txt folder)",
    )
    parser.add_argument(
        "--build-dir",
        type=pathlib.Path,
        help="clarabel build directory",
    )
    parser.add_argument(
        '-t',
        "--toolchain-path",
        type=pathlib.Path,
        help="toolchain setting path (.cmake file)",
    )
    parser.add_argument(
        '-i',
        "--install-dir",
        type=pathlib.Path,
        help="clarabel install directory",
    )
    parser.add_argument('-r', "--rm-cache-reconfig", action="store_true", help="delete cache and reconfigure")
    parser.add_argument('-rec', "--rerun-config", action="store_true", help="reconfigure")
    parser.add_argument('-rmc', "--rm-cache", action="store_true", help="remove cmake cache")
    parser.add_argument('-c',
        "--cmake-only",
        action="store_true",
        help="Stop once cmake terminates. Leave users a chance to adjust build options",
    )
    parser.add_argument("--build-demo", action="store_true", help="build demonstration examples")
    parser.add_argument("--build-test", action="store_true", help="build test cases")
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--release', action='store_true')
    group.add_argument('--debug', action='store_true')
    group.add_argument('--build-all', action='store_true')
    options = parser.parse_args()

    if options.rm_cache_reconfig:
        options.rerun_config = True
        options.rm_cache = True

    # print(options, str(options.install_dir))
    if options.build_all:
        options.debug = True
        options.release = True
    cmake_args = {}
    if options.toolchain_path is not None:
        cmake_args["CMAKE_TOOLCHAIN_FILE"] = str(options.toolchain_path)
    cmake_args["splitReleaseDebug"] = True
    build_clarabel(
        clarabel_path=str(options.base_path),
        build_dir=None if options.build_dir is None else str(options.build_dir),
        version=None,
        # cmake_python_library=None,
        # build_python=False,
        rerun_config=options.rerun_config,
        rm_cache=options.rm_cache,
        cmake_only=options.cmake_only,
        build_only=False,
        install_dir=None if options.install_dir is None else str(options.install_dir),
        build_test=options.build_test,
        build_demo=options.build_demo,
        build_release=options.release,
        build_debug=options.debug,
        **cmake_args
    )
