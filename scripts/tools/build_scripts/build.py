import argparse
import sys
from os.path import abspath, dirname
import os
import pathlib

# By appending build_root to sys.path, this module can import other torch
# modules even when run as a standalone script. i.e., it's okay either you
# do `python build_libtorch.py` or `python -m tools.build_libtorch`.
build_root = dirname(dirname(abspath(__file__)))
sys.path.append(build_root)

# from build_scripts.cmake.cmake import CMake
# type: ignore[import]
from build_scripts.cmake import _create_build_env, which, build_project

if __name__ == "__main__":
    # Placeholder for future interface. For now just gives a nice -h.
    parser = argparse.ArgumentParser(description="Build Sire", allow_abbrev=True)
    parser.add_argument(
        '-b',
        "--base-path",
        type=pathlib.Path,
        required=True,
        help="Sire base path (CMakeList.txt folder)",
    )
    parser.add_argument(
        "--build-dir",
        type=pathlib.Path,
        help="Sire build directory",
    )
    parser.add_argument(
        '-i',
        "--install-dir",
        type=pathlib.Path,
        help="Sire isntall directory",
    )
    parser.add_argument(
        '-a',
        "--aris-path",
        type=pathlib.Path,
        required=True,
        help="Aris install path",
    )
    parser.add_argument(
        '-f',
        "--fcl-path",
        type=pathlib.Path,
        required=True,
        help="hpp-fcl install path",
    )
    parser.add_argument(
        '-u',
        "--uuid-path",
        type=pathlib.Path,
        required=True,
        help="stduuid install path",
    )
    parser.add_argument(
        '-t',
        "--toolchain-path",
        type=pathlib.Path,
        help="toolchain setting path (.cmake file)",
    )
    parser.add_argument(
        "--http-proxy",
        default="",
        type=str,
        help="http proxy to env",
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
    parser.add_argument("--build-python", action="store_true", help="build python bindings")
    parser.add_argument(
        "--python-path",
        type=pathlib.Path,
        help="python.dev install path",
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--release', action='store_true')
    group.add_argument('--debug', action='store_true')
    group.add_argument('--build-all', action='store_true')
    options = parser.parse_args()
    
    my_env = _create_build_env()

    USE_NINJA = which("ninja", env=my_env) is not None
    if "CMAKE_GENERATOR" in my_env:
        USE_NINJA = my_env["CMAKE_GENERATOR"].lower() == "ninja"
    if options.http_proxy != "":
        my_env["HTTP_PROXY"] = options.http_proxy
        my_env["HTTPS_PROXY"] = options.http_proxy
    if USE_NINJA:
        my_env["CMAKE_GENERATOR"] = "ninja"

    if options.rm_cache_reconfig:
        options.rerun_config = True
        options.rm_cache = True

    # print(options, str(options.install_dir))
    if options.build_all:
        options.debug = True
        options.release = True

    cmake_args = {
        "BUILD_DEMO": True if options.build_demo else False,
        "BUILD_TEST": True if options.build_test else False,
        "BUILD_PYTHON": True if options.build_python else False,
    }
    if options.toolchain_path is not None:
        cmake_args["CMAKE_TOOLCHAIN_FILE"] = str(options.toolchain_path)

    for build_type in ["Debug", "Release"]:
        not_debug, not_release = False, False
        if not (build_type == "Debug" and options.debug):
            not_debug = True
        if not (build_type == "Release" and options.release):
            not_release = True
        if not_debug and not_release:
            continue

        cmake_args.update({
            "TARGET_ARIS_PATH": str(options.aris_path), 
            "TARGET_HPP_FCL_PATH": str(options.fcl_path / build_type), 
            "TARGET_STDUUID_PATH": str(options.uuid_path / build_type),
            # "TARGET_PYTHON_PATH": str(options.python_path),
            "CMAKE_BUILD_TYPE": build_type,
            })
        my_env["CMAKE_BUILD_TYPE"] = build_type
        build_project(
            project_path=str(options.base_path),
            build_dir=None if options.build_dir is None else str(options.build_dir),
            version=None,
            # cmake_python_library=None,
            # build_python=False,
            rerun_config=options.rerun_config,
            rm_cache=options.rm_cache,
            cmake_only=options.cmake_only,
            build_only=False,
            install_dir=None if options.install_dir is None else str(options.install_dir),
            env=my_env,
            **cmake_args
        )
        

    # if options.debug:
    #     os.environ['CMAKE_BUILD_TYPE'] = 'Debug'
    #     build_project(
    #         project_path=str(options.base_path),
    #         build_dir=None if options.build_dir is None else str(options.build_dir),
    #         install_dir=None if options.install_dir is None else str(options.install_dir),
    #         version=None,
    #         # cmake_python_library=None,
    #         # build_python=False,
    #         rerun_config=options.rerun_config,
    #         rm_cache=options.rm_cache,
    #         cmake_only=options.cmake_only,
    #         cmake_toolchain_file=None if options.toolchain_path is None else str(options.toolchain_path),
    #         env=my_env,
    #         TARGET_ARIS_PATH=str(options.aris_path),
    #         TARGET_HPP_FCL_PATH=str(options.fcl_path),
    #         CMAKE_BUILD_TYPE='Debug'
    #     )
    # if options.release:
    #     os.environ['CMAKE_BUILD_TYPE'] = 'Release'
    #     build_project(
    #         project_path=str(options.base_path),
    #         build_dir=None if options.build_dir is None else str(options.build_dir),
    #         install_dir=None if options.install_dir is None else str(options.install_dir),
    #         version=None,
    #         # cmake_python_library=None,
    #         # build_python=False,
    #         rerun_config=True if options.debug else options.rerun_config,
    #         rm_cache=True if options.debug else options.rm_cache,
    #         cmake_only=options.cmake_only,
    #         cmake_toolchain_file=None if options.toolchain_path is None else str(options.toolchain_path),
    #         env=my_env,
    #         TARGET_ARIS_PATH=str(options.aris_path),
    #         TARGET_HPP_FCL_PATH=str(options.fcl_path),
    #         CMAKE_BUILD_TYPE='Release'
    #     )
