# aris github link https://github.com/py0330/aris
import argparse
from typing import Optional
import sys
from os.path import abspath, dirname
import pathlib

build_root = dirname(dirname(abspath(__file__)))
sys.path.append(build_root)
from build_scripts.cmake.cmake import CMakeValue
from build_scripts.cmake import _create_build_env, which, build_project

def __create_aris_env():
    aris_env = _create_build_env()
    
    USE_NINJA = which("ninja", env=aris_env) is not None
    if "CMAKE_GENERATOR" in aris_env:
        USE_NINJA = aris_env["CMAKE_GENERATOR"].lower() == "ninja"

    if USE_NINJA:
        aris_env["CMAKE_GENERATOR"] = "ninja"

    return aris_env

def build_aris(
    aris_path: str,
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
    build_demo: bool = False,
    build_release: bool = False,
    build_debug: bool = False,
    **kwargs: CMakeValue,
) -> None:
    aris_env = __create_aris_env()
    cmake_args = { "BUILD_TESTS": build_test, "BUILD_DEMOS": build_demo }

    for build_type in ["Debug", "Release"]:
        not_debug, not_release = False, False
        if not (build_type == "Debug" and build_debug):
            not_debug = True
        if not (build_type == "Release" and build_release):
            not_release = True
        if not_debug and not_release:
            continue
        aris_env["CMAKE_BUILD_TYPE"] = build_type
        cmake_args["CMAKE_BUILD_TYPE"] = build_type
        build_project(
            project_path=str(aris_path),
            build_dir=build_dir,
            version=version,
            rerun_config=rerun_config,
            rm_cache=rm_cache,
            cmake_only=cmake_only,
            build_only=build_only,
            install_dir=install_dir,
            env=aris_env,
            **cmake_args,
            **kwargs
        )

def replace_string_in_win(file_path):
    """
    Replace all occurrences of a string in a file with a new string.

    :param file_path: Path to the file
    :param original_string: The string to be replaced
    :param new_string: The string to replace with
    """
    original_string = 'set(CMAKE_INSTALL_PREFIX "C:/aris/aris-${CMAKE_PROJECT_VERSION}")'
    # new_string = 'get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)' + '\n' + '  set(CMAKE_INSTALL_PREFIX "${PARENT_DIR}/install/aris/Release")'
    new_string = 'get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)'

    try:
        with open(file_path, 'r', encoding='gb2312') as file:
            file_contents = file.read()
        
        new_contents = file_contents.replace(original_string, new_string)
        
        with open(file_path, 'w', encoding='gb2312') as file:
            file.write(new_contents)
        
        print(f"Successfully replaced '{original_string}' with '{new_string}' in {file_path}")
    except Exception as e:
        print(f"An error occurred: {e}")


if __name__ == "__main__":
    # Placeholder for future interface. For now just gives a nice -h.
    parser = argparse.ArgumentParser(description="Build aris", allow_abbrev=True)
    parser.add_argument(
        '-b',
        "--base-path",
        type=pathlib.Path,
        required=True,
        help="Aris base path (CMakeList.txt folder)",
    )
    parser.add_argument(
        "--build-dir",
        type=pathlib.Path,
        help="Aris build directory",
    )
    parser.add_argument(
        '-i',
        "--install-dir",
        type=pathlib.Path,
        help="Aris isntall directory",
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
    replace_string_in_win(str(options.base_path) + '\CMakeLists.txt')

    if options.rm_cache_reconfig:
        options.rerun_config = True
        options.rm_cache = True

    # print(options, str(options.install_dir))
    if options.build_all:
        options.debug = True
        options.release = True
    
    build_aris(
        aris_path=str(options.base_path),
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
    )
