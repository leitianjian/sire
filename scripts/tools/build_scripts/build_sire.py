import os
from typing import Optional

from .cmake.env import check_env_flag
from .cmake.cmake import CMake, CMakeValue

def build_sire(
    sire_path: str,
    build_dir: Optional[str],
    install_dir: Optional[str],
    version: Optional[str],
    # cmake_python_library: Optional[str],
    # build_python: bool,
    rerun_config: bool,
    rm_cache: bool,
    cmake_only: bool,
    cmake_toolchain_file: Optional[str] = None,
    env: os._Environ[str] = os.environ,
    **kwargs: CMakeValue,
) -> None:
    cmake = CMake(base_dir=sire_path, build_dir=build_dir, install_dir=install_dir, env=env)
    cmake.defines(
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
        return
    cmake.build(env)
    # if build_python:
    #     caffe2_proto_dir = os.path.join(cmake.build_dir, "caffe2", "proto")
    #     for proto_file in glob(os.path.join(caffe2_proto_dir, "*.py")):
    #         if proto_file != os.path.join(caffe2_proto_dir, "__init__.py"):
    #             shutil.copy(proto_file, os.path.join("caffe2", "proto"))

    ######### set cmake configure option ##################
    '''
        # Store build options that are directly stored in environment variables
        build_options: Dict[str, CMakeValue] = {}

        # Build options that do not start with "BUILD_", "USE_", or "CMAKE_" and are directly controlled by env vars.
        # This is a dict that maps environment variables to the corresponding variable name in CMake.
        additional_options = {
            # Key: environment variable name. Value: Corresponding variable name to be passed to CMake. If you are
            # adding a new build option to this block: Consider making these two names identical and adding this option
            # in the block below.
            # "_GLIBCXX_USE_CXX11_ABI": "GLIBCXX_USE_CXX11_ABI",
            # "CUDNN_LIB_DIR": "CUDNN_LIBRARY",
            # "USE_CUDA_STATIC_LINK": "CAFFE2_STATIC_LINK_CUDA",
        }
        additional_options.update(
            {
                # Build options that have the same environment variable name and CMake variable name and that do not start
                # with "BUILD_", "USE_", or "CMAKE_". If you are adding a new build option, also make sure you add it to
                # CMakeLists.txt.
                var: var
                for var in (
                    # "UBSAN_FLAGS",
                    # "BLAS",
                    # "WITH_BLAS",
                    # "BUILDING_WITH_TORCH_LIBS",
                    # "CUDA_HOST_COMPILER",
                    # "CUDA_NVCC_EXECUTABLE",
                    # "CUDA_SEPARABLE_COMPILATION",
                    # "CUDNN_LIBRARY",
                    # "CUDNN_INCLUDE_DIR",
                    # "CUDNN_ROOT",
                    # "EXPERIMENTAL_SINGLE_THREAD_POOL",
                    # "INSTALL_TEST",
                    # "JAVA_HOME",
                    # "INTEL_MKL_DIR",
                    # "INTEL_OMP_DIR",
                    # "MKL_THREADING",
                    # "MKLDNN_CPU_RUNTIME",
                    # "MSVC_Z7_OVERRIDE",
                    # "CAFFE2_USE_MSVC_STATIC_RUNTIME",
                    # "Numa_INCLUDE_DIR",
                    # "Numa_LIBRARIES",
                    # "ONNX_ML",
                    # "ONNX_NAMESPACE",
                    # "ATEN_THREADING",
                    # "WERROR",
                    # "OPENSSL_ROOT_DIR",
                    # "STATIC_DISPATCH_BACKEND",
                    # "SELECTED_OP_LIST",
                    # "TORCH_CUDA_ARCH_LIST",
                    # "TRACING_BASED",
                )
            }
        )

        # Aliases which are lower priority than their canonical option
        low_priority_aliases = {
            # "CUDA_HOST_COMPILER": "CMAKE_CUDA_HOST_COMPILER",
            # "CUDAHOSTCXX": "CUDA_HOST_COMPILER",
            # "CMAKE_CUDA_HOST_COMPILER": "CUDA_HOST_COMPILER",
            # "CMAKE_CUDA_COMPILER": "CUDA_NVCC_EXECUTABLE",
            # "CUDACXX": "CUDA_NVCC_EXECUTABLE",
        }
        for var, val in my_env.items():
            # We currently pass over all environment variables that start with "BUILD_", "USE_", and "CMAKE_". This is
            # because we currently have no reliable way to get the list of all build options we have specified in
            # CMakeLists.txt. (`cmake -L` won't print dependent options when the dependency condition is not met.) We
            # will possibly change this in the future by parsing CMakeLists.txt ourselves (then additional_options would
            # also not be needed to be specified here).
            true_var = additional_options.get(var)
            if true_var is not None:
                build_options[true_var] = val
            elif var.startswith(("BUILD_", "USE_", "CMAKE_")) or var.endswith(
                ("EXITCODE", "EXITCODE__TRYRUN_OUTPUT")
            ):
                build_options[var] = val

            if var in low_priority_aliases:
                key = low_priority_aliases[var]
                if key not in build_options:
                    build_options[key] = val

        # The default value cannot be easily obtained in CMakeLists.txt. We set it here.
        # py_lib_path = sysconfig.get_path("purelib")
        # cmake_prefix_path = build_options.get("CMAKE_PREFIX_PATH", None)
        # if cmake_prefix_path:
        #     build_options["CMAKE_PREFIX_PATH"] = (
        #         py_lib_path + ";" + cast(str, cmake_prefix_path)
        #     )
        # else:
        #     build_options["CMAKE_PREFIX_PATH"] = py_lib_path

        # Some options must be post-processed. Ideally, this list will be shrunk to only one or two options in the
        # future, as CMake can detect many of these libraries pretty comfortably. We have them here for now before CMake
        # integration is completed. They appear here not in the CMake.defines call below because they start with either
        # "BUILD_" or "USE_" and must be overwritten here.
        build_options.update(
            {
                # Note: Do not add new build options to this dict if it is directly read from environment variable -- you
                # only need to add one in `CMakeLists.txt`. All build options that start with "BUILD_", "USE_", or "CMAKE_"
                # are automatically passed to CMake; For other options you can add to additional_options above.
                # "BUILD_PYTHON": build_python,
                # "BUILD_TEST": build_test,
                # "BUILD_DEMO": build_demo,
                # Most library detection should go to CMake script, except this one, which Python can do a much better job
                # due to NumPy's inherent Pythonic nature.
                # "USE_NUMPY": USE_NUMPY,
            }
        )
        # Options starting with CMAKE_
        cmake__options = {
            "CMAKE_INSTALL_PREFIX": self.install_dir,
        }

        # We set some CMAKE_* options in our Python build code instead of relying on the user's direct settings. Emit an
        # error if the user also attempts to set these CMAKE options directly.
        specified_cmake__options = set(build_options).intersection(cmake__options)
        if len(specified_cmake__options) > 0:
            print(
                ", ".join(specified_cmake__options)
                + " should not be specified in the environment variable. They are directly set by PyTorch build script."
            )
            sys.exit(1)
        build_options.update(cmake__options)

        self.defines(
            # PYTHON_EXECUTABLE=sys.executable,
            # PYTHON_LIBRARY=cmake_python_library,
            # PYTHON_INCLUDE_DIR=sysconfig.get_path("include"),
            SIRE_BUILD_VERSION=version,
            # NUMPY_INCLUDE_DIR=NUMPY_INCLUDE_DIR,
            **build_options,
        )
    '''