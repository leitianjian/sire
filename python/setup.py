#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import platform
import subprocess
import shutil
from pathlib import Path
from setuptools import setup, find_packages
import tomllib as toml
# import dotenv
from setuptools.command.install import install
from setuptools.command.build import build
from setuptools.command.develop import develop
from setuptools.command.build_ext import build_ext
from setuptools.command.editable_wheel import editable_wheel
# 项目根目录
PROJECT_ROOT = Path(__file__).parent.resolve()
# Add the current directory to the Python path so that we can import `tools`.
# This is required when running this script with a PEP-517-enabled build backend.
#
# From the PEP-517 documentation: https://peps.python.org/pep-0517
#
# > When importing the module path, we do *not* look in the directory containing
# > the source tree, unless that would be on `sys.path` anyway (e.g. because it
# > is specified in `PYTHONPATH`).
#
sys.path.insert(0, str(PROJECT_ROOT))  # this only affects the current process
# Add the current directory to PYTHONPATH so that we can import `tools` in subprocesses
os.environ["PYTHONPATH"] = os.pathsep.join(
    [
        str(PROJECT_ROOT),
        os.getenv("PYTHONPATH", ""),
    ]
).rstrip(os.pathsep)

from cmake_py import _create_build_env, which, build_project

# 配置加载器
class ConfigLoader:
    def __init__(self):
        self.project_root = Path(__file__).parent
        self.config = {}
        
        # 加载环境变量
        # dotenv.load_dotenv(self.project_root / ".env")
        
        # 确定配置环境
        # self.profile = os.getenv("CONFIG_PROFILE", "dev")
        
        # 加载配置
        self.load_base_config()
        # self.load_profile_config()
        self.load_local_config()
        self.apply_env_overrides()
    
    def load_base_config(self):
        """加载pyproject.toml中的基础配置"""
        base_path = self.project_root / "pyproject.toml"
        if base_path.exists():
            with open(base_path, "rb") as f:
                self.config = toml.load(f)
    
    def load_profile_config(self):
        """加载环境特定配置"""
        profile_path = self.project_root / "config" / f"{self.profile}.toml"
        if not profile_path.exists():
            print(f"警告: {profile_path} 配置文件不存在")
            return
        with open(profile_path, "rb") as f:
            profile_config = toml.load(f)
        
        # 处理包含关系
        if "include" in profile_config:
            include_path = self.project_root / "config" / profile_config["include"]
            if include_path.exists():
                with open(include_path, "rb") as f:
                    base_config = toml.load(f)
                    self.merge_configs(base_config, profile_config)
        
        # 合并到主配置
        self.merge_configs(self.config, profile_config)
        print(self.config)
    
    def load_local_config(self):
        """加载用户特定配置"""
        local_path = self.project_root / "local.toml"
        if local_path.exists():
            with open(local_path, "rb") as f:
                local_config = toml.load(f)
                self.merge_configs(self.config, local_config)
    
    def apply_env_overrides(self):
        """应用环境变量覆盖"""
        # 处理所有 ${VAR} 格式的占位符
        self.resolve_placeholders(self.config)
        
        # 特定环境变量覆盖
        for key in self.config.get("tool", {}).get("my_project", {}).get("build", {}).keys():
            env_key = f"BUILD_{key.upper()}"
            if env_key in os.environ:
                self.config["tool"]["my_project"]["build"][key] = os.environ[env_key]
    
    def resolve_placeholders(self, config):
        """递归解析配置中的环境变量占位符"""
        if isinstance(config, dict):
            for key, value in config.items():
                if isinstance(value, str):
                    config[key] = self.expand_vars(value)
                elif isinstance(value, (dict, list)):
                    self.resolve_placeholders(value)
        elif isinstance(config, list):
            for i, item in enumerate(config):
                if isinstance(item, str):
                    config[i] = self.expand_vars(item)
                elif isinstance(item, (dict, list)):
                    self.resolve_placeholders(item)
    
    def expand_vars(self, value):
        """展开环境变量占位符"""
        return os.path.expandvars(value)
    
    def merge_configs(self, base, update):
        """深度合并两个配置字典"""
        for key, value in update.items():
            if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                self.merge_configs(base[key], value)
            else:
                base[key] = value
    
    def get_build_config(self):
        """获取构建配置"""
        return self.config.get("tool", {}).get("sire", {}).get("build", {})

# 初始化配置
config_loader = ConfigLoader()
build_config = config_loader.get_build_config()
print(build_config)
# 平台特定的编译参数
def get_compile_args():
    if platform.system() == "Windows":
        return ["/O2", "/std:c++17", "/MP"]
    else:
        return ["-O3", "-std=c++17", "-fvisibility=hidden", "-fPIC"]

def get_link_args():
    if platform.system() == "Windows":
        return []
    else:
        return ["-Wl,-rpath=$ORIGIN"]

# 自定义 CMake 构建扩展

class CustomEditableWheel(editable_wheel):
    """自定义构建命令，集成 CMake"""
    
    def initialize_options(self):
        super().initialize_options()
    
    def finalize_options(self):
        super().finalize_options()
    
    def run(self):
        # 确保 CMake 可用
        self.run_command('build_ext')
        super().run()
    

class CMakeBuildExt(build_ext):
    """自定义构建命令，集成 CMake"""
    
    # user_options = build_ext.user_options + [
    #     ('cmake-args=', None, '额外的 CMake 参数'),
    #     ('build-type=', None, '构建类型 (Release, Debug, RelWithDebInfo)'),
    # ]
    
    # def initialize_options(self):
    #     super().initialize_options()
    #     self.cmake_args = ""
    #     self.build_type = "Release"
    
    # def finalize_options(self):
    #     super().finalize_options()
    
    def run(self):
        # 确保 CMake 可用
        try:
            subprocess.check_call(["cmake", "--version"])
        except OSError:
            raise RuntimeError("CMake 必须安装才能构建扩展")
        
        my_env = _create_build_env()
        USE_NINJA = which("ninja", env=my_env) is not None
        if "CMAKE_GENERATOR" in my_env:
            USE_NINJA = my_env["CMAKE_GENERATOR"].lower() == "ninja"
        if USE_NINJA:
            my_env["CMAKE_GENERATOR"] = "ninja"
        
        if build_config["cpp_build_type"].lower() == "all":
            build_config["debug"] = True
            build_config["release"] = True
        elif build_config["cpp_build_type"].lower() == "debug":
            build_config["debug"] = True
            build_config["release"] = False
        elif build_config["cpp_build_type"].lower() == "release":
            build_config["debug"] = False
            build_config["release"] = True
        py_install_type = build_config.get("py_install_type", "release").lower()
        assert(build_config[py_install_type], "py install type must be in cpp build type")

        cmake_args = {
            "BUILD_DEMO": True if build_config.get("build_demo", 0) else False,
            "BUILD_TEST": True if build_config.get("build_test", 0) else False,
            "BUILD_PYTHON": True if build_config.get("build_python", 0) else False,
            "SIRE_ENABLE_TRACY": True if build_config.get("sire_enable_tracy", 0) else False
        }
        print("[sire] cmake args:", cmake_args)
        if "toolchain_path" in build_config:
            cmake_args["CMAKE_TOOLCHAIN_FILE"] = str(build_config["toolchain_path"])
        install_dir = {}
        for build_type in ["Debug", "Release"]:
            not_debug, not_release = False, False
            if not (build_type == "Debug" and build_config.get("debug", 0)):
                not_debug = True
            if not (build_type == "Release" and build_config.get("release", 0)):
                not_release = True
            if not_debug and not_release:
                continue

            cmake_args.update({
                "TARGET_ARIS_PATH": str(build_config.get("aris_path", "")), 
                "TARGET_HPP_FCL_PATH": str(Path(build_config.get("fcl_path", "")) / build_type), 
                "TARGET_STDUUID_PATH": str(Path(build_config.get("uuid_path", "")) / build_type),
                # "TARGET_PYTHON_PATH": str(options.python_path),
                "CMAKE_BUILD_TYPE": build_type,
                })
            my_env["CMAKE_BUILD_TYPE"] = build_type
            print(build_config.get("cpp_base_path", ""))
            installDir = build_project(
                project_path=str(build_config.get("cpp_base_path", "")),
                build_dir=None if build_config.get("build_dir", None) is None else str(build_config.get("build_dir", "")),
                version=None,
                # cmake_python_library=None,
                # build_python=False,
                rerun_config=build_config.get("rerun_config", False),
                rm_cache=build_config.get("rm_cmake_cache", False),
                cmake_only=build_config.get("cmake_only", False),
                build_only=False,
                install_dir=None if build_config.get("install_dir", None) is None else str(build_config.get("install_dir", None)),
                env=my_env,
                **cmake_args
            )
            install_dir[build_type] = installDir
        ext_src = os.path.join(installDir, "python", py_install_type)
        ext_dest = os.path.join(PROJECT_ROOT, "src", "sire", "native")
        os.makedirs(ext_dest, exist_ok=True)
        shutil.copytree(ext_src, ext_dest, dirs_exist_ok=True)
        # 调用父类方法处理 Python 部分
        super().run()

# 自定义安装命令（添加后处理）
class CustomInstall(install):
    def run(self):
        raise RuntimeError("这就是个调试错误！看看我有没有被调用！")
        super().run()
        self.post_install()
    
    def post_install(self):
        """安装后处理"""
        print("安装完成！")
        print("运行测试: pytest")
        print("启动开发服务器: python -m my_package.app")

# # 自定义开发模式安装
# class CustomDevelop(develop):
#     def run(self):
#         super().run()
#         self.post_develop()
    
#     def post_develop(self):
#         """开发模式后处理"""
#         print("开发模式安装完成！")
#         print("C++ 扩展已构建在: build/")
#         print("修改 Python 代码后立即生效")
#         print("修改 C++ 代码后运行: python setup.py build_ext")

# 主设置函数
setup(
    # 包配置
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    include_package_data=True,
    package_data={
        "sire": ["*.pyd", "*.so", "*.dylib", "*.dll"]
    },
    zip_safe=False,
    
    # 自定义命令
    cmdclass={
        "build_ext": CMakeBuildExt,
        "install": CustomInstall,
        'editable_wheel': CustomEditableWheel
        # "develop": CustomDevelop
    },
)