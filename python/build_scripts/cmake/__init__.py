from .cmake import build_project
from .cmake_utils import which
from .env import _create_build_env

# if __name__ == '__main__':
#     print(which('cmake', ['C:\\Program Files\\Microsoft Visual Studio\\2022\\Community\\Common7\\IDE\\CommonExtensions\\Microsoft\\CMake\\CMake\\bin']))
#     os.environ["USE_NINJA"] = "False"
#     print(os.environ["USE_NINJA"], os.getenv("USE_NINJA"))
#     print(check_negative_env_flag("USE_NINJA"))
#     print(distutils._msvccompiler._get_vc_env('x64'))
