import warnings
import sys

# 版本信息
__version__ = "1.0.0"

# # 动态导入核心功能
# def __getattr__(name):
#     """延迟加载模块和函数"""
#     # if name in ['core', 'extensions']:
#         # 导入子模块
#     try:
#       module = __import__(f'sire.{name}')
#       setattr(sys.modules[__name__], name, module)
#       return module
#     except ImportError:
#       raise AttributeError(f"module 'sire' has no attribute '{name}'")
    # # 尝试从核心模块导入常用函数
    # if name in ['calculate', 'process', 'transform']:
    #     try:
    #         from .core import name as func
    #         setattr(sys.modules[__name__], name, func)
    #         return func

    
# import logging
# logger = logging.getLogger('sire')

# from . import visualize

# logger.debug("开始提升 visualize 模块的函数")

# # 提升函数
# for name in dir(visualize):
#     if not name.startswith('_'):
#         logger.debug(f"提升函数: {name}")
#         setattr(sys.modules[__name__], name, getattr(visualize, name))

# logger.debug("函数提升完成")
# 导入纯 Python 模块
from .py_module import visualize

# 将函数映射到当前命名空间
for name in dir(visualize):
    if not name.startswith('_'):
      setattr(sys.modules[__name__], name, getattr(visualize, name))

try:
  from .native import sire
  # 将函数映射到当前命名空间
  for name in dir(sire):
    if not name.startswith('_'):
      setattr(sys.modules[__name__], name, getattr(sire, name))
except ImportError as error:
  raise ImportError(
      f"Sire native extension could not be loaded by {sys.executable}. "
      "Build it with the same project venv using python/build_native.py. "
      f"Original error: {error}"
  ) from error
