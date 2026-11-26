# Sire Python SDK

Sire 的物理引擎使用 C++，Python 通过 pybind11 扩展调用 CPU 和可选的 CUDA 后端。构建前请查阅 [安装指南](BUILDING.md)。

```python
from sire import Simulator
from sire.sensor import Camera, CameraConfig
from sire.viewer import Viewer
```

Viewer 实时显示 Sire 场景，Camera 按仿真时间采集 RGB-D。Go2 策略播放、窗口操作及相机参数见仓库的 [sim2sim 指南](../sim2sim/README.md)。

C++ 应用可使用 `#include <sire.hpp>`，通过 CMake 的 `find_package(sire CONFIG REQUIRED)` 查找已安装的 Sire，并链接相应配置的 CMake 导出目标。
