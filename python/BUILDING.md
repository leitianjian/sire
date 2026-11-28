# 构建和安装 Sire Python SDK

在你运行 Sire 的 Python 环境中安装。构建脚本使用该环境的解释器，并按 Python ABI、环境和构建配置隔离缓存。

## 准备原生依赖

需要 CMake 3.22 或更新版本、C++17 编译器和 Python 开发头文件。原生依赖包括 ARIS、COAL（`hpp-fcl`）、stduuid、Eigen3、Assimp、Qhull、msgpack-cxx 和 Boost 的 serialization、filesystem。首次构建 Clarabel 需要 Rust/Cargo。

从仓库根目录初始化依赖。根据[依赖构建说明](../docs/cuda_implementation.md#linux-构建)安装仓库锁定的 ARIS、COAL 和 stduuid，或使用已有的安装前缀。

```bash
git submodule update --init --recursive
export BUILD_ARIS_PATH=/path/to/aris/install
export BUILD_FCL_PATH=/path/to/coal/install
export BUILD_UUID_PATH=/path/to/stduuid/install
```

在 Windows 上用 `$env:BUILD_ARIS_PATH` 等设置同名变量。已有 vcpkg 工具链时可设置 `BUILD_TOOLCHAIN_PATH`。也可通过 `CMAKE_PREFIX_PATH` 提供统一依赖前缀。

## 安装和验证

从仓库根目录运行：

```bash
python -m pip install -e './python[viewer]'
```

默认构建 CPU 版。`[viewer]` 安装 GLFW 和 PyOpenGL，Viewer 使用 OpenGL 3.3 或更新版本。只用物理仿真时可安装 `./python`。CUDA 版使用同一入口，在 Linux 上安装时启用 `SIRE_ENABLE_CUDA`，按目标 GPU 设置 `CMAKE_CUDA_ARCHITECTURES`。例如 RTX 40 系列：

```bash
SIRE_ENABLE_CUDA=ON CMAKE_CUDA_ARCHITECTURES=89 python -m pip install -e ./python
```

如果有多个 CUDA Toolkit，可传入 `CUDAToolkit_ROOT` 和 `CMAKE_CUDA_COMPILER` 来选择对应的 Toolkit 和 `nvcc`。编译 CUDA 版需要 CUDA Toolkit 12.4 或更新版本，运行需要兼容的 NVIDIA 驱动。

安装后可在任意工作目录验证。CUDA 版另外提供 `sire.SireCudaBatchStepper`。

```bash
python -c "import sire; from sire.viewer import Viewer; from sire.sensor import Camera; print(sire.__file__); print(sire.SireRLBatchStepper)"
```

### 真实感渲染依赖

先安装与 NVIDIA 驱动兼容的 CUDA 版 PyTorch，再在同一环境启用可选依赖。

```bash
python -m pip install -e './python[photoreal]'
```

`gsplat` 首次使用可能通过 Ninja 编译 CUDA 扩展。需要可用的 CUDA Toolkit，并可在运行前用 `CUDA_HOME` 选择 Toolkit、用 `MAX_JOBS=4` 限制编译并行。CUDA 版 Sire SDK 在 Linux 上可通过 CUDA/OpenGL 互操作把 3DGS 图像直接送进 Viewer 的纹理。CPU 版 Sire SDK 也能使用真实感模式，此时图像经过主机缓冲。经典 Viewer 只需 `[viewer]`。

## 日常开发和 wheel

修改 C++ 后可以用同一 Python 环境执行增量编译。使用 CUDA 版时，重编时传入和初次安装一致的 CUDA 选项。

```bash
python python/build_native.py --jobs 8
python python/build_native.py --dry-run
```

默认编译 Release。调试或性能分析分别使用 `--profile debug`、`--profile profile`。改过 Clarabel 源码后可设置 `BUILD_SIRE_CLARABEL_FORCE_SOURCE_BUILD=true` 强制重编。编译产物的 Python ABI 保留在文件名中，Linux 为 `.so`，Windows 为 `.pyd`。

可从完整仓库生成供目标平台和 Python 版本使用的 wheel：

```bash
python -m pip wheel --no-deps --wheel-dir dist ./python
python -m pip install dist/sire-0.1.0-cp311-cp311-linux_x86_64.whl
```

Linux wheel 随扩展安装 ARIS、COAL 和 Clarabel 的共享库，运行机器需要兼容的系统运行库。CUDA wheel 也需要兼容的 GPU 和驱动。开发环境已有构建工具时可加 `--no-build-isolation` 使用当前环境的工具。
