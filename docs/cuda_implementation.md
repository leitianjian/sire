# Sire CUDA 实现与使用指南

## 代码入口

| 位置 | 职责 |
| --- | --- |
| [`src/physics/cuda/`](../src/physics/cuda/) | GPU 模型、刚体动力学、接触历史和积分内核 |
| [`flat_ground_collision.cu`](../src/physics/cuda/flat_ground_collision.cu) · [`contact_trajectory.cu`](../src/physics/cuda/contact_trajectory.cu) · [`spectral_admm.cu`](../src/physics/cuda/spectral_admm.cu) | 平地碰撞、法向轨迹与分离搜索、Shifted Spectral ADMM |
| [`rigid_body_batch.cpp`](../src/physics/cuda/rigid_body_batch.cpp) | 场景上传、设备缓冲区、条件 CUDA Graph、批量步进与重置 |
| [`bindings_cuda.cpp`](../python/sire/bindings_cuda.cpp) | Python 批量步进接口 `SireCudaBatchStepper`，发布可供 PyTorch 使用的 CUDA 状态视图 |
| [`legged_robot_sire_cuda.py`](../demo/demo_python/SireRLGym/envs/base/legged_robot_sire_cuda.py) | CUDA 环境、动作与关节映射、观测、奖励和部分重置 |
| [`go2_config.py`](../demo/demo_python/SireRLGym/envs/go2/go2_config.py) · [`go2_rai_foot.xml`](../demo/demo_python/sirePaperDogRL/go2_rai_foot.xml) | GO2 环境及 PPO 默认配置、Sire 场景资产 |
| [`train.py`](../demo/demo_python/SireRLGym/scripts/train.py) · [`on_policy_runner.py`](../demo/demo_python/SireRLGym/runners/on_policy_runner.py) · [`benchmark_sire_batch.py`](../demo/demo_python/SireRLGym/test/benchmark_sire_batch.py) | 训练入口、PPO 采样循环、CPU/CUDA 步进基准 |

## 一个控制周期如何推进

1. 初始化时由 Sire 读取场景，将刚体树、地面几何、材料和电机关节约束上传到 GPU。每个环境保存独立的刚体状态、时钟和接触历史。
2. 每个物理事件先以 FP64 计算关节 PD 力矩、自由动力学和关节限位反力，再检测机器人与地面的接触。碰撞结果进入接触坐标系构造与历史匹配，用于处理初始穿透和为接触求解提供热启动。
3. 接触求解使用 FP64。法向轨迹用于搜索分离时间并确定可接受的推进间隔。当材料参数一致且满足比例刚度和阻尼条件时，使用解耦的阻尼模态计算法向轨迹，其余情况使用通用 Padé 矩阵指数。Shifted Spectral ADMM 用该轨迹给出的法向目标速度求解摩擦接触力。随后施加接触力，执行半隐式 Euler 积分和刚体树约束投影。条件 CUDA Graph 重复调度物理事件，直至活动环境到达本次控制周期的边界。
4. CUDA 默认按环境索引把批次划成连续分片，在单块 GPU 的 4 条 stream 上推进。分片保持原始环境顺序，动作与输出按同一索引对应。CUDA event 保证分片等待调用方的输入就绪，并在分片完成后让 PyTorch stream 等待全部结果。PPO 每个控制周期仍接收一个完整、顺序一致的环境批次。
5. PyTorch 从设备状态计算观测、奖励和终止条件。重置掩码只重置选中的环境及其时钟和接触历史。

## Linux 构建

需要 Git、CMake ≥ 3.22、支持 C++17 且与 CUDA Toolkit 兼容的宿主编译器、CUDA Toolkit ≥ 12.4、兼容的 NVIDIA 驱动，以及带开发头文件的 Python。首次从源码构建 Clarabel 还需要 Rust/Cargo。系统依赖包括 Eigen3、Assimp、Qhull、msgpack-cxx 和 Boost 的 serialization、filesystem 开发包。ARIS、COAL（`hpp-fcl`）与 stduuid 使用仓库锁定的子模块版本。

从仓库根目录执行：

```bash
git submodule update --init --recursive
python3 -m venv .venv
source .venv/bin/activate
```

在这个环境中安装支持 CUDA 的 PyTorch，再安装训练依赖。依赖清单包含 CMake、pybind11 和训练使用的 `rsl_rl` 版本。

```bash
python -m pip install -r demo/demo_python/SireRLGym/requirements.txt
python -c 'import torch; assert torch.cuda.is_available()'
```

将三个子模块安装到同一前缀，随后构建 Python 扩展。将 `SIRE_CUDA_ARCH` 设为目标 GPU 的计算能力编号，例如 RTX 40 系列使用 `89`。也可以复用已经构建的依赖前缀。

```bash
SIRE_DEPS_PREFIX="$PWD/build/deps/install"
SIRE_CUDA_ARCH=89

cmake -S third_party/aris -B build/deps/aris \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$SIRE_DEPS_PREFIX"
cmake --build build/deps/aris --target install --parallel

cmake -S third_party/hpp-fcl -B build/deps/coal \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$SIRE_DEPS_PREFIX" \
  -DBUILD_TESTING=OFF -DBUILD_PYTHON_INTERFACE=OFF
cmake --build build/deps/coal --target install --parallel

cmake -S third_party/stduuid -B build/deps/stduuid \
  -DCMAKE_INSTALL_PREFIX="$SIRE_DEPS_PREFIX" \
  -DUUID_ENABLE_INSTALL=ON -DUUID_BUILD_TESTS=OFF
cmake --build build/deps/stduuid --target install --parallel

cmake -S . -B build/cuda -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$SIRE_DEPS_PREFIX" \
  -DCMAKE_CUDA_ARCHITECTURES="$SIRE_CUDA_ARCH" \
  -DSIRE_ENABLE_CUDA=ON -DBUILD_PYTHON=ON \
  -DPYBIND11_FINDPYTHON=ON -DPython_EXECUTABLE="$(command -v python)" \
  -Dpybind11_DIR="$(python -m pybind11 --cmakedir)"
cmake --build build/cuda --target sire --parallel

export PYTHONPATH="$PWD/build/cuda/python_modules:$PWD/demo/demo_python${PYTHONPATH:+:$PYTHONPATH}"
python -c 'import sire; print(sire.__file__); print(sire.SireCudaBatchStepper)'
```

如果系统安装了多个 Toolkit，可在首次配置时通过 `CMAKE_CUDA_COMPILER` 指定 `nvcc`，并通过 `CUDAToolkit_ROOT` 指定对应的 Toolkit 根目录。依赖分开安装时，可以使用分号连接多个 `CMAKE_PREFIX_PATH` 前缀，或设置 `TARGET_ARIS_PATH`、`TARGET_HPP_FCL_PATH` 与 `TARGET_STDUUID_PATH`。

## 训练与步进基准

继续使用上面激活的 Python 环境与 `PYTHONPATH`。下面的命令从仓库根目录启动 1024 环境的 Go2 训练，每 100 轮存档一次。

```bash
python -m SireRLGym.scripts.train --task go2 --flat_terrain \
  --sire_backend cuda --rl_device cuda:0 --num_envs 1024 \
  --num_steps_per_env 24 --max_iterations 1000 --save_interval 100 --seed 1
```

| 参数 | 含义 |
| --- | --- |
| `--task go2 --flat_terrain` | 选择 GO2 平地任务 |
| `--sire_backend cuda --rl_device cuda:0` | 物理仿真与 PPO 使用同一块 GPU |
| `--sire_cuda_streams N` | CUDA 并行 stream 数。默认 4，上限为环境总数。设为 1 可运行单 stream |
| `--sire_backend cpu --sire_batch_threads N` | 使用 CPU 仿真并指定原生批量线程数，PPO 的设备仍由 `--rl_device` 指定 |
| `--num_envs`、`--num_steps_per_env` | 并行环境总数、每轮 PPO 从每个环境采集的控制步数 |
| `--max_iterations`、`--save_interval`、`--log_dir` | 目标迭代编号、模型保存间隔、日志根目录 |
| `--resume PATH` | 加载 checkpoint 并训练到 `--max_iterations` 指定的编号 |
| `--seed`、`--log_interval` | 随机种子、训练统计的输出间隔 |

Go2 默认物理步长为 1 ms，策略控制周期为 20 ms。环境和 PPO 默认值在
[`go2_config.py`](../demo/demo_python/SireRLGym/envs/go2/go2_config.py)，
流数由 `sim.sire_cuda_streams` 控制。原生基准中，每个环境完成一个控制周期计为一步。

```bash
python -m SireRLGym.test.benchmark_sire_batch --backend cuda --device cuda:0 \
  --flat-terrain --native-only --num-envs 256 --warmup-steps 5 --steps 30
```

追加 `--streams 1` 可对比单 stream。CPU 基准改用 `--backend cpu --device cpu --threads N`。若需要运行原生回归测试，以 `-DBUILD_TEST=ON` 重新配置并构建 CUDA 测试目标，再用 `ctest --test-dir build/cuda -R '^cuda_' --output-on-failure` 运行。逐事件对照和接触力回放见[接触实现与已锁定依赖](contact_implementation.md)。

## 适用范围

CUDA 路径目前支持无窗口 GO2 平地训练，使用 P 控制、固定摩擦与质量，关闭随机推力、地形或指令课程、地形高度测量、CPU 场景录制与求解器 trace。碰撞使用平地专用几何路径。箱体或圆柱在地面箱体的侧面和边缘遇到该路径未覆盖的接触时，环境会返回状态码。CPU 仍是训练入口的默认后端，也可用于现有的地形和调试功能。
