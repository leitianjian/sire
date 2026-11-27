# Sire CUDA

这是 Sire 的实验性 CUDA 后端，目标是在 GPU 上保留 Sire 的动力学与连续接触求解流程，并通过批量环境并行加速强化学习。当前支持 Go2 平地 PPO。物理状态、观测、奖励和策略更新使用同一块 GPU。

## 安装与使用

从仓库根目录按照[安装指南](python/BUILDING.md)准备原生依赖，并在目标 Python 环境安装 Sire：

```bash
python -m pip install -e './python[viewer]'
```

安装后，在任意工作目录都可以使用 `from sire import Simulator`、`from sire.viewer import Viewer` 和 `from sire.sensor import Camera`。Go2 的策略播放与 RGB-D 示例见 [sim2sim 使用说明](sim2sim/README.md#sire-单场景实时行走与-rgb-d)。

## CUDA 训练

在 Linux 上使用 CUDA Toolkit 12.4 或更新版本和 CUDA 版 PyTorch。按 [CUDA 实现与使用指南](docs/cuda_implementation.md)安装 CUDA 版 Sire 和训练依赖，然后从仓库根目录运行。

```bash
python demo/demo_python/SireRLGym/scripts/train.py --task go2 --flat_terrain \
  --sire_backend cuda --rl_device cuda:0 --num_envs 256 --max_iterations 3
```

CUDA 默认用 4 条 stream 并行推进环境，`--sire_cuda_streams 1` 可切换为单 stream。训练入口默认选择 CPU 后端，可用 `--sire_batch_threads` 设置 CPU 批量步进的线程数。

## 仓库结构

| 路径 | 内容 |
| --- | --- |
| `include/sire/`、`src/` | C++ 引擎、传感接口和 CUDA 后端 |
| `python/sire/` | pybind11 的 C++ 绑定 |
| `python/src/sire/` | 安装后的 Python SDK，包括 Viewer、Camera 和录制回放接口 |
| `demo/demo_python/SireRLGym/` | Go2 训练、播放与评估 |
| `sim2sim/` | 策略导出、仿真对照和 Go2 单场景行走 |
| `test/`、`python/test_native_build.py` | 原生回归与 Python 构建测试 |
| `docs/` | 接触求解、CUDA 实现、性能和发布媒体 |
| `demo/`、`scripts/` | C++/Python 示例、论文复现和开发工具。`scripts/dm_control/` 是场景转换使用的上游副本 |
| `third_party/`、`cmake_macros/` | 原生依赖与构建辅助 |

旧版浏览器示例所用的 `web_interface/` 可从 Git 提交 `8cec73e` 取出。复现这些示例时，在外部目录构建前端，再将示例的 `document_root` 指向该目录下构建后的静态资源。

## 论文与许可证

Sire 的高刚度连续接触方法发表于 IEEE Robotics and Automation Letters。[A Convergent Continuous Contact Solver With Explicit Separation Time for High-Stiffness Contact](https://doi.org/10.1109/LRA.2026.3692328)

本项目采用 [MIT 许可证](LICENSE.txt)。
