# Sire CUDA

这是 Sire 的实验性 CUDA 后端，目标是在 GPU 上保留 Sire 的动力学与连续接触求解流程，并通过批量环境并行加速强化学习。当前支持 Go2 平地 PPO。物理状态、观测、奖励和策略更新使用同一块 GPU。

## 快速开始

在 Linux 上使用 CUDA Toolkit 12.4 或更新版本和 CUDA 版 PyTorch。编译 Python 模块并设置 `PYTHONPATH` 后，在仓库根目录运行以下命令。[CUDA 实现与使用指南](docs/cuda_implementation.md)

```bash
python -m SireRLGym.scripts.train --task go2 --flat_terrain \
  --sire_backend cuda --rl_device cuda:0 --num_envs 256 --max_iterations 3
```

CUDA 默认用 4 条 stream 并行推进环境，`--sire_cuda_streams 1` 可切换为单 stream。训练入口默认选择 CPU 后端，可用 `--sire_batch_threads` 设置 CPU 批量步进的线程数。

## 论文与许可证

Sire 的高刚度连续接触方法发表于 IEEE Robotics and Automation Letters。[A Convergent Continuous Contact Solver With Explicit Separation Time for High-Stiffness Contact](https://doi.org/10.1109/LRA.2026.3692328)

本项目采用 [MIT 许可证](LICENSE.txt)。
