# Sire RL Training — 开发指南

> 最后更新: 2026-07-21

## 一、修改记录

### 1. Recorder 内存泄漏修复
- **文件**: `demo/demo_python/SireRLGym/runners/on_policy_runner.py` (~line 188)
- **问题**: `resetRL()` 只 reset timer/contacts/events，不 reset recorder。recorder 每个 record 存全部 part 的 pq+vs+as+forces (~4KB)，跨 iter 无限累积 → 50 iter 可达 3GB+
- **修复**: iter 结束时同时调 `resetRL()` + `resetRecorder()`
```python
for sl in self.env.sire_sim_loops:
    sl.resetRL()
    sl.resetRecorder()
```

### 2. `_reinit_sire_env` 简化
- **文件**: `demo/demo_python/SireRLGym/envs/base/legged_robot_sire.py`
- **问题**: physics bounds violation 时重建整个 Simulator（解析XML+init），旧 C++ 对象泄漏（pybind11 wrapper 等 Python GC 才释放）
- **修复**: 直接调 `self.sire_simulators[env_idx].reset()`，底层 `simReset()` 已清 recorder/contacts/events/timer 并恢复 model 初始状态

### 3. Physics Bounds Check: `vs` → `vp`
- **文件**: 同上 `_refresh_sim_tensors_sire()`
- **问题**: 空间速度 `vs = [v_O, ω]`，线速度分量含 `ω×p` 放大。机器人离原点远 + 有角速度时产生大量假阳性
- **例**: env15, p=[4.15,6.17,0.46], ω=[-53.9,-28.5,-3.0] → vs_z=214 m/s 但 vp_z≈0.02 m/s（身体根本没动）
- **修复**: 用 `vp`（身体线速度）判断，阈值 vp=100 m/s, ω=100 rad/s（从 200 降下来）
- **日志**: 打印 `vp` 和 `ω` 分量

### 4. 可视化时间戳修复
- **文件**: `demo/demo_python/SireRLGym/test/test_visualize_recording.py`
- **问题**: recorder 的 `timeIndex` 因 timer 反复 reset 而不可靠
- **修复**: 改用帧序号 × 0.001s 生成合成时间戳
```python
dt_frame = 0.001
frames["timeIndex"] = [i * dt_frame for i in range(n_frames)]
```

### 5. gc.collect()
- **文件**: `on_policy_runner.py`
- **说明**: 每 5 iter 调一次 `gc.collect()` 释放 pybind11 包装器

---

## 二、常用命令

### 训练
```powershell
# Sire 后端 (plane 地形)
cd demo\demo_python\SireRLGym\scripts
python train.py --task go2 --num_envs 64 --max_iterations 100 ^
    --log_dir SireRLGym\logs --seed 42 --visualize_interval 10

# Sire 后端 (trimesh 地形)
python train.py --task go2 --num_envs 64 --max_iterations 100 ^
    --log_dir SireRLGym\logs --seed 42 --visualize_interval 10

# RLGym (MuJoCo) 训练
cd demo\demo_python\RLGym\scripts
python train.py --task go2 --num_envs 64 --max_iterations 100 --seed 42
```
注意

### 可视化
```powershell
cd demo\demo_python\SireRLGym
python test/test_visualize_recording.py ^
    --recording scripts\SireRLGym\logs\rough_go2\expXX\vis\recording_XX.json
```
- recording 每 `visualize_interval` iter 自动保存

### 安装
```powershell
cd python
pip install -e . -v --no-build-isolation
```

---

## 三、关键配置

### 地形 (go2_config.py)
| 参数 | 值 | 说明 |
|------|-----|------|
| `mesh_type` | `'trimesh'` / `'plane'` / `'heightfield'` | 地形类型。`plane`=平面, `trimesh`=粗糙地形, `heightfield`=高度场, `slope`=斜面 |
| `curriculum` | `True` / `False` | 地形课程学习。**平地训练必须关闭**，仅粗糙地形/高速模式需要 |
| `measure_heights` | `True` / `False` | 高度测量 (plane 返回全零) |
| `init_state.pos` | `[0, 0, 0.34]` | 初始 spawn 位置 z=0.34m |

### 训练超参数建议

**env 数量与 iter 的关系**：
- **env 不能太低**：env 太少（如 64）会导致采样不足，学习效果差甚至诡异。IsaacGym 标准是 env=1024, iter=1500，一定能收敛
- **iter 无法替代 env**：用更多 iter 去弥补 env 不足是不可行的，采样多样性的缺失无法靠训练轮数解决
- **Sire 内存限制**：目前 Sire 是 CPU 仿真，64 env 已有内存压力。目标应优化到 1024 env

**iter 设置**：
- **平地**：1000+ iter 基本收敛
- **粗糙地形**：取决于地形难度和奖励设计，通常更多
- **实用策略**：设一个很大的值（如 10000），中途看收敛了就 Ctrl+C 停下（模型会按 save_interval 自动保存）

**curriculum（课程学习）**：
- 平地训练：**关掉**（`curriculum = False`，`mesh_type = 'plane'`）
- 粗糙地形/高速模式：启用（逐渐增加地形难度）
- curriculum 本质是给地形难度一个可放缩的维度

**save_interval**：
- 配置在 `legged_robot_config.py` 底部（`runner` 部分）
- 默认 100，建议改成 500

### 地形配置速查

| 场景 | mesh_type | curriculum | 说明 |
|------|-----------|------------|------|
| 平地 | `'plane'` | `False` | 最简单，先跑通这个 |
| 粗糙地形 | `'trimesh'` | `True` | 论文标准设置 |
| 高度场 | `'heightfield'` | `True` | 基于采样的地形 |
| 斜面 | `'slope'` | - | 固定角度的斜坡 |

### 物理参数
| 参数 | 值 |
|------|-----|
| sim_dt | 0.001 |
| ctrlT | 0.01 |
| decimation | 10 |
| kp | 25 |
| kd | 0.6 |
| action_scale | 0.25 |
| control_type | 'P' |

### Joint Limits (硬编码在 `_GO2_JOINT_LIMITS`)
| Joint | Range (rad) | Torque Limit (Nm) |
|-------|-------------|-------------------|
| hip | [-1.0472, 1.0472] | 23.7 |
| thigh (前腿) | [-1.5708, 3.4907] | 23.7 |
| thigh (后腿) | [-0.5236, 4.5379] | 23.7 |
| calf | [-2.7227, 0.83776] | 35.55 |

### Part Pool 映射
```
 0 = ground       1 = base (termination)
 2 = FL_hip       3 = FL_thigh (penalty)   4 = FL_calf (penalty)   5 = FL_foot
 6 = FR_hip       7 = FR_thigh (penalty)   8 = FR_calf (penalty)   9 = FR_foot
10 = RL_hip      11 = RL_thigh (penalty)  12 = RL_calf (penalty)  13 = RL_foot
14 = RR_hip      15 = RR_thigh (penalty)  16 = RR_calf (penalty)  17 = RR_foot
```

---

## 四、待修复问题

1. **Collision filter 需扩展到 26×26**: calf collision geometry (ID 14-25) 在现有 14×14 filter 范围外，碰撞处理异常 → 物理爆炸（可能是主因）
2. **Calf collision geometry 缺少属性**: `go2_rai_foot.xml` 中 calf 的 CylinderCollisionGeometry 缺少 `material="m1" contact_prop="{k:2.8e8,d:2000}"`
3. **Bounds threshold 待验证**: 100 m/s / 100 rad/s 在实际训练中是否合理

---

## 五、目录结构速览

```
d:\code\sire\
├── demo/demo_python/
│   ├── SireRLGym/          ← Sire 后端 RL 训练
│   │   ├── envs/base/legged_robot_sire.py   ← 核心环境
│   │   ├── envs/go2/go2_config.py            ← Go2 配置
│   │   ├── runners/on_policy_runner.py       ← 训练循环
│   │   ├── scripts/train.py                  ← 训练入口
│   │   └── test/test_visualize_recording.py  ← 可视化
│   ├── RLGym/              ← MuJoCo 后端 RL 训练 (参考)
│   └── sirePaperDogRL/     ← 单例 Sire 仿真 (dog.py)
├── python/sire/            ← pybind11 绑定
├── src/simulator/          ← C++ 仿真循环
└── include/sire/simulator/ ← C++ 头文件
```
