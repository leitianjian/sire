# Go2 策略播放与仿真对照

默认使用你选定的 `exp7/model_550.pt` 导出的 actor JIT。本文的 `run.sh` 入口只用于仿真，不连接机器人真机；不会启动训练、创建 Sire 环境或修改后台实验。新建的真机代码副本单独放在 `deploy_sire/`，请先阅读其 `README_REVIEW.md`，不要混淆两个入口。

## Sire 单场景实时行走与 RGB-D

在当前 Python 环境按 [SDK 安装指南](../python/BUILDING.md)安装 `./python[viewer]`，并准备 `torch` 和 `PyYAML`。新克隆需要准备 Go2 的 OBJ 网格和一个策略 checkpoint。网格取自 [MuJoCo Menagerie 的 `unitree_go2/assets`](https://github.com/google-deepmind/mujoco_menagerie/tree/main/unitree_go2/assets)。checkpoint 可用 [Go2 平地训练入口](../docs/cuda_implementation.md#训练与步进基准)生成，其旁边需有训练保存的 `config.yaml`。

从仓库根目录导出并运行，将下面两个路径换成自己的 checkpoint 和 Menagerie checkout。

```bash
python -m sim2sim.export_policy /path/to/run/model_1000.pt \
  --out /tmp/sire_go2_jit.pt
python -m sim2sim.walk_sire --policy /tmp/sire_go2_jit.pt \
  --mesh-dir /path/to/mujoco_menagerie/unitree_go2/assets \
  --cmd 0.25 0 0 --duration 10
```

也可以在任意目录运行 `python /path/to/sire/sim2sim/walk_sire.py`。

Sire CPU 求解单个平地场景，Sire Viewer 用 GLFW 和 OpenGL 绘制窗口和 RGB-D。左键旋转、右键平移、滚轮缩放、Esc 退出。右侧 RGB 和深度预览叠加在场景之上。`--cmd` 依次传入机身前向速度、侧向速度和偏航角速度。

- 在同一个 `walk_sire.py` 中用 `--render-mode classic|photoreal|none` 选择模式。`classic` 是默认的 GLFW Viewer。`none` 只跑 Sire 和策略，不加载渲染网格、不创建相机或窗口，也不等待墙钟时间。
- 将 JIT 和 OBJ 分别放到 `sim2sim/policies/go2_cuda_1024/model_1000_jit.pt` 和 `demo/demo_python/dogRL/assets` 后，可以省略相应参数。
- 用 `--camera-size`、`--camera-fps`、`--camera-fov-y` 或 `--camera-intrinsics FX FY CX CY`、`--camera-clip NEAR FAR` 调整相机。SDK 的 [`CameraConfig`](../python/src/sire/sensor.py) 设置安装连杆及相对位姿，`Camera.read()` 获取最近一帧。
- 用 `--save-rgbd frame.npz` 保存最后一帧，包含 RGB、米制光轴深度、有效像素掩码、仿真时间戳、帧编号和内参。`classic` 和 `photoreal` 加 `--headless` 时仍会使用 EGL 渲染 RGB-D。
- 三种模式都在 [`walk_sire.py`](walk_sire.py) 的同一循环中调用 `sim.step(action)`。渲染模式的窗口和相机使用 OpenGL 3.3 或更新版本。

```bash
python -m sim2sim.walk_sire --render-mode none --duration 10
python -m sim2sim.walk_sire --render-mode classic --headless --duration 2 --save-rgbd /tmp/classic.npz
```

### 真实感模式

在已有 CUDA 版 PyTorch 的 Python 环境按 [SDK 安装指南](../python/BUILDING.md#真实感渲染依赖)安装 `./python[photoreal]`。准备一份标准 3DGS PLY 和与它对齐的三角网格（PLY 或 OBJ）。例如从 [Habitat-GS scene56](https://huggingface.co/datasets/RukawaY/gs_scenes/tree/9d1b04d/val/scene56) 下载 `scene56.gs.ply` 和 `scene56.mesh.ply`，在同一目录保存 `room.json`。

```json
{
  "gaussians": "scene56.gs.ply",
  "depth_mesh": "scene56.mesh.ply",
  "position": [-2.0, 2.625, 0.1405],
  "rotation_wxyz": [0.5, 0.5, 0.5, 0.5],
  "scale": 0.25
}
```

两个资产共用这个资产到 Sire 世界坐标的变换。`position` 用米，`scale` 是每个资产单位对应的米数。示例把一段开阔地板对齐到 Sire 的起步位置；其他场景需要相应调整这三个值。

```bash
MAX_JOBS=4 python -m sim2sim.walk_sire --render-mode photoreal \
  --scene /path/to/room.json --cmd 0.10 0 0 --duration 4
```

场景的 RGB 由 CUDA 3DGS 渲染，Go2 用 PBR 网格渲染。`--scene` 用同一份资产的几何替换原地面碰撞盒，CPU Sire 可与扫描场景接触；`classic` 和 `none` 也可加载这个物理场景。另有 `--photoreal-scene` 用于在平地和演示盒上预览外观。`collision_mesh` 省略时和 `depth_mesh` 共用同一网格。

要用 `0.25 m/s` 回放这个示例，可先用 [`simplify_collision`](../docs/api_reference.md#sirescene_importsimplify_collision) 将原始网格简化到约 15 万面，再在 JSON 中设置 `"collision_mesh": "scene56.collision.ply"`。`depth_mesh` 继续使用原始扫描。

具备 CUDA/OpenGL 互操作的环境通过原生 SDK 在显存中传递图像，终端显示 `splat_transfer=cuda-gl`。通过主机缓冲的环境显示 `splat_transfer=host`。批量离屏使用 [`BatchCamera`](../docs/api_reference.md#siresensorbatchcamera)，直接采集 CUDA 张量。

## MuJoCo 平地与手柄对照

在 Sire 仓库根目录：

```bash
bash sim2sim/run.sh --policy sim2sim/policies/exp7/model_550_jit.pt
```

也可直接 `bash sim2sim/run.sh`，默认路径在 `sim2sim/config.yaml`。脚本使用已经安装 MuJoCo 3.2.3、torch 2.3.1、pygame 2.6.1 的 `/home/lqf/miniconda3/envs/unitree-rl/bin/python`，无需激活环境或设置 PYTHONPATH。如换机器，设置 `SIM2SIM_PYTHON=/path/to/python`。

- 左摇杆上下：前进/后退，最大 ±0.5 m/s。
- 左摇杆左右：左右平移，最大 ±0.5 m/s。
- 右摇杆左右：机身 yaw 角速度，最大 ±1 rad/s；不会被 heading 控制覆盖。
- 手柄 A / 窗口键盘 R：重置；Start / Space：暂停/继续；Esc / 关闭窗口 / Ctrl+C：退出。
- 摇杆回中或手柄断开：速度指令归零，策略继续执行零指令；不会保留断开前的运动指令。
- 绿色箭头是平移速度指令，蓝色箭头是实测速度；终端每秒显示 `cmd=[vx,vy,wz]`、机身系实测速度、位置、航向。
- 不按训练的 20 秒时限自动 reset，便于持续转向测试。高度低于 0.12 m 或机身倾斜超过约 78°，会报 `[fall]` 并暂停，按 A/R 重新测试；不偷偷重置掩盖摔倒。

参考 `go2_rl_gym/deploy/deploy_mujoco/deploy_go2.py`：轴编号 `[1,0,3]`，符号 `[-1,-1,-1]`，死区 0.1。当前主机手柄由 SDL 识别为 Xbox 360 Controller（6 轴 / 11 按键）。如果换手柄映射不同：

```bash
bash sim2sim/run.sh --inspect-joystick
# 打印轴和按键，移动摇杆核对；Ctrl+C 退出。需要时修改 config.yaml 或：
bash sim2sim/run.sh --axes 1 0 3 --max-cmd 0.5 0.5 1.0
```

## 换 checkpoint：先导出 actor-only JIT，再运行

沿用仓库已有 `SireRLGym/scripts/export_policy_jit.py` 的 `ActorPolicy + torch.jit.script`，只导出 actor，不带 critic、优化器、训练状态或随机采样的 std：

```bash
# 举例换成 exp23 的 500 轮（也可用其它存在的 model_N.pt）
bash sim2sim/run.sh export logs/flat_go2/exp23/model_500.pt \
  --out sim2sim/policies/exp23/model_500_jit.pt
bash sim2sim/run.sh --policy sim2sim/policies/exp23/model_500_jit.pt
```

导出时读取 checkpoint 旁边的 `config.yaml`，可用 `--training-config` 指定其它位置。默认输出为 `sim2sim/policies/<checkpoint父目录>/<checkpoint名>_jit.pt`；并行实验多个路径都叫 exp1 时，请显式指定不同 `--out`，避免重名。脚本拒绝覆盖已有导出。

JIT 内嵌 `deployment.yaml`，保存具名关节顺序、默认角、PD 参数、观测缩放、20 ms 控制周期和源 checkpoint 校验值；旁边的同名 YAML 只是方便查看。运行时仅 `torch.jit.load()`，无需原始 checkpoint 或训练配置。也兼容旧导出 JIT，需用 `--deployment-config path/to/profile.yaml` 显式给出对应接口，不能随意套用不同策略的配置。

45 维观测顺序：机身角速度 3、重力投影 3、速度指令 3、默认角偏差 12、关节速度 12、上次 action 12。当前 Sire 策略顺序为四个 hip、四个 thigh、四个 calf；通过关节名字映射到 MuJoCo 的逐腿排列，不使用 YAML 字典排序。MuJoCo `qvel[:3]` 从世界系转机身系，`qvel[3:6]` 已是机身角速度，**不再旋转第二次**。历史 263 维 critic 的兼容顺序沿用原有 exporter/play 规则；不确定来源时用导出选项 `--joint-order sire|go2_rl_gym` 明确指定。

MuJoCo 默认 2 ms × 10 子步（沿用参考 deploy），策略仍是训练的 20 ms / 50 Hz，PD 每个物理子步重算。可用 `--sim-dt 0.005` 改为 MuJoCo 5 ms × 4，不影响 Sire 训练步长。

## 无头回归与数据记录

```bash
# 实时 60 秒，持续转向；退出码 0=正常结束，2=观察到摔倒。
bash sim2sim/run.sh --headless --no-joystick --cmd 0 0 0.5 --steps 3000

# 同一固定指令比较不同 JIT，CSV 写 commands/实测速度/位置/yaw/fall/reset。
bash sim2sim/run.sh --policy sim2sim/policies/exp24/model_1000_jit.pt \
  --no-joystick --cmd 0.5 0 0 --steps 1500 --log sim2sim/logs/exp24_forward.csv

OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  /home/lqf/miniconda3/envs/unitree-rl/bin/python -m unittest discover -s sim2sim -p 'test_*.py'
```

默认实时、单推理线程；后台还在做计时实验时，请勿用 `--no-realtime` 批量跑对比。CSV 使用新建模式，不覆盖历史测试。窗口测试用于人工比较，不等于已经评定各策略优劣，更不代表真机部署已验证。

## 本机验证记录（2026-09-06，exp24 / 1000）

- 导出并从磁盘重新加载 JIT：33 组零值/随机输入相对原 actor 最大输出误差 0；导出约 753 KiB，只有 actor。
- 7 项回归通过：关节/电机排列、观测分块、多圈 yaw 加倾斜时的角速度坐标系（与 MuJoCo `mj_objectVelocity` 对照）、手柄轴和死区、断连归零、控制周期/reset、仅 JIT 加载与重复导出保护。
- 已实际启动 MuJoCo 窗口并读取已接入手柄，100 控制步后正常关闭。轴中立值已验证；手柄逐个实体按键/摇杆的人工操作仍需你自行体验。
- 固定 `cmd=[0,0,0.5]`，实时单线程 3000 步 / 60 s，转约 2.35 圈，无摔倒、无 reset，机身高度 0.314–0.339 m。5 s 后平均实测角速度约 **0.244 rad/s**：没有飞起不代表角速度跟踪已达标。
- 初始 yaw=90°，`cmd=[0.5,0,0]`，1000 步 / 20 s，无摔倒、无 reset。最终位置约 `(3.08,6.43,0.321)`，yaw 从 90° 漂到约 54°：本入口是直接速度控制，未用 heading 控制掩盖策略的偏航漂移。
- 原始记录在 `sim2sim/logs/exp24_turn_smoke.csv`、`sim2sim/logs/exp24_forward_yaw90_smoke.csv`。这是接口/稳定性检查，不是各训练 policy 的正式排名。
