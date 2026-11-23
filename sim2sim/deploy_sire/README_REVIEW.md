# deploy_sire：exp7 / 550 真机部署审查

目标目录是 **`/home/unitree/deploy_sire`，与 `/home/unitree/deploy_code` 并列**。
本目录参考工控机现有 `deploy_code/deploy_go2_moe_cts.py`，没有修改原目录、SDK、网口、服务或机器人运行模式。

## 先审查，再由操作者启动

本次只传输文件、检查源码、在工控机进行离线 TorchScript 推理。
**没有执行 `deploy_go2` / `deploy_go2.py`，没有创建 DDS 通道、调用运动 API 或发送电机命令。离线通过不等于真机已经验证安全。**

只做离线检查（无 SDK/DDS 导入，Controller 构造函数不执行）：

```bash
cd /home/unitree/deploy_sire
python3 offline_check.py
```

审查完成、按你们已有真机测试规程做好支撑、清场和急停准备后，才执行：

```bash
cd /home/unitree/deploy_sire
python3 deploy_go2 eth0
```

也支持 `python3 deploy_go2.py eth0`；无后缀文件只是同一入口的薄封装。
**命令一启动就会进入原代码的连接及运控释放流程，不是按 A 后才释放原生运控。**

原流程完整保留：配置读取 → `ChannelFactoryInitialize(0, eth0)` → 等待 LowState →
`SportClient` / `MotionSwitcherClient` 初始化 → 原有 `CheckMode / StandDown / ReleaseMode` 循环 →
零力矩等待 Start → 2 秒插值到默认姿态 → 保持姿态等待 A → 策略循环 → Select 或策略循环内 Ctrl+C → 原有阻尼命令一次发送。

## 本次差异，仅策略接口及明确批准的参数

- 策略：`policies/exp7_model_550_jit.pt`，只含 actor，45 维输入 / 12 维输出。不是 MoE，不使用 critic 或优化器。
- 策略 SHA256：`385f657aaae6040389e54a05e0ba78f38a6d52222f39f5babd7b611122791e75`。
- 关节顺序：FL/FR/RL/RR 四个 hip，然后四个 thigh，再四个 calf；**观测读取和动作发送共用同一个映射**。
- 对应实际电机编号：`[3,0,9,6, 4,1,10,7, 5,2,11,8]`。与旧 MoE 的逐腿排列不同，不能只换策略文件而沿用旧映射。
- 默认角：`[0.1,-0.1,0.1,-0.1, 0.8,0.8,1.0,1.0, -1.5,-1.5,-1.5,-1.5]`，物理姿态与参考相同，仅排列不同。
- 观测：机身 IMU 陀螺仪 ×0.25；原重力投影函数（wxyz）；command ×`[2,2,0.25]`；关节角偏差 ×1；关节速度 ×0.05；上次 action。未引入世界系角速度或 heading 控制。
- 手柄解码及符号不变：`vx=ly, vy=-lx, wz=-rx`。**物理 command 上限为 `[1,1,1]`**，输入限幅到 [-1,1]。物理上限与观测 `command_scale` 是两个参数。
- action_scale=0.25；观测/action 限幅 ±100，与选定 checkpoint 的训练配置一致。这不是新增的真机安全关节限位。
- 策略段 PD 按用户确认改为 **25 / 0.6**；原站起和保持段仍为 **40 / 0.6**。
- 原 `control_dt=0.02` 及 `time.sleep(control_dt)` 调度不变，不改变线程、通信频率或状态机。
- 配置在 DDS 初始化**之前**检查 JIT 校验值、内嵌接口、具名电机映射、PD、控制周期和缩放；配置不匹配时在连接前退出。只读验证，不触发硬件操作。

## 明确保留的原行为与风险（务必审查）

1. `common/command_helper.py`、`remote_controller.py`、`rotation_helper.py` 与原件逐字节一致，按用户要求没有更改。
2. **已核对的字段差异未修复**：原 `create_zero_cmd/create_damping_cmd` 写入 `.qd`，但工控机已安装 Go2 `MotorCmd_` 的目标速度字段是 `.dq`，不含 `.qd`。因此不能据“函数调用成功”认定目标速度字段已被清零。原正常策略循环会写 `.dq=0`，但这个事实不证明其它阶段或异常路径也正确。用户要求保留原函数，此问题列为审查项，不宣称已验证阻尼退出安全。
3. 沿用原异常处理：Select 和 Ctrl+C 的退出捕获位于策略循环，未扩展到等待 Start、站起、等待 A 阶段；其它异常没有新增全流程 finally 阻尼。没有新增断流看门狗、急停逻辑或额外动作限位。请按原代码的真实覆盖范围审查，不要把未实现的保护当作已具备。
4. 我没有现场验证 IMU 实际姿态、真实手柄、电机方向、机器人固件保护、力矩限制、原生运控释放、网络断流与实体急停。本目录的离线检查无法代替这些步骤。

字段和电机编号的核对依据：工控机实际安装的
`/home/unitree/unitree_sdk2_python/unitree_sdk2py/idl/unitree_go/msg/dds_/_MotorCmd_.py` 和
`example/go2/low_level/unitree_legged_const.py`；官方版本亦定义同样的
[MotorCmd 字段](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/idl/unitree_go/msg/dds_/_MotorCmd_.py)及
[电机编号](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/go2/low_level/unitree_legged_const.py)。

## 怎样核对“其它不改”

```bash
diff -u /home/unitree/deploy_code/deploy_go2_moe_cts.py /home/unitree/deploy_sire/deploy_go2.py
cmp /home/unitree/deploy_code/common/command_helper.py /home/unitree/deploy_sire/common/command_helper.py
cmp /home/unitree/deploy_code/common/remote_controller.py /home/unitree/deploy_sire/common/remote_controller.py
cmp /home/unitree/deploy_code/common/rotation_helper.py /home/unitree/deploy_sire/common/rotation_helper.py
```

`offline_check.py` 还会校验原源码 SHA256、10 项连接/启停代码一致性、模拟无线手柄包、各航向 IMU/重力观测、关节输入输出、PD、上限、跨 PyTorch 版本 JIT 推理一致性。
它用 AST 仅提取类和函数定义，**不导入部署主模块、不执行 Controller.__init__，发送函数只写入本地 Python 列表**。
结果在 `validation/local_report.json` 与 `validation/robot_report.json`，明确区分离线通过和遗留风险。

以后换策略需要一起检查配置和 checksum，不能只替换同名 JIT 绕过接口验证。本地可审查副本在 Sire 仓库 `sim2sim/deploy_sire/`，整个 `sim2sim/` 不被 Git 跟踪。
