# 单接触点回归模式

`PsVsSolver3` 和 `PsVsSolverV5`（包括继承 V5 的求解器）支持两种模式：

当前切向实验新增 `ShiftedSpectralAdmmContactSolver`，默认选择 `single_point`。
原 V3、V5 和未偏移的 Spectral ADMM 默认值不变。

| 配置值 | 穿深处理 | 本步时间 |
| --- | --- | --- |
| `height_field`（原求解器默认） | 保留当前多接触点前处理和 `1e-6` 截断 | 建议步长 |
| `single_point` | 按几何体对记录初始穿深，使用当前穿深与基准的差值 | 法向模型分离时间与建议步长的较小值 |

单接触点模式允许多个几何体对共同求解，但每个几何体对只能有一个接触点。
如果同一对返回多个点，会报出几何体 ID，避免多个点错误地共用穿深基准。
同一刚体上的不同几何体仍是不同的接触对。

## Python

切向研究默认使用 target-shifted Spectral ADMM：

```python
physicsEngine = sim.physicsEngine()
contactSolver = physicsEngine.addShiftedSpectralADMMSolver()
assert contactSolver.contactModelMode == "single_point"
# 继续配置材料、事件、模型，再初始化并运行仿真。
```

也可以在原 V5/ADMM 对象上设置同一个属性，或调用
`contactSolver.setContactModelMode("single_point")`。
**模式不改变切向求解算法**：原 ADMM v6 和未偏移的 Spectral ADMM 本来就不使用
DAE 法向目标速度；启用模式后恢复穿深前处理和接触时间，也不会改写这个约定。
V3、PIPG、Exact Coulomb 的现有目标速度接口继续接收本步法向模型结果。

`boxSliding.py` 已默认选择此求解器和单接触点模式，直接运行：

```powershell
python demo/demo_python/sire/boxSliding/boxSliding.py
```

未偏移的论文基线仍通过 `addSpectralADMMSolver()` 选择。若希望在相同的法向
前处理和接触时间下对比两种 ADMM，应也给论文基线设置
`contactSolver.contactModelMode = "single_point"`，并固定材料和初始条件。
V3 基线仍通过 `addPsVsSolver3()` 加 `contactModelMode = "single_point"` 使用。

## Shifted Spectral ADMM 的定义

求解使用冲量 `lambda = h * force`，令 `G = -sym(invM)`。
Sire 的 `v_target` 是穿透速度，期望法向相对速度为 `-v_target`。
因此自由速度按以下方式构造：

```text
g       = v0 - h*b
g_shift = g + E_N*v_target
w       = G*lambda + g_shift
0 <= lambda_N ⟂ w_N >= 0
```

`E_N` 仅把每个接触点的目标填入第 `3*i+2` 个分量。NCP 使用原有库仑锥与
De Saxce 修正。新旧版本共用同一个迭代核，唯一模型变化是上面的常量偏移；
谱 rho 更新、eta、锥投影、每轮 De Saxce 更新、三项停止残差、零初始化和
冲量到力的换算均一致。`v_target` 不乘除 `h`，不作为额外等式约束，也不添加
到 De Saxce 修正中。所有迭代变量均为本次调用的局部变量。

独立求解入口是 `sire.cptContactForceShiftedSpectralAdmm(...)`，参数和返回值
与 `cptContactForceSpectralAdmm(...)` 相同。`compare_solvers.py` 中对应名称为
`Shifted-Spectral-ADMM`，可与 `Spectral-ADMM` 对比。

## XML / C++

在原求解器元素上添加属性，例如：

```xml
<PsVsSolver3 __prop_name__="contact_solver" contact_model_mode="single_point">
    <!-- 原有 material_manager 配置 -->
</PsVsSolver3>
```

C++ 使用 `solver.setContactModelMode("single_point")`。
新求解器 XML 元素名为 `ShiftedSpectralAdmmContactSolver`，省略
`contact_model_mode` 时默认就是 `single_point`。
原 V3/V5 求解器不设置属性时继续使用原模式；新 shifted 求解器默认单接触点。
原 heightField 几何、碰撞回调和事件处理器保留；当前示例的事件映射
`{0:12, 1:13, 2:14}` 可直接使用，不必为了回归模式更换事件 ID。
V2 及更早的求解器不提供这个模式，仍保留它们原本的行为。

当前仿真入口是 `SimulationLoop::integrate()` 和 `handleContact()`。
对于示例使用的 Handler5，`integrate()` 分发到空的 handler 积分方法，
`handleContact()` 则分发到 `InitHandler5/StepHandler5/CtrlHandler5::handle()`，
完成碰撞检测并调用 `PhysicsEngine::integrateByContactInfo()`，最终进入
V3/V5 的 `cptContactSolverResult()`。穿深处理、接触时间求根、实际积分及
下一事件的创建均在这条调用链上完成，不依赖 `step()` 或 `stepPhysics()`。

## 状态和时间语义

- 新接触记录原始 `depth`，本步 `modifiedDepth` 为零。
- 持续接触使用 `depth - initDepth`；若差值为负，本步过滤，并降低穿深基准。
- 只在当前碰撞结果中存在的几何体对保留基准；空碰撞批次也会清理记录。
- `reset()`、`resetRL()`、`resetRLNoTimer()` 和模式切换清空基准。
- 每步速度从当前刚体模型重新计算，不从上一求解步继承 `x1t`、预测深度、
  预测速度，不补回已消失的接触点。本步仍计算 `x1t` 供目标速度使用。
- 求根使用完整增广矩阵指数，保留接触间耦合和常量外力项；扫描包括纯实特征值
  的情况，再对找到的穿深变号区间使用带区间保护的 Brent-Dekker 求根。
  标量函数取所有接触的最小穿深；没有找到分离根时使用建议步长。
- 扫描达到固定间隔后，在本次调用内只计算一次完整状态转移矩阵，随后用
  矩阵向量乘法推进。初始变间隔扫描、末尾不足固定间隔的扫描和求根均从
  当前区间左端状态局部推进；任何扫描分支发现负穿深时，都从本步初值直接
  复核，若复核不为负则用直接计算结果校正扫描状态。扫描点和区间宽度停止条件不变，
  状态转移矩阵及扫描状态不跨仿真步保存。浮点结果可能因递推舍入略有差异。
- 正的短接触时间不会因小于 `1e-6` 而被丢弃。实际时间统一用于目标计算、
  力求解、积分、计时和下一事件。

直接状态计算使用 `matrix_exp_multiply(n, A, v, result, t)`，计算
`exp(t*A)*v`。采用迹平移、基于矩阵 1-范数的缩放和 Taylor 阶数选择，
以双精度误差界控制截断；Taylor 路径只做矩阵向量乘法，不求逆、不构造
完整指数。工作量估计过大时回退到原 `matrix_exp_pade`；这是性能保护，
不放宽求根容差。保留完整增广矩阵的耦合和外力项，不要求矩阵可逆或可对角化。
原 `cptFormulaXComposeAb` 和固定扫描的状态转移矩阵仍用完整 Padé 指数。
同一次求根构造一个 `MatrixExpMultiplyWorkspace`，保存矩阵副本、迹平移、
范数和工作向量；其 `apply(v, result, t)` 不重复计算矩阵范数或分配 Taylor
工作数组。缩放子步和阶数仍随本次时间长度选择。Padé 回退使用独立矩阵缓冲区，
不覆盖准备好的矩阵，原独立函数接口仍可用。工作区不跨仿真步保存。
算法依据：[Al-Mohy 与 Higham (2011)](https://eprints.maths.manchester.ac.uk/1536/3/paper16.pdf)。

穿深基准是唯一用于跨步接触处理的物理记录。Recorder 和求解诊断日志仍可使用，
它们不参与下一步初值的构造。

## 验证

本次未配置、编译或运行 C++ 仿真/测试。新增 `single_point_contact_test` 源码覆盖：
新接触不取初始零点、欠阻尼分离、过阻尼分离、常量外力、不同接触对耦合、
短接触时间、持续接触、末尾不足固定扫描间隔时的分离、连续调用外力变化和
模式配置。启用 `BUILD_TEST` 后可自行编译并运行该目标。
同一测试目标也包含 shifted Spectral ADMM 用例：默认模式、零目标与基线一致、
偏移等价性、目标符号和冲量单位、滑动摩擦边界、分离无拉力及目标参数校验。

仿真回归建议检查单球落地/反弹、多个独立球脚、斜面滑动、分离后重新接触和
reset；另用原 heightField 配置检查默认模式。重点比较 `depth`、`modifiedDepth`、
实际 `dt`、模型时间和控制事件时间。

接触时间的 profile 位于 `ps_vs/contactEndTime/` 下：`fixedStepExponential`
记录固定间隔状态转移矩阵的构造（每次求根至多一次），`fixedStepAdvance`
记录后续矩阵向量推进；`scanAction` 记录变间隔扫描及末尾补齐的局部推进，
`exponentialAction` 只记录负穿深候选的初值复核。
`core/matrix_exp_multiply/prepare` 记录每次求根的一次工作区准备。
其内部 `core/matrix_exp_multiply/taylor` 和
`core/matrix_exp_multiply/padeFallback` 区分向量展开与完整指数回退。
`spectrum`、`scan` 和 `brent` 分别记录谱分析、扫描及求根；`scan` 包含
其内部的求根耗时。整体耗时见 `ps_vs/findSinglePointContactEndTime`。
求根中的 `rootAction` 从当前非负穿深的左端状态推进到试探点。
优先使用割线或逆二次插值，越界、进展不足或不适合插值时退回二分；
最多 80 轮，最后 48 轮只允许二分，保留原区间宽度容差。
零穿深不会直接终止求根：允许一次朝负穿深端的半容差探测，持续零值则退回二分。
返回负穿深侧的正时间端点，不跨仿真步缓存状态。

`contact_time.*` 曲线每次非空接触求根结束时记录：`n_contacts`、`suggest_dt`、
`max_rate`（最大特征值模）、`max_frequency`（最大虚部绝对值，rad/s）、
`max_scan_step`、`scan_evaluations`、`root_evaluations`、`interpolation_steps`
（包括一次零点探测）、`bisection_steps`、`confirmations` 和 `separation_found`。
无分离时求根次数也会记录为零，避免误读为上次调用的数据。

此优化主要减少已发现分离区间后的指数向量作用次数；没有分离的调用、谱分析、
固定转移矩阵及 Padé 回退本身不会加速。最小穿深的主导接触切换、平坦零值或
病态根会削弱插值收益，最差仍需二分并多付少量插值开销。
扫描仍不能保证检测到两个扫描点之间先负后正的极短分离区间。

时间求根以区间宽度为停止条件，不使用穿深残差阈值：
`max(32*epsilon*dt, min(1e-12, dt*1e-8))` 秒。
建议步长为 1 ms 或 2 ms 时，时间容差均为 1e-12 秒；它只约束数值求根
区间宽度，不代表物理接触时间具有该精度，也不控制扫描点之间的漏检。
Taylor 提前结束仍使用连续两项范数之和不超过
`0.5*epsilon*当前和向量的无穷范数`，另有原有阶数/缩放界；未放宽精度。

Taylor 内循环使用专用行主序矩阵向量计算，将项缩放、状态累加和范数检查
合并到每行输出，避免通用矩阵乘法调用及独立输出遍历。当前项向量在所有行
完成前保持不变，保留接触耦合和原址调用支持。时间缩放系数每个子步复用。
新增 `exp_action.*` profile 曲线：`time_interval` 为时间间隔绝对值，
`scaled_norm` 为时间乘迹平移后矩阵范数，`degree` 和 `scaling_steps` 为
所选阶数和预计缩放子步数，`matvecs` 为实际 Taylor 矩阵向量乘法次数，
`pade_fallback` 标识是否回退。回退时 `matvecs=0` 仅表示没有 Taylor 工作，
并非完整指数没有开销。零向量、零时间和空矩阵的提前返回不记录这些曲线。
启用 profile 时这些逐次记录也有开销；性能收益需要用重新编译后的场景确认。

`force_screw_test` 新增指数向量作用测试：零输入、原址输出、负时间、
Jordan 耦合、奇异增广外力、振荡缩放、大迹平移、Padé 回退和非有限值。
这些 C++ 测试尚未编译或运行。

## Padé 工作区与可选分段多项式扫描

`MatrixExpPadeWorkspace` 保留原六阶 Padé 系数和缩放规则，改为带部分选主元
的消元/回代，直接求解 `D * R = N` 的多个右端，不构造 `D` 的逆。
工作区复用缩放矩阵、幂次、分子、分母和临时矩阵，平方阶段同样复用缓冲区。
输入输出可以相同；零矩阵返回单位矩阵。原 `matrix_exp_pade` 函数仍可用，
内部创建临时工作区。指数作用的 Padé 回退持有可复用工作区，纯 Taylor 路径
不分配这些矩阵；固定扫描的完整转移矩阵使用本次求根的局部工作区。
新增 `core/matrix_exp_pade/solve` 和 `/square` 分别观察求解与平方成本。
该公共函数的其他调用者也使用新 Padé 实现，因此应一起做仿真回归。

V3/V5（包括 shifted ADMM）增加 `contact_time_method`：

- `exponential`：默认，原扫描与 Brent 路径，使用优化后的 Padé。
- `polynomial`：实验性分段 Taylor 轨迹扫描，困难计算回退到默认路径。

XML 在接触求解器元素上设置：

```xml
<ShiftedSpectralAdmmContactSolver __prop_name__="contact_solver"
    contact_model_mode="single_point" contact_time_method="polynomial">
    <!-- 保留原材料配置 -->
</ShiftedSpectralAdmmContactSolver>
```

Python 在初始化仿真前设置：

```python
contactSolver.setContactTimeMethod("polynomial")
# 回到基准：contactSolver.setContactTimeMethod("exponential")
```

boxSliding.py 已加入可取消注释的切换行。dog.py 使用的 go2_rai_foot.xml
显式设为 exponential，以便先比较 Padé 改动；改为 polynomial 后即可比较两种扫描。
height_field 分支不使用此接触时间选项。非法方法名抛出异常。

多项式模式对增广系统 `y'=B*y` 在局部区间生成系数，令 `u=(t-start)/h`：
`c0=y(start)`，`ck=(h/k)*B*c(k-1)`，通过 Horner 法求值。
一次优先准备最多四个最大扫描间隔的轨迹段，并先按矩阵范数限制段长；
失败后依次折半尝试中间长度，单扫描点最多四次尝试，最后一次使用当前扫描间隔。
同一次求根最多实际准备 16 次，缓存内的多项式求值不受此上限限制。最高 32 阶，使用首个省略项和矩阵无穷范数的几何上界
估计尾项，同时累计浮点舍入误差估计。`h*norm(B)>16` 时直接回退，避免在
明显不适合该低成本误差估计的段上构造长多项式。
穿深绝对值不足误差裕量时使用指数作用重新求值，不以不确定的非负值排除分离。
这些浮点误差估计不是区间算术证书，也没有消除局部状态传播的累计误差。

首版刻意保留原扫描点，不直接跳过整段，不以多项式根代替最终 Brent。
发现负穿深后仍从本步初值用指数作用复核，并丢弃多项式缓存；Brent 内部仍用
原指数作用。因此这版重点验证“多点复用轨迹”收益，尚未实现区间极小值检查，
仍有原端点扫描可能漏掉区间内短暂负穿深的限制。所有缓存都不跨仿真步。
强刚性、尺度不均衡或长期零穿深时可能频繁回退，速度可能比默认模式慢。

用 `contact_time.polynomial_hits`、`polynomial_fallbacks` 和
`polynomial_segments` 分别查看接受的多项式扫描次数、回退次数、段准备轮数
（每轮最多尝试四个区间），结合 `polynomialPrepare`、`polynomialEvaluate`
的 zone 耗时判断是否值得启用。默认模式这些计数为零。

本轮未编译或运行 C++ 测试。新增 Padé 零矩阵、Jordan、旋转、增广外力及工作区
复用测试，以及多项式模式的分离、持续接触、刚性、零穿深、短接触和配置测试。
独立 Python 数值原型通过 250 个 Padé 对照案例和 180 段多项式的三点检查；
这仅验证公式原型，不能替代 C++ 编译和实际场景回归。接触时间容差本轮未改变。

### 多项式重试成本控制

固定转移矩阵已经构造后，完整固定间隔直接使用矩阵向量推进，不再准备或求值
多项式；末尾不足固定间隔的扫描仍可尝试。矩阵范数限制在准备前检查，跳过
不可能通过的区间，并裁剪长段上限。累计舍入误差估计超过预算时立即终止展开。

一个扫描点准备或符号判断失败后，本次调用中不更短的后续区间直接回退。
这是避免重复成本的策略，不是“后续状态必然失败”的数学判断；更短的尾段在
总准备次数未耗尽时仍可重试。候选初值复核、Brent、原扫描点和容差不变。

新增 `contact_time.poly_*` 计数（只在 polynomial 模式的非空求根结束时发布）：
`prepare_calls` 为真实准备次数，最多 16；`norm_skips` 为范数限制跳过；
`error_failures`、`degree_failures`、`arithmetic_failures` 为准备/运算失败原因；
`uncertain_signs` 为穿深符号不确定；`retry_skips` 为重试门槛或总预算跳过；
`fixed_skips` 为固定转移矩阵优先使用次数。诊断为零时仍发布，切回 exponential
时不发布这组细分计数。`polynomial_fallbacks` 不包含主动优先使用固定矩阵的次数。
本轮补充刚性非活动模态与末尾分离的 C++ 回归用例，未编译或运行。
