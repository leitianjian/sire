"""
Clarabel debug script: 复现 C++ cptContactForceWithTargetState2 中的求解逻辑，
方便在 Python 中验证 C++ 计算结果。

用法：
    python clarabel_debug.py                          # 使用内置测试数据运行
    python clarabel_debug.py --load result.json       # 从 C++ 导出的 JSON 加载数据
"""

import argparse
import json
import sys
from typing import Optional

import clarabel
import numpy as np
import scipy.sparse as sparse


def solve_contact_forces(
    n: int,
    fri_coef: np.ndarray,      # [n]  摩擦系数 μ
    invM: np.ndarray,           # [3n, 3n]  逆惯量矩阵（已做 A-B 相减）
    v0: np.ndarray,             # [3n]  接触点初始速度
    v_target: np.ndarray,       # [n]   目标法向速度
    b: np.ndarray,              # [3n]  外力加速度项
    h: float,                   # 时间步长
    max_iters: int = 30,
    max_err: float = 1e-2,
    verbose: bool = True,
) -> tuple[np.ndarray, np.ndarray, int, float]:
    """
    完全复现 C++ cptContactForceWithTargetState2 的逻辑。

    Returns:
        contactFce: [3n]  接触力 (ft_x, ft_y, fn) 交替排列
        error:     最终收敛误差
        iters:     外循环迭代次数
    """

    # ---- Step 1: 计算 P 和 vFree ----
    W = invM.reshape(3 * n, 3 * n)
    P = -h * 0.5 * (W + W.T)                    # P = -h * (W + W^T) / 2
    vFree = v0 - h * b

    if verbose:
        print(f"=== Problem: n={n}, h={h:.6e} ===")
        print(f"P shape={P.shape}, vFree shape={vFree.shape}")
        print(f"fri_coef: {fri_coef}")
        print(f"v_target: {v_target}")
        print(f"vFree: {vFree}")

    # ---- Step 2: 拆分法向/切向索引 ----
    idx_n = [3 * i + 2 for i in range(n)]        # 法向 (Z)
    idx_t = []
    for i in range(n):
        idx_t.append(3 * i + 0)                   # 切向 X
        idx_t.append(3 * i + 1)                   # 切向 Y
    idx_n = np.array(idx_n, dtype=int)
    idx_t = np.array(idx_t, dtype=int)

    dim_n = len(idx_n)   # = n
    dim_t = len(idx_t)   # = 2n

    # ---- Step 3: 提取子矩阵 ----
    P_nn = P[np.ix_(idx_n, idx_n)]               # [n, n]
    P_nt = P[np.ix_(idx_n, idx_t)]               # [n, 2n]
    P_tt = P[np.ix_(idx_t, idx_t)]               # [2n, 2n]
    q_n = vFree[idx_n]                            # [n]
    q_t = vFree[idx_t]                            # [2n]

    if verbose:
        print(f"P_nn:\n{P_nn}")
        print(f"P_nt:\n{P_nt}")
        print(f"P_tt (first 4x4):\n{P_tt[:min(4, dim_t), :min(4, dim_t)]}")
        print(f"q_n: {q_n}")
        print(f"q_t: {q_t}")

    # ---- Step 4: Tikhonov 正则化求解 P_nn * x = rhs（秩亏稳定） ----
    # 与 C++ 中注释掉的 LDLT 正则化方案一致：
    #   const double reg_eps = 1e-8 * P_nn.diagonal().cwiseAbs().maxCoeff();
    #   Eigen::MatrixXd P_nn_reg = P_nn + I * reg_eps;
    #   Eigen::LDLT<Eigen::MatrixXd> P_nn_ldlt(P_nn_reg);
    reg_eps = 1e-8 * np.max(np.abs(np.diag(P_nn)))
    P_nn_reg = P_nn + np.eye(dim_n) * reg_eps
    if verbose:
        print(f"P_nn regularization: reg_eps={reg_eps:.2e}")

    def solve_pnn(rhs: np.ndarray) -> np.ndarray:
        """解 P_nn * x = rhs，等价于正则化后的 LDLT solve"""
        return np.linalg.solve(P_nn_reg, rhs)

    # ---- Step 5: 构建 Clarabel 问题（稀疏形式） ----
    # 变量 x = ft [2n], 锥变量 s [3n]
    # minimize  0.5 x^T P_tt x + q^T x
    # s.t.      A x + s = b,  s ∈ K
    #
    # K = ∏_{i=0}^{n-1} SOC(3)
    # A: [3n, 2n] 稀疏矩阵
    #   对每个接触点 i:
    #     A[3i+1, 2i+0] = -1.0
    #     A[3i+2, 2i+1] = -1.0
    #   其余为 0

    # 构建稀疏 A 矩阵 (CSC 格式，使用 scipy.sparse)
    A_rows = []
    A_cols = []
    A_vals = []
    for i in range(n):
        A_rows.append(3 * i + 1); A_cols.append(2 * i + 0); A_vals.append(-1.0)
        A_rows.append(3 * i + 2); A_cols.append(2 * i + 1); A_vals.append(-1.0)
    A_sparse = sparse.csc_matrix(
        (A_vals, (A_rows, A_cols)), shape=(3 * n, 2 * n)
    )

    # 锥: n 个 SOC(3)
    cones = [clarabel.SecondOrderConeT(3) for _ in range(n)]

    # P_tt 上三角（CSC）
    P_tt_sparse = sparse.csc_matrix(np.triu(P_tt))

    # Clarabel 设置
    settings = clarabel.DefaultSettings()
    settings.verbose = False
    # settings.max_iter = 30
    # settings.tol_gap_abs = 1e-4
    # settings.tol_feas = 1e-4

    # ---- Step 6: 外循环（不动点迭代） ----
    fn_val = np.zeros(n)
    ft_val = np.zeros(2 * n)
    error = -1.0
    iters_used = 0

    for iter in range(1, max_iters + 1):
        iters_used = iter

        # (a) 计算新的法向力
        rhs = -v_target - q_n - P_nt @ ft_val
        if verbose:
            print(f"\n--- iter {iter} ---")
            print(f"  rhs = {rhs}")

        fn_new = solve_pnn(rhs)
        if verbose:
            print(f"  fn_new = {fn_new}")
        fn_new = np.maximum(fn_new, 0.0)         # 非负约束
        fn_val = fn_new

        if verbose:
            print(f"  fn_new = {fn_val}")

        # (e) 更新切向 QP 的线性项
        q_tt_current = P_nt.T @ fn_val + q_t

        # (f) 更新锥约束右端 b
        b_eigen = np.zeros(3 * n)
        for i in range(n):
            b_eigen[3 * i] = fri_coef[i] * fn_val[i]

        if verbose:
            print(f"  q_tt_current = {q_tt_current}")
            print(f"  b_eigen = {b_eigen}")

        # (g) 求解 Clarabel
        solver = clarabel.DefaultSolver(
            P_tt_sparse, q_tt_current, A_sparse, b_eigen, cones, settings
        )
        solution = solver.solve()

        if verbose:
            print(f"  clarabel status: {solution.status}")
            print(f"  clarabel iters: {solution.iterations}")
            print(f"  clarabel solve_time: {solution.solve_time * 1e3:.3f} ms")

        if solution.status not in (clarabel.SolverStatus.Solved,
                                    clarabel.SolverStatus.AlmostSolved):
            print(f"  WARNING: Clarabel status = {solution.status}")

        ft_new = np.array(solution.x[:2 * n], dtype=np.float64)  # 前 2n 个是切向力

        # (h) 收敛判定
        error = np.linalg.norm(ft_new - ft_val)
        if verbose:
            print(f"  ft_new = {ft_new}")
            print(f"  error = {error:.6e}")

        if error < max_err:
            ft_val = ft_new
            break

        ft_val = ft_new

    # ---- Step 7: 组装最终接触力 ----
    contactFce = np.zeros(3 * n)
    for i in range(n):
        contactFce[3 * i + 0] = ft_val[2 * i + 0]     # ft_x
        contactFce[3 * i + 1] = ft_val[2 * i + 1]     # ft_y
        contactFce[3 * i + 2] = fn_val[i]              # fn

    if verbose:
        print(f"\n=== Final ===")
        print(f"  contactFce = {contactFce}")
        print(f"  error = {error:.6e}, iters = {iters_used}")

    return contactFce, error, iters_used


# ---------------------------------------------------------------------------
# 测试数据（模拟 C++ 中的典型场景）
# ---------------------------------------------------------------------------
def make_test_data(n: int = 2, seed: int = 42) -> dict:
    """生成合理的合成测试数据。"""
    rng = np.random.default_rng(seed)

    # 质量矩阵应当对称正定；C++ 中 invM 已经是负的逆惯量矩阵
    M = rng.normal(size=(3 * n, 3 * n))
    M = M @ M.T + np.eye(3 * n) * 1e-3
    invM = -np.linalg.inv(M).ravel()             # 负的逆惯量矩阵（与 C++ 一致）

    fri_coef = np.full(n, 0.3)                # μ = 0.3
    v0 = rng.uniform(-0.1, 0.1, size=3 * n)    # 初始接触速度（含穿透速度）
    v0[2::3] = -rng.uniform(0.01, 0.5, size=n) # 法向速度为负（正在穿透）
    v_target = -0.01 * np.ones(n)               # 目标法向速度（轻微分离或零）
    b = np.zeros(3 * n)                         # 外力项
    h = 0.001                                    # 1ms 时间步

    return {
        "n": n,
        "fri_coef": fri_coef,
        "invM": invM,
        "v0": v0,
        "v_target": v_target,
        "b": b,
        "h": h,
    }


def run_with_data(data: dict, verbose: bool = True):
    """使用给定数据运行求解器。"""
    contactFce, error, iters = solve_contact_forces(
        n=data["n"],
        fri_coef=data["fri_coef"],
        invM=data["invM"],
        v0=data["v0"],
        v_target=data["v_target"],
        b=data["b"],
        h=data["h"],
        max_iters=30,
        max_err=1e-2,
        verbose=verbose,
    )
    return contactFce, error, iters


# ---------------------------------------------------------------------------
# 从 C++ 导出的 JSON 加载数据
# ---------------------------------------------------------------------------
def load_from_cpp_json(filepath: str) -> Optional[dict]:
    """从 C++ 导出的 JSON 文件加载问题数据。

    期望 JSON 格式:
    {
        "n": 2,
        "fri_coef": [0.3, 0.3],
        "invM": [...],      // 扁平化的 3n x 3n 矩阵，行主序
        "v0": [...],
        "v_target": [...],
        "b": [...],
        "h": 0.001,
        "cResult": [...]    // (可选) C++ 计算结果，用于对比
    }
    """
    with open(filepath, "r") as f:
        j = json.load(f)

    required = ["n", "fri_coef", "invM", "v0", "v_target", "b", "h"]
    for k in required:
        if k not in j:
            print(f"JSON 缺少字段: {k}")
            return None

    data = {
        "n": int(j["n"]),
        "fri_coef": np.array(j["fri_coef"], dtype=np.float64),
        "invM": np.array(j["invM"], dtype=np.float64),
        "v0": np.array(j["v0"], dtype=np.float64),
        "v_target": np.array(j["v_target"], dtype=np.float64),
        "b": np.array(j["b"], dtype=np.float64),
        "h": float(j["h"]),
    }

    if "cResult" in j:
        data["cpp_result"] = np.array(j["cResult"], dtype=np.float64)

    return data


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Clarabel 接触力求解器 — Python 复现 C++ 逻辑"
    )
    parser.add_argument(
        "--load", type=str, default=None,
        help="从 C++ 导出的 JSON 文件加载数据"
    )
    parser.add_argument(
        "-n", type=int, default=2,
        help="接触点数量（使用内置测试数据时）"
    )
    parser.add_argument(
        "--seed", type=int, default=42,
        help="随机种子（使用内置测试数据时）"
    )
    parser.add_argument(
        "--quiet", action="store_true",
        help="减少输出"
    )
    args = parser.parse_args()

    verbose = not args.quiet

    if args.load:
        data = load_from_cpp_json(args.load)
        if data is None:
            sys.exit(1)
    else:
        data = make_test_data(n=args.n, seed=args.seed)

    contactFce, error, iters = run_with_data(data, verbose=verbose)

    # 如果提供了 C++ 结果，做对比
    if "cpp_result" in data:
        cpp = data["cpp_result"]
        diff = contactFce - cpp
        print(f"\n=== 与 C++ 结果对比 ===")
        print(f"  Python: {contactFce}")
        print(f"  C++:    {cpp}")
        print(f"  diff:   {diff}")
        print(f"  ||diff|| = {np.linalg.norm(diff):.6e}")

    return contactFce, error, iters


if __name__ == "__main__":
    main()
