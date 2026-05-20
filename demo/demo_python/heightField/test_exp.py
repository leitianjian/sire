import numpy as np
from scipy.linalg import expm

def cpt_formula_x_compose_ab(A: np.ndarray, b: np.ndarray, t: float, x0: np.ndarray) -> np.ndarray:
    """
    仿射系统 dx/dt = A x + b 的闭式传播。
    参数：
        A : (n, n) 系统矩阵
        b : (n,)   常值偏移向量
        t : float  传播时间
        x0: (n,)   初始状态
    返回：
        x1: (n,)   x(t)
    """
    n = A.shape[0]
    # 构建增广矩阵 Ab = [[A, b], [0, 0]] 尺寸 (n+1, n+1)
    Ab = np.zeros((n + 1, n + 1))
    Ab[:n, :n] = A
    Ab[:n, n] = b
    # 其余元素保持 0，即最后一行全 0

    # 构造增广初始向量 [x0; 1]
    x01 = np.zeros(n + 1)
    x01[:n] = x0
    x01[n] = 1.0

    # 计算矩阵指数 exp(Ab * t)
    exp_Ab_t = expm(Ab * t)
    print(exp_Ab_t)
    # 传播
    x1t = exp_Ab_t @ x01

    # 取前 n 个元素作为结果
    return x1t[:n]

# ---------- 使用示例 ----------
if __name__ == "__main__":
    # 假设 n2 = 3，随机生成测试数据
    n2 = 2
    # np.random.seed(0)
    A = np.array([[0, 1e+10], [-0.02, -500000]])
    b = np.array([0, 9.81])
    x0 = np.array([490.5, 1.00209e-15])
    min_time = 0.001

    x1 = cpt_formula_x_compose_ab(A, b, min_time, x0)
    print("A =\n", A)
    print("b =", b)
    print("x0 =", x0)
    print(f"x({min_time}) =", x1)

    eAt = np.array([0.670643, 13423.6, 161.55, -2.68472e-08, -0.000537374, 1.31686e-05, 0, 0, 1]).reshape(3, 3)
    print(eAt, np.hstack([x0, 1]))
    print(eAt @ np.hstack([x0, 1]).T)
    import sire
    print(sire.s_mm(3, 1, 3, eAt.ravel(), np.hstack([x0, 1])))