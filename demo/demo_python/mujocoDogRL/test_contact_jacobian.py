import numpy as np
import scipy.linalg
import mujoco
import numpy as np
import mujoco
import copy

def get_all_contact_jacobians(model, data, cons):
    """
    获取所有活跃接触的雅可比矩阵。
    返回:
        J_all: 堆叠后的整体雅可比 (3*ncon, nv)
        J_dict: 字典 {contact_index: J_i (3, nv)}
    """
    ncon = len(cons)

    nv = model.nv
    J_all = np.zeros((3 * ncon, nv), dtype=np.float64)
    J_dict = {}

    for i in range(ncon):
        con = cons[i]
        # con = data.contact[i]
        point = con["pos"].copy()
        body1 = model.geom_bodyid[con["geom1"]]
        body2 = model.geom_bodyid[con["geom2"]]
        # point[2] -= 0.109
        print(f"接触 {i}: 点 {point}, 物体1 (body {body1}), 物体2 (body {body2})")

        jacp1 = np.zeros((3, nv), dtype=np.float64)
        jacr1 = np.zeros((3, nv), dtype=np.float64)
        jacp2 = np.zeros((3, nv), dtype=np.float64)
        jacr2 = np.zeros((3, nv), dtype=np.float64)
        mujoco.mj_jac(model, data, jacp1, jacr1, point, body1)
        mujoco.mj_jac(model, data, jacp2, jacr2, point, body2)
        print("1", jacp2)

        J_i = jacp1 - jacp2   # 3 x nv
        J_all[3*i:3*i+3, :] = J_i
        J_dict[i] = J_i

    return J_all, J_dict


def get_contact_inertia_matrix_all(model, data, return_blocks=False, cons=None):
    """
    计算包含所有接触点的整体接触惯量矩阵 A = J_all * M⁻¹ * J_all^T。
    参数:
        return_blocks: 若为 True，同时返回每个接触点对应的 3x3 子矩阵块。
    返回:
        A_all: (3*ncon, 3*ncon) 的整体惯量矩阵
        blocks (可选): 列表 [A_11, A_22, ...]，每个子块 3x3
    """
    nv = model.nv
    # ncon = data.ncon
    # if ncon == 0:
    #     print("⚠️ 没有接触，无法计算接触惯量矩阵。")
    #     return None

    # 1. M⁻¹
    M_dense = np.zeros((nv, nv), dtype=np.float64)
    mujoco.mj_fullM(model, M_dense, data.qM)
    try:
        L = np.linalg.cholesky(M_dense)
        M_inv = scipy.linalg.cho_solve((L, True), np.eye(nv))
    except np.linalg.LinAlgError:
        print("⚠️ M 不正定，使用伪逆。")
        M_inv = np.linalg.pinv(M_dense)

    # 2. 所有接触的雅可比 J_all
    J_all, _ = get_all_contact_jacobians(model, data, cons)
    if J_all is None:
        return None

    # 3. 整体惯量 A_all
    A_all = J_all @ M_inv @ J_all.T

    if return_blocks:
        blocks = [A_all[3*i:3*i+3, 3*i:3*i+3] for i in range(len(cons))]
        return A_all, blocks
    else:
        return A_all

def compute_A_all_perturbation(model, data, epsilon=1e-4, cons=None):
    """
    通过数值扰动力估计所有接触点的整体接触惯量矩阵 A (3*ncon x 3*ncon)。
    使用 qfrc_applied 施加外力，避免接触约束干扰。
    """
    # if ncon is None:
        # ncon = data.ncon
    # if ncon == 0:
    #     raise RuntimeError("没有接触，无法估计。")
    
    # 预提取接触信息
    ncon = len(cons)
    contacts_info = []
    for i in range(ncon):
        # con = data.contact[i]
        con = cons[i]
        point = con["pos"].copy()
        body1 = model.geom_bodyid[con["geom1"]]
        body2 = model.geom_bodyid[con["geom2"]]
        contacts_info.append((point, body1, body2))
    
    # 1. 创建无接触约束的模型副本 (禁用所有几何体接触)
    model_nc = copy.deepcopy(model)
    model_nc.geom_contype[:] = 0
    model_nc.geom_conaffinity[:] = 0
    data_nc = mujoco.MjData(model_nc)
    
    # 2. 复制状态，冻结速度
    data_nc.qpos[:] = data.qpos[:]
    data_nc.qvel[:] = 0.0
    mujoco.mj_forward(model_nc, data_nc)   # 基线 (仅有重力、关节弹性等内力)
    baseline_qacc = data_nc.qacc.copy()
    
    # 保存基线状态，供每次扰动重置使用
    qpos0 = data_nc.qpos.copy()
    qvel0 = data_nc.qvel.copy()
    
    nv = model_nc.nv
    dim = 3 * ncon
    
    # 3. 整体平移雅可比 J_all (3*ncon, nv)
    J_all = np.zeros((dim, nv), dtype=np.float64)
    for i, (point, body1, body2) in enumerate(contacts_info):
        jacp1 = np.zeros((3, nv))
        jacr1 = np.zeros((3, nv))
        jacp2 = np.zeros((3, nv))
        jacr2 = np.zeros((3, nv))
        mujoco.mj_jac(model_nc, data_nc, jacp1, jacr1, point, body1)
        mujoco.mj_jac(model_nc, data_nc, jacp2, jacr2, point, body2)
        J_all[3*i:3*i+3, :] = jacp1 - jacp2
    
    # 4. 逐列扰动
    A_est = np.zeros((dim, dim))
    
    for k in range(dim):
        # 重置到基线状态
        data_nc.qpos[:] = qpos0
        data_nc.qvel[:] = qvel0
        data_nc.qfrc_applied[:] = 0.0
        
        # 构造试探接触力向量 f_k (dim,) 仅在 k 处非零
        f_pert_vec = np.zeros(dim)
        f_pert_vec[k] = epsilon
        
        # 将扰动力分解到各接触点，并累加到 qfrc_applied
        # 由于 mj_applyFT 要求二维数组 (3,1)，因此进行 reshape
        qfrc_temp = np.zeros((nv, 1), dtype=np.float64)  # 二维，用于累加
        for i, (point, body1, body2) in enumerate(contacts_info):
            f_i = f_pert_vec[3*i:3*i+3]  # (3,)
            if np.linalg.norm(f_i) > 1e-15:
                # 将一维力向量转换为 (3,1)
                force_2d = f_i.reshape(3, 1)
                torque_2d = np.zeros((3, 1))
                point_2d = point.reshape(3, 1)
                # 对 body1 施加 +f_i
                mujoco.mj_applyFT(model_nc, data_nc, force_2d, torque_2d, point_2d, body1, qfrc_temp)
                # 对 body2 施加 -f_i
                mujoco.mj_applyFT(model_nc, data_nc, -force_2d, torque_2d, point_2d, body2, qfrc_temp)
        
        # 将累加好的广义力写入 data_nc.qfrc_applied
        data_nc.qfrc_applied[:] = qfrc_temp.flatten()
        
        # 正向动力学
        mujoco.mj_forward(model_nc, data_nc)
        delta_qacc = data_nc.qacc - baseline_qacc
        
        # 接触点相对加速度增量 a_c = J_all * delta_qacc
        a_c = J_all @ delta_qacc
        
        # 填入 A 的第 k 列
        A_est[:, k] = a_c / epsilon
    
    return A_est

def group_by_diagonal(A):
    """
    将对称矩阵 A 的行和列重新排序，使相同的对角线元素聚集在一起。
    返回重排后的矩阵，以及重排后的对角线值列表。
    """
    # 1. 获取对角线值及对应的原始索引
    diag_vals = np.diag(A)
    # 2. 找出所有唯一值及其出现的索引
    unique_vals = np.unique(diag_vals)
    # 3. 按值的大小（或任意顺序）构造新顺序：相同值的索引放在一起
    new_order = []
    for val in unique_vals:
        indices = np.where(diag_vals == val)[0]
        new_order.extend(indices.tolist())
    new_order = np.array(new_order)[::-1]
    # 4. 同时重排行和列
    B = A[np.ix_(new_order, new_order)]
    return B, diag_vals[new_order]

# 示例用法
if __name__ == "__main__":
    # 加载一个简单的模型，例如 MuJoCo 自带的 humanoid
    model = mujoco.MjModel.from_xml_path("scene.xml")  # 请确保路径正确
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)  # 确保 qM 更新
    # for i in range(data.ncon):
    #     con = data.contact[i]
    #     point = con.pos.copy()
    #     body1 = model.geom_bodyid[con.geom1]
    #     body2 = model.geom_bodyid[con.geom2]
    #     print(f"接触 {i}: 点 {point}, geom {con.geom1} (body {body1}), geom {con.geom2} (body {body2})")
    # exit()
    qM_dense = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, qM_dense, data.qM)
    np.set_printoptions(precision=8, linewidth=np.inf, suppress=True)
    print("M 矩阵:")
    print(qM_dense)
    # perm = [12,13,14,15,16,17,0,1,2,3,4,5,6,7,8,9,10,11]
    perm = [6,9,12,15, 7,10,13,16, 8,11,14,17, 0,1,2,3,4,5]
    P = np.eye(18)[perm]
    A_perm = P @ qM_dense @ P.T
    print(A_perm)
    print(scipy.linalg.inv(A_perm))

    A_sire = [
    [0.047680, 0.000000, 0.000000, 0.000000, 0.000259, 0.000000, 0.000000, 0.000000, -0.000144, 0.000000, 0.000000, 0.000000, -0.000000, 0.123281, 0.108541, -0.002133, -0.020729, 0.023958],
    [0.000000, 0.047680, 0.000000, 0.000000, 0.000000, -0.000259, 0.000000, 0.000000, 0.000000, 0.000144, 0.000000, 0.000000, -0.000000, 0.123281, -0.108541, -0.002133, 0.020729, 0.023958],
    [0.000000, 0.000000, 0.047680, 0.000000, 0.000000, 0.000000, 0.000259, 0.000000, 0.000000, 0.000000, -0.000144, 0.000000, -0.000000, 0.123281, 0.108541, -0.002133, 0.021246, -0.023729],
    [0.000000, 0.000000, 0.000000, 0.047680, 0.000000, 0.000000, 0.000000, -0.000259, 0.000000, 0.000000, 0.000000, 0.000144, -0.000000, 0.123281, -0.108541, -0.002133, -0.021246, -0.023729],
    [0.000259, 0.000000, 0.000000, 0.000000, 0.038864, 0.000000, 0.000000, 0.000000, 0.013596, 0.000000, 0.000000, 0.000000, -0.123210, 0.000000, 0.002789, 0.000388, -0.016504, 0.017423],
    [0.000000, -0.000259, 0.000000, 0.000000, 0.000000, 0.038864, 0.000000, 0.000000, 0.000000, 0.013596, 0.000000, 0.000000, -0.123210, 0.000000, 0.002789, -0.000388, -0.016504, -0.017423],
    [0.000000, 0.000000, 0.000259, 0.000000, 0.000000, 0.000000, 0.038864, 0.000000, 0.000000, 0.000000, 0.013596, 0.000000, -0.123210, 0.000000, 0.002789, 0.000388, -0.015425, 0.017423],
    [0.000000, 0.000000, 0.000000, -0.000259, 0.000000, 0.000000, 0.000000, 0.038864, 0.000000, 0.000000, 0.000000, 0.013596, -0.123210, 0.000000, 0.002789, -0.000388, -0.015425, -0.017423],
    [-0.000144, 0.000000, 0.000000, 0.000000, 0.013596, 0.000000, 0.000000, 0.000000, 0.006326, 0.000000, 0.000000, 0.000000, -0.034131, 0.000000, -0.001520, -0.000215, -0.001298, 0.004838],
    [0.000000, 0.000144, 0.000000, 0.000000, 0.000000, 0.013596, 0.000000, 0.000000, 0.000000, 0.006326, 0.000000, 0.000000, -0.034131, 0.000000, -0.001520, 0.000215, -0.001298, -0.004838],
    [0.000000, 0.000000, -0.000144, 0.000000, 0.000000, 0.000000, 0.013596, 0.000000, 0.000000, 0.000000, 0.006326, 0.000000, -0.034131, 0.000000, -0.001520, -0.000215, -0.001886, 0.004838],
    [0.000000, 0.000000, 0.000000, 0.000144, 0.000000, 0.000000, 0.000000, 0.013596, 0.000000, 0.000000, 0.000000, 0.006326, -0.034131, 0.000000, -0.001520, 0.000215, -0.001886, -0.004838],
    [-0.000000, -0.000000, -0.000000, -0.000000, -0.123210, -0.123210, -0.123210, -0.123210, -0.034131, -0.034131, -0.034131, -0.034131, 15.206408, 0.000000, 0.000000, 0.000000, 6.236590, 0.000000],
    [0.123281, 0.123281, 0.123281, 0.123281, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 15.206408, 0.000000, -6.236590, 0.000000, 0.134960],
    [0.108541, -0.108541, 0.108541, -0.108541, 0.002789, 0.002789, 0.002789, 0.002789, -0.001520, -0.001520, -0.001520, -0.001520, 0.000000, 0.000000, 15.206408, 0.000000, -0.134960, 0.000000],
    [-0.002133, -0.002133, -0.002133, -0.002133, 0.000388, -0.000388, 0.000388, -0.000388, -0.000215, 0.000215, -0.000215, 0.000215, 0.000000, -6.236590, 0.000000, 2.813007, 0.000000, -0.058815],
    [-0.020729, 0.020729, 0.021246, -0.021246, -0.016504, -0.016504, -0.015425, -0.015425, -0.001298, -0.001298, -0.001886, -0.001886, 6.236590, 0.000000, -0.134960, 0.000000, 3.103988, 0.000000],
    [0.023958, 0.023958, -0.023729, -0.023729, 0.017423, -0.017423, 0.017423, -0.017423, 0.004838, -0.004838, 0.004838, -0.004838, 0.000000, 0.134960, 0.000000, -0.058815, 0.000000, 0.512906]
    ]
    A_sire = np.array(A_sire)
    print(scipy.linalg.inv(A_sire))
    cons = [{"geom1": 0, "geom2": 21, "pos": np.array([0.1914, 0.142, -0.109])}, 
            {"geom1": 0, "geom2": 34, "pos": np.array([0.1914, -0.142, -0.109])},
            {"geom1": 0, "geom2": 47, "pos": np.array([-0.1954, 0.142, -0.109])}, 
            {"geom1": 0, "geom2": 60, "pos": np.array([-0.1954, -0.142, -0.109])}]
    J_all, _ = get_all_contact_jacobians(model, data, cons)
    print(J_all)
    J_all_perm = J_all @ P.T
    print(J_all_perm)
    # print(np.allclose(A_perm, B, atol=1e-5))  # 应该近似为 True，除了那两个对角元素
    # print(group_by_diagonal(qM_dense[6:, 6:]))

    # # 运行几个仿真步，让系统产生一些接触
    # for _ in range(200):
    #     mujoco.mj_step(model, data)

    # 计算接触惯量矩阵 A
    # 执行计算
    # 解析解
    A_ana = get_contact_inertia_matrix_all(model, data, return_blocks=False, cons=cons)
    # 数值解
    A_num = compute_A_all_perturbation(model, data, epsilon=1e-5, cons=cons)

    print(A_ana)
    print(A_num)
    # if A.size > 0:
    #     print(f"\n接触惯量矩阵 A (维度: {A.shape}) 计算完成。")
    #     # 打印前两行的部分结果，作为示例
    #     print("A 矩阵的前 2x2 块:")
    #     print(A[:2, :2])
    # else:
    #     print("A 矩阵是空的。")