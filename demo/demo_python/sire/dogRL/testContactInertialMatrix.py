import sire
import numpy as np

def shrink_contact_inertia(A_ext: np.ndarray) -> np.ndarray:
    """
    将扩展接触空间惯量矩阵 (6*n x 6*n) 缩并为相对接触空间惯量矩阵 (3*n x 3*n)
    
    假设每个接触的排列为：
        - 前 3 维：物体 1 的接触点加速度
        - 后 3 维：物体 2 的接触点加速度
    
    相对加速度 = 物体1 - 物体2
    对应的线性变换为 S = [I_3, -I_3] （对每个接触块）
    
    因此 A_rel = S * A_ext * S^T
    可通过块运算直接转换为：
        A_rel_{ij} = A11_{ij} - A12_{ij} - A21_{ij} + A22_{ij}
    其中 A11, A12, A21, A22 是原矩阵 6×6 子块的分块。
    
    参数:
        A_ext: ndarray, shape (6*n_contact, 6*n_contact)
    返回:
        A_rel: ndarray, shape (3*n_contact, 3*n_contact)
    """
    
    n = A_ext.shape[0] // 6
    A_rel = np.zeros((3 * n, 3 * n))
    
    for i in range(n):
        for j in range(n):
            # 提取第 (i, j) 个 6×6 子块
            block = A_ext[6*i:6*i+6, 6*j:6*j+6]
            
            # 分块：A11(3x3), A12(3x3), A21(3x3), A22(3x3)
            A11 = block[0:3, 0:3]
            A12 = block[0:3, 3:6]
            A21 = block[3:6, 0:3]
            A22 = block[3:6, 3:6]
            
            # 相对惯量子块
            A_rel[3*i:3*i+3, 3*j:3*j+3] = A11 - A12 - A21 + A22
    
    return A_rel

cs = sire.ControlServer.instance()
sire.fromXmlFile(cs, 'D:/code/sire/demo/demo_paper/sire/dog/go2_can_work.xml')
cs.init()
simulator = sire.simulator(cs)
model = cs.model()

prt1Arr = [0, 0, 0, 0]
prt2Arr = [4, 7, 10, 13]
partidArr = [0, 4, 0, 7, 0, 10, 0, 13]
contactPointArr = [0.1914, 0.142, -0.109, 0.1914, -0.142,  -0.109,
                   -0.1954, 0.142, -0.109, -0.1954, -0.142,  -0.109]
v = [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
for i in range(model.numMotions()):
  model.motionPool()[i].mv = v[i]

model.forwardKinematicsVel()

for i in range(model.numLinks()):
    print(model.partPool()[i].getVs())
# print(model.partPool()[4].getVs())
exit()
print("fuck")
eye1 = np.eye(4)
eye2 = np.eye(4)
eye3 = np.eye(4)
eye4 = np.eye(4)
vector = np.vstack([eye1, eye2, eye3, eye4]).ravel()
A_out = model.cptContactInverseInertiaMatrix(4, partidArr, vector, contactPointArr)
print("fuck", A_out)
print(shrink_contact_inertia(np.array(A_out).reshape(24, 24)))
np.set_printoptions(precision=8, linewidth=np.inf, suppress=True)
model.cptProjectedMassMatrix()
motionForce = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
for i in range(model.numForces()):
    fce = model.forcePool()[i]
    if isinstance(fce, sire.SingleComponentForce):
        fce.fce = motionForce[i]
model.forwardDynamics()
for i in range(model.numMotions()):
    model.motion(i).updA()
boxAs = model.partPool()[1].getAs()
print(model.partPool()[1].getAs())
ap = sire.as2ap([0, 0, 0, 0, 0, 0], boxAs, [0, 0, 0.445])
print(ap)