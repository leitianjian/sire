import numpy as np
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf

def draw_arrow(vis, name, start, direction, length=0.1, headRatio=0.3, radius=0.02, scale=1, color=0xff0000):
    """
    Draw an arrow from `start` to `end` in MeshCat.
    
    Args:
        vis: MeshCat Visualizer instance
        name: Name of the arrow object
        start: Start point (3D vector)
        end: End point (3D vector)
        shaft_radius: Radius of the arrow shaft
        head_radius: Radius of the arrowhead base
        head_length: Length of the arrowhead
        color: RGB color of the arrow (e.g., 0xff0000 for red)
    """
    # 确保坐标是浮点数
    start = np.array(start, dtype=np.float64)
    direction = np.array(direction, dtype=np.float64)

    radius = radius * scale    
    direction /= np.linalg.norm(direction)
    length *= scale
    end = start + direction * length
    
    # 跳过零长度箭头
    if length < 1e-8:
        return
    
    # 创建箭头杆（圆柱体）
    shaft_length = length * (1 - headRatio)
    # 箭杆中点坐标
    shaft_pos = start + direction * (shaft_length / 2)
    shaft = g.Cylinder(shaft_length, radius)
    
    # 创建箭头头（使用圆柱体模拟圆锥体）
    # 通过设置顶部半径为0来创建圆锥体效果
    head = g.Cylinder(length * headRatio, radiusBottom=radius, radiusTop=0.0)
    
    # 箭头材质
    material = g.MeshBasicMaterial(color=color, wireframe=False)
    
    # 计算从z轴到方向向量的旋转矩阵
    rot_matrix = rotation_matrix_from_vectors([0, 1, 0], direction)
    
    # 杆的变换：将杆中心放在中点
    shaft_transform = tf.translation_matrix(shaft_pos)
    shaft_transform[:3, :3] = rot_matrix[:3, :3]
    
    # 头的变换：将头放在杆的末端
    # 注意：圆柱体默认沿y轴，需要旋转到z轴
    head_length = length * headRatio
    head_pos = end - direction * (head_length / 2)
    # head_initial_rot = tf.rotation_matrix(rot_matrix)
    head_transform = tf.translation_matrix(head_pos)
    head_transform[:3, :3] = rot_matrix[:3, :3]
    # head_full_transform = np.dot(head_transform, head_initial_rot)
    
    # 整体变换：移动到起点位置
    full_transform = tf.translation_matrix(start)
    
    # 将几何体添加到场景
    # vis[name].set_transform(full_transform)
    print("fuck", full_transform, shaft_transform, head_transform)
    
    # 分别设置几何体和变换
    vis[name]["shaft"].set_object(shaft, material)
    vis[name]["shaft"].set_transform(shaft_transform)
    
    vis[name]["head"].set_object(head, material)
    vis[name]["head"].set_transform(head_transform)

def rotation_matrix_from_vectors(a, b):
    """
    计算将向量 `a` 对齐到向量 `b` 的旋转矩阵。
    
    Args:
        a: 源向量 (3D)
        b: 目标向量 (3D)
        
    Returns:
        4x4 变换矩阵
    """
    a = np.array(a, dtype=np.float64) / np.linalg.norm(a)
    b = np.array(b, dtype=np.float64) / np.linalg.norm(b)
    
    if np.allclose(a, b):
        return np.eye(4)
    
    if np.allclose(a, -b):
        return tf.rotation_matrix(np.pi, [1, 0, 0])
    
    # 计算旋转轴和角度
    print(a, b)
    axis = np.cross(a, b)
    axis /= np.linalg.norm(axis)
    angle = np.arccos(np.dot(a, b))
    
    # 生成旋转矩阵
    return tf.rotation_matrix(angle, axis)
def s_pe_dot_pe(pe1, pe2):
  from scipy.spatial.transform import Rotation as R
  from scipy.spatial.transform import RigidTransform as TF
  tf0 = TF.from_components(
                translation=pe1[:3],
                rotation=R.from_euler("ZXZ", pe1[3:], degrees=False))
  tf1 = TF.from_components(
                translation=pe2[:3],
                rotation=R.from_euler("ZXZ", pe2[3:], degrees=False))
  tfResult = tf0 * tf1
  euler = tfResult.rotation.as_euler("ZXZ", degrees=False)
  return [tfResult.translation[0], tfResult.translation[1], tfResult.translation[2], euler[0], euler[1], euler[2]]
# 示例用法

def process_list(lst):
    abs_avg = [(abs(lst[2 * i]) + abs(lst[2 * i+1])) / 2 for i in range(int(len(lst) / 2))]
    
    duplicated = [val for avg in abs_avg for val in (avg, avg)]
    
    result = []
    for i in range(len(duplicated)):
        sign = 1 if lst[i] >= 0 else -1
        result.append(duplicated[i] * sign)
    
    return result

# 测试

if __name__ == "__main__":
    original = [0.0004213368993026857, -0.0008528004377880904, 0.00028666616453173425, -0.0009831516465715958, 0.0042507424415833975, -0.004340830747591274, -0.0063115594494359455, -0.0063115594494359455, 0.0004213368993026563, -0.0008528004377881339, 0.0002866661645316718, -0.0009831516465715932, 0.004250742441583344, -0.00434083074759122, -0.006311559449435989, -0.006311559449435989]
    result = process_list(original)
    print(result)  # 输出: [1.0, -1.0, 3.0, 3.0]
    # 创建可视化器
    print(s_pe_dot_pe([0.15,0.08,0.0140169,-0,0,-0], [-0.0639083004720105,-0.0695671360958659,0.0532795295363379,-0,0,-0]))

    # vis = meshcat.Visualizer().open()
    
    # # # 绘制从 (0, 0, 0) 到 (1, 1, 1) 的红色箭头
    # draw_arrow(vis, "arrow1", [1.0, 0.0, 0.0], [1.0, 1.0, 0], color=0xff0000)
    
    # # 绘制从 (0, 0, 0) 到 (0, 1, 0) 的绿色箭头
    # draw_arrow(vis, "arrow2", [1.0, 0.0, 0.0], [-1.0, 1.0, 0.0], color=0x00ff00)
    
    # # 绘制从 (0, 0, 0) 到 (1, 0, 0) 的蓝色箭头
    # draw_arrow(vis, "arrow3", [1.0, 0.0, 0.0], [0, 0.0, 1.0], color=0x0000ff)
    input("按 Enter 键退出程序...")