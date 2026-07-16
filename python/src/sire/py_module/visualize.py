import numpy as np
import time
import math
import meshcat.geometry as g
import meshcat.transformations as tf
from meshcat.animation import Animation
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import RigidTransform as TF
from PIL import Image
from meshcat.geometry import TriangularMeshGeometry

def pq2tfmatrix(pq):
  """
  Convert a pq to a transformation matrix
  :param pq: [position, quaternion]
  :return: transformation matrix
  """
  p = pq[:3]
  q = pq[3:]
  q.insert(0, q.pop())
  return tf.quaternion_matrix(q) + np.array([[0, 0, 0, p[0]],
                                             [0, 0, 0, p[1]],
                                             [0, 0, 0, p[2]],
                                             [0, 0, 0, 0]])

def pe3132tfmatrix(pe):
  """
  Convert a pe to a transformation matrix
  :param pe: [position, euler angles in ZXZ order]
  :return: transformation matrix
  """
  tf1 = TF.from_components(
                translation=pe[:3],
                rotation=R.from_euler("ZXZ", pe[3:], degrees=False))
  return tf1.as_matrix().flatten()

def buildHFieldMeshHeights(width, depth, nrow, ncol, heights):
    # 生成顶点
    x_vals = np.linspace(-width / 2, width / 2, ncol)
    y_vals = np.linspace(depth/2, -depth/2, nrow)
    xx, yy = np.meshgrid(x_vals, y_vals)
    zz = np.array(heights).reshape(nrow, ncol)

    vertices = np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=1).astype(np.float32)

    # 生成三角面索引
    faces = []
    for r in range(nrow - 1):
        for c in range(ncol - 1):
            a = r * ncol + c
            b = a + 1
            c_idx = a + ncol
            d = c_idx + 1
            faces.append([a, b, d])
            faces.append([a, d, c_idx])
    faces = np.array(faces, dtype=np.uint32)

    return TriangularMeshGeometry(vertices, faces)


def buildHFieldMeshFull(width, depth, nrow, ncol, heights, min_height):
    """
    生成 HeightField 的完整碰撞几何可视化网格，包括：
      - 顶面（实际高度）
      - 底面（min_height）
      - 四个侧面（连接顶面和底面边缘）

    min_height 即 coal 中 HeightField 的底面 z 坐标（本地坐标系）。
    """
    x_vals = np.linspace(-width / 2, width / 2, ncol)
    y_vals = np.linspace(depth / 2, -depth / 2, nrow)
    xx, yy = np.meshgrid(x_vals, y_vals)
    zz_top = np.array(heights).reshape(nrow, ncol).astype(np.float32)

    N = nrow * ncol  # 每层顶点数

    # ---- 顶点：先顶面，再底面 ----
    top_v = np.stack([xx.ravel(), yy.ravel(), zz_top.ravel()], axis=1)
    bot_v = np.stack(
        [xx.ravel(), yy.ravel(), np.full(N, min_height, dtype=np.float32)],
        axis=1,
    )
    vertices = np.concatenate([top_v, bot_v], axis=0)

    faces = []

    # ---- 顶面 (法线朝上) ----
    for r in range(nrow - 1):
        for c in range(ncol - 1):
            a = r * ncol + c
            b = a + 1
            d = a + ncol
            e = d + 1
            faces.append([a, b, e])
            faces.append([a, e, d])

    # ---- 底面 (法线朝下，翻转绕序) ----
    for r in range(nrow - 1):
        for c in range(ncol - 1):
            a = N + r * ncol + c
            b = a + 1
            d = a + ncol
            e = d + 1
            faces.append([a, e, b])
            faces.append([a, d, e])

    # ---- 侧面 ----
    # +y 侧边 (y = +depth/2, row=0)
    for c in range(ncol - 1):
        t0 = 0 * ncol + c          # top, row=0
        t1 = t0 + 1
        b0 = N + t0                # bottom, row=0
        b1 = N + t1
        faces.append([t0, b0, b1])
        faces.append([t0, b1, t1])

    # -y 侧边 (y = -depth/2, row=nrow-1)
    for c in range(ncol - 1):
        t0 = (nrow - 1) * ncol + c
        t1 = t0 + 1
        b0 = N + t0
        b1 = N + t1
        faces.append([t0, t1, b1])
        faces.append([t0, b1, b0])

    # -x 侧边 (x = -width/2, col=0)
    for r in range(nrow - 1):
        t0 = r * ncol + 0
        t1 = (r + 1) * ncol + 0
        b0 = N + t0
        b1 = N + t1
        faces.append([t0, t1, b1])
        faces.append([t0, b1, b0])

    # +x 侧边 (x = +width/2, col=ncol-1)
    for r in range(nrow - 1):
        t0 = r * ncol + (ncol - 1)
        t1 = (r + 1) * ncol + (ncol - 1)
        b0 = N + t0
        b1 = N + t1
        faces.append([t0, b0, b1])
        faces.append([t0, b1, t1])

    faces = np.array(faces, dtype=np.uint32)
    return TriangularMeshGeometry(vertices, faces)

def buildHFieldMeshPNG(png_path, width, depth, scale_z):
    """
    按照 MuJoCo 的 hfield 归一化方式从 PNG 生成三角网格。

    png_path  : MuJoCo 输出的高度场 PNG
    width     : 地形全长（X 方向，米）
    depth     : 地形全长（Y 方向，米）
    scale_z   : 高度缩放（对应 MuJoCo geom.size[2]）
    pos_z     : 垂直偏移（对应 MuJoCo geom.pos.z）
    """
    # 1. 读取 PNG 并转换为 [0, 1]（MuJoCo 用红色通道或灰度，256 或 65535）
    img = Image.open(png_path)
    if img.mode in ('I', 'I;16'):
        # 16 位灰度
        pixels = np.array(img, dtype=np.float64) / 65535.0
    elif img.mode == 'L':
        pixels = np.array(img, dtype=np.float64) / 255.0
    else:
        # RGB/RGBA -> 取红色通道
        pixels = np.array(img.convert('RGB'))[:, :, 0].astype(np.float64) / 255.0

    # 2. MuJoCo 风格的 min‑max 归一化到 [0, 1]
    emin = pixels.min()
    emax = pixels.max()
    if emin > emax:
        raise ValueError("Invalid height field data: min > max")
    pixels -= emin
    if emax - emin > 1e-10:  # MuJoCo 中用的是 mjEPS ≈ 1e-10
        pixels /= (emax - emin)

    rows, cols = pixels.shape

    # 3. 生成顶点（局部坐标，中心在 origin）
    # x_vals = np.linspace(-width / 2, width / 2, cols)
    # y_vals = np.linspace(-depth / 2, depth / 2, rows)
    # y_vals = np.linspace(depth/2, -depth/2, rows)    # 修正后
    # xx, yy = np.meshgrid(x_vals, y_vals)
    zz = pixels * scale_z

    return buildHFieldMeshHeights(width, depth, rows, cols, zz)

def robotInit(numLinks, resourcePath: str, displayInitJson, vis):
  robot = vis['robot']
  partInitConfig = displayInitJson['part_init_config']
  for i in range(numLinks):
    robot[str(i)].set_transform(pq2tfmatrix(partInitConfig[i]))

  geometryPool = displayInitJson['geometry_pool']
  for i in range(len(geometryPool)):
    geometry = geometryPool[i]
    meshcatGeo = robot[str(geometry['part_id'])][str(i)]
    meshcatGeo.set_transform(np.array(geometry['init_pm']).reshape(4, 4))
    if i % 2 == 1: 
      material = g.MeshPhongMaterial(color=0x0660FF)
    else:
      material = g.MeshPhongMaterial(color=0xD4D4D4)
    if str(geometry['part_id']) == "0":
      material = g.MeshPhongMaterial(color=0x755338)
    if(geometry['shape_type'] == 'box'):
      meshcatGeo.set_object(g.Box([geometry['length'], geometry['width'], geometry['height']]), material=material)
    elif(geometry['shape_type'] == 'capsule'):
      meshcatGeo.set_object(g.Cylinder(geometry['length'], geometry['radius']), material=material)
    elif(geometry['shape_type'] == 'cylinder'):
      meshcatGeo.set_object(g.Cylinder(geometry['length'], geometry['radius']), material=material)
    elif(geometry['shape_type'] == 'sphere'):
      meshcatGeo.set_object(g.Sphere(geometry['radius']), material=material)
    elif(geometry['shape_type'] == 'hfield'):
      meshcatGeo.set_object(buildHFieldMeshFull(
        width=geometry['x_dim'],
        depth=geometry['y_dim'],
        nrow=geometry['nrow'],
        ncol=geometry['ncol'],
        heights=geometry['heights'],
        min_height=geometry.get('min_height', 0.0),
      ), material=material)
    elif(geometry['shape_type'] == 'mesh'):
      ext = geometry['resource_path'].split('.')[-1]
      if ext == 'stl' or ext == 'STL':
        meshcatGeo.set_object(g.StlMeshGeometry.from_file(
          resourcePath + geometry['resource_path']), material=material)
      elif ext == 'obj':
        meshcatGeo.set_object(g.ObjMeshGeometry.from_file(
          resourcePath + geometry['resource_path']), material=material)
      else:
        print("Unknown mesh file type", ext)

def binarySearch(timeIndices, time):
  """
  Binary search to find the index of the closest time
  :param timeIndices: list of time indices
  :param time: target time
  :return: index of the closest time index
  """
  low = 0
  high = len(timeIndices) - 1
  while low <= high:
    mid = (low + high) // 2
    if math.isclose(timeIndices[mid], time):
      return mid
    if timeIndices[mid] < time:
      low = mid + 1
    else:
      high = mid - 1

  return min(int(low), len(timeIndices) - 1)
  
def s_pe_dot_pe(pe1, pe2):
  tf0 = TF.from_components(
                translation=pe1[:3],
                rotation=R.from_euler("ZXZ", pe1[3:], degrees=False))
  tf1 = TF.from_components(
                translation=pe2[:3],
                rotation=R.from_euler("ZXZ", pe2[3:], degrees=False))
  tfResult = tf0 * tf1
  euler = tfResult.rotation.as_euler("ZXZ", degrees=False)
  return [tfResult.translation[0], tfResult.translation[1], tfResult.translation[2], euler[0], euler[1], euler[2]]

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
    axis = np.cross(a, b)
    axis /= np.linalg.norm(axis)
    angle = np.arccos(np.dot(a, b))
    
    # 生成旋转矩阵
    return tf.rotation_matrix(angle, axis)

def create_scale_matrix(scale):
    S = np.eye(4)
    S[0, 0] = scale[0]
    S[1, 1] = scale[1]
    S[2, 2] = scale[2]
    return S

def draw_arrow_realtime(numJoints, vis, motor_pd, motor_force, length=0.1, headRatio=0.3, radius=0.02):
  arrow = vis['arrow']
  for i in range(numJoints):
    start = motor_pd[i][:3]
    direction = motor_pd[i][3:]
    scale = motor_force[i] * 100
    
    S = create_scale_matrix([scale, scale, scale])
    start = np.array(start, dtype=np.float64)
    direction = np.array(direction, dtype=np.float64)

    l = length * scale
    direction /= np.linalg.norm(direction)
    end = start + direction * l
    shaft_length = l * (1 - headRatio)
    # 箭杆中点坐标
    shaft_pos = start + direction * (shaft_length / 2)
    rot_matrix = rotation_matrix_from_vectors([0, 1, 0], direction)
    shaftPosMatrix = tf.translation_matrix(shaft_pos)

    head_length = l * headRatio
    head_pos = end - direction * (head_length / 2)
    headPosMatrix = tf.translation_matrix(head_pos)

    arrow[str(i)]["shaft"].set_transform(shaftPosMatrix @ rot_matrix @ S)
    arrow[str(i)]["head"].set_transform(headPosMatrix @ rot_matrix @ S)

def setRobotPqRealtime(numLinks, vis, pqs):
    """实时设置机器人姿态"""
    robot = vis['robot']
    for i in range(numLinks):
      robot[str(i)].set_transform(pq2tfmatrix(pqs[i]))

def setRobotPq(numLinks, frame, pqs):
  robot = frame['robot']
  for i in range(numLinks):
    robot[str(i)].set_transform(pq2tfmatrix(pqs[i]))

def animateRobotByRecords(numLinks, records, frameRate, vis):
  partpq = records['partPq']
  timeIndices = records['timeIndex']
  anim = Animation()
  anim.default_framerate = frameRate

  minTime = 0
  maxTime = timeIndices[-1]
  totalFrameNumber = int((maxTime - minTime) * anim.default_framerate)
  for i in range(totalFrameNumber):
    currentTime = minTime + i / anim.default_framerate
    currentIdx = binarySearch(timeIndices, currentTime)
    with anim.at_frame(vis, i) as frame:
      try:
        setRobotPq(numLinks, frame, partpq[currentIdx])
      except Exception as e:
        print("Error setting robot pq at time", currentTime, ":", e)

  vis.set_animation(anim)

def animateRobotByRecordsRealtime(numLinks, records, frameRate, vis):
    """实时循环渲染机器人动画"""
    partpq = records['partPq']
    timeIndices = records['timeIndex']
    motor_force = records['motor_force']
    motor_pd = records['motor_pd']
    
    minTime = 0
    maxTime = timeIndices[-1]
    frame_duration = 1.0 / frameRate
    
    print(f"开始实时动画渲染，总时长: {maxTime:.2f}秒，帧率: {frameRate}fps")
    
    start_time = time.time()
    current_time = minTime
    
    try:
        # while True:  # 无限循环，直到用户中断
          start_time = time.time()
          current_time = minTime
          while current_time <= maxTime:
              frame_start = time.time()

              # 查找当前时间对应的索引
              currentIdx = binarySearch(timeIndices, current_time)

              # 更新机器人姿态
              setRobotPqRealtime(numLinks, vis, partpq[currentIdx])

              # 更新箭头
              draw_arrow_realtime(16, vis, motor_pd[currentIdx], motor_force[currentIdx])

              # 计算并显示帧率
              elapsed = time.time() - start_time
              fps = (current_time - minTime) / elapsed if elapsed > 0 else 0
              print(f"时间: {current_time:.2f}/{maxTime:.2f}s | 实际FPS: {fps:.1f}", end='\r')

              # 控制帧率
              frame_time = time.time() - frame_start
              sleep_time = max(0, frame_duration - frame_time)
              time.sleep(sleep_time)

              # 更新时间
              current_time = minTime + elapsed
          # 循环播放时，添加短暂暂停
          # print("\n准备重新播放...")
          # time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\n动画已停止")
    finally:
        print("\n动画渲染完成")