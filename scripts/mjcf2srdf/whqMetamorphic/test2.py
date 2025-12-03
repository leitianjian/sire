import sys
import numpy as np
from math import sin, cos

FRAME_AXIS_POSITIONS = (
    np.array([[0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 1, 0], [0, 0, 0], [0, 0, 1]])
    .astype(np.float32)
    .T
)
FRAME_AXIS_COLORS = (
    np.array([[1, 0, 0], [1, 0.6, 0], [0, 1, 0], [0.6, 1, 0], [0, 0, 1], [0, 0.6, 1]])
    .astype(np.float32)
    .T
)


def visualize_frame(
    visualizer, name, tform, line_length=0.2, line_width=3, line_color=None
):
    import meshcat.geometry as mg
    import numpy as np
    """
    Visualizes a coordinate frame as an axis triad at a specified pose.

    Parameters
    ----------
        visualizer : `pinocchio.visualize.meshcat_visualizer.MeshcatVisualizer`
            The visualizer instance.
        name : str
            The name of the MeshCat component to add.
        tform : `pinocchio.SE3`
            The transform at which to display the frame.
        line_length : float, optional
            The length of the axes in the triad.
        line_width : float, optional
            The width of the axes in the triad.
        line_color : array-like, optional
            The line colors to use. If None, chooses default axes colors.
    """
    if line_color:
        color = np.array([line_color] * 6).T
    else:
        color = FRAME_AXIS_COLORS

    visualizer.viewer[name].set_object(
        mg.LineSegments(
            mg.PointsGeometry(
                position=line_length * FRAME_AXIS_POSITIONS,
                color=color,
            ),
            mg.LineBasicMaterial(
                linewidth=line_width,
                vertexColors=True,
            ),
        )
    )
    visualizer.viewer[name].set_transform(tform.homogeneous)


def visualize_frames(
    visualizer, prefix_name, tforms, line_length=0.2, line_width=3, line_color=None
):
    """
    Visualizes a set of coordinate frames as axis triads at specified poses.

    Parameters
    ----------
        visualizer : `pinocchio.visualize.meshcat_visualizer.MeshcatVisualizer`
            The visualizer instance.
        prefix_name : str
            The name of the MeshCat component to add.
        tforms : list[`pinocchio.SE3`]
            A list of transforms at which to display frames.
        line_length : float, optional
            The length of the axes in the triad.
        line_width : float, optional
            The width of the axes in the triad.
        line_color : array-like, optional
            The line colors to use. If None, chooses default axes colors.
    """
    visualizer.viewer[prefix_name].delete()
    for idx, tform in enumerate(tforms):
        visualize_frame(
            visualizer,
            f"{prefix_name}/frame{idx}",
            tform,
            line_length=line_length,
            line_width=line_width,
            line_color=line_color,
        )


def visualize_path(visualizer, name, tforms, line_width=3, line_color=[0.0, 0.0, 0.0]):
    """
    Visualizes a path of poses as lines containing only the translation component.

    Parameters
    ----------
        visualizer : `pinocchio.visualize.meshcat_visualizer.MeshcatVisualizer`
            The visualizer instance.
        name : str
            The name of the MeshCat component to add.
        tforms : list[`pinocchio.SE3`]
            A list of transforms representing the vertices of the path.
        line_width : float, optional
            The width of the axes in the triad.
        line_color : array-like, optional
            The line color to use.
    """
    visualize_paths(
        visualizer, name, [tforms], line_width=line_width, line_color=line_color
    )

def pq2tfmatrix(pq):
  import meshcat.transformations as tf
  import numpy as np
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
    
def robotInit(numLinks, resourcePath: str, displayInitJson, vis):
  import meshcat.geometry as g
  import numpy as np
  robot = vis['robot']
  partInitConfig = displayInitJson['part_init_config']
  for i in range(numLinks):
    robot[str(i)].set_transform(pq2tfmatrix(partInitConfig[i]))

  geometryPool = displayInitJson['geometry_pool']
  for i in range(len(geometryPool)):
    geometry = geometryPool[i]
    meshcatGeo = robot[str(geometry['part_id'])][str(geometry['geometry_id'])]
    meshcatGeo.set_transform(np.array(geometry['init_pm']).reshape(4, 4))
    if i % 2 == 1: 
      material = g.MeshPhongMaterial(color=0x0660FF)
    else:
      material = g.MeshPhongMaterial(color=0xD4D4D4)
    if i == 0:
      material = g.MeshPhongMaterial(color=0x755338)
    if(geometry['shape_type'] == 'box'):
      meshcatGeo.set_object(g.Box([geometry['length'], geometry['width'], geometry['height']]), material=material)
    elif(geometry['shape_type'] == 'capsule'):
      meshcatGeo.set_object(g.Cylinder(geometry['length'], geometry['radius']), material=material)
    elif(geometry['shape_type'] == 'sphere'):
      meshcatGeo.set_object(g.Sphere(geometry['radius']), material=material)
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

import numpy as np
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
import time

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
        while True:  # 无限循环，直到用户中断
          start_time = time.time()
          current_time = minTime
          while current_time <= maxTime:
              frame_start = time.time()

              # 查找当前时间对应的索引
              currentIdx = binarySearch(timeIndices, current_time)

              # 更新机器人姿态
              setRobotPqRealtime(numLinks, vis, partpq[currentIdx])

              # 更新箭头
              draw_arrow_realtime(14, vis, motor_pd[currentIdx], motor_force[currentIdx])

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
          print("\n准备重新播放...")
          time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\n动画已停止")
    finally:
        print("\n动画渲染完成")

def setRobotPqRealtime(numLinks, vis, pqs):
    """实时设置机器人姿态"""
    robot = vis['robot']
    for i in range(numLinks):
        robot[str(i)].set_transform(pq2tfmatrix(pqs[i]))

def draw_arrow_realtime(numJoints, vis, motor_pd, motor_force, length=0.1, headRatio=0.3, radius=0.02):
  arrow = vis['arrow']
  # print(numJoints, motor_pd, motor_force, len(motor_pd), len(motor_force))
  for i in range(numJoints):
    start = motor_pd[i][:3]
    direction = motor_pd[i][3:]
    scale = motor_force[i] / 20
    S = create_scale_matrix([scale, scale, scale])
    start = np.array(start, dtype=np.float64)
    direction = np.array(direction, dtype=np.float64)

    radius = radius * scale    
    direction /= np.linalg.norm(direction)
    length *= scale
    end = start + direction * length
    # arrow[str(i)].set_transform(S)
    shaft_length = length * (1 - headRatio)
    # 箭杆中点坐标
    shaft_pos = start + direction * (shaft_length / 2)
    rot_matrix = rotation_matrix_from_vectors([0, 1, 0], direction)
    shaft_transform = tf.translation_matrix(shaft_pos)
    shaft_transform[:3, :3] = rot_matrix[:3, :3]

    head_length = length * headRatio
    head_pos = end - direction * (head_length / 2)
    # head_initial_rot = tf.rotation_matrix(rot_matrix)
    head_transform = tf.translation_matrix(head_pos)
    head_transform[:3, :3] = rot_matrix[:3, :3]

    full_transform = tf.translation_matrix(start)
    # arrow[str(i)].set_transform(full_transform)
    if i == 0:
      # print(start, direction, scale)
      arrow[str(i)]["shaft"].set_transform(shaft_transform @ S)
      arrow[str(i)]["head"].set_transform(head_transform @ S)

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
    vis[name].set_transform(full_transform)
    
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

def arrowInit(numJoints, vis, length=0.1, headRatio=0.3, radius=0.02, color=0xff0000):
  import meshcat.geometry as g
  import numpy as np
  arrow = vis['arrow']
  for i in range(numJoints):
    arrow[str(i)].set_transform(np.eye(4))
    # arrow[str(i)].set_object(g.Group())
    arrow[str(i)]["shaft"].set_object(g.Cylinder(length * (1 - headRatio), radius), g.MeshPhongMaterial(color=0x00ff00, wireframe=False))
    arrow[str(i)]["head"].set_object(g.Cylinder(length * headRatio, radiusBottom=radius, radiusTop=0.0), g.MeshPhongMaterial(color=color, wireframe=False))

def draw_arrow(numJoints, frame, motor_pd, motor_force, length=0.1, headRatio=0.3, radius=0.02):
  arrow = frame['arrow']
  # print(numJoints, motor_pd, motor_force, len(motor_pd), len(motor_force))
  for i in range(numJoints):
    start = motor_pd[i][:3]
    direction = motor_pd[i][3:]
    scale = motor_force[i]
    S = create_scale_matrix([scale, scale, scale])
    start = np.array(start, dtype=np.float64)
    direction = np.array(direction, dtype=np.float64)
    if i == 3:
      print(start, direction)
    radius = radius * scale    
    direction /= np.linalg.norm(direction)
    length *= scale
    end = start + direction * length
    # arrow[str(i)].set_transform(S)
    shaft_length = length * (1 - headRatio)
    # 箭杆中点坐标
    shaft_pos = start + direction * (shaft_length / 2)
    rot_matrix = rotation_matrix_from_vectors([0, 1, 0], direction)
    shaft_transform = tf.translation_matrix(shaft_pos)
    shaft_transform[:3, :3] = rot_matrix[:3, :3]

    head_length = length * headRatio
    head_pos = end - direction * (head_length / 2)
    # head_initial_rot = tf.rotation_matrix(rot_matrix)
    head_transform = tf.translation_matrix(head_pos)
    head_transform[:3, :3] = rot_matrix[:3, :3]

    full_transform = tf.translation_matrix(start)
    # print(S)
    # arrow[str(i)].set_transform(full_transform)
    # arrow[str(i)]["shaft"].set_transform(shaft_transform @ S)
    arrow[str(i)]["head"].set_transform(head_transform @ S)
    
# def drawArrow(numJoints, frame, motor_pd, motor_force):
#   arrow = frame['arrow']
#   for i in range(numJoints):
#     draw_arrow(arrow, "arrow" + str(i), motor_pd[i][:3], motor_pd[i][3:], scale=motor_force[i])

def setRobotPq(numLinks, frame, pqs):
  robot = frame['robot']
  for i in range(numLinks):
    robot[str(i)].set_transform(pq2tfmatrix(pqs[i]))

def binarySearch(timeIndices, time):
  import math
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

def animateRobotByRecords(numLinks, records, frameRate, vis):
  from meshcat.animation import Animation
  partpq = records['partPq']
  timeIndices = records['timeIndex']
  motor_force = records['motor_force']
  motor_pd = records['motor_pd']
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
        # draw_arrow(14, frame, motor_pd[currentIdx], motor_force[currentIdx])
      except Exception as e:
        print("Error setting robot pq at time", currentTime, ":", e)

  vis.set_animation(anim)

# def jointTrajectory(time, TotalTime, start_q, target_q):
#    if (time >= TotalTime):
#       return target_q
#    size = len(target_q)
#    current_q = np.zeros(size)
#    for i in range(size):
#       current_q[i] = (target_q[i] - start_q[i]) * sin(np.pi * time / (2 * TotalTime)) + start_q[i]
#    return current_q

def jointTrajectory(time, TotalTime, start_q, target_q):
   if (time >= TotalTime):
      return target_q
   size = len(target_q)
   current_q = np.zeros(3 * size)
   for i in range(size):
      current_q[3 * i] = (target_q[i] - start_q[i]) * (sin(np.pi * time / TotalTime - np.pi / 2.0) + 1) / 2.0 + start_q[i]
      current_q[3 * i + 1] = (target_q[i] - start_q[i]) * (np.pi / TotalTime) * cos(np.pi * time / TotalTime - np.pi / 2.0) / 2.0
      current_q[3 * i + 2] = - (target_q[i] - start_q[i]) * (np.pi / TotalTime) * (np.pi / TotalTime) * sin(np.pi * time / TotalTime - np.pi / 2.0) / 2.0
   return current_q
  

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

import math

motorIdx = [0, 4, 2, 6, 3, 5, 1, 7, 11, 9, 13, 10, 12, 8]
# disable motor of bb joints
state_2_init = [0, 0, 0, 0, 0, 0, 0]
# enable motor of bb joints
state_1_init = [math.pi, math.pi, 0, 0, 0, 0, 0]
# state_1_init = [math.pi, 0, 0, 0, 0, 0, 0]
# enable motor of bs joint
state_3_init = [0, 0, 2.92, 2.92, -1.6, -1.6, -1.6]
# enable motor of ss joint
state_4_init = [math.pi, math.pi, math.pi * 2 / 3.0, math.pi * 2 / 3.0, -math.pi / 3.0, -math.pi / 3.0, - math.pi / 3.0]

trajectory_path = [state_3_init, state_2_init, state_1_init, state_4_init, state_1_init, state_2_init]

# trajectory_path = [state_1_init, state_2_init, state_1_init, state_2_init, state_1_init, state_2_init]

def main():
  # print(s_pe_dot_pe([0.15,0.08,0.0140169,-0,0,-0], [-0.0639083004720105,-0.0695671360958659,0.0532795295363379,-0,0,-0]))
  # print(s_pe_dot_pe([0.146363857335961,0.0524329845026821,0.0864603764909301,1.05278462481095e-06,1.57079334490724,3.12769227873903],[0.06,0.02,0.042,0.0139003748507651,1.57079334490724,3.14159160080517]))
  import pathlib
  currentDir = pathlib.Path(__file__).parent.resolve()
  # pyEnvPath = str((currentDir / "python").resolve())
  # sys.path.append(pyEnvPath)
  sys.path.append("D:/code/sire/install/python/release")
  import sire
  cs = sire.ControlServer.instance()
  configPath = str((currentDir / "metamophicArm1.xml").resolve())
  sire.fromXmlFile(cs, configPath)
  model = cs.model()

  # model.addFixedJointAbs(model.partPool()[0], model.partPool()[1], [0, 0, 0], [0, 0, 0])
  model.init()
  for i in range(model.numJoints()):
    print(model.joint(i))
    model.addMotion(model.joint(i))
    # actuator = sire.ActuatorSISO.add2Model(model, model.joint(i))
    # if actuator is not None:
    #   actuator.kp = 5
    #   actuator.kd = 0.05
  model.addFixedJointAbs(model.partPool()[0], model.partPool()[1], [0, 0, 0], [0, 0, 0])
  cs.init()
  print(sire.toXmlString(cs))
  simulator = sire.simulator(cs)
  model = cs.model()

  # 仿真控制循环
  motor_pd = []
  motor_force = []
  motor_a = []
  motor_v = []
  motor_p = []
  while(not simulator.isTimeout() and not simulator.isEventListEmpty()):
    # sim_time = sim_time
    sim_time = simulator.simTime()
    if sim_time <= 1:
      currentJoint = jointTrajectory(sim_time, 1, state_2_init * 2, trajectory_path[0] * 2)
    elif sim_time > 1 and sim_time <= 2:
      currentJoint = jointTrajectory(sim_time - 1, 1, trajectory_path[0] * 2, trajectory_path[1] * 2)
    elif sim_time > 2 and sim_time <= 3:
      currentJoint = jointTrajectory(sim_time - 2, 1, trajectory_path[1] * 2, trajectory_path[2] * 2)
    elif sim_time > 3 and sim_time <= 4: 
      currentJoint = jointTrajectory(sim_time - 3, 1, trajectory_path[2] * 2, trajectory_path[3] * 2)
    elif sim_time > 4 and sim_time <= 5: 
      currentJoint = jointTrajectory(sim_time - 4, 1, trajectory_path[3] * 2, trajectory_path[4] * 2)
    elif sim_time > 5 and sim_time <= 6: 
      currentJoint = jointTrajectory(sim_time - 5, 1, trajectory_path[4] * 2, trajectory_path[5] * 2)
    
    joint_pos_direction = []
    for i in range(14):
        joint = model.joint(i)
        joint_pos_direction.append(joint.posAndZDirection())

    # pd = joint_pos_direction[1][:3]
    # joint_pos_direction.insert(7, [pd[0], -pd[1], pd[2], 0, -1, 0])
    # pd = joint_pos_direction[9][:3]
    # joint_pos_direction.append([pd[0], -pd[1], pd[2], 0, 1, 0])
    # for i in range(14):
    #   print(i, joint_pos_direction[i])
    motor_pd.append(joint_pos_direction)
    # sim_time = simulator.simTime()
    # if sim_time < 2.0:
    #   # target_q = jointTrajectory(sim_time, 2.0, [0], [-0.003])
    #   target_q = jointTrajectory(sim_time, 2.0, [0], [-0.010])
    # elif sim_time < 3.0 and sim_time >= 2.0:
    #   target_q = jointTrajectory(sim_time - 2, 2.0, [-0.010], [0.002])
    # # elif sim_time < 4.0 and sim_time >= 3.0:
    # #   target_q = jointTrajectory(sim_time - 3, 1.0, [-0.01], [-0])
    mpInput = []
    for i in range(14):
      motion = model.motionPool()[motorIdx[i]]
      # if isinstance(motion, sire.ActuatorSISO):
          # motion.setDesiredValue(target_q[i])
          # mInput.append(motion.mp)
          # motion.desiredValue = currentJoint[i]
      motion.mp = currentJoint[3 * i]
      motion.mv = currentJoint[3 * i + 1]
      motion.ma = currentJoint[3 * i + 2]
      # motion.mp = 0
      # motion.mv = 0
      # motion.ma = 0
          # mpInput.append(motion.mp)

    simulator.step(1, False)

    f = []
    a = []
    v = []
    p = []
    for i in range(14):
      motion = model.motionPool()[motorIdx[i]]
      # if isinstance(motion, sire.ActuatorSISO):
          # motion.setDesiredValue(target_q[i])
          # mInput.append(motion.mp)
          # motion.desiredValue = currentJoint[i]
      f.append(motion.mf)
      a.append(motion.ma)
      v.append(motion.mv)
      p.append(motion.mp)
        # if i == 0: 
        #   print(motion.ma, motion.mf)
    # f.insert(7, 0)
    # f.append(0)
    motor_force.append(f)
    motor_a.append(a)
    motor_v.append(v)
    motor_p.append(p)
    
  
  simulator.recordsContactCptInfo()
  displayInitJson = model.displayInitJson()
  result = simulator.recordsToJson()
  result['motor_force'] = motor_force
  result['motor_pd'] = motor_pd
  # 加速度图
  idx = 0
  force = []
  p = []
  v = []
  a = []
  print(motor_force)
  for i in range(len(motor_force)):
     force.append(motor_force[i][idx])
     p.append(motor_p[i][idx])
     v.append(motor_v[i][idx])
     a.append(motor_a[i][idx])
  import matplotlib.pyplot as plt
  plt.subplot(2, 2, 1)
  plt.plot(result['timeIndex'], p, 'r-', linewidth=2)
  plt.axvline(x=result['timeIndex'][-1], color='r', linestyle='--', alpha=0.5)
  plt.title('Joint p vs Time')
  plt.xlabel('Time (s)')
  plt.ylabel('p (rad/s²)')
  plt.grid(True)

  plt.subplot(2, 2, 2)
  plt.plot(result['timeIndex'], v, 'r-', linewidth=2)
  plt.axvline(x=result['timeIndex'][-1], color='r', linestyle='--', alpha=0.5)
  plt.title('Joint v vs Time')
  plt.xlabel('Time (s)')
  plt.ylabel('v (rad/s²)')
  plt.grid(True)

  plt.subplot(2, 2, 3)
  plt.plot(result['timeIndex'], a, 'r-', linewidth=2)
  plt.axvline(x=result['timeIndex'][-1], color='r', linestyle='--', alpha=0.5)
  plt.title('Joint a vs Time')
  plt.xlabel('Time (s)')
  plt.ylabel('a (rad/s²)')
  plt.grid(True)

  plt.subplot(2, 2, 4)
  plt.plot(result['timeIndex'], force, 'r-', linewidth=2)
  plt.axvline(x=result['timeIndex'][-1], color='r', linestyle='--', alpha=0.5)
  plt.title('Joint Force vs Time')
  plt.xlabel('Time (s)')
  plt.ylabel('Force (rad/s²)')
  plt.grid(True)

  # 添加总标题
  plt.suptitle('Joint Trajectory Profile', fontsize=16)

  # 调整布局
  plt.tight_layout()
  plt.subplots_adjust(top=0.92)

  # 显示图表
  plt.show()
  print("Simulation finished, records loaded")
  # import meshcat
  # vis = meshcat.Visualizer()
  # import meshcat.transformations as tf
  # resourcePath = str((currentDir).resolve())
  # robotInit(model.numLinks(), resourcePath, displayInitJson, vis)
  # arrowInit(16, vis)
  # # forceInit(model.numJoints(), resourcePath, displayInitJson, vis)
  # # for i in range(len(result['partPq'])):
  # #   print(result['partPq'][i])
  # animateRobotByRecords(model.numLinks(), result, 1000, vis)
  # vis.jupyter_cell()
  # del simulator
  input("按 Enter 键退出程序...")
  # import json
  # with open("result.json", "w", encoding="utf-8") as f:
  #     json.dump(result, f, ensure_ascii=False, indent=2)

if __name__ == "__main__":
  main()