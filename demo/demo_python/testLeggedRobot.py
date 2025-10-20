import sys
import numpy as np
from math import sin, cos

def foot_trajectory(phase_time, swing_time, stance_time, step_height, step_length):
    """
    输入：
        phase_time: 当前相位时间（秒）
        swing_time: 腾空阶段总时间
        stance_time: 支撑阶段总时间
        step_height: 最大抬脚高度
        step_length: 一步前进的距离
    输出：
        相对于身体坐标系下的足端位置 [x, z]
    """
    total_time = swing_time + stance_time
    phase = phase_time / total_time  # 归一化相位 [0, 1)

    if phase <= swing_time / total_time:
        # === 腾空阶段（贝塞尔曲线） ===
        swing_phase = phase / (swing_time / total_time)

        P0 = np.array([0.0, 0.0])                  # 起点
        P1 = np.array([0.25 * step_length, step_height])
        P2 = np.array([0.75 * step_length, step_height])
        P3 = np.array([step_length, 0.0])          # 终点

        t = swing_phase
        foot_xy = (1 - t)**3 * P0 + 3*(1 - t)**2*t * P1 + 3*(1 - t)*t**2 * P2 + t**3 * P3
    else:
        foot_xy = np.array([-0.03, 0.0])

    # 输出为相对 body 坐标系的 (x, z)
    return np.array([foot_xy[0], 0.0, foot_xy[1]])  # 添加 y = 0 作为中间项（用于 3D）

def forward_kinematics(abduction_angle, hip_angle, knee_angle, L1=0.08505, L2=0.2, L3=0.2):
    r_y = L1
    # print(hip_angle, knee_angle)
    r_x = -L2 * sin(hip_angle) - L3 * sin(hip_angle + knee_angle)
    r_z = -L2 * cos(hip_angle) - L3 * cos(hip_angle + knee_angle)
    return np.array([r_x, r_y, r_z])

def inverse_kinematics(x, y, z, theta_init, l1=0.08505, l2=0.2, l3=0.2):
    max_iter = 10000
    tolerance = 1e-4
    theta = theta_init
    for i in range(max_iter):
        x_curr, y_curr, z_curr = forward_kinematics(theta[0], theta[1], theta[2])
        error = np.array([x - x_curr, y - y_curr, z - z_curr])
        error_num = np.sqrt((x - x_curr)**2 + (y - y_curr)**2 + (z - z_curr)**2)
        J = compute_J(theta, l1, l2, l3)
        delta_theta = np.linalg.pinv(J) @ error
        theta += delta_theta
        theta = (theta + np.pi) % (2 * np.pi) - np.pi
        if error_num < tolerance:
            break
        if i == max_iter - 1:
            print("Inverse kinematics failed to converge.")
            return None
    
    return theta

def compute_J(q, L1=0.08505, L2=0.2, L3=0.2):
    theta1, theta2, theta3 = q
    J = np.array([[0, - L3*cos(theta2 + theta3) - L2*cos(theta2), -L3*cos(theta2 + theta3)], 
                  [0, 0, 0], 
                  [0,  L3*sin(theta2 + theta3) + L2*sin(theta2),  L3*sin(theta2 + theta3)]])
    return J

class GaitParams:
  def __init__(self):
    self.amplitude = 0.3
    self.frequency = 1
    self.phase_offset = np.pi / 2

# def pq2tfmatrix(pq):
#   import meshcat.transformations as tf
#   import numpy as np
#   """
#   Convert a pq to a transformation matrix
#   :param pq: [position, quaternion]
#   :return: transformation matrix
#   """
#   p = pq[:3]
#   q = pq[3:]
#   q.insert(0, q.pop())
#   return tf.quaternion_matrix(q) + np.array([[0, 0, 0, p[0]],
#                                              [0, 0, 0, p[1]],
#                                              [0, 0, 0, p[2]],
#                                              [0, 0, 0, 0]])
    
# def robotInit(numLinks, resource_path, displayInitJson, vis):
#   import meshcat.geometry as g
#   import numpy as np
#   robot = vis['robot']
#   partInitConfig = displayInitJson['part_init_config']
#   for i in range(numLinks):
#     robot[str(i)].set_transform(pq2tfmatrix(partInitConfig[i]))

#   geometryPool = displayInitJson['geometry_pool']
#   for i in range(len(geometryPool)):
#     geometry = geometryPool[i]
#     meshcatGeo = robot[str(geometry['part_id'])][str(geometry['geometry_id'])]
#     meshcatGeo.set_transform(np.array(geometry['init_pm']).reshape(4, 4))
#     if i % 2 == 1: 
#       material = g.MeshPhongMaterial(color=0x0660FF)
#     else:
#       material = g.MeshPhongMaterial(color=0xD4D4D4)
#     if i == 0:
#       material = g.MeshPhongMaterial(color=0x755338)
#     if(geometry['shape_type'] == 'box'):
#       meshcatGeo.set_object(g.Box([geometry['length'], geometry['width'], geometry['height']]), material=material)
#     elif(geometry['shape_type'] == 'capsule'):
#       meshcatGeo.set_object(g.Cylinder(geometry['size'][0], geometry['size'][1]), material=material)
#     elif(geometry['shape_type'] == 'sphere'):
#       meshcatGeo.set_object(g.Sphere(geometry['radius']), material=material)
#     elif(geometry['shape_type'] == 'mesh'):
#       ext = geometry['resource_path'].split('.')[-1]
#       if ext == 'stl':
#         meshcatGeo.set_object(g.StlMeshGeometry.from_file(
#           resource_path, geometry['resource_path']), material=material)
#       elif ext == 'obj':
#         meshcatGeo.set_object(g.ObjMeshGeometry.from_file(
#           resource_path + geometry['resource_path']), material=material)
#       else:
#         print("Unknown mesh file type", ext)

# def setRobotPq(numLinks, frame, pqs):
#   robot = frame['robot']
#   for i in range(numLinks):
#     robot[str(i)].set_transform(pq2tfmatrix(pqs[i]))

# def binarySearch(timeIndices, time):
#   import math
#   """
#   Binary search to find the index of the closest time
#   :param timeIndices: list of time indices
#   :param time: target time
#   :return: index of the closest time index
#   """
#   low = 0
#   high = len(timeIndices) - 1
#   while low <= high:
#     mid = (low + high) // 2
#     if math.isclose(timeIndices[mid], time):
#       return mid
#     if timeIndices[mid] < time:
#       low = mid + 1
#     else:
#       high = mid - 1

#   return min(int(low), len(timeIndices) - 1)

# def animateRobotByRecords(numLinks, records, frameRate, vis):
#   from meshcat.animation import Animation
#   partpq = records['partPq']
#   timeIndices = records['timeIndex']
#   anim = Animation()
#   anim.default_framerate = frameRate

#   minTime = 0
#   maxTime = timeIndices[-1]
#   totalFrameNumber = int((maxTime - minTime) * anim.default_framerate)

#   for i in range(totalFrameNumber):
#     currentTime = minTime + i / anim.default_framerate
#     currentIdx = binarySearch(timeIndices, currentTime)
#     with anim.at_frame(vis, i) as frame:
#       try:
#         setRobotPq(numLinks, frame, partpq[currentIdx])
#       except Exception as e:
#         print("Error setting robot pq at time", currentTime, ":", e)
import sire

def getMotionsMp(model):
    motionsMp = np.zeros(model.numMotions())
    for i in range(model.numMotions()):
        motionsMp[i] = model.motion(i).mp
    return motionsMp

def getMotionsMv(model):
    motionsMv = np.zeros(model.numMotions())
    for i in range(model.numMotions()):
        motionsMv[i] = model.motion(i).mv
    return motionsMv

def getSingleComponentForceFce(model):
    fce = np.zeros(12)
    for i in range(12):
        f = model.force(i)
        if isinstance(f, sire.SingleComponentForce):
            fce[i] = f.fce
    return fce

def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd

#   vis.set_animation(anim)
def assignTau(model, tau):
    for i in range(len(tau)):
        fce = model.force(i)
        if isinstance(fce, sire.SingleComponentForce):
            fce.fce = tau[i]

def main():
  # sys.path.append("D:/code/sire/install/python/release")
  # print(sire.pq2tfmatrix([0,0,0,1,0,0,0]))
  cs = sire.ControlServer.instance()
  sire.fromXmlFile(cs, 'D:/code/sire/demo/demo_python/a1_modified.xml')
  cs.init()
  simulator = sire.simulator(cs)
  model = cs.model()
  gait = GaitParams()

  leg_phase = {
      0: 0.5,   # FR
      1: 0.0,  # FL
      2: 0.0,   # RR
      3: 0.5   # RL
  }

  init_angles = np.array([0, -0.9, 1.8])
  kps = [50, 50, 30, 50, 50, 30, 50, 50, 100, 50, 50, 100]
  kds = np.array([2, 2, 1, 2, 2, 1, 2, 2, 2, 2, 2, 2])
  # kps = np.array([20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0])
  # kds = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
  # 仿真控制循环
  simulator.simDuration = 3
  while(not simulator.isTimeout() and not simulator.isEventListEmpty()):
    isCtrl = simulator.integrate()
    sim_time = simulator.simTime()
    # target_q = [0, -0.9, 1.8, 0, -0.9, 1.8, 0, -0.9, 1.8, 0, -0.9, 1.8]
    target_q = np.array([0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8]) # 这个才是对的角度，目前的角度都反了
    # target_q = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] # 这个才是对的角度，目前的角度都反了
    # target_q = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01] # 这个才是对的角度，目前的角度都反了

    # # 控制四条腿的运动
    # duty_ratio = 0.75  # 支撑相比例
    # swing_time = (1.0 - duty_ratio) / gait.frequency
    # stance_time = duty_ratio / gait.frequency

    # leg_origins = {
    #           0: np.array([0.25, -0.1, -0.27]),  # FR
    #           1: np.array([0.25, 0.1, -0.27]),  # FL
    #           2: np.array([-0.25, -0.1, -0.27]),  # RR
    #           3: np.array([-0.25, 0.1, -0.27]),  # RL
    #       }
    # for leg in range(4):
    #   phase = (sim_time * gait.frequency + leg_phase[leg]) % 1.0
    #   side_sign = 1 if leg in [0, 2] else -1

    #   # 生成足端轨迹（相对身体）
    #   foot_target_local = foot_trajectory(
    #     phase * (swing_time + stance_time),
    #     swing_time=swing_time,
    #     stance_time=stance_time,
    #     step_height=0.1,
    #     step_length=-0.15
    #   )
    #   foot_relevent_xpos = [0, 0.085, -0.25]
    #   # foot_relevent_xpos[1] = foot_relevent_xpos[1] * side_sign
    #   foot_relevent_xpos = foot_relevent_xpos + foot_target_local
    #   x, y, z = foot_relevent_xpos

    #   ik_ans = inverse_kinematics(x, y, z, init_angles)
    #   if ik_ans is None:
    #     continue
    #   joint_angles = ik_ans
    #   joint_angles[0] = -0.1 * side_sign  # abduction补偿，因为正常站不稳
    #   fp = forward_kinematics(joint_angles[0], joint_angles[1], joint_angles[2])

    #   target_q[leg * 3 + 0] = joint_angles[0]
    #   target_q[leg * 3 + 1] = joint_angles[1]
    #   target_q[leg * 3 + 2] = joint_angles[2]

    # motionPool = model.motionPool()
    # for i in range(12):
    #   motion = model.motionPool()[i]
    #   if isinstance(motion, sire.ActuatorSISO):
    #     motion.desiredValue = target_q[i]
    motionMp = getMotionsMp(model)
    motionMv = getMotionsMv(model)
    tau = pd_control(target_q, motionMp, kps, np.zeros_like(kds), motionMv, kds)
    for i in range(12):
      motion = model.motionPool()[i]
      if isinstance(motion, sire.ActuatorSISO):
        motion.desiredValue = tau[i]
    simulator.handleContact()
    # print(tau, getSingleComponentForceFce(model))s
    # simulator.step(1, False)
  simulator.recordsContactCptInfo()
  displayInitJson = model.displayInitJson()
  result = simulator.recordsToJson()
  print("Simulation finished, records loaded")
  import meshcat
  vis = meshcat.Visualizer()
  resourcePath = "D:/code/sire/web_interface/public"
  sire.robotInit(model.numLinks(), resourcePath, displayInitJson, vis)
  sire.animateRobotByRecords(model.numLinks(), result, 1000, vis)
  # vis.jupyter_cell()
  del simulator
  input("按 Enter 键退出程序...")
  import json
  with open("result.json", "w", encoding="utf-8") as f:
      json.dump(result, f, ensure_ascii=False, indent=2)

if __name__ == "__main__":
  main()