import time

import numpy as np
import torch
import yaml
import sire
import json

LEGGED_GYM_ROOT_DIR = r"D:\code\sire\demo\demo_python\sirePaperDogRL"
def get_gravity_orientation(quaternion):
  qx = quaternion[0]
  qy = quaternion[1]
  qz = quaternion[2]
  qw = quaternion[3]

  gravity_orientation = np.zeros(3)

  gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
  gravity_orientation[1] = -2 * (qz * qy + qw * qx)
  gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)

  return gravity_orientation


def pd_control(target_q, q, kp, target_dq, dq, kd):
  """Calculates torques from position commands"""
  return (target_q - q) * kp + (target_dq - dq) * kd

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

def getMotionsMa(model):
  motionsMa = np.zeros(model.numMotions())
  for i in range(model.numMotions()):
    motionsMa[i] = model.motion(i).ma
  return motionsMa

def getMotionsMf(model):
  motionsMf = np.zeros(model.numMotions())
  for i in range(model.numMotions()):
    motionsMf[i] = model.motion(i).desiredValue
  return motionsMf

def getBodyQuat(model, i):
  return np.array(model.link(i).getPq()[3:])

def getBodyVa(model, i):
  """Body-origin [linear, angular] velocity expressed in body axes."""
  link = model.link(i)
  return np.asarray(sire.vs2bodyVa(link.getPq(), link.getVs()), dtype=np.float64)

def assignTau(model, tau):
  for i in range(len(tau)):
    fce = model.force(i)
    if isinstance(fce, sire.SingleComponentForce):
      fce.fce = tau[i]

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

def clampVec(vec, limitVec):
  lower = limitVec[0::2]
  upper = limitVec[1::2]
  return np.minimum(np.maximum(vec, lower), upper)

if __name__ == "__main__":
  # get config file name from command line
  # import argparse

  # parser = argparse.ArgumentParser()
  # parser.add_argument("config_file", type=str, help="config file name in the config folder")
  # args = parser.parse_args()
  config_file = "go2_can_work.yaml"
  with open(f"{LEGGED_GYM_ROOT_DIR}/{config_file}", "r") as f:
    config = yaml.load(f, Loader=yaml.FullLoader)
    policy_path = config["policy_path"].replace("{LEGGED_GYM_ROOT_DIR}", LEGGED_GYM_ROOT_DIR)
    xml_path = config["xml_path"].replace("{LEGGED_GYM_ROOT_DIR}", LEGGED_GYM_ROOT_DIR)

    simulation_duration = config["simulation_duration"]
    simulation_dt = config["simulation_dt"]
    ctrl_dt = config["ctrl_dt"]
    control_decimation = config["control_decimation"]

    kps = np.array(config["kps"], dtype=np.float32)
    kds = np.array(config["kds"], dtype=np.float32)

    default_angles = np.array(config["default_angles"], dtype=np.float32)

    lin_vel_scale = config["lin_vel_scale"]
    ang_vel_scale = config["ang_vel_scale"]
    dof_pos_scale = config["dof_pos_scale"]
    dof_vel_scale = config["dof_vel_scale"]
    action_scale = config["action_scale"]
    cmd_scale = np.array(config["cmd_scale"], dtype=np.float32)

    num_actions = config["num_actions"]
    num_obs = config["num_obs"]
        
    cmd = np.array(config["cmd_init"], dtype=np.float32)

  # define context variables
  action = np.zeros(num_actions, dtype=np.float32)
  target_dof_pos = default_angles.copy()
  obs = np.zeros(num_obs, dtype=np.float32)

  # Load robot model
  cs = sire.ControlServer.instance()
  sire.fromXmlFile(cs, r'D:\code\sire\demo\demo_python\sirePaperDogRL\go2_can_work.xml')
  cs.init()
  simulator = sire.simulationLoop(cs)
  model = cs.model()
  simulator.simDuration = simulation_duration
  simulator.deltaT = simulation_dt
  simulator.ctrlT = ctrl_dt
  print("Model info:")
  print(f"Number of joints: {model.numJoints()}")
  print(f"Joint names: {[model.joint(i).name for i in range(model.numJoints())]}")
  print(f"Number of actuators: {model.numMotions()}")
  print(f"Actuator names: {[model.motion(i).name for i in range(model.numMotions())]}")
  # load policy
  policy = torch.jit.load(policy_path)
  timeRecord = []
  motionMpRecords = []
  motionMvRecords = []
  motionMaRecords = []
  motionMfRecords = []
  timeRecord2 = []
  targetPosRecords = []
  action = np.zeros(num_actions, dtype=np.float32)
  obs = np.zeros(num_obs, dtype=np.float32)
  fceLimit = [-23.7, 23.7, -23.7, 23.7, -35.55, 35.55,
              -23.7, 23.7, -23.7, 23.7, -35.55, 35.55,
              -23.7, 23.7, -23.7, 23.7, -35.55, 35.55,
              -23.7, 23.7, -23.7, 23.7, -35.55, 35.55,]
  while(not simulator.isTimeout() and not simulator.isEventListEmpty()):
    sim_time = simulator.simTime()
    motionMp = getMotionsMp(model)
    motionMv = getMotionsMv(model)
    motionMa = getMotionsMa(model)
    motionMf = getMotionsMf(model)
    timeRecord.append(sim_time)
    motionMpRecords.append(motionMp)
    motionMvRecords.append(motionMv)
    motionMaRecords.append(motionMa)
    motionMfRecords.append(motionMf)

    isCtrl = simulator.headerIsCtrl()
    if isCtrl:# and counter != 0:
      timeRecord2.append(sim_time)
      qj = motionMp
      dqj = motionMv
      quat = getBodyQuat(model, 1)
      bodyVa = getBodyVa(model, 1)
      lin_vel = bodyVa[:3]
      ang_vel = bodyVa[3:]

      qj = (qj - default_angles) * dof_pos_scale
      dqj = dqj * dof_vel_scale
      gravity_orientation = get_gravity_orientation(quat)
      lin_vel = lin_vel * lin_vel_scale
      ang_vel = ang_vel * ang_vel_scale

      # obs[:3] = lin_vel
      # obs[3:6] = ang_vel
      # obs[6:9] = gravity_orientation
      # obs[9:12] = cmd * cmd_scale
      # obs[12 : 12 + num_actions] = qj
      # obs[12 + num_actions : 12 + 2 * num_actions] = dqj
      # obs[12 + 2 * num_actions : 12 + 3 * num_actions] = action
      obs[:3] = ang_vel
      obs[3:6] = gravity_orientation
      obs[6:9] = cmd * cmd_scale
      obs[9 : 9 + num_actions] = qj
      obs[9 + num_actions : 9 + 2 * num_actions] = dqj
      obs[9 + 2 * num_actions : 9 + 3 * num_actions] = action
      obs_tensor = torch.from_numpy(obs).unsqueeze(0)
      # policy inference
      action = policy(obs_tensor).detach().numpy().squeeze()
      # transform action to target_dof_pos
      target_dof_pos = action * action_scale + default_angles
      targetPosRecords.append(target_dof_pos)
      # if sim_time < 0.04:
      #     target_dof_pos = default_angles.copy()
    # if isCtrl:
    tau1 = pd_control(target_dof_pos, motionMp, kps, np.zeros_like(kds), motionMv, kds)
    tau = clampVec(tau1, fceLimit)
    print(tau1, tau)
    for i in range(12):
      motion = model.motionPool()[i]
      if isinstance(motion, sire.ActuatorSISO):
        motion.desiredValue = tau[i]
    print(sim_time)
    # print(motionMp, target_dof_pos, motionMv, tau)
    simulator.handleContact()

  simulator.recordsContactCptInfo()
  displayInitJson = model.displayInitJson()
  result = simulator.recordsToJson()
  print("Simulation finished, records loaded")
  import meshcat
  vis = meshcat.Visualizer()
  resourcePath = "D:/code/sire/demo/demo_python/dogRL"
  # print(displayInitJson)
  sire.robotInit(model.numLinks(), resourcePath, displayInitJson, vis)
  sire.animateRobotByRecords(model.numLinks(), result, 1000, vis)
  input("按 Enter 键保存数据...")
  import pathlib
  currentDir = pathlib.Path(__file__).parent.resolve()
  dataPath = str(currentDir.resolve())
  motionDataPath = str((currentDir / "motion_data").resolve())
  # partpq = result['partPq']
  # bodyHeightRecord = np.zeros((2, len(partpq)))
  # for j in range(len(partpq)):
  #     bodyHeightRecord[0, j] = result['timeIndex'][j]
  #     bodyHeightRecord[1, j] = partpq[j][1][2] + 0.1
  # python save json
  with open(str(currentDir) + "/simulation_result.json", "w") as f:
      json.dump(result, f)
  # np.savetxt(dataPath + f"/body_height.csv", bodyHeightRecord.transpose(), delimiter=",")
  for i in range(model.numMotions()):
      motionRecord = np.zeros((7, len(timeRecord)))
      motionRecord[0, :] = timeRecord
      motionRecord[5, :len(timeRecord2)] = timeRecord2
      for j in range(len(timeRecord)):
          motionRecord[1, j] = motionMpRecords[j][i]
          motionRecord[2, j] = motionMvRecords[j][i]
          motionRecord[3, j] = motionMaRecords[j][i]
          motionRecord[4, j] = motionMfRecords[j][i]
      for j in range(len(timeRecord2)):
          motionRecord[6, j] = targetPosRecords[j][i]
      np.savetxt(motionDataPath + f"/motion_{i}.csv", motionRecord.transpose(), delimiter=",")

    # with mujoco.viewer.launch_passive(m, d) as viewer:
    #     # Close the viewer automatically after simulation_duration wall-seconds.
    #     start = time.time()
    #     while viewer.is_running() and time.time() - start < simulation_duration:
    #         # print(d.qpos[2])
    #         step_start = time.time()
    #         tau = pd_control(target_dof_pos, d.qpos[7:], kps, np.zeros_like(kds), d.qvel[6:], kds)
    #         d.ctrl[:] = tau
    #         # mj_step can be replaced with code that also evaluates
    #         # a policy and applies a control signal before stepping the physics.
    #         mujoco.mj_step(m, d)

    #         counter += 1
    #         if counter % control_decimation == 0:
    #             # Apply control signal here.
                
    #             # create observation
    #             qj = d.qpos[7:]
    #             dqj = d.qvel[6:]
    #             quat = d.qpos[3:7]
    #             lin_vel = d.qvel[:3]
    #             ang_vel = d.qvel[3:6]

    #             qj = (qj - default_angles) * dof_pos_scale

    #             dqj = dqj * dof_vel_scale
    #             gravity_orientation = get_gravity_orientation(quat)
    #             lin_vel = lin_vel * lin_vel_scale
    #             ang_vel = ang_vel * ang_vel_scale

    #             obs[:3] = lin_vel
    #             obs[3:6] = ang_vel
    #             obs[6:9] = gravity_orientation
    #             obs[9:12] = cmd * cmd_scale
    #             obs[12 : 12 + num_actions] = qj
    #             obs[12 + num_actions : 12 + 2 * num_actions] = dqj
    #             obs[12 + 2 * num_actions : 12 + 3 * num_actions] = action
    #             obs_tensor = torch.from_numpy(obs).unsqueeze(0)
    #             # policy inference
    #             action = policy(obs_tensor).detach().numpy().squeeze()
    #             # transform action to target_dof_pos
    #             target_dof_pos = action * action_scale + default_angles

    #         # Pick up changes to the physics state, apply perturbations, update options from GUI.
    #         viewer.sync()

    #         # Rudimentary time keeping, will drift relative to wall clock.
    #         time_until_next_step = m.opt.timestep - (time.time() - step_start)
    #         if time_until_next_step > 0:
    #             time.sleep(time_until_next_step)