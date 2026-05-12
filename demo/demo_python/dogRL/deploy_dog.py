import time

import numpy as np
import torch
import yaml
import sire

LEGGED_GYM_ROOT_DIR = "D:/code/sire/demo/demo_python/dogRL"
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

def getBodyQuat(model, i):
    return np.array(model.link(i).getPq()[3:])

def getBodyVa(model, i):
    return np.array(sire.vs2va(model.link(i).getVs(), model.link(i).getPq()))

def assignTau(model, tau):
    for i in range(len(tau)):
        fce = model.force(i)
        if isinstance(fce, sire.SingleComponentForce):
            fce.fce = tau[i]

if __name__ == "__main__":
    # get config file name from command line
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("config_file", type=str, help="config file name in the config folder")
    args = parser.parse_args()
    config_file = args.config_file
    with open(f"{LEGGED_GYM_ROOT_DIR}/{config_file}", "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        policy_path = config["policy_path"].replace("{LEGGED_GYM_ROOT_DIR}", LEGGED_GYM_ROOT_DIR)
        xml_path = config["xml_path"].replace("{LEGGED_GYM_ROOT_DIR}", LEGGED_GYM_ROOT_DIR)

        simulation_duration = config["simulation_duration"]
        simulation_dt = config["simulation_dt"]
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

    counter = 0

    # Load robot model
    # print(sire.pq2tfmatrix([0,0,0,1,0,0,0]))
    cs = sire.ControlServer.instance()
    sire.fromXmlFile(cs, 'D:/code/sire/demo/demo_python/dogRL/go2_modified.xml')
    cs.init()
    simulator = sire.simulationLoop(cs)
    model = cs.model()
    simulator.simDuration = 10
    simulator.ctrlT = 0.002
    print(sire.toXmlString(cs))
    
    # 添加调试代码到deploy_dog.py中
    print("Model info:")
    print(f"Number of joints: {model.numJoints()}")
    print(f"Joint names: {[model.joint(i).name for i in range(model.numJoints())]}")
    print(f"Number of actuators: {model.numMotions()}")
    print(f"Actuator names: {[model.motion(i).name for i in range(model.numMotions())]}")
    counter = 0
    # load policy
    policy = torch.jit.load(policy_path)
    while(not simulator.isTimeout() and not simulator.isEventListEmpty()):
        isCtrl = simulator.integrate()
        sim_time = simulator.simTime()
        motionMp = getMotionsMp(model)
        motionMv = getMotionsMv(model)
        counter += 1
        if isCtrl:# and counter != 0:
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

        # if isCtrl:
        tau = pd_control(target_dof_pos, motionMp, kps, np.zeros_like(kds), motionMv, kds)
        for i in range(12):
          motion = model.motionPool()[i]
          if isinstance(motion, sire.ActuatorSISO):
            motion.desiredValue = tau[i]
        # print(sim_time)
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
    input("按 Enter 键退出程序...")
