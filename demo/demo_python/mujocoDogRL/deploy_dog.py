import time

import mujoco.viewer
import mujoco
import numpy as np
import torch
import yaml

paused = False
def key_callback(keycode):
    if chr(keycode) == ' ':
        global paused
        paused = not paused

LEGGED_GYM_ROOT_DIR = "D:/code/sire/demo/demo_python/mujocoDogRL"
def get_gravity_orientation(quaternion):
    qw = quaternion[0]
    qx = quaternion[1]
    qy = quaternion[2]
    qz = quaternion[3]

    gravity_orientation = np.zeros(3)

    gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)

    return gravity_orientation


def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd


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
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    m.opt.timestep = simulation_dt
    
    # 添加调试代码到deploy_dog.py中
    print("Model info:")
    print(f"Number of joints: {m.njnt}")
    print(f"Joint names: {[m.joint(i).name for i in range(m.njnt)]}")
    print(f"Number of actuators: {m.nu}")
    print(f"Actuator names: {[m.actuator(i).name for i in range(m.nu)]}")
    timeRecord = []
    motionMpRecords = []
    motionMvRecords = []
    motionMaRecords = []
    motionMfRecords = []
    # load policy
    policy = torch.jit.load(policy_path)
    action = np.zeros(num_actions, dtype=np.float32)
    obs = np.zeros(num_obs, dtype=np.float32)

    time_to_pause = [0, 0.1, 0.164, 0.166, 0.168]
    paused = True
    with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:
        # Close the viewer automatically after simulation_duration wall-seconds.
        start = time.time()
        while viewer.is_running() and d.time < simulation_duration:
            # print(d.qpos[2])

            # mj_step can be replaced with code that also evaluates
            # a policy and applies a control signal before stepping the physics.
            # print(d.time, target_dof_pos, obs)
            if not paused:
                step_start = time.time()
                tau = pd_control(target_dof_pos, d.qpos[7:], kps, np.zeros_like(kds), d.qvel[6:], kds)
                # d.ctrl[:] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

                d.ctrl[:] = tau
                mujoco.mj_step(m, d)
                timeRecord.append(d.time)
                motionMpRecords.append(d.qpos[7:].copy())
                motionMvRecords.append(d.qvel[6:].copy())
                motionMaRecords.append(d.qacc[6:].copy())
                motionMfRecords.append(tau.copy())

                counter += 1

                if counter % control_decimation == 0:
                    # Apply control signal here.

                    # create observation
                    qj = d.qpos[7:]
                    dqj = d.qvel[6:]
                    quat = d.qpos[3:7]
                    lin_vel = d.qvel[:3]
                    ang_vel = d.qvel[3:6]

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

                # Pick up changes to the physics state, apply perturbations, update options from GUI.
                viewer.sync()
                print(d.time)
                # print(d.time, any([abs(num - d.time) < 1e-8 for num in time_to_pause]))
                if any([abs(num - d.time) < 1e-8 for num in time_to_pause]):
                    paused = True

            # Rudimentary time keeping, will drift relative to wall clock.
            time_until_next_step = d.time + m.opt.timestep - (time.time() - start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
    import pathlib
    currentDir = pathlib.Path(__file__).parent.resolve()
    dataPath = str((currentDir / "motion_data").resolve())
    for i in range(m.nv - 6):
        motionRecord = np.zeros((5, len(timeRecord)))
        motionRecord[0, :] = timeRecord
        for j in range(len(timeRecord)):
            motionRecord[1, j] = motionMpRecords[j][i]
            motionRecord[2, j] = motionMvRecords[j][i]
            motionRecord[3, j] = motionMaRecords[j][i]
            motionRecord[4, j] = motionMfRecords[j][i]
        np.savetxt(dataPath + f"/motion_{i}.csv", motionRecord.transpose(), delimiter=",")