import os
import sys
from pathlib import Path

import numpy as np
import torch
import yaml
import sire

LEGGED_GYM_ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
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
    """Body-origin [linear, angular] velocity expressed in body axes."""
    link = model.link(i)
    return np.asarray(sire.vs2bodyVa(link.getPq(), link.getVs()), dtype=np.float64)

def assignTau(model, tau):
    for i in range(len(tau)):
        fce = model.force(i)
        if isinstance(fce, sire.SingleComponentForce):
            fce.fce = tau[i]


def build_dof_motion_map(model, joint_names):
    """Return motionPool indices in the policy's named DOF order."""
    joint_to_motion = {
        model.jointPool()[i].name: i for i in range(model.numMotions())
    }
    missing = [name for name in joint_names if name not in joint_to_motion]
    if missing:
        raise RuntimeError(
            f"Policy joints missing from Sire model: {missing}; "
            f"available={list(joint_to_motion)}"
        )
    return np.asarray([joint_to_motion[name] for name in joint_names], dtype=np.int32)


def initialize_training_state(model, motion_indices, default_angles, base_pos, base_quat):
    """Apply the deterministic counterpart of SireRLGym's episode reset."""
    motion_mps = np.asarray(sire.getMotionMps(model), dtype=np.float64)
    motion_mvs = np.zeros(model.numMotions(), dtype=np.float64)
    motion_mps[motion_indices] = default_angles
    sire.setMotionMps(model, motion_mps.tolist())
    sire.setMotionMvs(model, motion_mvs.tolist())
    model.forwardKinematics()
    model.forwardKinematicsVel()

    base = model.link(1)
    base.pq = np.concatenate([base_pos, base_quat])
    base.vs = np.zeros(6, dtype=np.float64)
    model.forwardKinematics()
    model.forwardKinematicsVel()

if __name__ == "__main__":
    # get config file name from command line
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("config_file", type=str, help="config file name in the config folder")
    parser.add_argument(
        "--policy",
        type=str,
        default=None,
        help="Override policy_path from YAML with a TorchScript policy file.",
    )
    parser.add_argument(
        "--cmd",
        type=float,
        nargs=3,
        default=None,
        metavar=("VX", "VY", "YAW"),
        help="Override cmd_init from YAML.",
    )
    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="Run the physics/policy regression without starting MeshCat.",
    )
    args = parser.parse_args()
    config_file = args.config_file
    with open(f"{LEGGED_GYM_ROOT_DIR}/{config_file}", "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        policy_path = config["policy_path"].replace("{LEGGED_GYM_ROOT_DIR}", LEGGED_GYM_ROOT_DIR)
        if args.policy is not None:
            policy_path = str(Path(args.policy).expanduser().resolve())
        xml_path = config["xml_path"].replace("{LEGGED_GYM_ROOT_DIR}", LEGGED_GYM_ROOT_DIR)

        simulation_duration = config["simulation_duration"]
        simulation_dt = config["simulation_dt"]
        ctrl_dt = config["ctrl_dt"]
        control_decimation = config["control_decimation"]

        kps = np.array(config["kps"], dtype=np.float32)
        kds = np.array(config["kds"], dtype=np.float32)

        default_angles = np.array(config["default_angles"], dtype=np.float32)
        joint_names = list(config["joint_names"])
        torque_limits = np.array(config["torque_limits"], dtype=np.float32)
        base_pos = np.array(config["base_pos"], dtype=np.float64)
        base_quat = np.array(config["base_quat"], dtype=np.float64)

        lin_vel_scale = config["lin_vel_scale"]
        ang_vel_scale = config["ang_vel_scale"]
        dof_pos_scale = config["dof_pos_scale"]
        dof_vel_scale = config["dof_vel_scale"]
        action_scale = config["action_scale"]
        cmd_scale = np.array(config["cmd_scale"], dtype=np.float32)

        num_actions = config["num_actions"]
        num_obs = config["num_obs"]
        
        cmd = np.array(
            args.cmd if args.cmd is not None else config["cmd_init"],
            dtype=np.float32,
        )

    # define context variables
    action = np.zeros(num_actions, dtype=np.float32)
    target_dof_pos = default_angles.copy()
    obs = np.zeros(num_obs, dtype=np.float32)

    # Load robot model
    # print(sire.pq2tfmatrix([0,0,0,1,0,0,0]))
    sim = sire.Simulator()
    sire.fromXmlFile(sim, xml_path)
    sim.init()
    simulator = sim.simulationLoop()
    model = sim.model()
    simulator.simDuration = simulation_duration
    simulator.deltaT = simulation_dt
    simulator.ctrlT = ctrl_dt
    expected_ctrl_dt = simulation_dt * control_decimation
    if not np.isclose(ctrl_dt, expected_ctrl_dt, rtol=0.0, atol=1e-12):
        raise ValueError(
            f"ctrl_dt={ctrl_dt} must equal simulation_dt*control_decimation="
            f"{expected_ctrl_dt}"
        )
    motion_indices = build_dof_motion_map(model, joint_names)
    initialize_training_state(
        model, motion_indices, default_angles, base_pos, base_quat
    )
    print("Model info:")
    print(f"Number of joints: {model.numJoints()}")
    print(f"Joint names: {[model.joint(i).name for i in range(model.numJoints())]}")
    print(f"Number of actuators: {model.numMotions()}")
    print(f"Actuator names: {[model.motion(i).name for i in range(model.numMotions())]}")
    # load policy
    policy = torch.jit.load(policy_path)
    action = np.zeros(num_actions, dtype=np.float32)
    obs = np.zeros(num_obs, dtype=np.float32)
    next_ctrl_time = 0.0
    control_step = 0
    min_base_z = float(model.link(1).getPq()[2])
    while(not simulator.isTimeout() and not simulator.isEventListEmpty()):
        sim_time = simulator.simTime()
        isCtrl = sim_time >= next_ctrl_time - 1e-12
        # Reorder XML motionPool state into the exact DOF order used by PPO.
        motionMp = getMotionsMp(model)[motion_indices]
        motionMv = getMotionsMv(model)[motion_indices]

        if isCtrl:
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
            with torch.no_grad():
                action = policy(obs_tensor).numpy().squeeze()
            if action.shape != (num_actions,) or not np.all(np.isfinite(action)):
                raise RuntimeError(
                    f"Invalid policy action at t={sim_time:.6f}: {action}"
                )
            # transform action to target_dof_pos
            target_dof_pos = action * action_scale + default_angles
            next_ctrl_time += ctrl_dt
            control_step += 1
            # if sim_time < 0.04:
            #     target_dof_pos = default_angles.copy()
        # if isCtrl:
        tau = pd_control(target_dof_pos, motionMp, kps, np.zeros_like(kds), motionMv, kds)
        tau = np.clip(tau, -torque_limits, torque_limits)
        for dof_i, motion_i in enumerate(motion_indices):
          motion = model.motionPool()[int(motion_i)]
          if isinstance(motion, sire.ActuatorSISO):
            motion.desiredValue = float(tau[dof_i])
        base_pq = np.asarray(model.link(1).getPq(), dtype=np.float64)
        min_base_z = min(min_base_z, float(base_pq[2]))
        if not np.all(np.isfinite(base_pq)):
            raise RuntimeError(f"Non-finite base state at t={sim_time:.6f}: {base_pq}")
        if isCtrl and (control_step == 1 or control_step % 50 == 0):
            print(
                f"sim_time={sim_time:.3f}s base_xyz="
                f"[{base_pq[0]:.3f}, {base_pq[1]:.3f}, {base_pq[2]:.3f}]"
            )
        # print(motionMp, target_dof_pos, motionMv, tau)
        simulator.handleContact()
        # part_as_before = []
        # for i in range(model.numLinks()):
        #     part = model.partPool()[i]
        #     part_as_before.append(part.getAs())
        # model.forwardDynamics()
        # part_as_after = []
        # for i in range(model.numLinks()):
        #     part = model.partPool()[i]
        #     part_as_after.append(part.getAs())
        # for i in range(12):
        #   motion = model.motionPool()[i].updA()
        # print(isCtrl, sim_time, target_dof_pos, obs)

    simulator.recordsContactCptInfo()
    result = simulator.recordsToJson()
    final_pq = np.asarray(model.link(1).getPq(), dtype=np.float64)
    print(
        f"Simulation finished at t={simulator.simTime():.3f}s "
        f"base_xyz=[{final_pq[0]:.3f}, {final_pq[1]:.3f}, {final_pq[2]:.3f}] "
        f"min_base_z={min_base_z:.3f}"
    )
    resourcePath = LEGGED_GYM_ROOT_DIR
    try:
        if args.no_viz:
            raise ModuleNotFoundError("disabled by --no-viz")
        import meshcat
        displayInitJson = sim.displayInitJson()
        vis = meshcat.Visualizer()
        # print(displayInitJson)
        sire.robotInit(model.numLinks(), resourcePath, displayInitJson, vis)
        sire.animateRobotByRecords(model.numLinks(), result, 1000, vis)
        print(f"Meshcat visualizer running at: {vis.url()}")
        # Keep the process (and the meshcat server) alive so the animation
        # stays viewable in the browser. No CSV is written to disk anymore.
        if sys.stdin.isatty():
            input("按 Enter 键退出可视化...")
    except ModuleNotFoundError as e:
        print(f"[viz skipped] meshcat visualization unavailable: {e}")

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
