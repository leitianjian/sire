from pathlib import Path
import hashlib
import zipfile

import numpy as np
import yaml

LEGGED_GYM_ROOT_DIR = str(Path(__file__).resolve().parent)


class Config:
    def __init__(self, file_path) -> None:
        with open(file_path, "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        self.control_dt = config["control_dt"]
        self.joint2motor_idx = config["joint2motor_idx"]
        self.msg_type = config["msg_type"]
        self.imu_type = config["imu_type"]
        self.lowcmd_topic = config["lowcmd_topic"]
        self.lowstate_topic = config["lowstate_topic"]
        self.policy_path = config["policy_path"].replace("{LEGGED_GYM_ROOT_DIR}", LEGGED_GYM_ROOT_DIR)
        self.kps = np.array(config["kps"], dtype=np.float32)
        self.kds = np.array(config["kds"], dtype=np.float32)
        self.default_angles = np.array(config["default_angles"], dtype=np.float32)
        self.obs_scales_ang_vel = config["obs_scales_ang_vel"]
        self.obs_scales_dof_pos = config["obs_scales_dof_pos"]
        self.obs_scales_dof_vel = config["obs_scales_dof_vel"]
        self.command_scale = config["command_scale"]
        self.action_scale = config["action_scale"]
        self.num_actions = config["num_actions"]
        self.num_obs = config["num_obs"]
        self.max_cmd = np.array(config["max_cmd"], dtype=np.float32)
        self.clip_observations = config["clip_observations"]
        self.clip_actions = config["clip_actions"]
        self.policy_joint_names = config["policy_joint_names"]

        # Read-only input-contract checks. Config is constructed BEFORE the
        # existing main calls ChannelFactoryInitialize; no SDK import here.
        self._validate_policy_contract(config["policy_sha256"])

    def _validate_policy_contract(self, expected_sha256):
        path = Path(self.policy_path)
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected_sha256:
            raise ValueError("Policy SHA256 mismatch; review the selected JIT and deployment config before connecting.")
        with zipfile.ZipFile(str(path)) as archive:
            names = [name for name in archive.namelist() if name.endswith("/extra/deployment.yaml")]
            if len(names) != 1:
                raise ValueError("Expected the reviewed Sire actor JIT with embedded deployment.yaml.")
            profile = yaml.safe_load(archive.read(names[0]))
        if self.num_obs != 45 or self.num_actions != 12:
            raise ValueError("Go2 Sire actor interface must be 45 observations / 12 actions.")
        if self.msg_type != "go" or self.imu_type != "torso":
            raise ValueError("This deployment uses the reference Go2 torso IMU interface.")
        if self.policy_joint_names != profile["policy_joint_names"]:
            raise ValueError("Policy joint order differs from JIT metadata.")
        # Verified against the installed Unitree SDK LegID constants, not the
        # MuJoCo qpos order. Motor order is FR, FL, RR, RL; policy is grouped.
        motor_names = [f"{leg}_{joint}_joint" for leg in ("FR", "FL", "RR", "RL")
                       for joint in ("hip", "thigh", "calf")]
        if self.joint2motor_idx != [motor_names.index(name) for name in self.policy_joint_names]:
            raise ValueError("joint2motor_idx does not match the reviewed Go2 named motor mapping.")

        def same(name, actual, expected):
            actual = np.asarray(actual)
            expected = np.asarray(expected)
            if actual.shape != expected.shape or not np.isfinite(actual).all() or not np.allclose(actual, expected, rtol=1e-6, atol=1e-7):
                raise ValueError(f"{name} differs from the selected policy's interface: {actual} != {expected}")

        env = profile["env_cfg"]
        control = env["control"]
        norm = env["normalization"]
        scales = norm["obs_scales"]
        if control["control_type"] != "P":
            raise ValueError("Expected a position-PD policy.")
        same("control_dt", self.control_dt, env["sim"]["dt"] * control["decimation"])
        same("default_angles", self.default_angles, [env["init_state"]["default_joint_angles"][name] for name in self.policy_joint_names])
        same("kps", self.kps, [control["stiffness"]["joint"]] * 12)
        same("kds", self.kds, [control["damping"]["joint"]] * 12)
        same("command_scale", self.command_scale, [scales["lin_vel"], scales["lin_vel"], scales["ang_vel"]])
        same("obs_scales_ang_vel", self.obs_scales_ang_vel, scales["ang_vel"])
        same("obs_scales_dof_pos", self.obs_scales_dof_pos, scales["dof_pos"])
        same("obs_scales_dof_vel", self.obs_scales_dof_vel, scales["dof_vel"])
        same("action_scale", self.action_scale, control["action_scale"])
        same("clip_observations", self.clip_observations, norm["clip_observations"])
        same("clip_actions", self.clip_actions, norm["clip_actions"])
        same("max_cmd", self.max_cmd, [1.0, 1.0, 1.0])

