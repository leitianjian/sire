"""Run a Sire-trained locomotion actor on a Unitree Go2 low-level controller.

Copy this file, ``deploy_go2.yaml``, and the exported TorchScript policy to the
robot controller, then run::

    python3 deploy_go2.py eth0

Use ``--config`` if the YAML file is stored elsewhere. The controller keeps the
same joint and observation ordering used by the Sire RLGym Go2 task.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import struct
import time

import numpy as np
import torch
import yaml

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_, unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_ as LowCmdGo
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_ as LowStateGo
from unitree_sdk2py.utils.crc import CRC


POS_STOP_F = 2.146e9
VEL_STOP_F = 16000.0


class Config:
    def __init__(self, file_path: str | Path):
        path = Path(file_path).expanduser().resolve()
        with path.open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)

        self.control_dt = float(config["control_dt"])
        self.joint2motor_idx = list(config["joint2motor_idx"])
        self.lowcmd_topic = str(config["lowcmd_topic"])
        self.lowstate_topic = str(config["lowstate_topic"])
        self.policy_path = path.parent / config["policy_path"]
        self.kps = np.asarray(config["kps"], dtype=np.float32)
        self.kds = np.asarray(config["kds"], dtype=np.float32)
        self.default_angles = np.asarray(config["default_angles"], dtype=np.float32)
        self.obs_scales_ang_vel = float(config["obs_scales_ang_vel"])
        self.obs_scales_dof_pos = float(config["obs_scales_dof_pos"])
        self.obs_scales_dof_vel = float(config["obs_scales_dof_vel"])
        self.command_scale = np.asarray(config["command_scale"], dtype=np.float32)
        self.action_scale = float(config["action_scale"])
        self.num_actions = int(config["num_actions"])
        self.num_obs = int(config["num_obs"])
        self.max_cmd = np.asarray(config["max_cmd"], dtype=np.float32)
        self.clip_observations = float(config["clip_observations"])
        self.clip_actions = float(config["clip_actions"])
        self._validate()

    def _validate(self) -> None:
        if self.num_actions != 12:
            raise ValueError(f"Go2 deployment requires 12 actions, got {self.num_actions}")
        if self.num_obs != 45:
            raise ValueError(f"This observation layout requires 45 values, got {self.num_obs}")
        for name in ("joint2motor_idx", "kps", "kds", "default_angles"):
            if len(getattr(self, name)) != self.num_actions:
                raise ValueError(f"{name} must contain {self.num_actions} values")
        if len(set(self.joint2motor_idx)) != self.num_actions:
            raise ValueError("joint2motor_idx must contain 12 unique motor indices")
        if self.command_scale.shape != (3,) or self.max_cmd.shape != (3,):
            raise ValueError("command_scale and max_cmd must each contain 3 values")
        if not self.policy_path.is_file():
            raise FileNotFoundError(f"TorchScript policy not found: {self.policy_path}")


class KeyMap:
    start = 2
    select = 3
    A = 8


class RemoteController:
    def __init__(self):
        self.lx = 0.0
        self.ly = 0.0
        self.rx = 0.0
        self.ry = 0.0
        self.button = [0] * 16

    def set(self, data) -> None:
        keys = struct.unpack("H", data[2:4])[0]
        for index in range(16):
            self.button[index] = (keys & (1 << index)) >> index
        self.lx = struct.unpack("f", data[4:8])[0]
        self.rx = struct.unpack("f", data[8:12])[0]
        self.ry = struct.unpack("f", data[12:16])[0]
        self.ly = struct.unpack("f", data[20:24])[0]


def get_gravity_orientation(quaternion) -> np.ndarray:
    """Return projected gravity from a Unitree w/x/y/z quaternion."""
    qw, qx, qy, qz = quaternion
    return np.asarray(
        [
            2 * (-qz * qx + qw * qy),
            -2 * (qz * qy + qw * qx),
            1 - 2 * (qw * qw + qz * qz),
        ],
        dtype=np.float32,
    )


def create_zero_cmd(cmd) -> None:
    for motor in cmd.motor_cmd:
        motor.q = 0.0
        motor.dq = 0.0
        motor.kp = 0.0
        motor.kd = 0.0
        motor.tau = 0.0


def create_damping_cmd(cmd) -> None:
    for motor in cmd.motor_cmd:
        motor.q = 0.0
        motor.dq = 0.0
        motor.kp = 0.0
        motor.kd = 8.0
        motor.tau = 0.0


def init_cmd_go2(cmd: LowCmdGo) -> None:
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0
    for index in range(12):
        cmd.motor_cmd[index].mode = 0x0A
        cmd.motor_cmd[index].q = POS_STOP_F
        cmd.motor_cmd[index].dq = VEL_STOP_F
        cmd.motor_cmd[index].kp = 0.0
        cmd.motor_cmd[index].kd = 0.0
        cmd.motor_cmd[index].tau = 0.0


class Controller:
    def __init__(self, config: Config) -> None:
        self.config = config
        self.remote_controller = RemoteController()
        self.use_remote_controller = True

        self.policy = torch.jit.load(str(config.policy_path), map_location="cpu").eval()
        self._warm_up()

        self.qj = np.zeros(config.num_actions, dtype=np.float32)
        self.dqj = np.zeros(config.num_actions, dtype=np.float32)
        self.action = np.zeros(config.num_actions, dtype=np.float32)
        self.obs = np.zeros(config.num_obs, dtype=np.float32)
        self.cmd = np.asarray([0.8, 0.0, 0.0], dtype=np.float32)

        self.low_cmd = unitree_go_msg_dds__LowCmd_()
        self.low_state = unitree_go_msg_dds__LowState_()
        self.lowcmd_publisher = ChannelPublisher(config.lowcmd_topic, LowCmdGo)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber(config.lowstate_topic, LowStateGo)
        self.lowstate_subscriber.Init(self.low_state_handler, 10)

        self.wait_for_low_state()
        init_cmd_go2(self.low_cmd)
        self._release_sport_mode()

    def _warm_up(self) -> None:
        sample = torch.zeros((1, self.config.num_obs), dtype=torch.float32)
        with torch.inference_mode():
            for _ in range(10):
                result = self.policy(sample)
        result = result[0] if isinstance(result, tuple) else result
        if tuple(result.shape) != (1, self.config.num_actions):
            raise ValueError(
                "Policy output shape mismatch: "
                f"expected (1, {self.config.num_actions}), got {tuple(result.shape)}"
            )
        print("Network has been warmed up.")

    def _release_sport_mode(self) -> None:
        sport_client = SportClient()
        sport_client.SetTimeout(5.0)
        sport_client.Init()

        motion_switcher_client = MotionSwitcherClient()
        motion_switcher_client.SetTimeout(5.0)
        motion_switcher_client.Init()

        _, result = motion_switcher_client.CheckMode()
        while result["name"]:
            sport_client.StandDown()
            motion_switcher_client.ReleaseMode()
            _, result = motion_switcher_client.CheckMode()
            time.sleep(1.0)
        print("Released sport mode.")

    def wait_for_low_state(self) -> None:
        while self.low_state.tick == 0:
            time.sleep(self.config.control_dt)
        print("Successfully connected to the robot.")

    def low_state_handler(self, msg: LowStateGo) -> None:
        self.low_state = msg
        self.remote_controller.set(self.low_state.wireless_remote)

    def send_cmd(self, cmd: LowCmdGo) -> None:
        cmd.crc = CRC().Crc(cmd)
        self.lowcmd_publisher.Write(cmd)

    def zero_torque_state(self) -> None:
        print("Enter zero torque state; press START to continue.")
        while self.remote_controller.button[KeyMap.start] != 1:
            create_zero_cmd(self.low_cmd)
            self.send_cmd(self.low_cmd)
            time.sleep(self.config.control_dt)

    def move_to_default_pos(self) -> None:
        print("Moving to default position.")
        num_steps = int(2.0 / self.config.control_dt)
        initial = np.asarray(
            [self.low_state.motor_state[index].q for index in self.config.joint2motor_idx],
            dtype=np.float32,
        )
        for step in range(num_steps):
            alpha = step / num_steps
            target = initial * (1.0 - alpha) + self.config.default_angles * alpha
            self._write_position_command(target, np.full(12, 40.0), np.full(12, 0.6))
            time.sleep(self.config.control_dt)

    def default_pos_state(self) -> None:
        print("Holding default position; press A to run the policy.")
        while self.remote_controller.button[KeyMap.A] != 1:
            self._write_position_command(
                self.config.default_angles, np.full(12, 40.0), np.full(12, 0.6)
            )
            time.sleep(self.config.control_dt)

    def _write_position_command(self, target, kps, kds) -> None:
        for index, motor_idx in enumerate(self.config.joint2motor_idx):
            motor = self.low_cmd.motor_cmd[motor_idx]
            motor.q = float(target[index])
            motor.dq = 0.0
            motor.kp = float(kps[index])
            motor.kd = float(kds[index])
            motor.tau = 0.0
        self.send_cmd(self.low_cmd)

    def run(self) -> None:
        for index, motor_idx in enumerate(self.config.joint2motor_idx):
            self.qj[index] = self.low_state.motor_state[motor_idx].q
            self.dqj[index] = self.low_state.motor_state[motor_idx].dq

        ang_vel = np.asarray(self.low_state.imu_state.gyroscope, dtype=np.float32)
        ang_vel *= self.config.obs_scales_ang_vel
        gravity = get_gravity_orientation(self.low_state.imu_state.quaternion)

        if self.use_remote_controller:
            self.cmd[:] = (
                self.remote_controller.ly,
                -self.remote_controller.lx,
                -self.remote_controller.rx,
            )
        self.cmd = np.clip(self.cmd, -1.0, 1.0) * self.config.max_cmd

        self.obs[0:3] = ang_vel
        self.obs[3:6] = gravity
        self.obs[6:9] = self.cmd * self.config.command_scale
        self.obs[9:21] = (
            self.qj - self.config.default_angles
        ) * self.config.obs_scales_dof_pos
        self.obs[21:33] = self.dqj * self.config.obs_scales_dof_vel
        self.obs[33:45] = self.action
        np.clip(
            self.obs,
            -self.config.clip_observations,
            self.config.clip_observations,
            out=self.obs,
        )

        with torch.inference_mode():
            result = self.policy(torch.from_numpy(self.obs).unsqueeze(0))
        result = result[0] if isinstance(result, tuple) else result
        self.action = result.detach().cpu().numpy().reshape(-1).astype(np.float32)
        np.clip(
            self.action,
            -self.config.clip_actions,
            self.config.clip_actions,
            out=self.action,
        )
        target = self.config.default_angles + self.action * self.config.action_scale
        self._write_position_command(target, self.config.kps, self.config.kds)
        time.sleep(self.config.control_dt)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("net", help="network interface connected to the Go2, e.g. eth0")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).with_name("deploy_go2.yaml"),
        help="deployment YAML (default: deploy_go2.yaml beside this script)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = Config(args.config)
    ChannelFactoryInitialize(0, args.net)
    controller = Controller(config)

    try:
        controller.zero_torque_state()
        controller.move_to_default_pos()
        controller.default_pos_state()
        while controller.remote_controller.button[KeyMap.select] != 1:
            controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        create_damping_cmd(controller.low_cmd)
        controller.send_cmd(controller.low_cmd)
        print("Exit")


if __name__ == "__main__":
    main()
