"""Deploy a Go2 policy and record synchronized paper-evaluation telemetry.

This additive wrapper imports ``deploy_go2.py`` and leaves the original
deployment, export, and build scripts unchanged.  Example:

    python3 deploy_go2_paper.py eth0 --method sire_full \
        --task velocity_sweep --trial-id 01

It writes append-only JSONL from a background thread plus a metadata sidecar.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import queue
import re
import socket
import threading
import time
from typing import Any, Dict, Iterable, Optional

import numpy as np
import torch
import yaml

import deploy_go2 as base

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_


SCHEMA_VERSION = 1
FOOT_NAMES_UNITREE = ("FR", "FL", "RR", "RL")
FOOT_NAMES_POLICY = ("FL", "FR", "RL", "RR")
FOOT_UNITREE_TO_POLICY = (1, 0, 3, 2)


def _safe_name(value: str) -> str:
    result = re.sub(r"[^A-Za-z0-9_.-]+", "-", value.strip())
    return result.strip("-._") or "unnamed"


def _float_list(values: Iterable[Any]) -> list:
    return [float(value) for value in values]


def _int_list(values: Iterable[Any]) -> list:
    return [int(value) for value in values]


def _flatten(values: Any) -> list:
    if isinstance(values, (list, tuple, np.ndarray)):
        result = []
        for value in values:
            result.extend(_flatten(value))
        return result
    return [float(values)]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_json(path: Path, value: Dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    temporary.replace(path)


class AsyncJsonlLogger:
    """Bounded, non-blocking producer with one disk-writer thread."""

    _STOP = object()

    def __init__(self, path: Path, queue_size: int, flush_every: int) -> None:
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._queue = queue.Queue(maxsize=queue_size)
        self._flush_every = max(1, flush_every)
        self._error = None  # type: Optional[BaseException]
        self.written = 0
        self.dropped = 0
        self._thread = threading.Thread(
            target=self._writer, name="paper-jsonl-writer", daemon=True
        )
        self._thread.start()

    def submit(self, sample: Dict[str, Any]) -> None:
        if self._error is not None:
            raise RuntimeError("paper log writer failed") from self._error
        try:
            self._queue.put_nowait(sample)
        except queue.Full:
            self.dropped += 1

    def _writer(self) -> None:
        try:
            with self.path.open("x", encoding="utf-8", buffering=1024 * 1024) as stream:
                pending = 0
                while True:
                    item = self._queue.get()
                    try:
                        if item is self._STOP:
                            break
                        stream.write(json.dumps(item, separators=(",", ":"), allow_nan=False))
                        stream.write("\n")
                        self.written += 1
                        pending += 1
                        if pending >= self._flush_every:
                            stream.flush()
                            pending = 0
                    finally:
                        self._queue.task_done()
                stream.flush()
        except BaseException as error:
            self._error = error

    def close(self) -> None:
        while True:
            try:
                self._queue.put(self._STOP, timeout=0.5)
                break
            except queue.Full:
                if self._error is not None:
                    raise RuntimeError("paper log writer failed") from self._error
        self._thread.join()
        if self._error is not None:
            raise RuntimeError("paper log writer failed") from self._error


class ContactHysteresis:
    def __init__(self, on_threshold: float, off_threshold: float) -> None:
        if off_threshold > on_threshold:
            raise ValueError("contact-off must not exceed contact-on")
        self.on_threshold = on_threshold
        self.off_threshold = off_threshold
        self.state = [False] * 4

    def update(self, forces: Iterable[float]) -> list:
        for index, force in enumerate(forces):
            threshold = self.off_threshold if self.state[index] else self.on_threshold
            self.state[index] = float(force) >= threshold
        return [int(value) for value in self.state]


class PaperController(base.Controller):
    def __init__(self, config: Any, logger: AsyncJsonlLogger,
                 contact_on: float, contact_off: float) -> None:
        self.paper_logger = logger
        self._sport_state = None
        self._sport_receive_ns = 0
        self._last_sample_ns = 0
        self._sequence = 0
        self._scheduled_start_ns = 0
        self._skipped_deadlines = 0
        self._total_skipped_deadlines = 0
        self._contacts = ContactHysteresis(contact_on, contact_off)
        super().__init__(config)
        self._sport_subscriber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self._sport_subscriber.Init(self._sport_state_handler, 10)

    def _sport_state_handler(self, msg: SportModeState_) -> None:
        self._sport_state = msg
        self._sport_receive_ns = time.monotonic_ns()

    def set_schedule_context(self, scheduled_start_ns: int,
                             skipped_deadlines: int) -> None:
        self._scheduled_start_ns = scheduled_start_ns
        self._skipped_deadlines = skipped_deadlines
        self._total_skipped_deadlines += skipped_deadlines

    def _write_targets(self, target: np.ndarray) -> None:
        for index, motor_index in enumerate(self.config.joint2motor_idx):
            motor = self.low_cmd.motor_cmd[motor_index]
            motor.q = float(target[index])
            motor.dq = 0.0
            motor.kp = float(self.config.kps[index])
            motor.kd = float(self.config.kds[index])
            motor.tau = 0.0
        self.send_cmd(self.low_cmd)

    def _sport_sample(self, now_ns: int) -> Dict[str, Any]:
        state = self._sport_state
        if state is None:
            return {"valid": False, "age_ms": None}
        stamp = state.stamp
        return {
            "valid": True,
            "age_ms": (now_ns - self._sport_receive_ns) * 1.0e-6,
            "stamp_sec": int(stamp.sec),
            "stamp_nsec": int(stamp.nanosec),
            "position": _float_list(state.position),
            "velocity": _float_list(state.velocity),
            "yaw_speed": float(state.yaw_speed),
            "body_height": float(state.body_height),
            "foot_force": _int_list(state.foot_force),
            "foot_position_body": _flatten(state.foot_position_body),
            "foot_speed_body": _flatten(state.foot_speed_body),
            "mode": int(state.mode),
            "gait_type": int(state.gait_type),
        }

    def run(self) -> None:
        cycle_start_ns = time.monotonic_ns()
        wall_time_ns = time.time_ns()
        period_ns = max(1, int(round(float(self.config.control_dt) * 1.0e9)))
        scheduled_start_ns = (
            self._scheduled_start_ns
            if self._scheduled_start_ns > 0 else cycle_start_ns
        )
        command_deadline_ns = scheduled_start_ns + period_ns
        state = self.low_state
        indices = self.config.joint2motor_idx
        q = np.asarray([state.motor_state[i].q for i in indices], dtype=np.float32)
        dq = np.asarray([state.motor_state[i].dq for i in indices], dtype=np.float32)
        tau_est = np.asarray([state.motor_state[i].tau_est for i in indices], dtype=np.float32)
        motor_temperature = [int(state.motor_state[i].temperature) for i in indices]
        motor_lost = [int(state.motor_state[i].lost) for i in indices]
        self.qj[:] = q
        self.dqj[:] = dq

        gyro = np.asarray(state.imu_state.gyroscope, dtype=np.float32)
        accelerometer = np.asarray(state.imu_state.accelerometer, dtype=np.float32)
        quaternion = np.asarray(state.imu_state.quaternion, dtype=np.float32)
        rpy = np.asarray(state.imu_state.rpy, dtype=np.float32)
        gravity = base.get_gravity_orientation(quaternion)

        if self.use_remote_controller:
            self.cmd[:] = (self.remote_controller.ly, -self.remote_controller.lx,
                           -self.remote_controller.rx)
        self.cmd = np.clip(self.cmd, -1.0, 1.0) * self.config.max_cmd

        self.obs[0:3] = gyro * self.config.obs_scales_ang_vel
        self.obs[3:6] = gravity
        self.obs[6:9] = self.cmd * self.config.command_scale
        self.obs[9:21] = (q - self.config.default_angles) * self.config.obs_scales_dof_pos
        self.obs[21:33] = dq * self.config.obs_scales_dof_vel
        self.obs[33:45] = self.action
        np.clip(self.obs, -self.config.clip_observations,
                self.config.clip_observations, out=self.obs)

        inference_start_ns = time.monotonic_ns()
        with torch.inference_mode():
            result = self.policy(torch.from_numpy(self.obs).unsqueeze(0))
        inference_end_ns = time.monotonic_ns()
        result = result[0] if isinstance(result, tuple) else result
        action = result.detach().cpu().numpy().reshape(-1).astype(np.float32)
        np.clip(action, -self.config.clip_actions, self.config.clip_actions, out=action)
        self.action = action
        target = self.config.default_angles + action * self.config.action_scale
        if hasattr(self, "target_dof_pos"):
            self.target_dof_pos[:] = target
        self._write_targets(target)
        command_sent_ns = time.monotonic_ns()

        force_unitree = _float_list(state.foot_force)
        force_est_unitree = _float_list(state.foot_force_est)
        force_policy = [force_unitree[i] for i in FOOT_UNITREE_TO_POLICY]
        force_est_policy = [force_est_unitree[i] for i in FOOT_UNITREE_TO_POLICY]
        contact = self._contacts.update(force_est_policy)
        sample_period_ms = None if self._last_sample_ns == 0 else (
            cycle_start_ns - self._last_sample_ns) * 1.0e-6
        self._last_sample_ns = cycle_start_ns

        self.paper_logger.submit({
            "schema_version": SCHEMA_VERSION,
            "sequence": self._sequence,
            "wall_time_ns": wall_time_ns,
            "monotonic_ns": cycle_start_ns,
            "scheduled_start_ns": scheduled_start_ns,
            "robot_tick": int(state.tick),
            "sample_period_ms": sample_period_ms,
            "control_dt_s": float(self.config.control_dt),
            "schedule_lateness_ms": max(
                0.0, (cycle_start_ns - scheduled_start_ns) * 1.0e-6
            ),
            "inference_ms": (inference_end_ns - inference_start_ns) * 1.0e-6,
            "compute_to_send_ms": (command_sent_ns - cycle_start_ns) * 1.0e-6,
            "deadline_miss": bool(command_sent_ns > command_deadline_ns),
            "deadline_overrun_ms": max(
                0.0, (command_sent_ns - command_deadline_ns) * 1.0e-6
            ),
            "skipped_deadlines": self._skipped_deadlines,
            "total_skipped_deadlines": self._total_skipped_deadlines,
            "command": _float_list(self.cmd),
            "remote_axes": [float(self.remote_controller.lx),
                            float(self.remote_controller.ly),
                            float(self.remote_controller.rx),
                            float(self.remote_controller.ry)],
            "remote_buttons": _int_list(self.remote_controller.button),
            "observation": _float_list(self.obs),
            "action": _float_list(action),
            "target_q": _float_list(target),
            "joint_q": _float_list(q),
            "joint_dq": _float_list(dq),
            "joint_tau_est": _float_list(tau_est),
            "motor_temperature": motor_temperature,
            "motor_lost": motor_lost,
            "imu": {
                "quaternion_wxyz": _float_list(quaternion),
                "rpy": _float_list(rpy),
                "gyroscope": _float_list(gyro),
                "accelerometer": _float_list(accelerometer),
                "accelerometer_norm": float(np.linalg.norm(accelerometer)),
                "projected_gravity": _float_list(gravity),
            },
            "foot_force_unitree_order": force_unitree,
            "foot_force_est_unitree_order": force_est_unitree,
            "foot_force_policy_order": force_policy,
            "foot_force_est_policy_order": force_est_policy,
            "foot_contact_policy_order": contact,
            "power_v": float(state.power_v),
            "power_a": float(state.power_a),
            "sport_state": self._sport_sample(command_sent_ns),
        })
        self._sequence += 1


def _default_config() -> Path:
    directory = Path(__file__).resolve().parent
    remote_name = directory / "config.yaml"
    return remote_name if remote_name.is_file() else directory / "deploy_go2.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("net", help="network interface connected to the Go2, e.g. eth0")
    parser.add_argument("--config", type=Path, default=_default_config())
    parser.add_argument(
        "--policy", type=Path, default=None,
        help="TorchScript policy override; leaves the YAML file unchanged",
    )
    parser.add_argument("--method", required=True,
                        help="sire_full, sire_unshifted/no_event, or mujoco")
    parser.add_argument("--task", required=True,
                        help="stand, trot, velocity_sweep, or accel_stop")
    parser.add_argument("--trial-id", required=True, help="repeat identifier")
    parser.add_argument("--robot-id", default="go2")
    parser.add_argument("--notes", default="")
    parser.add_argument(
        "--expected-duration", type=float, default=None,
        help="planned policy-phase duration in seconds, recorded but not enforced",
    )
    parser.add_argument("--output-dir", type=Path, default=Path("paper_logs"))
    parser.add_argument("--contact-on", type=float, default=20.0)
    parser.add_argument("--contact-off", type=float, default=10.0)
    parser.add_argument("--queue-size", type=int, default=4096)
    parser.add_argument("--flush-every", type=int, default=25)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config_path = args.config.expanduser().resolve()
    config = base.Config(config_path)
    policy_path = (
        args.policy.expanduser().resolve()
        if args.policy is not None
        else Path(config.policy_path).expanduser().resolve()
    )
    if not policy_path.is_file():
        raise FileNotFoundError("TorchScript policy not found: " + str(policy_path))
    config.policy_path = str(policy_path)
    timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    stem = "_".join((timestamp, _safe_name(args.method), _safe_name(args.task),
                     _safe_name(args.trial_id)))
    output_dir = args.output_dir.expanduser().resolve()
    log_path = output_dir / (stem + ".jsonl")
    metadata_path = output_dir / (stem + ".metadata.json")
    if log_path.exists() or metadata_path.exists():
        raise FileExistsError("refusing to overwrite an existing trial: " + stem)

    with config_path.open(encoding="utf-8") as stream:
        config_snapshot = yaml.safe_load(stream)
    metadata = {
        "schema_version": SCHEMA_VERSION,
        "status": "initializing",
        "method": args.method,
        "task": args.task,
        "trial_id": args.trial_id,
        "robot_id": args.robot_id,
        "notes": args.notes,
        "expected_duration_s": args.expected_duration,
        "hostname": socket.gethostname(),
        "network_interface": args.net,
        "start_wall_time_ns": time.time_ns(),
        "config_path": str(config_path),
        "config": config_snapshot,
        "policy_path": str(policy_path),
        "policy_sha256": _sha256(policy_path),
        "foot_names_unitree_order": list(FOOT_NAMES_UNITREE),
        "foot_names_policy_order": list(FOOT_NAMES_POLICY),
        "contact_on": args.contact_on,
        "contact_off": args.contact_off,
        "sport_state_topic": "rt/sportmodestate",
        "low_state_topic": str(config.lowstate_topic),
        "scheduler": "absolute_monotonic_deadline",
        "log_path": str(log_path),
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    _atomic_json(metadata_path, metadata)
    logger = AsyncJsonlLogger(log_path, args.queue_size, args.flush_every)
    controller = None
    exit_reason = "unknown"
    try:
        ChannelFactoryInitialize(0, args.net)
        controller = PaperController(config, logger, args.contact_on, args.contact_off)
        controller.zero_torque_state()
        controller.move_to_default_pos()
        controller.default_pos_state()
        metadata["status"] = "recording"
        metadata["recording_start_wall_time_ns"] = time.time_ns()
        recording_start_ns = time.monotonic_ns()
        _atomic_json(metadata_path, metadata)
        print("Paper recording started: " + str(log_path), flush=True)
        period_ns = max(1, int(round(float(config.control_dt) * 1.0e9)))
        next_start_ns = time.monotonic_ns()
        while controller.remote_controller.button[base.KeyMap.select] != 1:
            now_ns = time.monotonic_ns()
            if now_ns < next_start_ns:
                time.sleep((next_start_ns - now_ns) * 1.0e-9)
                now_ns = time.monotonic_ns()

            # If more than one complete period late, advance to the latest
            # scheduled tick instead of issuing several rapid catch-up commands.
            skipped_deadlines = 0
            lateness_ns = now_ns - next_start_ns
            if lateness_ns >= period_ns:
                skipped_deadlines = int(lateness_ns // period_ns)
                next_start_ns += skipped_deadlines * period_ns

            controller.set_schedule_context(next_start_ns, skipped_deadlines)
            controller.run()
            next_start_ns += period_ns
        exit_reason = "remote_select"
    except KeyboardInterrupt:
        exit_reason = "keyboard_interrupt"
    except BaseException:
        exit_reason = "exception"
        raise
    finally:
        if controller is not None:
            base.create_damping_cmd(controller.low_cmd)
            controller.send_cmd(controller.low_cmd)
        logger.close()
        recording_end_ns = time.monotonic_ns()
        metadata.update({
            "status": "complete" if exit_reason != "exception" else "failed",
            "exit_reason": exit_reason,
            "end_wall_time_ns": time.time_ns(),
            "samples_written": logger.written,
            "samples_dropped": logger.dropped,
            "total_skipped_deadlines": (
                controller._total_skipped_deadlines
                if controller is not None else 0
            ),
            "recording_duration_s": (
                (recording_end_ns - recording_start_ns) * 1.0e-9
                if "recording_start_ns" in locals() else None
            ),
        })
        _atomic_json(metadata_path, metadata)
        print("Paper recording stopped: samples={} dropped={} metadata={}".format(
              logger.written, logger.dropped, metadata_path), flush=True)
        print("Exit", flush=True)


if __name__ == "__main__":
    main()
