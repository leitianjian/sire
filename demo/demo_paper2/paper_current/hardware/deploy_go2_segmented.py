#!/usr/bin/env python3
"""Run one reproducible Go2 hardware segment, then allow manual recovery.

This script is intended to be copied beside ``deploy_go2.py`` and
``deploy_go2_paper.py`` on the robot computer. It reuses their low-level
controller and telemetry implementation.

Experiment mode:
  * A is a deadman switch and must remain held.
  * SELECT immediately sends a damping command.
  * Commands come from one named segment in a protocol YAML.
  * Releasing A invalidates the trial; the interrupted trial never resumes.

Recovery mode:
  * Experimental logging is closed before recovery begins.
  * Hold A to drive with the joystick through the same locomotion policy.
  * Release A to command zero velocity.
  * Press SELECT to leave recovery mode and send a damping command.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import socket
import tempfile
import threading
import time
from typing import Any

import numpy as np
import yaml

import deploy_go2 as base
import deploy_go2_paper as paper

from unitree_sdk2py.core.channel import ChannelFactoryInitialize


SCHEMA_VERSION = 2


class ProtocolError(ValueError):
    pass


class SafetyAbort(RuntimeError):
    pass


@dataclass(frozen=True)
class Phase:
    name: str
    duration_s: float
    command_from: np.ndarray
    command_to: np.ndarray

    def command_at(self, elapsed_s: float) -> np.ndarray:
        if self.duration_s <= 0.0:
            return self.command_to.copy()
        alpha = float(np.clip(elapsed_s / self.duration_s, 0.0, 1.0))
        return self.command_from * (1.0 - alpha) + self.command_to * alpha


@dataclass(frozen=True)
class Segment:
    protocol_id: str
    segment_id: str
    phases: tuple[Phase, ...]
    boundary_mode: str
    distance_limit_m: float | None

    @property
    def duration_s(self) -> float:
        return sum(phase.duration_s for phase in self.phases)

    def sample(self, elapsed_s: float) -> tuple[int, Phase, float, np.ndarray]:
        remaining = max(0.0, elapsed_s)
        for index, phase in enumerate(self.phases):
            if remaining < phase.duration_s or index == len(self.phases) - 1:
                phase_elapsed = min(remaining, phase.duration_s)
                return index, phase, phase_elapsed, phase.command_at(phase_elapsed)
            remaining -= phase.duration_s
        raise AssertionError("segment must contain at least one phase")


def _command3(value: Any, field: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float32)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ProtocolError(f"{field} must contain three finite values")
    if np.any(np.abs(result) > 1.0):
        raise ProtocolError(f"{field} is normalized and must remain within [-1, 1]")
    return result


def load_segment(path: Path, segment_id: str) -> tuple[Segment, dict[str, Any]]:
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ProtocolError("protocol YAML must contain a mapping")
    protocol_id = str(document.get("protocol_id", "")).strip()
    if not protocol_id:
        raise ProtocolError("protocol_id is required")
    segments = document.get("segments")
    if not isinstance(segments, dict) or segment_id not in segments:
        raise ProtocolError(f"unknown segment {segment_id!r}")
    spec = segments[segment_id]
    if not isinstance(spec, dict) or not isinstance(spec.get("phases"), list):
        raise ProtocolError(f"segment {segment_id!r} must define a phases list")

    phases: list[Phase] = []
    previous = np.zeros(3, dtype=np.float32)
    names: set[str] = set()
    for index, raw in enumerate(spec["phases"]):
        if not isinstance(raw, dict):
            raise ProtocolError(f"phase {index} must be a mapping")
        name = str(raw.get("name", f"phase_{index + 1}")).strip()
        if not name or name in names:
            raise ProtocolError(f"phase names must be nonempty and unique: {name!r}")
        names.add(name)
        duration_s = float(raw.get("duration_s", 0.0))
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ProtocolError(f"phase {name!r} duration_s must be positive")
        if "command" in raw and "command_to" in raw:
            raise ProtocolError(f"phase {name!r} cannot define both command and command_to")
        if "command" in raw:
            command_from = _command3(raw["command"], f"{name}.command")
            command_to = command_from.copy()
        elif "command_to" in raw:
            command_from = previous.copy()
            command_to = _command3(raw["command_to"], f"{name}.command_to")
        else:
            raise ProtocolError(f"phase {name!r} requires command or command_to")
        phases.append(Phase(name, duration_s, command_from, command_to))
        previous = command_to.copy()

    if not phases:
        raise ProtocolError("segment must contain at least one phase")
    if np.linalg.norm(phases[0].command_from, ord=np.inf) > 1.0e-9:
        raise ProtocolError("the first phase must begin at zero command")
    if np.linalg.norm(phases[-1].command_to, ord=np.inf) > 1.0e-9:
        raise ProtocolError("the final phase must end at zero command")

    boundary_mode = str(spec.get("boundary_mode", "operator_deadman"))
    if boundary_mode not in {"operator_deadman", "onboard_position"}:
        raise ProtocolError(
            "boundary_mode must be operator_deadman or onboard_position"
        )
    raw_limit = spec.get("distance_limit_m")
    distance_limit = None if raw_limit is None else float(raw_limit)
    if distance_limit is not None and (
        not math.isfinite(distance_limit) or distance_limit <= 0.0
    ):
        raise ProtocolError("distance_limit_m must be positive when provided")
    if boundary_mode == "onboard_position" and distance_limit is None:
        raise ProtocolError(
            f"segment {segment_id!r} requires a venue-specific distance_limit_m"
        )
    return (
        Segment(
            protocol_id,
            segment_id,
            tuple(phases),
            boundary_mode,
            distance_limit,
        ),
        document,
    )


class ContextLogger:
    """Attach immutable trial context to paper samples and support log cutoff."""

    def __init__(self, sink: paper.AsyncJsonlLogger) -> None:
        self.sink = sink
        self.enabled = True
        self.context: dict[str, Any] = {}
        self.last_sample: dict[str, Any] | None = None
        self.foot_contact_source = "none"
        self.foot_contact_valid = False
        self.foot_order_confirmed = False
        self._foot_contacts: paper.ContactHysteresis | None = None

    def set_context(self, **values: Any) -> None:
        self.context = dict(values)

    def update_context(self, **values: Any) -> None:
        self.context.update(values)

    def configure_foot_contact(
        self,
        source: str,
        valid: bool,
        foot_order_confirmed: bool,
        on_threshold: float,
        off_threshold: float,
    ) -> None:
        self.foot_contact_source = source
        self.foot_contact_valid = valid
        self.foot_order_confirmed = foot_order_confirmed
        self._foot_contacts = (
            paper.ContactHysteresis(on_threshold, off_threshold)
            if valid and source in {"raw", "estimated"}
            else None
        )

    def submit(self, sample: dict[str, Any]) -> None:
        enriched = dict(sample)
        enriched["foot_contact_source"] = self.foot_contact_source
        enriched["foot_contact_valid"] = self.foot_contact_valid
        enriched["foot_force_order_confirmed"] = self.foot_order_confirmed
        if not self.foot_order_confirmed:
            enriched["foot_force_policy_order"] = None
            enriched["foot_force_est_policy_order"] = None
        if self._foot_contacts is None:
            enriched["foot_contact_policy_order"] = None
        else:
            force_field = (
                "foot_force_policy_order"
                if self.foot_contact_source == "raw"
                else "foot_force_est_policy_order"
            )
            enriched["foot_contact_policy_order"] = self._foot_contacts.update(
                enriched[force_field]
            )
        enriched.update(self.context)
        enriched["schema_version"] = SCHEMA_VERSION
        self.last_sample = enriched
        if self.enabled:
            self.sink.submit(enriched)

    @property
    def dropped(self) -> int:
        return self.sink.dropped


class RawLowStateRecorder:
    """Non-blocking high-rate recorder called directly from the DDS callback."""

    def __init__(self, sink: paper.AsyncJsonlLogger) -> None:
        self.sink = sink
        self._lock = threading.Lock()
        self.enabled = False
        self.protocol_id = ""
        self.segment_id = ""
        self.repeat = 0
        self.policy_seed = 0
        self.checkpoint_iteration = 0
        self.trial_start_ns = 0
        self.sequence = 0
        self.error: str | None = None
        self.phase = ""
        self.phase_index = -1
        self.planned_normalized_command = [0.0, 0.0, 0.0]

    def start(
        self,
        *,
        protocol_id: str,
        segment_id: str,
        repeat: int,
        policy_seed: int,
        checkpoint_iteration: int,
        trial_start_ns: int,
    ) -> None:
        with self._lock:
            self.protocol_id = protocol_id
            self.segment_id = segment_id
            self.repeat = repeat
            self.policy_seed = policy_seed
            self.checkpoint_iteration = checkpoint_iteration
            self.trial_start_ns = trial_start_ns
            self.sequence = 0
            self.error = None
            self.enabled = True

    def update_context(
        self,
        *,
        phase: str,
        phase_index: int,
        planned_normalized_command: np.ndarray,
    ) -> None:
        with self._lock:
            self.phase = phase
            self.phase_index = phase_index
            self.planned_normalized_command = paper._float_list(
                planned_normalized_command
            )

    def stop(self) -> None:
        with self._lock:
            self.enabled = False

    @staticmethod
    def _motor_sample(motor: Any) -> dict[str, Any]:
        return {
            "mode": int(motor.mode),
            "q": float(motor.q),
            "dq": float(motor.dq),
            "ddq": float(motor.ddq),
            "tau_est": float(motor.tau_est),
            "q_raw": float(motor.q_raw),
            "dq_raw": float(motor.dq_raw),
            "ddq_raw": float(motor.ddq_raw),
            "temperature": int(motor.temperature),
            "lost": int(motor.lost),
        }

    def submit(self, msg: Any, monotonic_ns: int, wall_time_ns: int) -> None:
        with self._lock:
            if not self.enabled:
                return
            imu = msg.imu_state
            sample = {
                "schema_version": 1,
                "sequence": self.sequence,
                "protocol_id": self.protocol_id,
                "segment_id": self.segment_id,
                "repeat": self.repeat,
                "policy_seed": self.policy_seed,
                "checkpoint_iteration": self.checkpoint_iteration,
                "phase": self.phase,
                "phase_index": self.phase_index,
                "planned_normalized_command": self.planned_normalized_command,
                "monotonic_ns": monotonic_ns,
                "wall_time_ns": wall_time_ns,
                "trial_elapsed_s": (
                    monotonic_ns - self.trial_start_ns
                ) * 1.0e-9,
                "robot_tick": int(msg.tick),
                "head": [int(value) for value in msg.head],
                "level_flag": int(msg.level_flag),
                "version": [int(value) for value in msg.version],
                "bandwidth": int(msg.bandwidth),
                "imu": {
                    "quaternion_wxyz": paper._float_list(imu.quaternion),
                    "rpy": paper._float_list(imu.rpy),
                    "gyroscope_rad_s": paper._float_list(imu.gyroscope),
                    "accelerometer_m_s2": paper._float_list(imu.accelerometer),
                    "temperature": int(imu.temperature),
                },
                "motor_state_unitree_order": [
                    self._motor_sample(motor) for motor in msg.motor_state
                ],
                "foot_force_raw_indices": [
                    float(value) for value in msg.foot_force
                ],
                "foot_force_est_indices": [
                    float(value) for value in msg.foot_force_est
                ],
                "power_v": float(msg.power_v),
                "power_a": float(msg.power_a),
                "wireless_remote": [int(value) for value in msg.wireless_remote],
            }
            try:
                self.sink.submit(sample)
            except BaseException as error:
                self.error = f"{type(error).__name__}: {error}"
                self.enabled = False
            else:
                self.sequence += 1

    @property
    def written(self) -> int:
        return self.sink.written

    @property
    def dropped(self) -> int:
        return self.sink.dropped


class SegmentedController(paper.PaperController):
    def __init__(
        self,
        config: Any,
        logger: ContextLogger,
        raw_recorder: RawLowStateRecorder,
        contact_on: float,
        contact_off: float,
    ) -> None:
        self._low_state_receive_ns = 0
        self.raw_recorder = raw_recorder
        super().__init__(config, logger, contact_on, contact_off)
        self._motor_lost_baseline = [
            int(motor.lost) for motor in self.low_state.motor_state
        ]

    def LowStateHandler(self, msg: Any) -> None:
        """Match the callback name registered by the robot's deploy_go2.py."""
        monotonic_ns = time.monotonic_ns()
        wall_time_ns = time.time_ns()
        self._low_state_receive_ns = monotonic_ns
        super().LowStateHandler(msg)
        self.raw_recorder.submit(msg, monotonic_ns, wall_time_ns)

    def low_state_handler(self, msg: Any) -> None:
        """Compatibility callback for Unitree examples using snake case."""
        monotonic_ns = time.monotonic_ns()
        wall_time_ns = time.time_ns()
        self._low_state_receive_ns = monotonic_ns
        super().low_state_handler(msg)
        self.raw_recorder.submit(msg, monotonic_ns, wall_time_ns)

    def run_normalized_command(self, command: np.ndarray) -> None:
        self.use_remote_controller = False
        self.cmd[:] = command
        super().run()

    def run_manual_recovery(self, enabled: bool) -> None:
        if enabled:
            self.use_remote_controller = True
        else:
            self.use_remote_controller = False
            self.cmd[:] = 0.0
        super().run()

    def sport_position(self, max_age_ms: float) -> np.ndarray | None:
        if self._sport_state is None or self._sport_receive_ns <= 0:
            return None
        age_ms = (time.monotonic_ns() - self._sport_receive_ns) * 1.0e-6
        if age_ms > max_age_ms:
            return None
        position = np.asarray(self._sport_state.position, dtype=float)
        return position if position.shape == (3,) and np.all(np.isfinite(position)) else None


def assess_telemetry(
    controller: SegmentedController,
    logger: ContextLogger,
    duration_s: float,
) -> dict[str, Any]:
    """Collect zero-command diagnostics without writing experiment samples."""
    logger.enabled = False
    scheduler = AbsoluteScheduler(float(controller.config.control_dt))
    raw_samples: list[list[float]] = []
    estimated_samples: list[list[float]] = []
    sport_valid = 0
    steps = max(1, int(math.ceil(duration_s / controller.config.control_dt)))
    for _ in range(steps):
        scheduled_ns, skipped = scheduler.wait()
        controller.set_schedule_context(scheduled_ns, skipped)
        controller.run_normalized_command(np.zeros(3, dtype=np.float32))
        raw = [float(value) for value in controller.low_state.foot_force]
        estimated = [float(value) for value in controller.low_state.foot_force_est]
        raw_samples.append(raw)
        estimated_samples.append(estimated)
        sport_age_ms = (
            math.inf
            if controller._sport_receive_ns <= 0
            else (time.monotonic_ns() - controller._sport_receive_ns) * 1.0e-6
        )
        sport_valid += int(sport_age_ms <= 200.0)

    def summarize(values: list[list[float]]) -> dict[str, Any]:
        array = np.asarray(values, dtype=float)
        minimum = np.min(array, axis=0)
        maximum = np.max(array, axis=0)
        nonzero_fraction = np.mean(np.abs(array) > 1.0e-9, axis=0)
        usable = (np.max(np.abs(array), axis=0) > 1.0) & (
            nonzero_fraction > 0.05
        )
        return {
            "minimum_unitree_order": paper._float_list(minimum),
            "maximum_unitree_order": paper._float_list(maximum),
            "nonzero_fraction_unitree_order": paper._float_list(nonzero_fraction),
            "usable_mask_unitree_order": [bool(value) for value in usable],
            "all_four_usable": bool(np.all(usable)),
        }

    return {
        "samples": steps,
        "duration_s": steps * float(controller.config.control_dt),
        "raw": summarize(raw_samples),
        "estimated": summarize(estimated_samples),
        "sport_state_valid_fraction": sport_valid / steps,
    }


def select_contact_source(
    requested: str,
    report: dict[str, Any],
    foot_order_confirmed: bool,
) -> tuple[str, bool]:
    if requested == "none":
        return "none", False
    if not foot_order_confirmed:
        if requested != "auto":
            raise SafetyAbort("foot_force_order_has_not_been_confirmed")
        return "none", False
    if requested == "auto":
        if report["estimated"]["all_four_usable"]:
            return "estimated", True
        if report["raw"]["all_four_usable"]:
            return "raw", True
        return "none", False
    if not report[requested]["all_four_usable"]:
        raise SafetyAbort(f"requested_{requested}_foot_force_is_not_valid_for_all_feet")
    return requested, True


def wait_for_deadman_rearm(
    controller: SegmentedController,
) -> None:
    """Require an A release followed by a fresh A press before experiment start."""
    print("Release A to arm the trial.", flush=True)
    scheduler = AbsoluteScheduler(float(controller.config.control_dt))
    while (
        controller.remote_controller.button[base.KeyMap.A] == 1
        and controller.remote_controller.button[base.KeyMap.select] != 1
    ):
        scheduled_ns, skipped = scheduler.wait()
        controller.set_schedule_context(scheduled_ns, skipped)
        controller.run_normalized_command(np.zeros(3, dtype=np.float32))
    if controller.remote_controller.button[base.KeyMap.select] == 1:
        raise SafetyAbort("remote_select_before_trial")
    print("Hold A to start the selected segment.", flush=True)
    while (
        controller.remote_controller.button[base.KeyMap.A] != 1
        and controller.remote_controller.button[base.KeyMap.select] != 1
    ):
        scheduled_ns, skipped = scheduler.wait()
        controller.set_schedule_context(scheduled_ns, skipped)
        controller.run_normalized_command(np.zeros(3, dtype=np.float32))
    if controller.remote_controller.button[base.KeyMap.select] == 1:
        raise SafetyAbort("remote_select_before_trial")


class AbsoluteScheduler:
    def __init__(self, period_s: float) -> None:
        self.period_ns = max(1, int(round(period_s * 1.0e9)))
        self.next_start_ns = time.monotonic_ns()

    def wait(self) -> tuple[int, int]:
        now_ns = time.monotonic_ns()
        if now_ns < self.next_start_ns:
            time.sleep((self.next_start_ns - now_ns) * 1.0e-9)
            now_ns = time.monotonic_ns()
        skipped = 0
        lateness_ns = now_ns - self.next_start_ns
        if lateness_ns >= self.period_ns:
            skipped = int(lateness_ns // self.period_ns)
            self.next_start_ns += skipped * self.period_ns
        scheduled_ns = self.next_start_ns
        self.next_start_ns += self.period_ns
        return scheduled_ns, skipped


def run_cycle(
    controller: SegmentedController,
    scheduler: AbsoluteScheduler,
    command: np.ndarray,
) -> None:
    scheduled_ns, skipped = scheduler.wait()
    controller.set_schedule_context(scheduled_ns, skipped)
    controller.run_normalized_command(command)


def safety_reason(
    controller: SegmentedController,
    logger: ContextLogger,
    args: argparse.Namespace,
    consecutive_deadline_misses: int,
) -> tuple[str | None, int]:
    now_ns = time.monotonic_ns()
    low_age_ms = (
        math.inf
        if controller._low_state_receive_ns <= 0
        else (now_ns - controller._low_state_receive_ns) * 1.0e-6
    )
    if low_age_ms > args.max_low_state_age_ms:
        return f"low_state_stale:{low_age_ms:.2f}ms", consecutive_deadline_misses
    indices = controller.config.joint2motor_idx
    lost_increments = []
    for index in indices:
        current = int(controller.low_state.motor_state[index].lost)
        baseline = int(controller._motor_lost_baseline[index])
        if current < baseline:
            controller._motor_lost_baseline[index] = current
            baseline = current
        if current > baseline:
            lost_increments.append((index, current - baseline))
    lost_abort_increment = args.motor_lost_abort_increment
    lost_exceeded = [
        (index, increment)
        for index, increment in lost_increments
        if increment >= lost_abort_increment
    ]
    if lost_exceeded:
        detail = ",".join(f"{index}:+{increment}" for index, increment in lost_exceeded)
        return f"motor_lost_increment:{detail}", consecutive_deadline_misses
    if args.max_motor_temp is not None and any(
        int(controller.low_state.motor_state[index].temperature) > args.max_motor_temp
        for index in indices
    ):
        return "motor_temperature", consecutive_deadline_misses
    rpy = np.asarray(controller.low_state.imu_state.rpy, dtype=float)
    if args.max_abs_roll is not None and abs(float(rpy[0])) > args.max_abs_roll:
        return "roll_limit", consecutive_deadline_misses
    if args.max_abs_pitch is not None and abs(float(rpy[1])) > args.max_abs_pitch:
        return "pitch_limit", consecutive_deadline_misses
    if logger.dropped > 0:
        return "logger_dropped_sample", consecutive_deadline_misses
    if controller.raw_recorder.error is not None:
        return "raw_logger_error", consecutive_deadline_misses
    if controller.raw_recorder.dropped > 0:
        return "raw_logger_dropped_sample", consecutive_deadline_misses
    sample = logger.last_sample or {}
    if bool(sample.get("deadline_miss", False)):
        consecutive_deadline_misses += 1
    else:
        consecutive_deadline_misses = 0
    if consecutive_deadline_misses >= args.max_consecutive_deadline_misses:
        return "consecutive_deadline_misses", consecutive_deadline_misses
    return None, consecutive_deadline_misses


def request_damping(controller: SegmentedController | None) -> str | None:
    if controller is None:
        return None
    try:
        base.create_damping_cmd(controller.low_cmd)
        controller.send_cmd(controller.low_cmd)
    except BaseException as error:
        return f"{type(error).__name__}: {error}"
    return None


def controlled_zero(
    controller: SegmentedController,
    scheduler: AbsoluteScheduler,
    logger: ContextLogger,
    initial_command: np.ndarray,
    duration_s: float,
    reason: str,
) -> None:
    steps = max(1, int(math.ceil(duration_s / controller.config.control_dt)))
    for step in range(steps):
        if controller.remote_controller.button[base.KeyMap.select] == 1:
            raise SafetyAbort("remote_select_during_controlled_stop")
        alpha = (step + 1) / steps
        command = initial_command * (1.0 - alpha)
        logger.update_context(
            record_class="experimental_abort_stop",
            eligible_for_analysis=False,
            phase="abort_ramp_down",
            abort_reason=reason,
            planned_normalized_command=paper._float_list(command),
        )
        controller.raw_recorder.update_context(
            phase="abort_ramp_down",
            phase_index=-1,
            planned_normalized_command=command,
        )
        run_cycle(controller, scheduler, command)


def manual_recovery(
    controller: SegmentedController,
    logger: ContextLogger,
    metadata: dict[str, Any],
    metadata_path: Path,
    args: argparse.Namespace,
) -> None:
    logger.enabled = False
    print(
        "Experimental log closed. Release A fully before recovery is armed.",
        flush=True,
    )
    recovery_start_ns = time.monotonic_ns()
    scheduler = AbsoluteScheduler(float(controller.config.control_dt))
    consecutive_deadline_misses = 0
    recovery_exit_reason = "remote_select"
    while (
        controller.remote_controller.button[base.KeyMap.A] == 1
        and controller.remote_controller.button[base.KeyMap.select] != 1
    ):
        scheduled_ns, skipped = scheduler.wait()
        controller.set_schedule_context(scheduled_ns, skipped)
        controller.run_manual_recovery(False)
        reason, consecutive_deadline_misses = safety_reason(
            controller,
            logger,
            args,
            consecutive_deadline_misses,
        )
        if reason is not None:
            recovery_exit_reason = f"safety_abort:{reason}"
            break
    if recovery_exit_reason == "remote_select":
        print(
            "Recovery armed: hold A to drive with the joystick; release A for "
            "zero command; press SELECT to exit.",
            flush=True,
        )
    while controller.remote_controller.button[base.KeyMap.select] != 1:
        if recovery_exit_reason.startswith("safety_abort:"):
            break
        scheduled_ns, skipped = scheduler.wait()
        controller.set_schedule_context(scheduled_ns, skipped)
        enabled = controller.remote_controller.button[base.KeyMap.A] == 1
        controller.run_manual_recovery(enabled)
        reason, consecutive_deadline_misses = safety_reason(
            controller,
            logger,
            args,
            consecutive_deadline_misses,
        )
        if reason is not None:
            recovery_exit_reason = f"safety_abort:{reason}"
            break
    metadata["manual_recovery_duration_s"] = (
        time.monotonic_ns() - recovery_start_ns
    ) * 1.0e-9
    metadata["manual_recovery_used"] = True
    metadata["manual_recovery_exit_reason"] = recovery_exit_reason
    paper._atomic_json(metadata_path, metadata)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("net", help="network interface connected to the Go2")
    parser.add_argument(
        "--config",
        required=True,
        type=Path,
        help="deployment YAML; pass deploy_sire_1200.yaml for the current policies",
    )
    parser.add_argument("--policy", required=True, type=Path)
    parser.add_argument("--protocol", required=True, type=Path)
    parser.add_argument("--segment", required=True)
    parser.add_argument("--repeat", required=True, type=int)
    parser.add_argument("--policy-seed", required=True, type=int)
    parser.add_argument("--checkpoint-iteration", default=1200, type=int)
    parser.add_argument("--robot-id", default="go2")
    parser.add_argument("--notes", default="")
    parser.add_argument("--output-dir", type=Path, default=Path("segmented_logs"))
    parser.add_argument("--contact-on", type=float, default=20.0)
    parser.add_argument("--contact-off", type=float, default=10.0)
    parser.add_argument(
        "--foot-contact-source",
        choices=("auto", "raw", "estimated", "none"),
        default="auto",
        help=(
            "derived contact source; auto requires all four channels to pass "
            "the standing telemetry check"
        ),
    )
    parser.add_argument(
        "--confirm-foot-order-fr-fl-rr-rl",
        action="store_true",
        help=(
            "confirm that LowState foot-force indices were empirically verified "
            "as FR,FL,RR,RL on this robot; required before deriving contact labels"
        ),
    )
    parser.add_argument("--telemetry-preflight-s", type=float, default=1.0)
    parser.add_argument("--queue-size", type=int, default=4096)
    parser.add_argument("--flush-every", type=int, default=25)
    parser.add_argument("--raw-queue-size", type=int, default=32768)
    parser.add_argument("--raw-flush-every", type=int, default=250)
    parser.add_argument("--abort-ramp-s", type=float, default=0.20)
    parser.add_argument("--max-low-state-age-ms", type=float, default=100.0)
    parser.add_argument("--max-sport-state-age-ms", type=float, default=200.0)
    parser.add_argument("--max-consecutive-deadline-misses", type=int, default=3)
    parser.add_argument("--motor-lost-abort-increment", type=int, default=10)
    parser.add_argument("--max-motor-temp", type=int, default=None)
    parser.add_argument("--max-abs-roll", type=float, default=None)
    parser.add_argument("--max-abs-pitch", type=float, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.repeat < 1:
        raise ValueError("--repeat must be positive")
    if args.motor_lost_abort_increment < 1:
        raise ValueError("--motor-lost-abort-increment must be positive")
    config_path = args.config.expanduser().resolve()
    policy_path = args.policy.expanduser().resolve()
    protocol_path = args.protocol.expanduser().resolve()
    segment, protocol_snapshot = load_segment(protocol_path, args.segment)

    if not policy_path.is_file():
        raise FileNotFoundError(policy_path)

    timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    stem = "_".join(
        (
            timestamp,
            paper._safe_name(segment.protocol_id),
            paper._safe_name(segment.segment_id),
            f"seed-{args.policy_seed}",
            f"repeat-{args.repeat:02d}",
        )
    )
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    log_path = output_dir / f"{stem}.jsonl"
    raw_log_path = output_dir / f"{stem}.lowstate.jsonl"
    metadata_path = output_dir / f"{stem}.metadata.json"
    if log_path.exists() or raw_log_path.exists() or metadata_path.exists():
        raise FileExistsError(f"refusing to overwrite trial {stem}")

    with config_path.open("r", encoding="utf-8") as stream:
        config_snapshot = yaml.safe_load(stream)
    # base.Config validates policy_path during construction. Build an effective
    # temporary snapshot so --policy remains a real override even if the YAML's
    # historical policy path is absent on the robot computer.
    effective_config = dict(config_snapshot)
    effective_config["policy_path"] = str(policy_path)
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        suffix=".yaml",
        prefix="go2_segmented_",
        delete=False,
    ) as temporary:
        yaml.safe_dump(effective_config, temporary, sort_keys=False)
        effective_config_path = Path(temporary.name)
    try:
        config = base.Config(effective_config_path)
    finally:
        effective_config_path.unlink(missing_ok=True)
    metadata: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "status": "initializing",
        "protocol_id": segment.protocol_id,
        "protocol_path": str(protocol_path),
        "protocol_sha256": paper._sha256(protocol_path),
        "protocol": protocol_snapshot,
        "segment_id": segment.segment_id,
        "planned_duration_s": segment.duration_s,
        "distance_limit_m": segment.distance_limit_m,
        "boundary_mode": segment.boundary_mode,
        "repeat": args.repeat,
        "policy_seed": args.policy_seed,
        "checkpoint_iteration": args.checkpoint_iteration,
        "policy_path": str(policy_path),
        "policy_sha256": paper._sha256(policy_path),
        "config_path": str(config_path),
        "config_sha256": paper._sha256(config_path),
        "config": config_snapshot,
        "effective_config": effective_config,
        "robot_id": args.robot_id,
        "notes": args.notes,
        "hostname": socket.gethostname(),
        "network_interface": args.net,
        "start_wall_time_ns": time.time_ns(),
        "command_source": "protocol_normalized_then_scaled_by_config_max_cmd",
        "deadman_button": "A",
        "emergency_stop_button": "SELECT",
        "manual_recovery_logged_as_experiment": False,
        "log_path": str(log_path),
        "raw_lowstate_log_path": str(raw_log_path),
        "raw_lowstate_schema": (
            "all 20 Unitree motor states plus IMU, raw/estimated foot arrays, "
            "robot tick, and local monotonic/wall clocks"
        ),
    }
    paper._atomic_json(metadata_path, metadata)

    sink = paper.AsyncJsonlLogger(log_path, args.queue_size, args.flush_every)
    raw_sink = paper.AsyncJsonlLogger(
        raw_log_path,
        args.raw_queue_size,
        args.raw_flush_every,
    )
    logger = ContextLogger(sink)
    raw_recorder = RawLowStateRecorder(raw_sink)
    controller: SegmentedController | None = None
    trial_start_ns: int | None = None
    trial_end_ns: int | None = None
    last_command = np.zeros(3, dtype=np.float32)
    exit_reason = "initialization_failed"
    eligible = False
    abort_reason: str | None = None
    damping_command_error: str | None = None
    logger_close_errors: list[str] = []
    try:
        ChannelFactoryInitialize(0, args.net)
        controller = SegmentedController(
            config,
            logger,
            raw_recorder,
            args.contact_on,
            args.contact_off,
        )
        controller.zero_torque_state()
        controller.move_to_default_pos()
        controller.default_pos_state()
        if controller.remote_controller.button[base.KeyMap.select] == 1:
            raise SafetyAbort("remote_select_before_trial")

        if args.foot_contact_source == "none":
            telemetry_report = {
                "skipped": True,
                "reason": "foot_contact_source_none",
            }
            contact_source, contact_valid = "none", False
        else:
            telemetry_report = assess_telemetry(
                controller,
                logger,
                args.telemetry_preflight_s,
            )
            contact_source, contact_valid = select_contact_source(
                args.foot_contact_source,
                telemetry_report,
                args.confirm_foot_order_fr_fl_rr_rl,
            )
        logger.configure_foot_contact(
            contact_source,
            contact_valid,
            args.confirm_foot_order_fr_fl_rr_rl,
            args.contact_on,
            args.contact_off,
        )
        metadata["telemetry_preflight"] = telemetry_report
        metadata["foot_contact_source"] = contact_source
        metadata["foot_contact_valid"] = contact_valid
        metadata["foot_force_order_assumption"] = "FR,FL,RR,RL"
        metadata["foot_force_order_confirmed"] = (
            args.confirm_foot_order_fr_fl_rr_rl
        )
        paper._atomic_json(metadata_path, metadata)
        if telemetry_report.get("skipped", False):
            print(
                "Telemetry preflight skipped: foot_contact_source=none",
                flush=True,
            )
        else:
            print(
                "Telemetry preflight: "
                f"raw={telemetry_report['raw']['usable_mask_unitree_order']} "
                f"estimated={telemetry_report['estimated']['usable_mask_unitree_order']} "
                f"sport_valid_fraction="
                f"{telemetry_report['sport_state_valid_fraction']:.2f} "
                f"contact_source={contact_source}",
                flush=True,
            )

        wait_for_deadman_rearm(controller)

        start_position = (
            controller.sport_position(args.max_sport_state_age_ms)
            if segment.boundary_mode == "onboard_position"
            else None
        )
        if segment.boundary_mode == "onboard_position" and start_position is None:
            raise SafetyAbort("fresh_sport_position_required_for_distance_limit")

        logger.enabled = True
        metadata["status"] = "recording"
        metadata["recording_start_wall_time_ns"] = time.time_ns()
        paper._atomic_json(metadata_path, metadata)
        print(f"Segment recording started: {log_path}", flush=True)

        scheduler = AbsoluteScheduler(float(config.control_dt))
        trial_start_ns = time.monotonic_ns()
        initial_phase_index, initial_phase, _, initial_command = segment.sample(0.0)
        raw_recorder.update_context(
            phase=initial_phase.name,
            phase_index=initial_phase_index,
            planned_normalized_command=initial_command,
        )
        raw_recorder.start(
            protocol_id=segment.protocol_id,
            segment_id=segment.segment_id,
            repeat=args.repeat,
            policy_seed=args.policy_seed,
            checkpoint_iteration=args.checkpoint_iteration,
            trial_start_ns=trial_start_ns,
        )
        consecutive_deadline_misses = 0
        while True:
            if controller.remote_controller.button[base.KeyMap.select] == 1:
                abort_reason = "remote_select"
                raise SafetyAbort(abort_reason)
            if controller.remote_controller.button[base.KeyMap.A] != 1:
                abort_reason = "deadman_released"
                exit_reason = "operator_abort"
                controlled_zero(
                    controller,
                    scheduler,
                    logger,
                    last_command,
                    args.abort_ramp_s,
                    abort_reason,
                )
                break

            elapsed_s = (time.monotonic_ns() - trial_start_ns) * 1.0e-9
            if elapsed_s >= segment.duration_s:
                logger.set_context(
                    record_class="experimental",
                    eligible_for_analysis=True,
                    protocol_id=segment.protocol_id,
                    segment_id=segment.segment_id,
                    repeat=args.repeat,
                    policy_seed=args.policy_seed,
                    checkpoint_iteration=args.checkpoint_iteration,
                    phase_index=len(segment.phases),
                    phase="protocol_complete_zero",
                    phase_elapsed_s=0.0,
                    segment_elapsed_s=elapsed_s,
                    planned_normalized_command=[0.0, 0.0, 0.0],
                    command_source="protocol",
                    abort_reason=None,
                )
                raw_recorder.update_context(
                    phase="protocol_complete_zero",
                    phase_index=len(segment.phases),
                    planned_normalized_command=np.zeros(3, dtype=np.float32),
                )
                run_cycle(
                    controller,
                    scheduler,
                    np.zeros(3, dtype=np.float32),
                )
                exit_reason = "protocol_completed"
                eligible = True
                break
            phase_index, phase, phase_elapsed_s, command = segment.sample(elapsed_s)
            last_command = command.copy()
            logger.set_context(
                record_class="experimental",
                eligible_for_analysis=True,
                protocol_id=segment.protocol_id,
                segment_id=segment.segment_id,
                repeat=args.repeat,
                policy_seed=args.policy_seed,
                checkpoint_iteration=args.checkpoint_iteration,
                phase_index=phase_index,
                phase=phase.name,
                phase_elapsed_s=phase_elapsed_s,
                segment_elapsed_s=elapsed_s,
                planned_normalized_command=paper._float_list(command),
                command_source="protocol",
                abort_reason=None,
            )
            raw_recorder.update_context(
                phase=phase.name,
                phase_index=phase_index,
                planned_normalized_command=command,
            )
            run_cycle(controller, scheduler, command)

            reason, consecutive_deadline_misses = safety_reason(
                controller,
                logger,
                args,
                consecutive_deadline_misses,
            )
            if reason is not None:
                abort_reason = reason
                raise SafetyAbort(reason)

            if segment.boundary_mode == "onboard_position":
                position = controller.sport_position(args.max_sport_state_age_ms)
                if position is None:
                    abort_reason = "sport_state_stale_during_distance_guard"
                    raise SafetyAbort(abort_reason)
                displacement = float(np.linalg.norm(position - start_position))
                if displacement >= float(segment.distance_limit_m):
                    abort_reason = "distance_limit"
                    exit_reason = "boundary_abort"
                    controlled_zero(
                        controller,
                        scheduler,
                        logger,
                        last_command,
                        args.abort_ramp_s,
                        abort_reason,
                    )
                    break
        trial_end_ns = time.monotonic_ns()
    except KeyboardInterrupt:
        exit_reason = "keyboard_interrupt"
        abort_reason = "keyboard_interrupt"
    except SafetyAbort as error:
        exit_reason = "safety_abort"
        abort_reason = str(error)
    except BaseException as error:
        exit_reason = "exception"
        abort_reason = f"{type(error).__name__}: {error}"
        raise
    finally:
        if trial_start_ns is not None and trial_end_ns is None:
            trial_end_ns = time.monotonic_ns()
        if exit_reason not in {
            "protocol_completed",
            "operator_abort",
            "boundary_abort",
        }:
            damping_command_error = request_damping(controller)
        logger.enabled = False
        raw_recorder.stop()
        for name, current_sink in (("policy", sink), ("raw_lowstate", raw_sink)):
            try:
                current_sink.close()
            except BaseException as error:
                logger_close_errors.append(
                    f"{name}: {type(error).__name__}: {error}"
                )
        if (
            sink.dropped > 0
            or raw_sink.dropped > 0
            or raw_recorder.error is not None
            or logger_close_errors
        ):
            eligible = False
            if abort_reason is None:
                if logger_close_errors:
                    abort_reason = "logger_close_error"
                elif raw_recorder.error is not None:
                    abort_reason = "raw_logger_error"
                elif raw_sink.dropped > 0:
                    abort_reason = "raw_logger_dropped_sample"
                else:
                    abort_reason = "logger_dropped_sample"
            if exit_reason == "protocol_completed":
                exit_reason = "logging_invalid"
                damping_command_error = (
                    damping_command_error or request_damping(controller)
                )
        metadata.update(
            {
                "status": "complete" if eligible else "invalid",
                "eligible_for_analysis": eligible,
                "exit_reason": exit_reason,
                "abort_reason": abort_reason,
                "end_wall_time_ns": time.time_ns(),
                "samples_written": sink.written,
                "samples_dropped": sink.dropped,
                "raw_lowstate_samples_written": raw_sink.written,
                "raw_lowstate_samples_dropped": raw_sink.dropped,
                "raw_lowstate_recorder_error": raw_recorder.error,
                "logger_close_errors": logger_close_errors,
                "damping_command_error": damping_command_error,
                "raw_lowstate_mean_rate_hz": (
                    raw_sink.written
                    / ((trial_end_ns - trial_start_ns) * 1.0e-9)
                    if trial_start_ns is not None
                    and trial_end_ns is not None
                    and trial_end_ns > trial_start_ns
                    else None
                ),
                "total_skipped_deadlines": (
                    controller._total_skipped_deadlines
                    if controller is not None
                    else 0
                ),
                "experimental_duration_s": (
                    (trial_end_ns - trial_start_ns) * 1.0e-9
                    if trial_start_ns is not None and trial_end_ns is not None
                    else None
                ),
            }
        )
        paper._atomic_json(metadata_path, metadata)

    if controller is None:
        return
    allow_recovery = exit_reason in {
        "protocol_completed",
        "operator_abort",
        "boundary_abort",
    }
    try:
        if allow_recovery:
            manual_recovery(controller, logger, metadata, metadata_path, args)
    finally:
        damping_command_error = request_damping(controller)
        if damping_command_error is not None:
            metadata["damping_command_error"] = damping_command_error
        metadata["manual_recovery_allowed"] = allow_recovery
        metadata["recovery_exit_wall_time_ns"] = time.time_ns()
        paper._atomic_json(metadata_path, metadata)
        print(
            f"Trial {exit_reason}: policy_samples={sink.written} "
            f"policy_dropped={sink.dropped} raw_samples={raw_sink.written} "
            f"raw_dropped={raw_sink.dropped}; "
            f"metadata={metadata_path}",
            flush=True,
        )


if __name__ == "__main__":
    main()
