#!/usr/bin/env python3
"""Run a persistent-controller Go2 paper session for one policy seed."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import socket
import tempfile
import time
from typing import Any

import numpy as np
import yaml

import deploy_go2 as base
import deploy_go2_paper as paper
import deploy_go2_segmented as segmented

from unitree_sdk2py.core.channel import ChannelFactoryInitialize


PAPER_SEGMENTS = (
    "paper_v3_stand_08",
    "paper_v3_fore_aft_050",
    "paper_v3_lateral_pair_050",
    "paper_v3_yaw_pair_080",
)

PAPER_SEGMENT_GUIDANCE = {
    "paper_v3_stand_08": (
        "STAND for 8 s; center the robot with at least 0.75 m clearance on all sides."
    ),
    "paper_v3_fore_aft_050": (
        "FORWARD then BACKWARD at 0.50 m/s; maximum ideal forward excursion "
        "1.75 m. Place the robot with at least 2.4 m clear in front and 0.8 m behind."
    ),
    "paper_v3_lateral_pair_050": (
        "LEFT then RIGHT at 0.50 m/s; maximum ideal left excursion 1.75 m. "
        "Leave at least 2.4 m clear to the left and 0.8 m to the right."
    ),
    "paper_v3_yaw_pair_080": (
        "COUNTERCLOCKWISE then CLOCKWISE at 0.80 rad/s; maximum ideal angular "
        "excursion 2.8 rad. Center the robot in a 2 m by 2 m clear area."
    ),
}


class NullSink:
    """Bootstrap sink used before the first trial installs real loggers."""

    written = 0
    dropped = 0

    def submit(self, _sample: dict[str, Any]) -> None:
        return

    def close(self) -> None:
        return


def print_next_trial(
    repeat: int,
    segment: segmented.Segment,
    queue_position: int,
    queue_size: int,
) -> None:
    phase_text = " -> ".join(
        f"{phase.name} {phase.duration_s:g}s" for phase in segment.phases
    )
    guidance = PAPER_SEGMENT_GUIDANCE.get(
        segment.segment_id,
        "Review the protocol phases and provide conservative clearance.",
    )
    print("", flush=True)
    print("=" * 72, flush=True)
    print(
        f"NEXT TRIAL {queue_position}/{queue_size}: repeat={repeat} "
        f"segment={segment.segment_id}",
        flush=True,
    )
    print(f"PLACEMENT: {guidance}", flush=True)
    print(f"PHASES: {phase_text}", flush=True)
    print("Reposition with A, release A, inspect all feet, then press START.", flush=True)
    print("=" * 72, flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("net")
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--policy", required=True, type=Path)
    parser.add_argument("--protocol", required=True, type=Path)
    parser.add_argument("--seed", required=True, type=int, choices=(1, 2, 3, 4))
    parser.add_argument("--repeats", required=True, type=int)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--checkpoint-iteration", default=1200, type=int)
    parser.add_argument("--robot-id", default="go2")
    parser.add_argument("--notes", default="")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--queue-size", type=int, default=4096)
    parser.add_argument("--flush-every", type=int, default=25)
    parser.add_argument("--raw-queue-size", type=int, default=32768)
    parser.add_argument("--raw-flush-every", type=int, default=250)
    parser.add_argument("--abort-ramp-s", type=float, default=0.20)
    parser.add_argument("--max-low-state-age-ms", type=float, default=100.0)
    parser.add_argument("--max-consecutive-deadline-misses", type=int, default=3)
    parser.add_argument("--motor-lost-abort-increment", type=int, default=10)
    parser.add_argument("--max-motor-temp", type=int, default=None)
    parser.add_argument("--max-abs-roll", type=float, default=None)
    parser.add_argument("--max-abs-pitch", type=float, default=None)
    parser.add_argument("--pretrial-settle-min-s", type=float, default=2.0)
    parser.add_argument("--pretrial-settle-timeout-s", type=float, default=5.0)
    parser.add_argument("--pretrial-stable-window-s", type=float, default=0.5)
    parser.add_argument("--pretrial-max-abs-roll-pitch", type=float, default=0.20)
    parser.add_argument("--pretrial-max-body-rate", type=float, default=0.25)
    parser.add_argument("--pretrial-max-joint-speed-rms", type=float, default=0.25)
    return parser.parse_args()


def valid_trial_exists(
    output_dir: Path,
    seed: int,
    segment_id: str,
    repeat: int,
    protocol_id: str | None = None,
) -> Path | None:
    for path in sorted(output_dir.glob("*.metadata.json")):
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        if (
            value.get("policy_seed") == seed
            and value.get("segment_id") == segment_id
            and value.get("repeat") == repeat
            and (
                protocol_id is None
                or value.get("protocol_id") == protocol_id
            )
            and value.get("eligible_for_analysis") is True
            and value.get("exit_reason") == "protocol_completed"
        ):
            return path
    return None


def build_config(config_path: Path, policy_path: Path) -> tuple[Any, dict[str, Any]]:
    with config_path.open("r", encoding="utf-8") as stream:
        snapshot = yaml.safe_load(stream)
    effective = dict(snapshot)
    effective["policy_path"] = str(policy_path)
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        suffix=".yaml",
        prefix="go2_paper_session_",
        delete=False,
    ) as temporary:
        yaml.safe_dump(effective, temporary, sort_keys=False)
        temporary_path = Path(temporary.name)
    try:
        config = base.Config(temporary_path)
    finally:
        temporary_path.unlink(missing_ok=True)
    return config, snapshot


def settle_before_trial(
    controller: segmented.SegmentedController,
    logger: segmented.ContextLogger,
    args: argparse.Namespace,
) -> tuple[bool, dict[str, Any]]:
    """Hold zero command until body and joint motion remain stably small."""
    scheduler = segmented.AbsoluteScheduler(float(controller.config.control_dt))
    required_stable = max(
        1,
        int(math.ceil(args.pretrial_stable_window_s / controller.config.control_dt)),
    )
    stable_count = 0
    misses = 0
    start_ns = time.monotonic_ns()
    last_metrics: dict[str, Any] = {}
    print(
        "Pre-trial settling: keep A released and visually confirm all feet are down.",
        flush=True,
    )
    while True:
        if controller.remote_controller.button[base.KeyMap.select] == 1:
            raise segmented.SafetyAbort("remote_select_during_pretrial_settle")
        if controller.remote_controller.button[base.KeyMap.A] == 1:
            return False, {
                "ready": False,
                "reason": "A_pressed_during_settle",
                "elapsed_s": (time.monotonic_ns() - start_ns) * 1.0e-9,
            }

        segmented.run_cycle(
            controller,
            scheduler,
            np.zeros(3, dtype=np.float32),
        )
        reason, misses = segmented.safety_reason(
            controller,
            logger,
            args,
            misses,
        )
        if reason is not None:
            raise segmented.SafetyAbort(reason)

        rpy = np.asarray(controller.low_state.imu_state.rpy, dtype=float)
        gyro = np.asarray(controller.low_state.imu_state.gyroscope, dtype=float)
        indices = controller.config.joint2motor_idx
        joint_dq = np.asarray(
            [controller.low_state.motor_state[index].dq for index in indices],
            dtype=float,
        )
        elapsed_s = (time.monotonic_ns() - start_ns) * 1.0e-9
        last_metrics = {
            "abs_roll_rad": abs(float(rpy[0])),
            "abs_pitch_rad": abs(float(rpy[1])),
            "body_rate_norm_rad_s": float(np.linalg.norm(gyro)),
            "joint_speed_rms_rad_s": float(
                math.sqrt(float(np.mean(np.square(joint_dq))))
            ),
        }
        stable = (
            max(last_metrics["abs_roll_rad"], last_metrics["abs_pitch_rad"])
            <= args.pretrial_max_abs_roll_pitch
            and last_metrics["body_rate_norm_rad_s"]
            <= args.pretrial_max_body_rate
            and last_metrics["joint_speed_rms_rad_s"]
            <= args.pretrial_max_joint_speed_rms
        )
        stable_count = stable_count + 1 if stable else 0
        if elapsed_s >= args.pretrial_settle_min_s and stable_count >= required_stable:
            summary = {
                "ready": True,
                "reason": "stable",
                "elapsed_s": elapsed_s,
                "stable_window_s": stable_count * controller.config.control_dt,
                **last_metrics,
            }
            print(f"Pre-trial state ready: {json.dumps(summary)}", flush=True)
            return True, summary
        if elapsed_s >= args.pretrial_settle_timeout_s:
            summary = {
                "ready": False,
                "reason": "stability_timeout",
                "elapsed_s": elapsed_s,
                "stable_window_s": stable_count * controller.config.control_dt,
                **last_metrics,
            }
            print(f"Pre-trial state not ready: {json.dumps(summary)}", flush=True)
            return False, summary


def set_trial_context(
    logger: segmented.ContextLogger,
    raw_recorder: segmented.RawLowStateRecorder,
    segment: segmented.Segment,
    repeat: int,
    seed: int,
    checkpoint_iteration: int,
    phase_index: int,
    phase: segmented.Phase,
    phase_elapsed_s: float,
    segment_elapsed_s: float,
    command: np.ndarray,
) -> None:
    logger.set_context(
        record_class="experimental",
        eligible_for_analysis=True,
        protocol_id=segment.protocol_id,
        segment_id=segment.segment_id,
        repeat=repeat,
        policy_seed=seed,
        checkpoint_iteration=checkpoint_iteration,
        phase_index=phase_index,
        phase=phase.name,
        phase_elapsed_s=phase_elapsed_s,
        segment_elapsed_s=segment_elapsed_s,
        planned_normalized_command=paper._float_list(command),
        command_source="protocol",
        abort_reason=None,
    )
    raw_recorder.update_context(
        phase=phase.name,
        phase_index=phase_index,
        planned_normalized_command=command,
    )


def run_trial(
    controller: segmented.SegmentedController,
    config: Any,
    config_snapshot: dict[str, Any],
    config_path: Path,
    policy_path: Path,
    protocol_path: Path,
    protocol_snapshot: dict[str, Any],
    segment: segmented.Segment,
    repeat: int,
    queue_index: int,
    pretrial_state: dict[str, Any],
    args: argparse.Namespace,
) -> tuple[bool, str, Path]:
    timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    stem = "_".join(
        (
            timestamp,
            paper._safe_name(args.session_id),
            paper._safe_name(segment.segment_id),
            f"seed-{args.seed}",
            f"repeat-{repeat:02d}",
        )
    )
    log_path = args.output_dir / f"{stem}.jsonl"
    raw_path = args.output_dir / f"{stem}.lowstate.jsonl"
    metadata_path = args.output_dir / f"{stem}.metadata.json"
    if any(path.exists() for path in (log_path, raw_path, metadata_path)):
        raise FileExistsError(f"refusing to overwrite trial {stem}")

    sink = paper.AsyncJsonlLogger(log_path, args.queue_size, args.flush_every)
    raw_sink = paper.AsyncJsonlLogger(
        raw_path,
        args.raw_queue_size,
        args.raw_flush_every,
    )
    logger = segmented.ContextLogger(sink)
    logger.enabled = False
    logger.configure_foot_contact("none", False, False, 20.0, 10.0)
    raw_recorder = segmented.RawLowStateRecorder(raw_sink)
    controller.paper_logger = logger
    controller.raw_recorder = raw_recorder

    metadata = {
        "schema_version": segmented.SCHEMA_VERSION,
        "status": "initializing",
        "session_id": args.session_id,
        "session_queue_index": queue_index,
        "protocol_id": segment.protocol_id,
        "protocol_path": str(protocol_path),
        "protocol_sha256": paper._sha256(protocol_path),
        "protocol": protocol_snapshot,
        "segment_id": segment.segment_id,
        "planned_duration_s": segment.duration_s,
        "boundary_mode": segment.boundary_mode,
        "repeat": repeat,
        "policy_seed": args.seed,
        "checkpoint_iteration": args.checkpoint_iteration,
        "policy_path": str(policy_path),
        "policy_sha256": paper._sha256(policy_path),
        "config_path": str(config_path),
        "config_sha256": paper._sha256(config_path),
        "config": config_snapshot,
        "robot_id": args.robot_id,
        "notes": args.notes,
        "hostname": socket.gethostname(),
        "network_interface": args.net,
        "start_wall_time_ns": time.time_ns(),
        "foot_contact_source": "none",
        "foot_contact_valid": False,
        "manual_recovery_logged_as_experiment": False,
        "persistent_controller_session": True,
        "pretrial_default_reset": {
            "performed": False,
            "reason": "training_default_angles_are_not_a_loaded_equilibrium",
        },
        "pretrial_state": pretrial_state,
        "motor_lost_abort_increment": args.motor_lost_abort_increment,
        "log_path": str(log_path),
        "raw_lowstate_log_path": str(raw_path),
        "motor_lost_baseline_unitree_order": list(
            controller._motor_lost_baseline
        ),
    }
    paper._atomic_json(metadata_path, metadata)

    trial_start_ns: int | None = None
    trial_end_ns: int | None = None
    last_command = np.zeros(3, dtype=np.float32)
    exit_reason = "initialization_failed"
    abort_reason: str | None = None
    eligible = False
    logger_close_errors: list[str] = []
    damping_error: str | None = None
    skipped_deadlines_at_start = controller._total_skipped_deadlines
    try:
        initial_index, initial_phase, _, initial_command = segment.sample(0.0)
        set_trial_context(
            logger,
            raw_recorder,
            segment,
            repeat,
            args.seed,
            args.checkpoint_iteration,
            initial_index,
            initial_phase,
            0.0,
            0.0,
            initial_command,
        )
        metadata["status"] = "recording"
        metadata["recording_start_wall_time_ns"] = time.time_ns()
        paper._atomic_json(metadata_path, metadata)
        logger.enabled = True
        trial_start_ns = time.monotonic_ns()
        raw_recorder.start(
            protocol_id=segment.protocol_id,
            segment_id=segment.segment_id,
            repeat=repeat,
            policy_seed=args.seed,
            checkpoint_iteration=args.checkpoint_iteration,
            trial_start_ns=trial_start_ns,
        )
        scheduler = segmented.AbsoluteScheduler(float(config.control_dt))
        consecutive_deadline_misses = 0
        print(f"Recording trial: {segment.segment_id}, repeat {repeat}", flush=True)
        while True:
            if controller.remote_controller.button[base.KeyMap.select] == 1:
                raise segmented.SafetyAbort("remote_select")
            if controller.remote_controller.button[base.KeyMap.A] != 1:
                abort_reason = "deadman_released"
                exit_reason = "operator_abort"
                segmented.controlled_zero(
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
                logger.update_context(
                    phase="protocol_complete_zero",
                    phase_index=len(segment.phases),
                    phase_elapsed_s=0.0,
                    segment_elapsed_s=elapsed_s,
                    planned_normalized_command=[0.0, 0.0, 0.0],
                )
                raw_recorder.update_context(
                    phase="protocol_complete_zero",
                    phase_index=len(segment.phases),
                    planned_normalized_command=np.zeros(3, dtype=np.float32),
                )
                segmented.run_cycle(
                    controller,
                    scheduler,
                    np.zeros(3, dtype=np.float32),
                )
                exit_reason = "protocol_completed"
                eligible = True
                break

            phase_index, phase, phase_elapsed_s, command = segment.sample(elapsed_s)
            last_command = command.copy()
            set_trial_context(
                logger,
                raw_recorder,
                segment,
                repeat,
                args.seed,
                args.checkpoint_iteration,
                phase_index,
                phase,
                phase_elapsed_s,
                elapsed_s,
                command,
            )
            segmented.run_cycle(controller, scheduler, command)
            reason, consecutive_deadline_misses = segmented.safety_reason(
                controller,
                logger,
                args,
                consecutive_deadline_misses,
            )
            if reason is not None:
                raise segmented.SafetyAbort(reason)
        trial_end_ns = time.monotonic_ns()
    except KeyboardInterrupt:
        exit_reason = "keyboard_interrupt"
        abort_reason = "keyboard_interrupt"
        damping_error = segmented.request_damping(controller)
    except segmented.SafetyAbort as error:
        exit_reason = "safety_abort"
        abort_reason = str(error)
        damping_error = segmented.request_damping(controller)
    except BaseException as error:
        exit_reason = "exception"
        abort_reason = f"{type(error).__name__}: {error}"
        damping_error = segmented.request_damping(controller)
        raise
    finally:
        if trial_start_ns is not None and trial_end_ns is None:
            trial_end_ns = time.monotonic_ns()
        logger.enabled = False
        raw_recorder.stop()
        for name, current in (("policy", sink), ("raw_lowstate", raw_sink)):
            try:
                current.close()
            except BaseException as error:
                logger_close_errors.append(
                    f"{name}: {type(error).__name__}: {error}"
                )
        if (
            sink.dropped
            or raw_sink.dropped
            or raw_recorder.error is not None
            or logger_close_errors
        ):
            eligible = False
            if exit_reason == "protocol_completed":
                exit_reason = "logging_invalid"
            abort_reason = abort_reason or "logging_invalid"
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
                "damping_command_error": damping_error,
                "trial_skipped_deadlines": (
                    controller._total_skipped_deadlines
                    - skipped_deadlines_at_start
                ),
                "session_total_skipped_deadlines": (
                    controller._total_skipped_deadlines
                ),
                "experimental_duration_s": (
                    (trial_end_ns - trial_start_ns) * 1.0e-9
                    if trial_start_ns is not None and trial_end_ns is not None
                    else None
                ),
                "raw_lowstate_mean_rate_hz": (
                    raw_sink.written
                    / ((trial_end_ns - trial_start_ns) * 1.0e-9)
                    if trial_start_ns is not None
                    and trial_end_ns is not None
                    and trial_end_ns > trial_start_ns
                    else None
                ),
            }
        )
        paper._atomic_json(metadata_path, metadata)
    return eligible, exit_reason, metadata_path


def standing_recovery(
    controller: segmented.SegmentedController,
    logger: segmented.ContextLogger,
    args: argparse.Namespace,
    *,
    allow_continue: bool,
) -> bool:
    """Keep policy position control active while the operator repositions."""
    logger.enabled = False
    scheduler = segmented.AbsoluteScheduler(float(controller.config.control_dt))
    misses = 0
    print("Release A fully to arm standing recovery.", flush=True)
    while controller.remote_controller.button[base.KeyMap.A] == 1:
        if controller.remote_controller.button[base.KeyMap.select] == 1:
            return False
        scheduled_ns, skipped = scheduler.wait()
        controller.set_schedule_context(scheduled_ns, skipped)
        controller.run_manual_recovery(False)

    print(
        "Standing recovery: hold A and use the joystick; release A for zero "
        "command.",
        flush=True,
    )
    if allow_continue:
        print(
            "Visually confirm a safe four-foot stance, then press START with A "
            "released to begin zero-command policy settling.",
            flush=True,
        )
    else:
        print("After securing the robot, press SELECT to end with damping.", flush=True)

    start_released = controller.remote_controller.button[base.KeyMap.start] != 1
    while True:
        if controller.remote_controller.button[base.KeyMap.select] == 1:
            return False
        a_pressed = controller.remote_controller.button[base.KeyMap.A] == 1
        start_pressed = controller.remote_controller.button[base.KeyMap.start] == 1
        if not start_pressed:
            start_released = True
        if allow_continue and start_released and start_pressed and not a_pressed:
            controller.run_manual_recovery(False)
            return True
        scheduled_ns, skipped = scheduler.wait()
        controller.set_schedule_context(scheduled_ns, skipped)
        controller.run_manual_recovery(a_pressed)
        reason, misses = segmented.safety_reason(
            controller,
            logger,
            args,
            misses,
        )
        if reason is not None:
            print(f"Recovery safety abort: {reason}", flush=True)
            controller._recovery_safety_event = {
                "reason": reason,
                "wall_time_ns": time.time_ns(),
                "robot_tick": int(controller.low_state.tick),
                "motor_lost_unitree_order": [
                    int(motor.lost) for motor in controller.low_state.motor_state
                ],
                "motor_lost_baseline_unitree_order": list(
                    controller._motor_lost_baseline
                ),
                "motor_temperature_unitree_order": [
                    int(motor.temperature)
                    for motor in controller.low_state.motor_state
                ],
            }
            print(
                json.dumps(controller._recovery_safety_event, indent=2),
                flush=True,
            )
            segmented.request_damping(controller)
            return False


def main() -> None:
    args = parse_args()
    if args.repeats < 1:
        raise ValueError("--repeats must be positive")
    if args.motor_lost_abort_increment < 1:
        raise ValueError("--motor-lost-abort-increment must be positive")
    if not (
        0.0 < args.pretrial_settle_min_s <= args.pretrial_settle_timeout_s
    ):
        raise ValueError("pretrial settle times must satisfy 0 < minimum <= timeout")
    if not (0.0 < args.pretrial_stable_window_s <= args.pretrial_settle_timeout_s):
        raise ValueError("pretrial stable window must be positive and within timeout")
    if min(
        args.pretrial_max_abs_roll_pitch,
        args.pretrial_max_body_rate,
        args.pretrial_max_joint_speed_rms,
    ) <= 0.0:
        raise ValueError("pretrial stability values must be positive")
    if not args.session_id or any(
        character not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-"
        for character in args.session_id
    ):
        raise ValueError("--session-id contains unsupported characters")

    config_path = args.config.expanduser().resolve()
    policy_path = args.policy.expanduser().resolve()
    protocol_path = args.protocol.expanduser().resolve()
    args.output_dir = args.output_dir.expanduser().resolve()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    config, config_snapshot = build_config(config_path, policy_path)

    segments = []
    protocol_snapshot = None
    for segment_id in PAPER_SEGMENTS:
        segment, snapshot = segmented.load_segment(protocol_path, segment_id)
        segments.append(segment)
        protocol_snapshot = snapshot
    queue = [
        (repeat, segment)
        for repeat in range(1, args.repeats + 1)
        for segment in segments
        if valid_trial_exists(
            args.output_dir,
            args.seed,
            segment.segment_id,
            repeat,
            segment.protocol_id,
        ) is None
    ]
    if not queue:
        print("Session already complete; no controller was started.", flush=True)
        return

    session_metadata_path = args.output_dir / "session.json"
    session_metadata = {
        "session_id": args.session_id,
        "policy_seed": args.seed,
        "repeats": args.repeats,
        "protocol_path": str(protocol_path),
        "protocol_sha256": paper._sha256(protocol_path),
        "policy_path": str(policy_path),
        "policy_sha256": paper._sha256(policy_path),
        "config_path": str(config_path),
        "config_sha256": paper._sha256(config_path),
        "motor_lost_abort_increment": args.motor_lost_abort_increment,
        "pretrial_settle": {
            "default_pose_reset": False,
            "initial_condition": "zero_command_policy_equilibrium",
            "minimum_s": args.pretrial_settle_min_s,
            "timeout_s": args.pretrial_settle_timeout_s,
            "stable_window_s": args.pretrial_stable_window_s,
            "max_abs_roll_pitch_rad": args.pretrial_max_abs_roll_pitch,
            "max_body_rate_rad_s": args.pretrial_max_body_rate,
            "max_joint_speed_rms_rad_s": args.pretrial_max_joint_speed_rms,
        },
        "status": "initializing",
        "pending_trials_at_start": [
            {"repeat": repeat, "segment": segment.segment_id}
            for repeat, segment in queue
        ],
        "start_wall_time_ns": time.time_ns(),
    }
    if session_metadata_path.exists():
        previous_session = json.loads(
            session_metadata_path.read_text(encoding="utf-8")
        )
        identity_fields = (
            "session_id",
            "policy_seed",
            "repeats",
            "protocol_sha256",
            "policy_sha256",
            "config_sha256",
            "motor_lost_abort_increment",
            "pretrial_settle",
        )
        mismatches = [
            field
            for field in identity_fields
            if previous_session.get(field) != session_metadata.get(field)
        ]
        if mismatches:
            raise RuntimeError(
                "session directory belongs to a different acquisition: "
                + ", ".join(mismatches)
            )
        session_metadata["original_start_wall_time_ns"] = previous_session.get(
            "original_start_wall_time_ns",
            previous_session.get("start_wall_time_ns"),
        )
    paper._atomic_json(session_metadata_path, session_metadata)

    bootstrap_sink = NullSink()
    bootstrap_logger = segmented.ContextLogger(bootstrap_sink)
    bootstrap_logger.enabled = False
    bootstrap_raw = segmented.RawLowStateRecorder(bootstrap_sink)
    controller: segmented.SegmentedController | None = None
    session_ok = False
    try:
        ChannelFactoryInitialize(0, args.net)
        controller = segmented.SegmentedController(
            config,
            bootstrap_logger,
            bootstrap_raw,
            20.0,
            10.0,
        )
        controller.zero_torque_state()
        controller.move_to_default_pos()
        controller.default_pos_state()
        print("Persistent position control is active for the complete session.")

        print("Initial standing recovery is available before the first trial.")
        print_next_trial(queue[0][0], queue[0][1], 1, len(queue))
        if not standing_recovery(
            controller,
            bootstrap_logger,
            args,
            allow_continue=True,
        ):
            print("Session ended during initial standing recovery.", flush=True)
            return

        stop_requested = False
        for queue_index, (repeat, segment) in enumerate(queue):
            while True:
                ready, pretrial_state = settle_before_trial(
                    controller,
                    controller.paper_logger,
                    args,
                )
                if ready:
                    break
                print(
                    "Pre-trial state was not stable; returning to standing recovery.",
                    flush=True,
                )
                if not standing_recovery(
                    controller,
                    controller.paper_logger,
                    args,
                    allow_continue=True,
                ):
                    stop_requested = True
                    break
            if stop_requested:
                break
            segmented.wait_for_deadman_rearm(controller)
            eligible, reason, metadata_path = run_trial(
                controller,
                config,
                config_snapshot,
                config_path,
                policy_path,
                protocol_path,
                protocol_snapshot,
                segment,
                repeat,
                queue_index,
                pretrial_state,
                args,
            )
            print(
                f"Trial result: eligible={eligible} reason={reason} "
                f"metadata={metadata_path}",
                flush=True,
            )
            if reason in {"safety_abort", "keyboard_interrupt", "exception"}:
                break
            if not eligible:
                print("Invalid trial stops the session after standing recovery.")
                standing_recovery(
                    controller,
                    controller.paper_logger,
                    args,
                    allow_continue=False,
                )
                break
            has_next = queue_index + 1 < len(queue)
            if not has_next:
                session_ok = True
                standing_recovery(
                    controller,
                    controller.paper_logger,
                    args,
                    allow_continue=False,
                )
                break
            next_repeat, next_segment = queue[queue_index + 1]
            print_next_trial(
                next_repeat,
                next_segment,
                queue_index + 2,
                len(queue),
            )
            if not standing_recovery(
                controller,
                controller.paper_logger,
                args,
                allow_continue=True,
            ):
                break
        else:
            session_ok = True
    except KeyboardInterrupt:
        segmented.request_damping(controller)
    except BaseException:
        segmented.request_damping(controller)
        raise
    finally:
        if controller is not None:
            segmented.request_damping(controller)
        session_metadata.update(
            {
                "status": "complete" if session_ok else "stopped",
                "end_wall_time_ns": time.time_ns(),
                "recovery_safety_event": (
                    getattr(controller, "_recovery_safety_event", None)
                    if controller is not None
                    else None
                ),
            }
        )
        paper._atomic_json(session_metadata_path, session_metadata)


if __name__ == "__main__":
    main()
