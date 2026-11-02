#!/usr/bin/env python3
"""Collect a complete operator-commanded Go2 coverage session."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import socket
import time
from typing import Any

import numpy as np
import yaml

import deploy_go2 as base
import deploy_go2_paper as paper
import deploy_go2_paper_session as paper_session
import deploy_go2_segmented as segmented

from unitree_sdk2py.core.channel import ChannelFactoryInitialize


@dataclass(frozen=True)
class CoverageSpec:
    protocol_id: str
    stable_entry_s: float
    required_stable_s: float
    maximum_session_s: float
    print_interval_s: float
    classes: dict[str, dict[str, Any]]
    order: tuple[str, ...]
    snapshot: dict[str, Any]


def load_coverage(path: Path, seed: int) -> CoverageSpec:
    snapshot = yaml.safe_load(path.read_text(encoding="utf-8"))
    order = snapshot["orders"].get(seed, snapshot["orders"].get(str(seed)))
    if not isinstance(order, list) or not order:
        raise ValueError(f"coverage order missing for seed {seed}")
    classes = snapshot["classes"]
    if any(name not in classes for name in order):
        raise ValueError("coverage order references an unknown class")
    spec = CoverageSpec(
        protocol_id=str(snapshot["protocol_id"]),
        stable_entry_s=float(snapshot["stable_entry_s"]),
        required_stable_s=float(snapshot["required_stable_s"]),
        maximum_session_s=float(snapshot["maximum_session_s"]),
        print_interval_s=float(snapshot["print_interval_s"]),
        classes=classes,
        order=tuple(str(name) for name in order),
        snapshot=snapshot,
    )
    if min(
        spec.stable_entry_s,
        spec.required_stable_s,
        spec.maximum_session_s,
        spec.print_interval_s,
    ) <= 0.0:
        raise ValueError("coverage durations must be positive")
    return spec


class CoverageTracker:
    def __init__(self, spec: CoverageSpec) -> None:
        self.spec = spec
        self.states = {
            name: {
                "match_start_ns": None,
                "accepted_start_ns": None,
                "accepted_start_sequence": None,
            }
            for name in spec.order
        }
        self.completed: list[dict[str, Any]] = []
        self.just_completed: dict[str, Any] | None = None

    @property
    def complete(self) -> bool:
        return len(self.completed_names) >= len(self.spec.order)

    @property
    def completed_names(self) -> set[str]:
        return {str(item["class"]) for item in self.completed}

    @property
    def missing(self) -> tuple[str, ...]:
        completed = self.completed_names
        return tuple(name for name in self.spec.order if name not in completed)

    @property
    def target(self) -> str | None:
        missing = self.missing
        return missing[0] if missing else None

    @property
    def target_index(self) -> int:
        target = self.target
        return len(self.spec.order) if target is None else self.spec.order.index(target)

    def command_matches(self, name: str, command: np.ndarray) -> bool:
        rule = self.spec.classes[name]
        if rule["axis"] == "none":
            return bool(np.max(np.abs(command)) <= float(rule["max_abs_command"]))
        axis = int(rule["axis"])
        signed = float(command[axis]) * int(rule["sign"])
        lower, upper = (float(value) for value in rule["magnitude"])
        cross = np.delete(np.abs(command), axis)
        return bool(
            lower <= signed <= upper
            and np.max(cross) <= float(rule["max_cross_axis"])
        )

    def observed_class(self, command: np.ndarray) -> str:
        matches = [
            name for name in self.spec.classes if self.command_matches(name, command)
        ]
        return matches[0] if len(matches) == 1 else "unclassified"

    def observe(self, sample: dict[str, Any]) -> dict[str, Any]:
        command = np.asarray(sample["command"], dtype=float)
        now_ns = int(sample["monotonic_ns"])
        sequence = int(sample["sequence"])
        target = self.target
        observed = self.observed_class(command)
        self.just_completed = None
        candidate = observed if observed in self.spec.classes else None
        for name, state in self.states.items():
            if name != candidate:
                state["match_start_ns"] = None
                state["accepted_start_ns"] = None
                state["accepted_start_sequence"] = None
        if candidate is None:
            matched_s = 0.0
            accepted_s = 0.0
        else:
            state = self.states[candidate]
            if state["match_start_ns"] is None:
                state["match_start_ns"] = now_ns
            matched_s = (now_ns - int(state["match_start_ns"])) * 1.0e-9
            if matched_s >= self.spec.stable_entry_s:
                if state["accepted_start_ns"] is None:
                    state["accepted_start_ns"] = now_ns
                    state["accepted_start_sequence"] = sequence
                accepted_s = (now_ns - int(state["accepted_start_ns"])) * 1.0e-9
            else:
                accepted_s = 0.0

            if accepted_s >= self.spec.required_stable_s:
                occurrence = 1 + sum(
                    item["class"] == candidate for item in self.completed
                )
                satisfies_required = candidate not in self.completed_names
                completion = {
                    "class": candidate,
                    "planned_order_index": self.spec.order.index(candidate),
                    "completion_index": len(self.completed),
                    "occurrence": occurrence,
                    "satisfies_required_coverage": satisfies_required,
                    "accepted_start_sequence": state["accepted_start_sequence"],
                    "accepted_end_sequence": sequence,
                    "accepted_start_monotonic_ns": state["accepted_start_ns"],
                    "accepted_end_monotonic_ns": now_ns,
                    "accepted_duration_s": accepted_s,
                }
                self.completed.append(completion)
                self.just_completed = completion
                state["match_start_ns"] = None
                state["accepted_start_ns"] = None
                state["accepted_start_sequence"] = None

        return {
            "coverage_target": target,
            "coverage_observed_class": observed,
            "coverage_target_match": candidate is not None,
            "coverage_counted_class": candidate,
            "coverage_matched_s": matched_s,
            "coverage_accepted_s": accepted_s,
            "coverage_complete": self.complete,
        }


class OperatorLogger:
    def __init__(
        self,
        sink: paper.AsyncJsonlLogger,
        tracker: CoverageTracker,
        context: dict[str, Any],
    ) -> None:
        self.sink = sink
        self.tracker = tracker
        self.context = context
        self.enabled = True
        self.tracking_enabled = False
        self.last_sample: dict[str, Any] | None = None

    def submit(self, sample: dict[str, Any]) -> None:
        enriched = dict(sample)
        enriched.update(self.context)
        if self.tracking_enabled:
            enriched.update(self.tracker.observe(enriched))
        else:
            enriched.update(
                {
                    "coverage_target": self.tracker.target,
                    "coverage_observed_class": "not_tracking",
                    "coverage_target_match": False,
                    "coverage_counted_class": None,
                    "coverage_matched_s": 0.0,
                    "coverage_accepted_s": 0.0,
                    "coverage_complete": self.tracker.complete,
                }
            )
        enriched["schema_version"] = segmented.SCHEMA_VERSION
        enriched["foot_contact_source"] = "none"
        enriched["foot_contact_valid"] = False
        enriched["foot_force_order_confirmed"] = False
        enriched["foot_force_policy_order"] = None
        enriched["foot_force_est_policy_order"] = None
        enriched["foot_contact_policy_order"] = None
        self.last_sample = enriched
        if self.enabled:
            self.sink.submit(enriched)

    @property
    def dropped(self) -> int:
        return self.sink.dropped


def print_target(tracker: CoverageTracker) -> None:
    target = tracker.target
    if target is None:
        print("All required motion classes have been collected.", flush=True)
        return
    rule = tracker.spec.classes[target]
    print("", flush=True)
    print("=" * 72, flush=True)
    print(
        f"TARGET {tracker.target_index + 1}/{len(tracker.spec.order)}: "
        f"{target.upper()}",
        flush=True,
    )
    print(str(rule["description"]), flush=True)
    print(
        f"Hold the target band for {tracker.spec.stable_entry_s:.2f} s to "
        f"stabilize, then {tracker.spec.required_stable_s:.2f} s for coverage.",
        flush=True,
    )
    completed = [
        name for name in tracker.spec.order if name in tracker.completed_names
    ]
    remaining = list(tracker.spec.order[tracker.target_index :])
    print(f"COLLECTED SO FAR: {completed or ['none']}", flush=True)
    print(f"STILL REQUIRED: {remaining}", flush=True)
    print("=" * 72, flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("net")
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--policy", required=True, type=Path)
    parser.add_argument("--coverage", required=True, type=Path)
    parser.add_argument("--seed", required=True, type=int, choices=(1, 2, 3, 4))
    parser.add_argument("--repeat", required=True, type=int)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--notes", default="")
    parser.add_argument("--queue-size", type=int, default=8192)
    parser.add_argument("--flush-every", type=int, default=25)
    parser.add_argument("--raw-queue-size", type=int, default=65536)
    parser.add_argument("--raw-flush-every", type=int, default=250)
    parser.add_argument("--max-low-state-age-ms", type=float, default=100.0)
    parser.add_argument("--max-consecutive-deadline-misses", type=int, default=3)
    parser.add_argument("--motor-lost-abort-increment", type=int, default=10)
    parser.add_argument("--max-motor-temp", type=int, default=None)
    parser.add_argument("--max-abs-roll", type=float, default=None)
    parser.add_argument("--max-abs-pitch", type=float, default=None)
    parser.add_argument("--abort-ramp-s", type=float, default=0.20)
    parser.add_argument("--pretrial-settle-min-s", type=float, default=2.0)
    parser.add_argument("--pretrial-settle-timeout-s", type=float, default=5.0)
    parser.add_argument("--pretrial-stable-window-s", type=float, default=0.5)
    parser.add_argument("--pretrial-max-abs-roll-pitch", type=float, default=0.20)
    parser.add_argument("--pretrial-max-body-rate", type=float, default=0.25)
    parser.add_argument("--pretrial-max-joint-speed-rms", type=float, default=0.25)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.repeat < 1:
        raise ValueError("--repeat must be positive")
    config_path = args.config.expanduser().resolve()
    policy_path = args.policy.expanduser().resolve()
    coverage_path = args.coverage.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    config, config_snapshot = paper_session.build_config(config_path, policy_path)
    coverage = load_coverage(coverage_path, args.seed)

    stem = f"{time.strftime('%Y%m%d_%H%M%S')}_{paper._safe_name(args.session_id)}"
    log_path = output_dir / f"{stem}.jsonl"
    raw_path = output_dir / f"{stem}.lowstate.jsonl"
    metadata_path = output_dir / f"{stem}.metadata.json"
    if any(path.exists() for path in (log_path, raw_path, metadata_path)):
        raise FileExistsError(stem)

    sink = paper.AsyncJsonlLogger(log_path, args.queue_size, args.flush_every)
    raw_sink = paper.AsyncJsonlLogger(
        raw_path,
        args.raw_queue_size,
        args.raw_flush_every,
    )
    tracker = CoverageTracker(coverage)
    context = {
        "record_class": "operator_coverage",
        "eligible_for_analysis": True,
        "protocol_id": coverage.protocol_id,
        "session_id": args.session_id,
        "repeat": args.repeat,
        "policy_seed": args.seed,
        "checkpoint_iteration": 1200,
        "command_source": "operator_joystick",
    }
    logger = OperatorLogger(sink, tracker, context)
    raw_recorder = segmented.RawLowStateRecorder(raw_sink)
    controller: segmented.SegmentedController | None = None
    start_ns: int | None = None
    end_ns: int | None = None
    exit_reason = "initialization_failed"
    eligible = False
    logger_close_errors: list[str] = []
    metadata = {
        "schema_version": segmented.SCHEMA_VERSION,
        "status": "initializing",
        "session_id": args.session_id,
        "repeat": args.repeat,
        "policy_seed": args.seed,
        "checkpoint_iteration": 1200,
        "policy_path": str(policy_path),
        "policy_sha256": paper._sha256(policy_path),
        "config_path": str(config_path),
        "config_sha256": paper._sha256(config_path),
        "config": config_snapshot,
        "coverage_path": str(coverage_path),
        "coverage_sha256": paper._sha256(coverage_path),
        "coverage": coverage.snapshot,
        "coverage_order": list(coverage.order),
        "motor_lost_abort_increment": args.motor_lost_abort_increment,
        "pretrial_settle": {
            "minimum_s": args.pretrial_settle_min_s,
            "timeout_s": args.pretrial_settle_timeout_s,
            "stable_window_s": args.pretrial_stable_window_s,
            "max_abs_roll_pitch_rad": args.pretrial_max_abs_roll_pitch,
            "max_body_rate_rad_s": args.pretrial_max_body_rate,
            "max_joint_speed_rms_rad_s": args.pretrial_max_joint_speed_rms,
        },
        "notes": args.notes,
        "hostname": socket.gethostname(),
        "network_interface": args.net,
        "log_path": str(log_path),
        "raw_lowstate_log_path": str(raw_path),
        "start_wall_time_ns": time.time_ns(),
    }
    paper._atomic_json(metadata_path, metadata)

    try:
        ChannelFactoryInitialize(0, args.net)
        controller = segmented.SegmentedController(
            config,
            logger,
            raw_recorder,
            20.0,
            10.0,
        )
        logger.enabled = False
        controller.zero_torque_state()
        controller.move_to_default_pos()
        controller.default_pos_state()
        print("Initial standing recovery is available before collection.")
        if not paper_session.standing_recovery(
            controller,
            logger,
            args,
            allow_continue=True,
        ):
            exit_reason = "operator_ended_before_collection"
            segmented.request_damping(controller)
            return
        ready, pretrial_state = paper_session.settle_before_trial(
            controller,
            logger,
            args,
        )
        if not ready:
            exit_reason = "pretrial_state_not_ready"
            paper_session.standing_recovery(
                controller,
                logger,
                args,
                allow_continue=False,
            )
            segmented.request_damping(controller)
            return

        metadata["pretrial_state"] = pretrial_state
        metadata["motor_lost_baseline_unitree_order"] = list(
            controller._motor_lost_baseline
        )
        metadata["status"] = "recording"
        paper._atomic_json(metadata_path, metadata)
        start_ns = time.monotonic_ns()
        raw_recorder.update_context(
            phase=f"target_{tracker.target}",
            phase_index=tracker.target_index,
            planned_normalized_command=np.zeros(3, dtype=np.float32),
        )
        raw_recorder.start(
            protocol_id=coverage.protocol_id,
            segment_id="operator_coverage",
            repeat=args.repeat,
            policy_seed=args.seed,
            checkpoint_iteration=1200,
            trial_start_ns=start_ns,
        )
        logger.tracking_enabled = True
        logger.enabled = True
        scheduler = segmented.AbsoluteScheduler(float(config.control_dt))
        misses = 0
        last_print_ns = 0
        print_target(tracker)
        while not tracker.complete:
            if controller.remote_controller.button[base.KeyMap.select] == 1:
                raise segmented.SafetyAbort("remote_select")
            a_pressed = controller.remote_controller.button[base.KeyMap.A] == 1
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
                raise segmented.SafetyAbort(reason)
            sample = logger.last_sample or {}
            command = np.asarray(sample.get("command", [0.0, 0.0, 0.0]), dtype=float)
            raw_recorder.update_context(
                phase=f"target_{tracker.target or 'complete'}",
                phase_index=tracker.target_index,
                planned_normalized_command=command,
            )
            if tracker.just_completed is not None:
                window_kind = (
                    "COLLECTED"
                    if tracker.just_completed["satisfies_required_coverage"]
                    else "ADDITIONAL WINDOW"
                )
                print(
                    f"{window_kind}: {tracker.just_completed['class']} "
                    f"occurrence={tracker.just_completed['occurrence']} "
                    f"({tracker.just_completed['accepted_duration_s']:.2f} s)",
                    flush=True,
                )
                print_target(tracker)
            now_ns = time.monotonic_ns()
            if now_ns - last_print_ns >= int(coverage.print_interval_s * 1.0e9):
                print(
                        f"target={sample.get('coverage_target')} "
                        f"observed={sample.get('coverage_observed_class')} "
                        f"counting={sample.get('coverage_counted_class')} "
                    f"progress={sample.get('coverage_accepted_s', 0.0):.2f}/"
                    f"{coverage.required_stable_s:.2f}s "
                    f"command={np.round(command, 2).tolist()}",
                    flush=True,
                )
                last_print_ns = now_ns
            if (now_ns - start_ns) * 1.0e-9 >= coverage.maximum_session_s:
                exit_reason = "coverage_timeout"
                break

        end_ns = time.monotonic_ns()
        eligible = tracker.complete
        exit_reason = "coverage_completed" if eligible else exit_reason
    except KeyboardInterrupt:
        exit_reason = "keyboard_interrupt"
        segmented.request_damping(controller)
    except segmented.SafetyAbort as error:
        exit_reason = f"safety_abort:{error}"
        segmented.request_damping(controller)
    except BaseException:
        exit_reason = "exception"
        segmented.request_damping(controller)
        raise
    finally:
        if start_ns is not None and end_ns is None:
            end_ns = time.monotonic_ns()
        logger.tracking_enabled = False
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
            if exit_reason == "coverage_completed":
                exit_reason = "logging_invalid"
        metadata.update(
            {
                "status": "complete" if eligible else "invalid",
                "eligible_for_analysis": eligible,
                "exit_reason": exit_reason,
                "coverage_windows": tracker.completed,
                "actual_completion_order": [
                    item["class"]
                    for item in tracker.completed
                    if item["satisfies_required_coverage"]
                ],
                "missing_classes": list(tracker.missing),
                "samples_written": sink.written,
                "samples_dropped": sink.dropped,
                "raw_lowstate_samples_written": raw_sink.written,
                "raw_lowstate_samples_dropped": raw_sink.dropped,
                "raw_lowstate_recorder_error": raw_recorder.error,
                "logger_close_errors": logger_close_errors,
                "session_total_skipped_deadlines": (
                    controller._total_skipped_deadlines
                    if controller is not None
                    else 0
                ),
                "raw_lowstate_mean_rate_hz": (
                    raw_sink.written / ((end_ns - start_ns) * 1.0e-9)
                    if start_ns is not None
                    and end_ns is not None
                    and end_ns > start_ns
                    else None
                ),
                "duration_s": (
                    (end_ns - start_ns) * 1.0e-9
                    if start_ns is not None and end_ns is not None
                    else None
                ),
                "end_wall_time_ns": time.time_ns(),
            }
        )
        paper._atomic_json(metadata_path, metadata)

    if controller is not None and exit_reason in {
        "coverage_completed",
        "coverage_timeout",
        "logging_invalid",
    }:
        print("Experimental logs are closed; standing recovery remains active.", flush=True)
        paper_session.standing_recovery(
            controller,
            logger,
            args,
            allow_continue=False,
        )
    segmented.request_damping(controller)
    print(f"Session result: {exit_reason}; metadata={metadata_path}", flush=True)


if __name__ == "__main__":
    main()
