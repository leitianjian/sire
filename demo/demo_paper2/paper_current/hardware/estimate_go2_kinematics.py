#!/usr/bin/env python3
"""Offline contact-aided Go2 onboard kinematic state estimate.

The result is an onboard estimate, not external ground truth. Body-frame foot
position and velocity come from the public Go2 geometry and measured joints.
Base velocity uses zero-velocity constraints for selected stance feet and IMU
propagation when no stance foot is available. Horizontal position is integrated
and therefore drifts. Height assumes a locally flat ground plane.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


LEGS = ("FR", "FL", "RR", "RL")
MOTOR_INDICES = {
    "FR": (0, 1, 2),
    "FL": (3, 4, 5),
    "RR": (6, 7, 8),
    "RL": (9, 10, 11),
}
HIP_ORIGINS = {
    "FR": np.array([0.1934, -0.0465, 0.0]),
    "FL": np.array([0.1934, 0.0465, 0.0]),
    "RR": np.array([-0.1934, -0.0465, 0.0]),
    "RL": np.array([-0.1934, 0.0465, 0.0]),
}
SIDE = {"FR": -1.0, "FL": 1.0, "RR": -1.0, "RL": 1.0}
HIP_LINK_M = 0.0955
THIGH_M = 0.213
CALF_M = 0.213


def rotation_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rotation_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def quaternion_rotation_wxyz(quaternion: list[float]) -> np.ndarray:
    w, x, y, z = (float(value) for value in quaternion)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1.0e-12:
        raise ValueError("invalid zero-norm IMU quaternion")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def leg_kinematics(
    leg: str,
    q: np.ndarray,
    dq: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return body-frame foot position, relative velocity, and Jacobian."""
    q0, q1, q2 = (float(value) for value in q)
    rx = rotation_x(q0)
    ry1 = rotation_y(q1)
    ry12 = rotation_y(q1 + q2)
    hip_origin = HIP_ORIGINS[leg]
    thigh_origin = hip_origin + rx @ np.array([0.0, SIDE[leg] * HIP_LINK_M, 0.0])
    calf_origin = thigh_origin + rx @ ry1 @ np.array([0.0, 0.0, -THIGH_M])
    foot = calf_origin + rx @ ry12 @ np.array([0.0, 0.0, -CALF_M])

    axis_hip = np.array([1.0, 0.0, 0.0])
    axis_thigh = rx @ np.array([0.0, 1.0, 0.0])
    axis_calf = rx @ ry1 @ np.array([0.0, 1.0, 0.0])
    jacobian = np.column_stack(
        (
            np.cross(axis_hip, foot - hip_origin),
            np.cross(axis_thigh, foot - thigh_origin),
            np.cross(axis_calf, foot - calf_origin),
        )
    )
    return foot, jacobian @ dq, jacobian


class ForceContactSelector:
    def __init__(self, on: float, off: float, force_order: tuple[str, ...]) -> None:
        if off > on:
            raise ValueError("force-off must not exceed force-on")
        if sorted(force_order) != sorted(LEGS):
            raise ValueError("force-order must name FR,FL,RR,RL exactly once")
        self.on = on
        self.off = off
        self.force_order = force_order
        self.state = {leg: False for leg in LEGS}

    def update(self, values: list[float]) -> list[str]:
        by_leg = dict(zip(self.force_order, (float(value) for value in values)))
        for leg in LEGS:
            threshold = self.off if self.state[leg] else self.on
            self.state[leg] = by_leg[leg] >= threshold
        return [leg for leg in LEGS if self.state[leg]]


def kinematic_contacts(
    positions: dict[str, np.ndarray],
    rotation_world_body: np.ndarray,
    tolerance_m: float,
) -> list[str]:
    world_relative_z = {
        leg: float((rotation_world_body @ position)[2])
        for leg, position in positions.items()
    }
    minimum = min(world_relative_z.values())
    return [
        leg
        for leg in LEGS
        if world_relative_z[leg] <= minimum + tolerance_m
    ]


def robust_base_velocity_body(
    contacts: list[str],
    positions: dict[str, np.ndarray],
    velocities: dict[str, np.ndarray],
    omega_body: np.ndarray,
    residual_limit_m_s: float,
) -> tuple[np.ndarray | None, list[str], float]:
    if not contacts:
        return None, [], math.nan
    candidates = {
        leg: -(velocities[leg] + np.cross(omega_body, positions[leg]))
        for leg in contacts
    }
    stacked = np.vstack(list(candidates.values()))
    center = np.median(stacked, axis=0)
    residuals = {
        leg: float(np.linalg.norm(value - center))
        for leg, value in candidates.items()
    }
    accepted = [leg for leg in contacts if residuals[leg] <= residual_limit_m_s]
    if not accepted:
        accepted = [min(contacts, key=residuals.get)]
    estimate = np.mean(np.vstack([candidates[leg] for leg in accepted]), axis=0)
    rms = math.sqrt(
        sum(float(np.dot(candidates[leg] - estimate, candidates[leg] - estimate)) for leg in accepted)
        / len(accepted)
    )
    return estimate, accepted, rms


class ContactAidedKalmanFilter:
    """Position/velocity/accelerometer-bias filter with contact updates."""

    def __init__(self) -> None:
        self.state = np.zeros(9, dtype=float)
        self.covariance = np.diag(
            [10.0, 10.0, 0.25, 1.0, 1.0, 1.0, 0.5, 0.5, 0.5]
        )

    def predict(
        self,
        dt: float,
        measured_acceleration_world: np.ndarray,
        acceleration_noise: float,
        bias_random_walk: float,
    ) -> None:
        if dt <= 0.0:
            return
        position = self.state[0:3]
        velocity = self.state[3:6]
        bias = self.state[6:9]
        acceleration = measured_acceleration_world - bias
        self.state[0:3] = position + velocity * dt + 0.5 * acceleration * dt * dt
        self.state[3:6] = velocity + acceleration * dt

        transition = np.eye(9)
        transition[0:3, 3:6] = np.eye(3) * dt
        transition[0:3, 6:9] = -0.5 * np.eye(3) * dt * dt
        transition[3:6, 6:9] = -np.eye(3) * dt
        process = np.zeros((9, 9), dtype=float)
        process[0:3, 0:3] = np.eye(3) * (0.25 * acceleration_noise**2 * dt**4)
        process[3:6, 3:6] = np.eye(3) * (acceleration_noise**2 * dt**2)
        process[6:9, 6:9] = np.eye(3) * (bias_random_walk**2 * dt)
        self.covariance = transition @ self.covariance @ transition.T + process

    def _update(self, measurement: np.ndarray, observation: np.ndarray, noise: np.ndarray) -> None:
        innovation = measurement - observation @ self.state
        innovation_covariance = observation @ self.covariance @ observation.T + noise
        gain = self.covariance @ observation.T @ np.linalg.inv(innovation_covariance)
        self.state = self.state + gain @ innovation
        identity = np.eye(9)
        correction = identity - gain @ observation
        self.covariance = (
            correction @ self.covariance @ correction.T + gain @ noise @ gain.T
        )

    def update_velocity(self, velocity_world: np.ndarray, standard_deviation: float) -> None:
        observation = np.zeros((3, 9), dtype=float)
        observation[:, 3:6] = np.eye(3)
        self._update(
            np.asarray(velocity_world, dtype=float),
            observation,
            np.eye(3) * max(standard_deviation, 1.0e-4) ** 2,
        )

    def update_height(self, height_m: float, standard_deviation: float) -> None:
        observation = np.zeros((1, 9), dtype=float)
        observation[0, 2] = 1.0
        self._update(
            np.asarray([height_m], dtype=float),
            observation,
            np.asarray([[max(standard_deviation, 1.0e-4) ** 2]], dtype=float),
        )


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0])
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_lowstate_log", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--contact-source",
        choices=("kinematic_lowest", "raw", "estimated"),
        default="kinematic_lowest",
    )
    parser.add_argument(
        "--force-order",
        default=None,
        help="confirmed raw array order, e.g. FR,FL,RR,RL; required for force contacts",
    )
    parser.add_argument("--force-on", type=float, default=20.0)
    parser.add_argument("--force-off", type=float, default=10.0)
    parser.add_argument("--height-tolerance", type=float, default=0.025)
    parser.add_argument("--contact-residual-limit", type=float, default=0.60)
    parser.add_argument("--max-imu-propagation", type=float, default=0.15)
    parser.add_argument("--gravity", type=float, default=9.81)
    parser.add_argument("--filter-mode", choices=("direct", "kalman"), default="kalman")
    parser.add_argument("--acceleration-noise", type=float, default=1.5)
    parser.add_argument("--bias-random-walk", type=float, default=0.05)
    parser.add_argument("--contact-velocity-noise", type=float, default=0.08)
    parser.add_argument("--height-noise", type=float, default=0.025)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    raw_rows = []
    invalid_json_lines = 0
    for line in args.raw_lowstate_log.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            raw_rows.append(json.loads(line))
        except json.JSONDecodeError:
            invalid_json_lines += 1
    if len(raw_rows) < 2:
        raise RuntimeError("raw LowState log requires at least two samples")
    timestamps_ns = np.asarray(
        [int(row["monotonic_ns"]) for row in raw_rows],
        dtype=np.int64,
    )
    timestamp_differences_ns = np.diff(timestamps_ns)
    if np.any(timestamp_differences_ns <= 0):
        raise RuntimeError("raw LowState timestamps must be strictly increasing")
    sequences = np.asarray([int(row["sequence"]) for row in raw_rows], dtype=np.int64)
    sequence_gaps = int(np.maximum(np.diff(sequences) - 1, 0).sum())
    median_sample_rate_hz = 1.0e9 / float(np.median(timestamp_differences_ns))

    force_selector = None
    if args.contact_source in {"raw", "estimated"}:
        if args.force_order is None:
            raise ValueError("--force-order is required for force-based contacts")
        force_selector = ForceContactSelector(
            args.force_on,
            args.force_off,
            tuple(item.strip() for item in args.force_order.split(",")),
        )

    output_rows: list[dict[str, Any]] = []
    position_world = np.zeros(3, dtype=float)
    velocity_world = np.zeros(3, dtype=float)
    previous_velocity_world = velocity_world.copy()
    previous_ns = int(raw_rows[0]["monotonic_ns"])
    last_contact_ns = previous_ns
    initial_ns = previous_ns
    kalman_filter = ContactAidedKalmanFilter()

    for raw in raw_rows:
        monotonic_ns = int(raw["monotonic_ns"])
        dt = max(0.0, (monotonic_ns - previous_ns) * 1.0e-9)
        previous_ns = monotonic_ns
        imu = raw["imu"]
        rotation_world_body = quaternion_rotation_wxyz(imu["quaternion_wxyz"])
        omega_body = np.asarray(imu["gyroscope_rad_s"], dtype=float)
        specific_force_body = np.asarray(imu["accelerometer_m_s2"], dtype=float)
        acceleration_world = (
            rotation_world_body @ specific_force_body
            + np.array([0.0, 0.0, -args.gravity])
        )
        motors = raw["motor_state_unitree_order"]

        foot_positions: dict[str, np.ndarray] = {}
        foot_velocities: dict[str, np.ndarray] = {}
        for leg in LEGS:
            indices = MOTOR_INDICES[leg]
            q = np.asarray([motors[index]["q"] for index in indices], dtype=float)
            dq = np.asarray([motors[index]["dq"] for index in indices], dtype=float)
            foot, foot_velocity, _ = leg_kinematics(leg, q, dq)
            foot_positions[leg] = foot
            foot_velocities[leg] = foot_velocity

        if args.contact_source == "kinematic_lowest":
            proposed_contacts = kinematic_contacts(
                foot_positions,
                rotation_world_body,
                args.height_tolerance,
            )
        else:
            field = (
                "foot_force_raw_indices"
                if args.contact_source == "raw"
                else "foot_force_est_indices"
            )
            proposed_contacts = force_selector.update(raw[field])

        velocity_body, contacts, residual = robust_base_velocity_body(
            proposed_contacts,
            foot_positions,
            foot_velocities,
            omega_body,
            args.contact_residual_limit,
        )
        kinematic_velocity_world = (
            rotation_world_body @ velocity_body
            if velocity_body is not None else None
        )
        if velocity_body is not None:
            last_contact_ns = monotonic_ns
            height_values = [
                -float((rotation_world_body @ foot_positions[leg])[2])
                for leg in contacts
            ]
            height_estimate = float(np.median(height_values))
        else:
            since_contact = (monotonic_ns - last_contact_ns) * 1.0e-9
            height_estimate = math.nan

        if args.filter_mode == "kalman":
            kalman_filter.predict(
                dt,
                acceleration_world,
                args.acceleration_noise,
                args.bias_random_walk,
            )
            if kinematic_velocity_world is not None:
                measurement_noise = args.contact_velocity_noise + (
                    residual if math.isfinite(residual) else 0.0
                )
                kalman_filter.update_velocity(
                    kinematic_velocity_world,
                    measurement_noise,
                )
                kalman_filter.update_height(height_estimate, args.height_noise)
                estimate_mode = "contact_kalman"
                valid = True
            else:
                since_contact = (monotonic_ns - last_contact_ns) * 1.0e-9
                estimate_mode = "imu_prediction"
                valid = since_contact <= args.max_imu_propagation
            position_world = kalman_filter.state[0:3].copy()
            velocity_world = kalman_filter.state[3:6].copy()
            acceleration_bias_world = kalman_filter.state[6:9].copy()
        else:
            acceleration_bias_world = np.zeros(3, dtype=float)
            if kinematic_velocity_world is not None:
                velocity_world = kinematic_velocity_world
                estimate_mode = "contact_kinematic"
                valid = True
            else:
                since_contact = (monotonic_ns - last_contact_ns) * 1.0e-9
                if since_contact <= args.max_imu_propagation:
                    velocity_world = velocity_world + acceleration_world * dt
                    estimate_mode = "short_imu_propagation"
                    valid = True
                else:
                    velocity_world[:] = math.nan
                    estimate_mode = "invalid_no_contact"
                    valid = False
            if dt > 0.0 and np.all(np.isfinite(velocity_world)):
                integration_start_velocity = (
                    previous_velocity_world
                    if np.all(np.isfinite(previous_velocity_world))
                    else velocity_world
                )
                position_world += 0.5 * (
                    integration_start_velocity + velocity_world
                ) * dt
                if math.isfinite(height_estimate):
                    position_world[2] = height_estimate
            previous_velocity_world = velocity_world.copy()

        velocity_body_estimate = rotation_world_body.T @ velocity_world
        contact_velocity_body_measurement = (
            velocity_body if velocity_body is not None else np.full(3, math.nan)
        )
        contact_velocity_world_measurement = (
            kinematic_velocity_world
            if kinematic_velocity_world is not None else np.full(3, math.nan)
        )

        row: dict[str, Any] = {
            "sequence": int(raw["sequence"]),
            "robot_tick": int(raw["robot_tick"]),
            "monotonic_ns": monotonic_ns,
            "time_s": (monotonic_ns - initial_ns) * 1.0e-9,
            "phase": raw.get("phase", ""),
            "contact_source": args.contact_source,
            "proposed_contacts": ";".join(proposed_contacts),
            "accepted_contacts": ";".join(contacts),
            "accepted_contact_count": len(contacts),
            "contact_velocity_residual_m_s": residual,
            "estimate_mode": estimate_mode,
            "estimate_valid": int(valid),
            "base_velocity_world_x_m_s": float(velocity_world[0]),
            "base_velocity_world_y_m_s": float(velocity_world[1]),
            "base_velocity_world_z_m_s": float(velocity_world[2]),
            "base_velocity_body_x_m_s": float(velocity_body_estimate[0]),
            "base_velocity_body_y_m_s": float(velocity_body_estimate[1]),
            "base_velocity_body_z_m_s": float(velocity_body_estimate[2]),
            "contact_velocity_body_x_m_s": float(contact_velocity_body_measurement[0]),
            "contact_velocity_body_y_m_s": float(contact_velocity_body_measurement[1]),
            "contact_velocity_body_z_m_s": float(contact_velocity_body_measurement[2]),
            "contact_velocity_world_x_m_s": float(contact_velocity_world_measurement[0]),
            "contact_velocity_world_y_m_s": float(contact_velocity_world_measurement[1]),
            "contact_velocity_world_z_m_s": float(contact_velocity_world_measurement[2]),
            "base_position_world_x_m": float(position_world[0]),
            "base_position_world_y_m": float(position_world[1]),
            "base_position_world_z_m": float(position_world[2]),
            "base_height_flat_ground_m": float(position_world[2]),
            "acceleration_bias_world_x_m_s2": float(acceleration_bias_world[0]),
            "acceleration_bias_world_y_m_s2": float(acceleration_bias_world[1]),
            "acceleration_bias_world_z_m_s2": float(acceleration_bias_world[2]),
            "imu_acceleration_world_x_m_s2": float(acceleration_world[0]),
            "imu_acceleration_world_y_m_s2": float(acceleration_world[1]),
            "imu_acceleration_world_z_m_s2": float(acceleration_world[2]),
        }
        for leg in LEGS:
            foot_position_world = position_world + rotation_world_body @ foot_positions[leg]
            foot_velocity_world = velocity_world + rotation_world_body @ (
                foot_velocities[leg] + np.cross(omega_body, foot_positions[leg])
            )
            for axis, suffix in enumerate(("x", "y", "z")):
                row[f"{leg}_foot_position_body_{suffix}_m"] = float(
                    foot_positions[leg][axis]
                )
                row[f"{leg}_foot_velocity_body_{suffix}_m_s"] = float(
                    foot_velocities[leg][axis]
                )
                row[f"{leg}_foot_position_world_{suffix}_m"] = float(
                    foot_position_world[axis]
                )
                row[f"{leg}_foot_velocity_world_{suffix}_m_s"] = float(
                    foot_velocity_world[axis]
                )
        output_rows.append(row)

    write_csv(args.output, output_rows)
    valid_rows = [row for row in output_rows if row["estimate_valid"]]
    summary = {
        "input": str(args.raw_lowstate_log.resolve()),
        "output": str(args.output.resolve()),
        "samples": len(output_rows),
        "invalid_json_lines": invalid_json_lines,
        "valid_samples": len(valid_rows),
        "valid_fraction": len(valid_rows) / len(output_rows),
        "duration_s": (int(timestamps_ns[-1]) - int(timestamps_ns[0])) * 1.0e-9,
        "median_sample_rate_hz": median_sample_rate_hz,
        "sequence_gaps": sequence_gaps,
        "contact_source": args.contact_source,
        "filter_mode": args.filter_mode,
        "acceleration_noise_m_s2": args.acceleration_noise,
        "bias_random_walk_m_s2_sqrt_s": args.bias_random_walk,
        "contact_velocity_noise_m_s": args.contact_velocity_noise,
        "height_noise_m": args.height_noise,
        "mean_contact_residual_m_s": (
            float(
                np.nanmean(
                    [row["contact_velocity_residual_m_s"] for row in valid_rows]
                )
            )
            if valid_rows
            else math.nan
        ),
        "final_integrated_xy_m": [
            output_rows[-1]["base_position_world_x_m"],
            output_rows[-1]["base_position_world_y_m"],
        ],
        "interpretation": (
            "onboard contact-aided kinematic estimate; horizontal position is "
            "integrated and drifting; not external ground truth"
        ),
        "slip_limitation": (
            "stance-foot world velocity is part of the estimator constraint and "
            "is not an independent slip measurement"
        ),
        "contact_limitation": (
            "kinematic_lowest contact selection is a geometry-based heuristic"
            if args.contact_source == "kinematic_lowest"
            else "force contacts depend on the supplied calibration and leg order"
        ),
    }
    summary_path = args.output.with_suffix(".summary.json")
    summary_path.write_text(
        json.dumps(summary, indent=2, allow_nan=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, allow_nan=True))


if __name__ == "__main__":
    main()
