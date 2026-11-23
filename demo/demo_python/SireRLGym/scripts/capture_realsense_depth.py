from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np


def _require_realsense():
    try:
        import pyrealsense2 as rs
    except ImportError as exc:  # pragma: no cover - hardware dependency
        raise ImportError(
            "pyrealsense2 is not installed in the current environment. "
            "Please install librealsense Python bindings in the `unitree-rl` env first."
        ) from exc
    return rs


def _make_camera_pose(x: float, y: float, z: float, roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)

    forward = np.array(
        [
            math.cos(pitch) * math.cos(yaw),
            math.cos(pitch) * math.sin(yaw),
            math.sin(pitch),
        ],
        dtype=np.float32,
    )
    forward /= np.linalg.norm(forward)

    world_up = np.array([0.0, 0.0, 1.0], dtype=np.float32)
    right = np.cross(forward, world_up)
    right /= np.linalg.norm(right)
    camera_up = np.cross(right, forward)
    camera_up /= np.linalg.norm(camera_up)
    down = -camera_up

    if abs(roll) > 1e-6:
        c = math.cos(roll)
        s = math.sin(roll)
        right_rot = c * right + s * down
        down_rot = -s * right + c * down
        right, down = right_rot, down_rot

    rotation = np.stack([right, down, forward], axis=1)
    transform = np.eye(4, dtype=np.float32)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray([x, y, z], dtype=np.float32)
    return transform


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture a short RealSense depth sequence and save it as npz.")
    parser.add_argument("--duration", type=float, default=4.0, help="Capture duration in seconds.")
    parser.add_argument("--fps", type=int, default=15, help="Depth stream FPS.")
    parser.add_argument("--width", type=int, default=640, help="Depth frame width.")
    parser.add_argument("--height", type=int, default=480, help="Depth frame height.")
    parser.add_argument("--warmup-frames", type=int, default=20, help="Frames to discard before recording.")
    parser.add_argument("--depth-min", type=float, default=0.18, help="Minimum valid depth in meters.")
    parser.add_argument("--depth-max", type=float, default=4.0, help="Maximum valid depth in meters.")
    parser.add_argument("--camera-height", type=float, default=0.40, help="Camera height in world frame.")
    parser.add_argument("--camera-pitch-deg", type=float, default=-32.0, help="Camera pitch in degrees.")
    parser.add_argument("--camera-roll-deg", type=float, default=0.0, help="Camera roll in degrees.")
    parser.add_argument("--camera-yaw-deg", type=float, default=0.0, help="Camera yaw in degrees.")
    parser.add_argument("--timeout-ms", type=int, default=5000, help="Frame wait timeout in milliseconds.")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("resources/scene_inputs/realsense_capture.npz"),
        help="Where to save the captured depth sequence.",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    rs = _require_realsense()

    pipeline = rs.pipeline()
    rs_config = rs.config()
    rs_config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)

    output_path = args.output.expanduser()
    if not output_path.is_absolute():
        output_path = (Path.cwd() / output_path).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    pose = _make_camera_pose(
        x=0.0,
        y=0.0,
        z=args.camera_height,
        roll_deg=args.camera_roll_deg,
        pitch_deg=args.camera_pitch_deg,
        yaw_deg=args.camera_yaw_deg,
    )

    profile = pipeline.start(rs_config)
    try:
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = float(depth_sensor.get_depth_scale())
        depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        intr = depth_profile.get_intrinsics()

        intrinsics = np.asarray(
            [
                intr.width,
                intr.height,
                intr.fx,
                intr.fy,
                intr.ppx,
                intr.ppy,
                args.depth_min,
                args.depth_max,
            ],
            dtype=np.float32,
        )

        metadata = {
            "source": "realsense_d435i",
            "duration_s": float(args.duration),
            "fps": int(args.fps),
            "width": int(intr.width),
            "height": int(intr.height),
            "depth_scale": depth_scale,
            "camera_height": float(args.camera_height),
            "camera_pitch_deg": float(args.camera_pitch_deg),
            "camera_roll_deg": float(args.camera_roll_deg),
            "camera_yaw_deg": float(args.camera_yaw_deg),
            "capture_mode": "static_camera_depth_sequence",
        }

        for _ in range(max(0, args.warmup_frames)):
            pipeline.wait_for_frames(timeout_ms=args.timeout_ms)

        depth_frames: list[np.ndarray] = []
        pose_frames: list[np.ndarray] = []
        timestamps: list[float] = []
        start_t = time.time()
        deadline = start_t + float(args.duration)
        frame_idx = 0

        while time.time() < deadline:
            frameset = pipeline.wait_for_frames(timeout_ms=args.timeout_ms)
            depth_frame = frameset.get_depth_frame()
            if not depth_frame:
                continue

            depth_raw = np.asanyarray(depth_frame.get_data())
            depth_m = depth_raw.astype(np.float32) * depth_scale
            depth_m[(depth_m < args.depth_min) | (depth_m > args.depth_max)] = 0.0

            depth_frames.append(depth_m)
            pose_frames.append(pose.copy())
            timestamps.append(float(time.time() - start_t))
            frame_idx += 1

            if frame_idx % 10 == 0:
                print(f"capture_progress frames={frame_idx} elapsed_s={timestamps[-1]:.2f}", flush=True)

        if not depth_frames:
            raise RuntimeError("No depth frames were captured.")

        metadata["captured_frames"] = len(depth_frames)
        np.savez_compressed(
            output_path,
            depth_stack=np.stack(depth_frames, axis=0).astype(np.float32),
            pose_stack=np.stack(pose_frames, axis=0).astype(np.float32),
            timestamps=np.asarray(timestamps, dtype=np.float32),
            intrinsics=intrinsics,
            metadata_json=json.dumps(metadata, ensure_ascii=True),
        )
    finally:
        pipeline.stop()

    print(f"saved_capture={output_path}", flush=True)
    print(f"captured_frames={metadata['captured_frames']}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
