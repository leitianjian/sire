"""Run one Go2 with a saved policy, live Sire poses, and an RGB-D camera."""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path
import time

import numpy as np
import sire
import torch
from sire.sensor import Camera, CameraConfig
from sire.viewer import MeshRenderer, SireScene, Viewer, graphics_context

if __package__:
    from .go2_policy import load_jit
else:
    from go2_policy import load_jit


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "demo/demo_python/sirePaperDogRL/go2_rai_foot.xml"
ASSETS = ROOT / "demo/demo_python/dogRL/assets"
POLICY = ROOT / "sim2sim/policies/go2_cuda_1024/model_1000_jit.pt"
BOX_CENTER = (1.65, 0.48, 0.20)
BOX_SIZE = (0.30, 0.35, 0.40)


def _joint_gains(names, settings):
    return [next(value for pattern, value in settings.items() if pattern in name) for name in names]


class Go2Simulation:
    """Advance Sire by one policy control interval per step(action)."""

    def __init__(self, profile):
        env = profile["env_cfg"]
        control = env["control"]
        if control["control_type"] != "P":
            raise ValueError("Expected a Go2 position-PD policy")
        dt = float(env["sim"]["dt"])
        self.control_dt = dt * int(control["decimation"])
        names = profile["policy_joint_names"]
        start = env["init_state"]
        default_q = np.array([start["default_joint_angles"][name] for name in names])

        self.sim = sire.Simulator()
        sire.fromXmlFile(self.sim, str(SCENE))
        self.loop = self.sim.simulationLoop()
        self.loop.deltaT = dt
        self.loop.ctrlT = self.control_dt
        self.model = self.sim.model()
        model = self.model
        ground = model.link(0)
        self.base = model.link(1)
        box_pose = np.eye(4)
        box_pose[:3, 3] = BOX_CENTER
        box_transform = box_pose.ravel().tolist()
        ground.addBoxGeometry(ground.id, *BOX_SIZE, box_transform)
        self.sim.physicsEngine().addBoxGeometry(*BOX_SIZE, ground.id, False, box_transform,
                                               True, "m1", "{k:2.8e8,d:2000}")
        self.sim.init()

        motion_ids = {model.jointPool()[i].name: i for i in range(model.numMotions())}
        indices = np.array([motion_ids[name] for name in names], dtype=np.int32)
        motions = [model.motion(int(index)) for index in indices]
        feet = [model.link(i).id for i in range(model.nbody) if model.link(i).name.endswith("_foot")]
        self.stepper = sire.SireRLBatchStepper(
            [self.sim], indices, feet, "P", control["action_scale"],
            _joint_gains(names, control["stiffness"]), _joint_gains(names, control["damping"]),
            default_q, [motion.maxForce for motion in motions],
            [motion.minPosition for motion in motions], [motion.maxPosition for motion in motions],
            dt, self.control_dt, 1,
        )

        joints = np.zeros(model.numMotions())
        joints[indices] = default_q
        sire.setMotionMps(model, joints.tolist())
        sire.setMotionMvs(model, np.zeros_like(joints).tolist())
        self.base.pq = [*start["pos"], *start["rot"]]
        self.base.vs = sire.vp2vs(start["pos"], start["lin_vel"], start["ang_vel"])
        model.forwardKinematics()
        model.forwardKinematicsVel()
        self.joint_positions = default_q
        self.joint_velocities = np.zeros_like(default_q)

    @property
    def sim_time(self):
        return self.loop.simTime()

    @property
    def base_angular_velocity(self):
        return np.asarray(self.base.vs[3:])

    def step(self, action):
        """Apply the 12 normalized joint commands in the deployment profile's order."""
        _, positions, velocities, *_ = self.stepper.step(action[None, :])
        if self.stepper.recoveredEnvIds:
            raise RuntimeError(self.stepper.recoveredErrors[0])
        self.joint_positions, self.joint_velocities = positions[0], velocities[0]


class Go2Controller:
    def __init__(self, actor, profile):
        self.actor = actor
        env = profile["env_cfg"]
        angles = env["init_state"]["default_joint_angles"]
        self.default_q = np.array([angles[name] for name in profile["policy_joint_names"]])
        self.last_action = np.zeros(len(self.default_q), dtype=np.float32)
        normalization = env["normalization"]
        self.scales = normalization["obs_scales"]
        self.clip_obs = normalization["clip_observations"]
        self.clip_actions = normalization["clip_actions"]
        self.command_scale = np.array([self.scales["lin_vel"], self.scales["lin_vel"], self.scales["ang_vel"]])

    def act(self, sim, command, base_rotation):
        """Map the current Go2 state and velocity command to a policy action."""
        obs = np.concatenate((base_rotation.T @ sim.base_angular_velocity * self.scales["ang_vel"],
                              -base_rotation[2], command * self.command_scale,
                              (sim.joint_positions - self.default_q) * self.scales["dof_pos"],
                              sim.joint_velocities * self.scales["dof_vel"], self.last_action))
        obs = np.clip(obs, -self.clip_obs, self.clip_obs).astype(np.float32)
        with torch.inference_mode():
            action = self.actor(torch.from_numpy(obs[None, :])).numpy()
        self.last_action = np.clip(action[0], -self.clip_actions, self.clip_actions)
        return self.last_action


def _go2_style(item, geometry):
    if item.mesh is not None:
        if item.mesh.stem.endswith("_0") or item.mesh.stem == "foot":
            return replace(item, color=(0.14, 0.17, 0.19))
    elif item.body == "ground" and not geometry.isDynamic and max(item.size[:2]) >= 10:
        return replace(item, color=(0.55, 0.59, 0.62), checker=True)
    return item


def run_walk(args, config, sim, controller, window):
    scene = SireScene(sim.model, args.mesh_dir, style=_go2_style)
    renderer = MeshRenderer(scene.items)
    try:
        poses = scene.poses()
        camera = Camera(config, lambda optics, pose: renderer.capture(poses, optics, pose),
                        deferred_render=lambda optics, pose: renderer.prepare_capture(poses, optics, pose))
        viewer = Viewer(window, renderer) if window else None
        update_camera = camera.update_deferred if viewer else camera.update
        command = np.array(args.cmd)
        start = time.perf_counter()
        initial_time = sim.sim_time
        base = poses["base"]
        initial_position = base[:3, 3].copy()
        min_height = initial_position[2]
        update_camera(initial_time, poses)
        while True:
            elapsed = sim.sim_time - initial_time
            if viewer:
                time.sleep(max(0, start + elapsed - time.perf_counter()))
                if not viewer.sync(poses, base[:3, 3], capture_preview=True):
                    break
            if args.duration and elapsed >= args.duration - 1e-9:
                break
            action = controller.act(sim, command, base[:3, :3])
            sim.step(action)
            poses = scene.poses()
            update_camera(sim.sim_time, poses)
            base = poses["base"]
            min_height = min(min_height, base[2, 3])

        frame = camera.read()
        if args.save_rgbd:
            np.savez_compressed(args.save_rgbd, rgb=frame.rgb, depth=frame.depth, valid=frame.valid,
                                sim_time=frame.sim_time, frame_id=frame.frame_id, intrinsics=config.pinhole)
        delta = base[:3, 3] - initial_position
        print(f"sim={elapsed:.2f}s wall={time.perf_counter() - start:.2f}s "
              f"walk_xy=({delta[0]:.3f}, {delta[1]:.3f})m min_height={min_height:.3f}m "
              f"rgbd_frames={frame.frame_id + 1} valid_depth={frame.valid.mean():.1%}")
    finally:
        renderer.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", type=Path, default=POLICY, help="Go2 TorchScript actor")
    parser.add_argument("--deployment-config", type=Path, help="Override the actor's embedded deployment config")
    parser.add_argument("--mesh-dir", type=Path, default=ASSETS, help="Directory containing the Go2 OBJ meshes")
    parser.add_argument("--cmd", type=float, nargs=3, default=(0.25, 0, 0), metavar=("VX", "VY", "YAW_RATE"))
    parser.add_argument("--duration", type=float, default=15, help="Simulated seconds, or 0 to run until window close")
    parser.add_argument("--headless", action="store_true", help="Capture RGB-D without opening the desktop window")
    parser.add_argument("--save-rgbd", type=Path, help="Save the latest RGB-D frame as an NPZ file")
    parser.add_argument("--camera-size", type=int, nargs=2, default=(640, 480), metavar=("WIDTH", "HEIGHT"))
    parser.add_argument("--camera-fps", type=float, default=30)
    parser.add_argument("--camera-fov-y", type=float, default=60, help="Vertical field of view in degrees")
    parser.add_argument("--camera-intrinsics", type=float, nargs=4, metavar=("FX", "FY", "CX", "CY"))
    parser.add_argument("--camera-clip", type=float, nargs=2, default=(.05, 20), metavar=("NEAR", "FAR"))
    args = parser.parse_args()
    if not np.isfinite(args.duration) or args.duration < 0 or (args.headless and args.duration == 0):
        parser.error("--duration must be finite and nonnegative (positive in headless mode)")
    if not np.isfinite(args.cmd).all():
        parser.error("--cmd needs three finite values")

    # Robot axes are +X forward, +Y left, +Z up. The camera tilts down by 10 degrees.
    pitch = np.deg2rad(10)
    s, c = np.sin(pitch), np.cos(pitch)
    mount = np.array([[0, s, -c, .34], [-1, 0, 0, 0], [0, c, s, .07], [0, 0, 0, 1]])
    config = CameraConfig(*args.camera_size, fps=args.camera_fps, fov_y=args.camera_fov_y,
                          intrinsics=args.camera_intrinsics, near=args.camera_clip[0], far=args.camera_clip[1],
                          parent="base", local_pose=mount)
    torch.set_num_threads(1)
    actor, profile = load_jit(args.policy, args.deployment_config)
    sim = Go2Simulation(profile)
    controller = Go2Controller(actor, profile)
    with graphics_context(args.headless, title="Sire Go2 | RGB-D") as window:
        run_walk(args, config, sim, controller, window)


if __name__ == "__main__":
    main()
