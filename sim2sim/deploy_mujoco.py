"""Local Go2 flat-ground sim2sim, adapted from go2_rl_gym/deploy/deploy_mujoco.

Loads actor-only TorchScript via torch.jit.load, as in the reference deployer.
No checkpoint, training environment or Sire native extension is needed at runtime.
This is a simulator-only viewer, not a hardware driver.
"""
from __future__ import annotations

import argparse
import contextlib
import csv
import queue
import time
from pathlib import Path

import mujoco
import numpy as np
import torch
import yaml

ROOT = Path(__file__).resolve().parents[1]
LEGS = ("FL", "FR", "RL", "RR")
JOINT_ORDERS = {
    "sire": tuple(f"{leg}_{joint}_joint" for joint in ("hip", "thigh", "calf") for leg in LEGS),
    "go2_rl_gym": tuple(f"{leg}_{joint}_joint" for leg in LEGS for joint in ("hip", "thigh", "calf")),
}


def resolve_path(value):
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (ROOT / path).resolve()


def rotation_matrix(quat_wxyz):
    out = np.empty(9, dtype=np.float64)
    mujoco.mju_quat2Mat(out, np.asarray(quat_wxyz, dtype=np.float64))
    return out.reshape(3, 3)


def joystick_command(axes, axis_ids, signs, max_command, deadzone):
    values = np.asarray([axes[i] for i in axis_ids], dtype=np.float32)
    if not np.isfinite(values).all():
        return np.zeros(3, dtype=np.float32)
    values[np.abs(values) < deadzone] = 0.0
    return np.asarray(np.clip(values, -1, 1) * signs * max_command, dtype=np.float32)


def load_jit(path, deployment_config=None):
    extra = {"deployment.yaml": ""}
    try:
        actor = torch.jit.load(str(path), map_location="cpu", _extra_files=extra).eval()
    except RuntimeError as exc:
        raise ValueError("Expected an actor JIT, not a training checkpoint. Use: bash sim2sim/run.sh export <checkpoint>") from exc
    if deployment_config:
        profile = yaml.safe_load(deployment_config.read_text())
    elif extra["deployment.yaml"]:
        profile = yaml.safe_load(extra["deployment.yaml"])
    else:
        raise ValueError("JIT has no deployment metadata; pass --deployment-config or export using sim2sim/run.sh export.")
    if profile["num_observations"] != 45 or profile["num_actions"] != 12:
        raise ValueError("This deployment expects 45 observations and 12 actions.")
    if not all(torch.isfinite(p).all().item() for p in actor.parameters()):
        raise ValueError("JIT contains nonfinite actor parameters.")
    with torch.inference_mode():
        sample = actor(torch.zeros(1, 45))
    if not isinstance(sample, torch.Tensor) or sample.shape != (1, 12) or not torch.isfinite(sample).all():
        raise ValueError("JIT must map a [1, 45] observation to a finite [1, 12] action tensor.")
    return actor, profile


class Controller:
    def __init__(self, config):
        import pygame
        self.pg = pygame
        self.config = config
        self.js = None
        self.previous_buttons = set()
        self.last_probe = -1.0
        # No pygame window or audio initialization; MuJoCo owns the only window.
        pygame.display.init()
        pygame.joystick.init()
        self.connect()

    def connect(self):
        self.last_probe = time.monotonic()
        index = int(self.config["index"])
        if self.pg.joystick.get_count() <= index:
            return
        js = self.pg.joystick.Joystick(index)
        js.init()
        if min(self.config["axes"]) < 0 or max(self.config["axes"]) >= js.get_numaxes():
            js.quit()
            raise ValueError("Configured joystick axis is unavailable; use --inspect-joystick.")
        self.js = js
        self.previous_buttons = set()
        print(f"[joystick] {js.get_name()}: {js.get_numaxes()} axes, {js.get_numbuttons()} buttons", flush=True)

    def poll(self):
        self.pg.event.pump()
        # Drain events to avoid queue growth; removal must zero commands even
        # when SDL still exposes cached axis values from the detached device.
        for event in self.pg.event.get():
            if event.type == self.pg.JOYDEVICEREMOVED and self.js is not None:
                if event.instance_id == self.js.get_instance_id():
                    self.js.quit()
                    self.js = None
                    self.previous_buttons = set()
                    print("[joystick] disconnected; command = 0", flush=True)
        if self.js is None and time.monotonic() - self.last_probe > 1:
            self.connect()
        if self.js is None:
            return np.zeros(3, dtype=np.float32), set(), []
        try:
            axes = [self.js.get_axis(i) for i in range(self.js.get_numaxes())]
            buttons = {i for i in range(self.js.get_numbuttons()) if self.js.get_button(i)}
        except self.pg.error:
            self.js.quit()
            self.js = None
            self.previous_buttons = set()
            print("[joystick] read failed; command = 0", flush=True)
            return np.zeros(3, dtype=np.float32), set(), []
        pressed = buttons - self.previous_buttons
        self.previous_buttons = buttons
        cfg = self.config
        cmd = joystick_command(axes, cfg["axes"], cfg["signs"], cfg["max_command"], cfg["deadzone"])
        return cmd, pressed, axes

    def close(self):
        self.pg.quit()


class Go2Simulation:
    def __init__(self, policy_path, xml_path, simulation_dt, deployment_config=None, yaw_deg=0):
        self.actor, profile = load_jit(policy_path, deployment_config)
        self.profile = profile
        self.iteration = profile.get("iteration")
        env_cfg = profile["env_cfg"]
        control = env_cfg["control"]
        if control["control_type"] != "P":
            raise ValueError("Only the current Go2 position-PD policy is supported.")
        self.control_dt = float(env_cfg["sim"]["dt"]) * int(control["decimation"])
        self.decimation = round(self.control_dt / simulation_dt)
        if simulation_dt <= 0 or self.decimation < 1 or not np.isclose(self.decimation * simulation_dt, self.control_dt):
            raise ValueError("MuJoCo simulation_dt must divide the trained policy control period exactly.")
        self.names = tuple(profile["policy_joint_names"])
        if len(self.names) != 12 or set(self.names) != set(JOINT_ORDERS["sire"]):
            raise ValueError("Invalid deployment policy_joint_names; expected the 12 named Go2 joints.")
        # YAML serializes the training joint dictionary alphabetically. Its
        # iteration order is NOT the policy order: always address joints by name.
        self.default_q = np.array([env_cfg["init_state"]["default_joint_angles"][n] for n in self.names])

        def gains(values):
            result = []
            for name in self.names:
                matches = [value for pattern, value in values.items() if pattern in name]
                if len(matches) != 1:
                    raise ValueError(f"Ambiguous/missing PD gain for {name}: {values}")
                result.append(matches[0])
            return np.array(result, dtype=np.float64)

        self.kp = gains(control["stiffness"])
        self.kd = gains(control["damping"])
        self.action_scale = float(control["action_scale"])
        norm = env_cfg["normalization"]
        self.scales = norm["obs_scales"]
        self.command_scale = np.array([self.scales["lin_vel"], self.scales["lin_vel"], self.scales["ang_vel"]])
        self.clip_obs, self.clip_actions = float(norm["clip_observations"]), float(norm["clip_actions"])
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.model.opt.timestep = simulation_dt
        self.data = mujoco.MjData(self.model)
        m = self.model
        joint_ids = np.array([m.joint(n).id for n in self.names])
        self.qpos_ids = m.jnt_qposadr[joint_ids].copy()
        self.qvel_ids = m.jnt_dofadr[joint_ids].copy()
        actuator_ids = []
        for jid in joint_ids:
            matches = np.flatnonzero((m.actuator_trntype == mujoco.mjtTrn.mjTRN_JOINT) & (m.actuator_trnid[:, 0] == jid))
            if len(matches) != 1:
                raise ValueError(f"Expected one direct motor for joint {m.joint(jid).name}.")
            actuator_ids.append(matches[0])
        self.actuator_ids = np.array(actuator_ids)
        if not np.allclose(m.actuator_gear[self.actuator_ids], [1, 0, 0, 0, 0, 0]):
            raise ValueError("This Go2 deployment requires unit-gear motors.")
        if not np.all(m.jnt_actfrclimited[joint_ids]):
            raise ValueError("Go2 joint actuator force limits are missing.")
        self.torque_limits = m.jnt_actfrcrange[joint_ids].copy()
        root_joint = m.joint("root").id
        self.root_qpos = int(m.jnt_qposadr[root_joint])
        self.root_qvel = int(m.jnt_dofadr[root_joint])
        self.base_body = m.body("base").id
        self.initial_position = np.array(env_cfg["init_state"]["pos"], dtype=float)
        yaw = np.deg2rad(yaw_deg) / 2
        self.initial_quat = np.array([np.cos(yaw), 0, 0, np.sin(yaw)])
        self.action = np.zeros(12, dtype=np.float32)
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.root_qpos:self.root_qpos + 3] = self.initial_position
        self.data.qpos[self.root_qpos + 3:self.root_qpos + 7] = self.initial_quat
        self.data.qpos[self.qpos_ids] = self.default_q
        self.action.fill(0)
        mujoco.mj_forward(self.model, self.data)

    def state(self):
        r = self.root_qpos
        quat = self.data.qpos[r + 3:r + 7]
        rot = rotation_matrix(quat)
        v = self.data.qvel[self.root_qvel:self.root_qvel + 6]
        # MuJoCo free joint: linear velocity is WORLD; angular is already BODY.
        # Inverse-rotating v[3:6] again recreates the historical turning bug.
        return self.data.qpos[r:r + 3].copy(), rot, rot.T @ v[:3], v[3:6].copy()

    def observation(self, command):
        _, rot, _, angular = self.state()
        obs = np.concatenate((angular * self.scales["ang_vel"], rot.T @ [0, 0, -1],
                              np.asarray(command) * self.command_scale,
                              (self.data.qpos[self.qpos_ids] - self.default_q) * self.scales["dof_pos"],
                              self.data.qvel[self.qvel_ids] * self.scales["dof_vel"], self.action))
        if not np.isfinite(obs).all():
            raise FloatingPointError("Nonfinite policy observation; stopping simulation.")
        return np.clip(obs, -self.clip_obs, self.clip_obs).astype(np.float32)

    def step(self, command):
        obs = torch.from_numpy(self.observation(command)).unsqueeze(0)
        with torch.inference_mode():
            action = self.actor(obs).squeeze(0).numpy()
        if not np.isfinite(action).all():
            raise FloatingPointError("Nonfinite policy action; stopping simulation.")
        self.action = np.clip(action, -self.clip_actions, self.clip_actions)
        target = self.default_q + self.action * self.action_scale
        previous_time = self.data.time
        for _ in range(self.decimation):
            torque = self.kp * (target - self.data.qpos[self.qpos_ids]) - self.kd * self.data.qvel[self.qvel_ids]
            self.data.ctrl[self.actuator_ids] = np.clip(torque, self.torque_limits[:, 0], self.torque_limits[:, 1])
            mujoco.mj_step(self.model, self.data)
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise FloatingPointError("Nonfinite MuJoCo state; stopping simulation.")
        if not np.isclose(self.data.time, previous_time + self.control_dt):
            raise FloatingPointError("MuJoCo reset its time unexpectedly (possible unstable state).")
        for warning in (mujoco.mjtWarning.mjWARN_BADQPOS, mujoco.mjtWarning.mjWARN_BADQVEL, mujoco.mjtWarning.mjWARN_BADQACC):
            if self.data.warning[warning].number:
                raise FloatingPointError(f"MuJoCo numerical warning: {warning}")

    def fallen(self):
        position, rot, _, _ = self.state()
        return position[2] < 0.12 or rot[2, 2] < 0.2


def draw_commands(viewer, sim, command):
    position, rot, linear, _ = sim.state()
    start = position + np.array([0, 0, 0.2])
    scene = viewer.user_scn
    scene.ngeom = 0
    for velocity, color in (([command[0], command[1], 0], [0.1, 1, 0.1, 1]),
                            ([linear[0], linear[1], 0], [0.1, 0.4, 1, 1])):
        delta = rot @ np.asarray(velocity) * 0.7
        if np.linalg.norm(delta) < 0.01:
            continue
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.eye(3).ravel(), np.array(color, dtype=np.float32))
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW, 0.015, start, start + delta)
        scene.ngeom += 1


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", type=Path, default=Path(__file__).with_name("config.yaml"))
    p.add_argument("--policy", "-p", help="Actor-only JIT; relative paths are resolved from the repository root")
    p.add_argument("--deployment-config", type=Path, help="Explicit profile for an old JIT without embedded metadata")
    p.add_argument("--sim-dt", type=float, help="MuJoCo physics dt; policy period is read from training config")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--no-realtime", action="store_true", help="Run as fast as possible; avoid while timing background training")
    p.add_argument("--steps", type=int, default=0, help="Control steps; 0 runs until viewer closes / Ctrl+C")
    p.add_argument("--no-joystick", action="store_true")
    p.add_argument("--cmd", nargs=3, type=float, default=[0, 0, 0], metavar=("VX", "VY", "WZ"), help="Fixed body-frame command; requires --no-joystick")
    p.add_argument("--joystick-index", type=int)
    p.add_argument("--axes", nargs=3, type=int, metavar=("VX_AXIS", "VY_AXIS", "WZ_AXIS"))
    p.add_argument("--max-cmd", nargs=3, type=float, metavar=("VX", "VY", "WZ"))
    p.add_argument("--inspect-joystick", action="store_true", help="Print axes/buttons without loading physics/policy")
    p.add_argument("--initial-yaw-deg", type=float, default=0)
    p.add_argument("--log", type=Path, help="Optional CSV with commands, measured velocities, height, yaw, falls/resets")
    return p


def main():
    args = parser().parse_args()
    cfg = yaml.safe_load(args.config.read_text())
    joy_cfg = cfg["joystick"]
    for key, value in (("index", args.joystick_index), ("axes", args.axes), ("max_command", args.max_cmd)):
        if value is not None:
            joy_cfg[key] = value
    if args.steps < 0 or joy_cfg["index"] < 0 or not 0 <= joy_cfg["deadzone"] < 1:
        raise ValueError("Invalid step count / joystick index / deadzone.")
    if not np.isfinite(args.cmd).all() or not np.isfinite(joy_cfg["max_command"]).all() or min(joy_cfg["max_command"]) < 0:
        raise ValueError("Commands and maximum speeds must be finite (maxima nonnegative).")
    if not args.no_joystick and any(args.cmd):
        raise ValueError("--cmd requires --no-joystick; joystick yaw is never replaced by heading control.")
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    with contextlib.ExitStack() as stack:
        controller = None
        if not args.no_joystick or args.inspect_joystick:
            controller = Controller(joy_cfg)
            stack.callback(controller.close)
            if controller.js is None:
                print("[joystick] none connected; command = 0, waiting for hotplug", flush=True)
        if args.inspect_joystick:
            i = 0
            while not args.steps or i < args.steps:
                command, pressed, axes = controller.poll()
                print(f"axes={np.round(axes, 3).tolist()} pressed={sorted(pressed)} command={np.round(command, 3).tolist()}", flush=True)
                i += 1
                time.sleep(0.2)
            return 0
        policy_path = resolve_path(args.policy or cfg["policy"])
        sim_dt = args.sim_dt if args.sim_dt is not None else float(cfg["simulation_dt"])
        if not np.isfinite(sim_dt) or sim_dt <= 0:
            raise ValueError("simulation_dt must be finite and positive")
        sim = Go2Simulation(policy_path, resolve_path(cfg["xml"]), sim_dt,
                            args.deployment_config, args.initial_yaw_deg)
        print(f"[policy JIT] {policy_path} (iteration={sim.iteration})\n[source] {sim.profile.get('source_checkpoint', 'external JIT')}\n"
              f"[timing] MuJoCo dt={sim_dt:g}, decimation={sim.decimation}, policy dt={sim.control_dt:g}\n"
              f"[joint order] {', '.join(sim.names)}\n"
              "[controls] left stick: vx/vy; right stick: yaw rate; A / R: reset; Start / Space: pause; Esc: exit\n"
              "[display] green arrow=command, blue=measured; velocities in body frame; no episode time-limit reset", flush=True)
        viewer = None
        keys = queue.SimpleQueue()
        if not args.headless:
            import mujoco.viewer
            viewer = stack.enter_context(mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=keys.put))
            with viewer.lock():
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                viewer.cam.trackbodyid = sim.base_body
                viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 2.5, -20, 60
        csv_writer = None
        log_file = None
        if args.log:
            args.log.parent.mkdir(parents=True, exist_ok=True)
            # Exclusive create: comparing policies must not overwrite a past run.
            log_file = stack.enter_context(args.log.open("x", newline=""))
            csv_writer = csv.writer(log_file)
            csv_writer.writerow(["step", "sim_time", "reset_count", "cmd_vx", "cmd_vy", "cmd_wz", "vx", "vy", "wz", "x", "y", "z", "yaw", "fallen"])
        step = resets = falls = 0
        paused = fall_latched = False
        start = time.monotonic()
        next_log = 0.0
        command = np.asarray(args.cmd, dtype=np.float32)
        while not args.steps or step < args.steps:
            tick = time.monotonic()
            if viewer and not viewer.is_running():
                break
            pressed = set()
            if controller:
                command, pressed, _ = controller.poll()
            key_set = set()
            while not keys.empty():
                key_set.add(keys.get())
            if 256 in key_set:
                break
            if joy_cfg["reset_button"] in pressed or ord("R") in key_set:
                sim.reset()
                resets += 1
                paused = fall_latched = False
                print(f"[reset] manual reset #{resets}", flush=True)
            if joy_cfg["pause_button"] in pressed or 32 in key_set:
                paused = not paused
                print(f"[pause] {paused}", flush=True)
            if not paused:
                sim.step(command)
                step += 1
                position, rot, linear, angular = sim.state()
                fallen = sim.fallen()
                yaw = float(np.arctan2(rot[1, 0], rot[0, 0]))
                if csv_writer:
                    csv_writer.writerow([step, sim.data.time, resets, *command, linear[0], linear[1], angular[2], *position, yaw, int(fallen)])
                if fallen and not fall_latched:
                    falls += 1
                    fall_latched = paused = True
                    print(f"[fall] step={step}, height={position[2]:.3f}; paused, press A / R to reset", flush=True)
                    if args.headless:
                        break
            if viewer:
                with viewer.lock():
                    draw_commands(viewer, sim, command)
                viewer.sync()
            if tick >= next_log:
                position, rot, linear, angular = sim.state()
                print(f"[state] step={step} t={sim.data.time:.2f} cmd={np.round(command, 3).tolist()} "
                      f"vel=[{linear[0]:.3f}, {linear[1]:.3f}, {angular[2]:.3f}] "
                      f"xyz={np.round(position, 3).tolist()} yaw={np.rad2deg(np.arctan2(rot[1,0], rot[0,0])):.1f} paused={paused}", flush=True)
                if log_file:
                    log_file.flush()
                next_log = tick + 1
            if not args.no_realtime or paused:
                time.sleep(max(0, sim.control_dt - (time.monotonic() - tick)))
        position, _, linear, angular = sim.state()
        print(f"[summary] steps={step} sim_time={sim.data.time:.3f} wall_time={time.monotonic()-start:.3f} "
              f"manual_resets={resets} falls={falls} xyz={position.tolist()} "
              f"velocity={[float(linear[0]), float(linear[1]), float(angular[2])]}", flush=True)
        return 2 if falls else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nStopped by user.")
