"""
Reproduce the first 20 control steps of Sire Go2 training in a standalone script.

Matches the LeggedRobotSire training pipeline exactly:
- Same random initialization (_reset_dofs, _reset_root_states)
- Same PD controller (P-control, kp=25, kd=0.6)
- Same event-loop (handleContact loop)
- Same post-step math: quat_rotate_inverse for base_lin_vel, projected_gravity, etc.
- Same diagnostic format as _post_physics_step_sire

Usage:
    # With seed, auto-detects XML from latest exp
    python test/test_repro_20steps.py --seed 42

    # Explicit XML + seed + random actions (like training)
    python test/test_repro_20steps.py --xml path/to/scene.xml --seed 42 --action random

    # Zero actions (PD holds default pose, for isolating physics)
    python test/test_repro_20steps.py --seed 42 --action zero --out repro.json
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
_SIREGYM = _HERE.parent
_DEMO_PY = _SIREGYM.parent
sys.path.insert(0, str(_DEMO_PY))

import sire

# ══════════════════════════════════════════════════════════════════════
#  Go2 config constants (matches GO2RoughCfg)
# ══════════════════════════════════════════════════════════════════════
DEFAULT_JOINT_ANGLES = np.array([
    0.1,   # FL_hip_joint
    -0.1,  # FR_hip_joint
    0.1,   # RL_hip_joint
    -0.1,  # RR_hip_joint
    0.8,   # FL_thigh_joint
    0.8,   # FR_thigh_joint
    1.0,   # RL_thigh_joint
    1.0,   # RR_thigh_joint
    -1.5,  # FL_calf_joint
    -1.5,  # FR_calf_joint
    -1.5,  # RL_calf_joint
    -1.5,  # RR_calf_joint
])
DOF_NAMES = [
    'FL_hip_joint', 'FR_hip_joint', 'RL_hip_joint', 'RR_hip_joint',
    'FL_thigh_joint', 'FR_thigh_joint', 'RL_thigh_joint', 'RR_thigh_joint',
    'FL_calf_joint', 'FR_calf_joint', 'RL_calf_joint', 'RR_calf_joint',
]
NUM_DOF = 12
KP = 25.0
KD = 0.6
ACTION_SCALE = 0.25
SIM_DT = 0.001
DECIMATION = 10
CTRL_DT = SIM_DT * DECIMATION  # 0.01 s

# Base init state (GO2RoughCfg.init_state)
BASE_INIT_POS = np.array([0.0, 0.0, 0.44])          # [x, y, z]
BASE_INIT_QUAT = np.array([0.0, 0.0, 0.0, 1.0])      # [qx,qy,qz,qw] Aris scalar-last
BASE_INIT_LIN_VEL = np.array([0.0, 0.0, 0.0])
BASE_INIT_ANG_VEL = np.array([0.0, 0.0, 0.0])
INIT_YAW_RANGE = [-3.1415926, 3.1415926]

# Terrain spawn (GO2RoughCfg.terrain)
BORDER = 2.0
TERRAIN_LENGTH = 10.0
TERRAIN_WIDTH = 10.0
SPAWN_OFFSET_X = 2.0
SPAWN_OFFSET_Y = 5.0
SPAWN_RAND_X_RANGE = [-1.0, 1.0]
SPAWN_RAND_Y_RANGE = [-1.0, 1.0]

# ══════════════════════════════════════════════════════════════════════
#  Aris quaternion math (scalar-last: [qx, qy, qz, qw]) — pure numpy
#  Equivalent to SireRLGym/utils/math.py but without torch dependency.
# ══════════════════════════════════════════════════════════════════════

def _quat_conjugate(q: np.ndarray) -> np.ndarray:
    """Conjugate for scalar-last (x,y,z,w): negate x,y,z, keep w."""
    out = q.copy()
    out[..., :3] *= -1.0
    return out


def _quat_mul(q: np.ndarray, r: np.ndarray) -> np.ndarray:
    """Quaternion multiplication for scalar-last (x,y,z,w)."""
    x0, y0, z0, w0 = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    x1, y1, z1, w1 = r[..., 0], r[..., 1], r[..., 2], r[..., 3]
    return np.stack([
        w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
        w0 * y1 + y0 * w1 + z0 * x1 - x0 * z1,
        w0 * z1 + z0 * w1 + x0 * y1 - y0 * x1,
        w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
    ], axis=-1)


def _quat_apply(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector v by quaternion q (scalar-last)."""
    zeros = np.zeros_like(v[..., :1])
    v_as_quat = np.concatenate([v, zeros], axis=-1)
    return _quat_mul(_quat_mul(q, v_as_quat), _quat_conjugate(q))[..., :3]


def quat_rotate_inverse(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Inverse-rotate vector v by quaternion q (scalar-last).
    Equivalent to: quat_apply(quat_conjugate(q), v)
    This is the same math as SireRLGym.utils.math.quat_rotate_inverse.
    """
    return _quat_apply(_quat_conjugate(q), v)


# ══════════════════════════════════════════════════════════════════════
#  Helpers
# ══════════════════════════════════════════════════════════════════════

def find_generated_xml() -> str | None:
    """Find the most recent generated XML from the vis/ directories."""
    logs_base = _SIREGYM / "scripts" / "SireRLGym" / "logs" / "rough_go2"
    if not logs_base.is_dir():
        logs_base = Path.cwd() / "logs" / "rough_go2"
    if not logs_base.is_dir():
        return None
    exp_dirs = sorted(
        [d for d in logs_base.iterdir() if d.is_dir() and d.name.startswith("exp")],
        key=lambda d: int(d.name[3:]) if d.name[3:].isdigit() else 0,
        reverse=True,
    )
    for exp_dir in exp_dirs:
        vis_dir = exp_dir / "vis"
        xml_path = vis_dir / "sire_first20_scene.xml"
        if xml_path.exists():
            print(f"[repro] Found scene XML: {xml_path}")
            return str(xml_path)
    base_model = _SIREGYM.parent / "sirePaperDogRL" / "go2_rai_foot.xml"
    if base_model.exists():
        print("[repro] ⚠️  No generated scene XML found, using base model (no terrain)")
        return str(base_model)
    return None


def compute_spawn_origin(terrain_x: float, terrain_y: float) -> np.ndarray:
    """Compute env_origin spawn point from HeightField center position.
    
    HeightField pm = terrain center = (border + total/2, border + total/2).
    Spawn point = (border + spawn_offset_x, border + spawn_offset_y).
    → spawn = terrain_center - total/2 + spawn_offset.
    """
    spawn_x = terrain_x - TERRAIN_LENGTH * 0.5 + SPAWN_OFFSET_X
    spawn_y = terrain_y - TERRAIN_WIDTH * 0.5 + SPAWN_OFFSET_Y
    return np.array([spawn_x, spawn_y, 0.0])


# ══════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════

def main():
    p = argparse.ArgumentParser(
        description="Reproduce first 20 Sire sim steps — matches LeggedRobotSire training"
    )
    p.add_argument("--xml", type=str, default=None,
                   help="Path to generated Sire XML (with terrain)")
    p.add_argument("--seed", type=int, default=42,
                   help="Random seed (matches train.py --seed)")
    p.add_argument("--out", type=str, default="repro_20steps.json",
                   help="Output recording path")
    p.add_argument("--steps", type=int, default=20,
                   help="Number of control steps")
    p.add_argument("--action", type=str, default="zero",
                   choices=["zero", "random", "replay"],
                   help="Action mode: zero=hold default pose, random=uniform noise "
                        "(mimics untrained policy), replay=load from --replay-file")
    p.add_argument("--replay-file", type=str, default=None,
                   help="Path to sire_first20_replay.json (initial state + actions)")
    p.add_argument("--headless", action="store_true", default=False,
                   help="Skip meshcat viewer")
    args = p.parse_args()

    # ── Load replay data if provided ──
    replay_data = None
    if args.replay_file:
        with open(args.replay_file) as f:
            replay_data = json.load(f)
        print(f"[repro] Loaded replay: {len(replay_data['actions'])} actions, "
              f"dofs={replay_data['dof_names']}")
        args.action = "replay"
        args.steps = len(replay_data['actions'])

    # ── Seed ──
    rng = np.random.RandomState(args.seed)
    print(f"[repro] Random seed: {args.seed}")

    # ── Resolve XML ──
    xml_path = args.xml
    if not xml_path:
        xml_path = os.environ.get("SIRE_REPRO_XML", "")
    if not xml_path or not Path(xml_path).exists():
        xml_path = find_generated_xml()
    if not xml_path or not Path(xml_path).exists():
        print("[repro] ERROR: No XML found.")
        sys.exit(1)
    print(f"[repro] XML: {xml_path}")

    # ── Create simulator ──
    sim = sire.Simulator()
    sire.fromXmlFile(sim, xml_path)
    model = sim.model()
    sloop = sim.simulationLoop()
    sloop.deltaT = SIM_DT
    sloop.ctrlT = CTRL_DT
    sim.init()

    pe = sim.physicsEngine()
    print(f"[repro] nbody={model.nbody}  nMotions={model.numMotions()}  "
          f"nGeom={pe.numGeometries()}")

    # ── Build name→part_id map ──
    part_name_to_idx: dict[str, int] = {}
    for pid in range(model.nbody):
        part_name_to_idx[model.partPool()[pid].name] = pid

    # ── Build dof_name → motion_idx map ──
    dof_to_motion: dict[str, int] = {}
    for mot_idx in range(model.numMotions()):
        joint = model.jointPool()[mot_idx]
        if joint.name in DOF_NAMES:
            dof_to_motion[joint.name] = mot_idx
    print(f"[repro] Found {len(dof_to_motion)}/{NUM_DOF} DOF→motion mappings")

    # ── Find terrain center: parse HeightField pm from XML directly ──
    # The C++ geometryPool type name may not match 'HeightField', so we
    # extract the pm attribute from the XML element.
    import xml.etree.ElementTree as ET
    terrain_x, terrain_y, terrain_z = 7, 7, 0.0
    tree = ET.parse(xml_path)
    pe_elem = tree.getroot().find('PhysicsEngine')
    if pe_elem is not None:
        gpo = pe_elem.find('GeometryPoolObject')
        if gpo is not None:
            for child in gpo:
                if 'HeightField' in child.tag:
                    pm_str = child.get('pm', '')
                    # pm="{1,0,0,7,0,1,0,7,0,0,1,0,0,0,0,1}"
                    parts = [float(x.strip('{} ')) for x in pm_str.split(',') if x.strip('{} ')]
                    if len(parts) >= 12:
                        terrain_x, terrain_y, terrain_z = parts[3], parts[7], parts[11]
                        print(f"[repro] HeightField (XML): center=({terrain_x:.2f}, "
                              f"{terrain_y:.2f}, {terrain_z:.2f})")
                        break

    # ══════════════════════════════════════════════════════════════════
    #  Initialize robot state
    # ══════════════════════════════════════════════════════════════════

    base_part = model.partPool()[1]

    if replay_data is not None:
        # ── Exact replay: use saved initial state ──
        init = replay_data['init_state']
        base_part.pq = init['base_pq']
        base_part.vs = init['base_vs']
        for name, angle in init['dof_pos'].items():
            mi = dof_to_motion.get(name)
            if mi is not None:
                model.motionPool()[mi].mp = float(angle)
                model.motionPool()[mi].mv = 0.0
        model.forwardKinematics()
        model.forwardKinematicsVel()

        base_pos = np.array(init['base_pq'][:3])
        base_vel = np.array(init['base_vs'])
        print(f"[repro] Init (replay): base pos=({base_pos[0]:.3f},{base_pos[1]:.3f},{base_pos[2]:.3f})")
        print(f"[repro] Init (replay): base vel=({base_vel[0]:.3f},{base_vel[1]:.3f},{base_vel[2]:.3f})")
    else:
        # ── Random init (matching _reset_dofs + _reset_root_states) ──
        jitter = rng.uniform(0.5, 1.5, size=NUM_DOF)
        init_dof_pos = DEFAULT_JOINT_ANGLES * jitter
        for j, name in enumerate(DOF_NAMES):
            mi = dof_to_motion.get(name)
            if mi is not None:
                model.motionPool()[mi].mp = float(init_dof_pos[j])
                model.motionPool()[mi].mv = 0.0

        env_origin = compute_spawn_origin(terrain_x, terrain_y)
        spawn_rx = rng.uniform(*SPAWN_RAND_X_RANGE)
        spawn_ry = rng.uniform(*SPAWN_RAND_Y_RANGE)
        base_pos = BASE_INIT_POS.copy()
        base_pos[:2] += env_origin[:2]
        base_pos[0] += spawn_rx
        base_pos[1] += spawn_ry

        yaw = rng.uniform(*INIT_YAW_RANGE)
        hy = 0.5 * yaw
        yaw_quat = np.array([0.0, 0.0, math.sin(hy), math.cos(hy)])
        base_quat = _quat_mul(yaw_quat, BASE_INIT_QUAT)
        base_vel = rng.uniform(-0.5, 0.5, size=6)

        base_part.pq = np.concatenate([base_pos, base_quat]).tolist()
        base_part.vs = base_vel.tolist()
        model.forwardKinematics()
        model.forwardKinematicsVel()

        print(f"[repro] Init base pos: ({base_pos[0]:.3f}, {base_pos[1]:.3f}, {base_pos[2]:.3f})")
        print(f"[repro] Init base quat: [{base_quat[0]:.4f}, {base_quat[1]:.4f}, "
              f"{base_quat[2]:.4f}, {base_quat[3]:.4f}]")
        print(f"[repro] Init base vel (world): ({base_vel[0]:.3f}, {base_vel[1]:.3f}, "
              f"{base_vel[2]:.3f})")

    # ── Precompute actions ──
    if args.action == "replay":
        actions_seq = np.array(replay_data['actions'], dtype=np.float64)
        print(f"[repro] Action mode: replay ({len(actions_seq)} actions from training)")
    elif args.action == "random":
        actions_seq = rng.uniform(-1.0, 1.0, size=(args.steps, NUM_DOF))
        print("[repro] Action mode: random (uniform [-1,1], mimics untrained policy)")
    else:
        actions_seq = np.zeros((args.steps, NUM_DOF))
        print("[repro] Action mode: zero (PD holds default pose)")

    # ══════════════════════════════════════════════════════════════════
    #  Simulation loop
    # ══════════════════════════════════════════════════════════════════
    num_ctrl_steps = args.steps
    gravity_vec = np.array([0.0, 0.0, -1.0])

    print(f"\n[repro] ══ Running {num_ctrl_steps} control steps ══\n")

    for step in range(1, num_ctrl_steps + 1):
        action = actions_seq[step - 1]  # [NUM_DOF]

        # ── Inner event loop (matching SireRLGym step()) ──
        substep_count = 0
        while True:
            # PD torque update
            for j, name in enumerate(DOF_NAMES):
                mi = dof_to_motion[name]
                mot = model.motionPool()[mi]
                mp = float(mot.mp)
                mv = float(mot.mv)
                target = DEFAULT_JOINT_ANGLES[j] + action[j] * ACTION_SCALE
                tau = KP * (target - mp) - KD * mv
                if isinstance(mot, sire.ActuatorSISO):
                    mot.desiredValue = float(tau)

            sloop.handleContact()
            substep_count += 1

            if sloop.headerIsCtrl():
                # Torque update before ctrl event too
                for j, name in enumerate(DOF_NAMES):
                    mi = dof_to_motion[name]
                    mot = model.motionPool()[mi]
                    mp = float(mot.mp)
                    mv = float(mot.mv)
                    target = DEFAULT_JOINT_ANGLES[j] + action[j] * ACTION_SCALE
                    tau = KP * (target - mp) - KD * mv
                    if isinstance(mot, sire.ActuatorSISO):
                        mot.desiredValue = float(tau)

                sloop.handleContact()
                substep_count += 1
                break

        # ════════════════════════════════════════════════════════════
        #  Post-step: read state & compute derived quantities
        #  (matches _post_physics_step_sire exactly)
        # ════════════════════════════════════════════════════════════
        pq = np.array(base_part.pq, dtype=np.float64)   # [x,y,z, qx,qy,qz,qw]
        vs = np.array(base_part.vs, dtype=np.float64)   # [vx,vy,vz, wx,wy,wz]

        world_pos = pq[:3]
        world_quat = pq[3:7]
        world_vel = vs[:3]    # world-frame linear velocity = root_states[:, 7:10]
        world_ang = vs[3:6]   # world-frame angular velocity = root_states[:, 10:13]

        # base_lin_vel = quat_rotate_inverse(base_quat, root_states[:, 7:10])
        base_lin_vel = quat_rotate_inverse(world_quat, world_vel)
        # base_ang_vel = quat_rotate_inverse(base_quat, root_states[:, 10:13])
        base_ang_vel = quat_rotate_inverse(world_quat, world_ang)
        # projected_gravity = quat_rotate_inverse(base_quat, gravity_vec)
        projected_gravity = quat_rotate_inverse(world_quat, gravity_vec)

        # ── Contact pairs ──
        try:
            cr = sloop.lastContactPairResultsWithPartIds()
            n_contacts = len(cr)
            contact_details = []
            for k, (pa, pb, fx, fy, fz, px, py, pz) in enumerate(cr[:3]):
                f_mag = math.sqrt(fx*fx + fy*fy + fz*fz)
                contact_details.append(
                    f"  c{k}: part{pa}↔part{pb} f=({fx:.1f},{fy:.1f},{fz:.1f}) "
                    f"|f|={f_mag:.1f} pos=({px:.3f},{py:.3f},{pz:.3f})"
                )
        except Exception:
            n_contacts = 0
            contact_details = ["  (no contact API)"]

        # ── Foot positions ──
        foot_parts_map = {'FL': 'FL_calf', 'FR': 'FR_calf',
                          'RL': 'RL_calf', 'RR': 'RR_calf'}
        foot_info = []
        for label, pname in foot_parts_map.items():
            pid = part_name_to_idx.get(pname)
            if pid is not None:
                pm = model.partPool()[pid].getPm()
                foot_info.append(f"{label}_z={pm[11]:.4f}")

        # ── Diagnostic print (same format as training) ──
        tilt = math.sqrt(projected_gravity[0]**2 + projected_gravity[1]**2)
        print(
            f"[step {step:3d}] "
            f"world_vel: vx={world_vel[0]:.3f} vy={world_vel[1]:.3f} vz={world_vel[2]:.3f}\n"
            f"  base_vel:  vx={base_lin_vel[0]:.3f} vy={base_lin_vel[1]:.3f} vz={base_lin_vel[2]:.3f}\n"
            f"  base_ang:  wx={base_ang_vel[0]:.3f} wy={base_ang_vel[1]:.3f} wz={base_ang_vel[2]:.3f}\n"
            f"  quat: qx={world_quat[0]:.4f} qy={world_quat[1]:.4f} qz={world_quat[2]:.4f} qw={world_quat[3]:.4f}\n"
            f"  base_z={world_pos[2]:.3f} tilt={tilt:.4f} "
            f"proj_grav=({projected_gravity[0]:.3f},{projected_gravity[1]:.3f},{projected_gravity[2]:.3f})\n"
            f"  substeps={substep_count} n_contacts={n_contacts}  "
            f"feet: {' '.join(foot_info)}",
            flush=True,
        )
        for cd in contact_details:
            print(cd, flush=True)

        # ── Stability check ──
        if world_pos[2] < -5.0 or abs(world_vel[2]) > 100.0:
            print(f"[repro] ⚠️  Robot unstable at step {step}! Aborting.", flush=True)
            break

    # ══════════════════════════════════════════════════════════════════
    #  Save recording
    # ══════════════════════════════════════════════════════════════════
    result = sloop.recordsToJson()
    display_init = sim.displayInitJson()
    rec = {
        "nlinks": int(model.nbody),
        "display_init": display_init,
        "frames": result,
    }
    out_path = Path(args.out)
    with open(out_path, "w") as f:
        json.dump(rec, f)
    times = result.get("timeIndex", [])
    t_info = f"{times[0]:.3f}~{times[-1]:.3f}" if times else "empty"
    print(f"\n[repro] Recording → {out_path}  (frames={len(times)}, time={t_info})")

    # ── Viewer ──
    if not args.headless:
        try:
            import meshcat
            resource_path = "D:/code/sire/demo/demo_python/dogRL"
            vis = meshcat.Visualizer()
            sire.robotInit(model.nbody, resource_path, display_init, vis)
            frames = rec["frames"]
            t0 = frames["timeIndex"][0]
            frames["timeIndex"] = [t - t0 for t in frames["timeIndex"]]
            sire.animateRobotByRecords(model.nbody, frames, 1000, vis)
            input("[repro] Press Enter to exit viewer...")
        except ImportError:
            print("[repro] meshcat not installed.")

    # ── Summary ──
    pq_final = np.array(base_part.pq)
    vs_final = np.array(base_part.vs)
    print(f"\n[repro] ══ Summary (seed={args.seed}) ══")
    print(f"  XML: {xml_path}")
    print(f"  Steps: {num_ctrl_steps} ctrl × {DECIMATION} substeps")
    print(f"  Action mode: {args.action}")
    print(f"  Final base z: {pq_final[2]:.4f}")
    print(f"  Final base vz (world): {vs_final[2]:.3f}")
    if n_contacts > 0 and all("|f|=0.0" in cd for cd in contact_details):
        print("\n  ⚠️  CONTACT PAIRS DETECTED BUT ALL FORCES ARE ZERO!")
        print("  → PsVsSolver3 finds penetration but force_W is zero.")
        print("  → Check src/physics/contact/ps_vs_solver3.cpp")


if __name__ == "__main__":
    main()


if __name__ == "__main__":
    main()
