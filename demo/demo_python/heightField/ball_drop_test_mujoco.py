"""Ball drop on HeightField — MuJoCo equivalent of ball_drop_test.py (Sire).

Mirrors the Sire simulation from ball_drop_hf.xml:
  - HeightField terrain: 10×10 m, scale_z=0.2, min_height=-0.02, centered at (7,7,0)
  - Ball: mass=1, radius=0.05, pos=(7,7,0.5), vel=(0,0,-0.5)
  - Contact: stiff (≈k=2.8e8) with friction cof=0.6
  - Gravity: (0,0,-9.81), dt=0.001s
"""

import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))

# ── Try to import MuJoCo ──────────────────────────────────────────────────────
try:
    import mujoco
except ImportError:
    print("MuJoCo not found. Install with:  pip install mujoco")
    sys.exit(1)

try:
    import mujoco.viewer
except ImportError:
    print("MuJoCo viewer not available.")
    sys.exit(1)

XML = os.path.join(_HERE, "ball_drop_hf_mujoco.xml")

if not os.path.exists(XML):
    print(f"ERROR: model file not found: {XML}")
    sys.exit(1)

# ── Build model & data ─────────────────────────────────────────────────────────
model = mujoco.MjModel.from_xml_path(XML)
data = mujoco.MjData(model)

# ── Locate ball body & geom ────────────────────────────────────────────────────
ball_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ball")
ball_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ball_geom")
ground_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")

print(f"ball_body_id  = {ball_body_id}")
print(f"ball_geom_id  = {ball_geom_id}")
print(f"ground_geom_id = {ground_geom_id}")

# The free joint adds 7 qpos entries (x,y,z,qw,qx,qy,qz) and 6 qvel (vx,vy,vz,wx,wy,wz).
# The ball is body 1 (0 = world), so its qpos offset is at model.jnt_qposadr[...].
qpos_adr = model.jnt_qposadr[model.body_jntadr[ball_body_id]]
qvel_adr = model.jnt_dofadr[model.body_dofadr[ball_body_id]]

# Set initial velocity: (0, 0, -0.5) m/s
data.qvel[qvel_adr:qvel_adr + 3] = [0.0, 0.0, -0.5]

print(f"\nBall init: z={data.qpos[qpos_adr + 2]:.4f}  "
      f"vz={data.qvel[qvel_adr + 2]:.3f}")
print(f"HeightField: geom='ground'")
print()


def get_ball_ground_contacts(m, d, ball_gid, ground_gid):
    """Return raw MuJoCo contacts between ball and ground — NO Python filtering.

    Returns list of dicts: {idx, pos, dist, frame, force, friction}
    where idx is the index in d.contact[], dist is signed (negative = penetration).
    """
    raw = []
    for i in range(d.ncon):
        c = d.contact[i]
        g1, g2 = c.geom1, c.geom2
        if (g1 == ball_gid and g2 == ground_gid) or (g1 == ground_gid and g2 == ball_gid):
            force = np.zeros(6, dtype=np.float64)
            mujoco.mj_contactForce(m, d, i, force)
            raw.append({
                "idx": i,
                "pos": c.pos.copy(),
                "dist": float(c.dist),
                "frame": c.frame.copy(),
                "force": force[:3].copy(),
                "friction": c.friction.copy(),
            })
    return raw


def contact_summary_raw(m, d, ball_gid, ground_gid):
    """Return (max_penetration, raw_contact_count, total_normal_force_z).

    NO Python-level filtering/clustering — raw MuJoCo data as-is.
    Penetration depth = -dist (positive when interpenetrating).
    """
    contacts = get_ball_ground_contacts(m, d, ball_gid, ground_gid)
    if not contacts:
        return 0.0, 0, 0.0
    pen = -min(c["dist"] for c in contacts)  # most negative dist → largest penetration
    fz = sum(c["force"][2] for c in contacts)
    return pen, len(contacts), fz


def print_contact_details(m, d, ball_gid, ground_gid, t):
    """Print raw MuJoCo contact details for ball↔ground — NO Python filtering."""
    contacts = get_ball_ground_contacts(m, d, ball_gid, ground_gid)
    n_bg = len(contacts)

    print(f"  ── Raw MuJoCo ball↔ground contacts at t={t:.4f}s "
          f"(total d.ncon={d.ncon}, ball↔ground={n_bg}) ──")

    if n_bg == 0:
        print("    (none)")
        return

    # Header
    print(f"    {'idx':>4s}  {'pos.x':>9s}  {'pos.y':>9s}  {'pos.z':>9s}  "
          f"{'dist':>10s}  {'fx':>10s}  {'fy':>10s}  {'fz':>10s}  "
          f"{'frame_nz':>9s}")

    for c in contacts:
        pos = c["pos"]
        f = c["force"]
        # frame[0:3] = contact normal, so frame[2] = normal.z component
        nz = c["frame"][2]
        print(f"    {c['idx']:4d}  {pos[0]:9.5f}  {pos[1]:9.5f}  {pos[2]:9.5f}  "
              f"{c['dist']:10.6f}  {f[0]:10.2f}  {f[1]:10.2f}  {f[2]:10.2f}  "
              f"{nz:9.4f}")

    # Compute pairwise distances between contact positions
    if n_bg >= 2:
        print(f"    Pairwise contact distances:")
        for i in range(n_bg):
            for j in range(i + 1, n_bg):
                d_ij = np.linalg.norm(contacts[i]["pos"] - contacts[j]["pos"])
                print(f"      contact[{contacts[i]['idx']}] ↔ contact[{contacts[j]['idx']}]: "
                      f"{d_ij:.6f} m")


# ── Main simulation loop (inside viewer, so we see it live) ──────────────────
MAX_STEPS = 8000  # 8 seconds at 0.001s dt

# Visual refresh: sync viewer every N physics steps → ~30 FPS
VISUAL_STEPS = 33  # 33 × 0.001s ≈ 33ms per frame

print("\nLaunching interactive viewer — watching ball drop in real time ...\n")
print(f"{'t(s)':>8s}  {'ball_z':>8s}  {'vz':>8s}  {'pen(mm)':>9s}  "
      f"{'ncon_bg':>7s}  {'ncon_all':>8s}  {'fz(N)':>10s}")

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Point camera at the terrain center so it's not a black void
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -25
    viewer.cam.distance = 3.0
    viewer.cam.lookat[:] = [7.0, 7.0, 0.15]

    last_render = time.time()
    multi_contact_events = []  # (t, ncon_bg, ncon_all) for steps with >=2 ball↔ground contacts

    for step in range(MAX_STEPS):
        # Stop if user closed the viewer window
        if not viewer.is_running():
            print("  Viewer closed by user.")
            break

        mujoco.mj_step(model, data)

        # Sync viewer at ~30 FPS so it doesn't race through
        if step % VISUAL_STEPS == 0:
            viewer.sync()
            # Maintain real-time pacing
            now = time.time()
            elapsed = now - last_render
            target = VISUAL_STEPS * model.opt.timestep  # ~33ms
            if elapsed < target:
                time.sleep(target - elapsed)
            last_render = now

        t = data.time
        bz = data.qpos[qpos_adr + 2]
        bvz = data.qvel[qvel_adr + 2]

        # Raw MuJoCo ball↔ground contacts — NO Python filtering
        pen, ncon_bg, fz = contact_summary_raw(model, data, ball_geom_id, ground_geom_id)
        pen_mm = pen * 1000  # convert to mm for readability

        # Record & print ball↔ground multi-contact events (>=2 raw contacts)
        if ncon_bg >= 2:
            multi_contact_events.append((t, ncon_bg, data.ncon))
            print_contact_details(model, data, ball_geom_id, ground_geom_id, t)

        if step % 100 == 0:  # every 0.1 s (same as Sire test)
            marker = f"  <-- {ncon_bg} ball↔ground raw contacts" if ncon_bg >= 2 else ""
            print(f"{t:8.3f}  {bz:8.4f}  {bvz:8.3f}  {pen_mm:9.3f}  "
                  f"{ncon_bg:7d}  {data.ncon:8d}  {fz:10.2f}{marker}")

        # Stop if ball fell through
        if bz < -5.0:
            print(f"  Ball fell through terrain at t={t:.3f}s")
            break
        if t > 8:
            break

    # ── Multi-contact summary ────────────────────────────────────────────
    print(f"\nFinal: t={data.time:.3f}s  ball_z={data.qpos[qpos_adr + 2]:.4f}")
    if multi_contact_events:
        print(f"\nBall↔ground multi-contact events (raw MuJoCo, >=2 contacts):")
        print(f"  Total occurrences: {len(multi_contact_events)} steps")
        # Show unique (time, ncon_bg, ncon_all) snapshots, sampling if too many
        if len(multi_contact_events) > 20:
            print(f"  First few:")
            for t, nbg, nall in multi_contact_events[:5]:
                print(f"    t={t:.4f}s  ball↔ground={nbg}  total_ncon={nall}")
            print(f"  Last few:")
            for t, nbg, nall in multi_contact_events[-5:]:
                print(f"    t={t:.4f}s  ball↔ground={nbg}  total_ncon={nall}")
            max_nbg = max(nbg for _, nbg, _ in multi_contact_events)
            print(f"  Max simultaneous ball↔ground raw contacts: {max_nbg}")
        else:
            for t, nbg, nall in multi_contact_events:
                print(f"    t={t:.4f}s  ball↔ground={nbg}  total_ncon={nall}")
    else:
        print("\nNo ball↔ground multi-contact events detected (raw ncon_bg never >= 2).")

    # Keep viewer alive so user can orbit / inspect the scene
    print("Simulation finished — viewer stays open. Close the window to exit.")
    while viewer.is_running():
        time.sleep(0.1)
