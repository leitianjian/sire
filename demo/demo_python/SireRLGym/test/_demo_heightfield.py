"""
Demo: visualize robot on HeightField terrain (training config).
Loads the exact XML used by training, positions robot on the heightfield,
steps with zero actions, renders in MeshCat.

Usage: python test/_demo_heightfield.py
"""
import sys, os
import numpy as np

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_TEST_DIR))
sys.path.insert(0, os.path.dirname(os.path.dirname(_TEST_DIR)))

import sire

XML_PATH = r"C:\Users\leiti\AppData\Local\Temp\mujoco_rl_terrain\sire_terrain_znf9cxb4.xml"

sim = sire.Simulator()
sire.fromXmlFile(sim, XML_PATH)
sim.init()

m = sim.model()
pe = sim.physicsEngine()
sl = sim.simulationLoop()

# ---- Show HeightField config ----
print("=" * 60)
print("HeightField terrain config:")
print("=" * 60)
gp = pe.geometryPool
for idx in range(pe.numGeometries()):
    g = gp[idx]
    if g is not None and g.prtId == 0:  # ground
        pm = np.array(g.getPm()).reshape(4, 4)
        print(f"  Type: {type(g).__name__}")
        print(f"  Center: ({pm[0,3]:.1f}, {pm[1,3]:.1f}, {pm[2,3]:.1f})")
        if hasattr(g, 'x_dim'):
            print(f"  Size: {g.x_dim}x{g.y_dim} m")
            print(f"  scale_z={g.scale_z}  min_height={g.min_height}")

# ---- Move robot onto the heightfield ----
base = m.partPool()[1]
base.pq = (7.0, 7.0, 0.44, base.pq[3], base.pq[4], base.pq[5], base.pq[6])
base.vs = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

# ---- Set default standing joint angles (same as training _reset_dofs) ----
default_angles = [
    0.1,   # actuator_0: FL_hip
    0.8,   # actuator_1: FL_thigh
    -1.5,  # actuator_2: FL_calf
    -0.1,  # actuator_3: FR_hip
    0.8,   # actuator_4: FR_thigh
    -1.5,  # actuator_5: FR_calf
    0.1,   # actuator_6: RL_hip
    1.0,   # actuator_7: RL_thigh
    -1.5,  # actuator_8: RL_calf
    -0.1,  # actuator_9: RR_hip
    1.0,   # actuator_10: RR_thigh
    -1.5,  # actuator_11: RR_calf
]
for j, angle in enumerate(default_angles):
    m.motionPool()[j].mp = angle
    m.motionPool()[j].mv = 0.0

m.forwardKinematics()
m.forwardKinematicsVel()

# ---- Check foot positions vs HeightField surface ----
print(f"\n  Robot placed at (7, 7, 0.44), standing pose")
print(f"  Foot positions:")
sphere_off = np.array([-0.002, 0.0, -0.213, 1.0])
foot_parts = [(4, 'FL'), (7, 'FR'), (10, 'RL'), (13, 'RR')]
for pid, name in foot_parts:
    pm = np.array(m.partPool()[pid].getPm()).reshape(4, 4)
    foot_z = (pm @ sphere_off)[2]
    foot_bottom = foot_z - 0.022
    status = 'PENETRATE!' if foot_bottom < 0 else 'ABOVE'
    print(f"    {name}: bottom_z={foot_bottom:.4f} {status}")

# ---- Gently step with zero actions (record for MeshCat) ----
print("\n  Stepping (zero actions, gravity only)...")
print(f"  {'sim_time':>10s}  {'base_z':>8s}  {'vz':>8s}  contacts")
for step in range(20):
    try:
        while not sl.headerIsCtrl():
            sl.handleContact()
        sl.handleContact()
    except RuntimeError as e:
        print(f"  physics crash at t={sl.simTime():.3f}s: {e}")
        break
    cr = sl.lastContactPairResults()
    gc = sum(1 for c in cr if c[0] == 0 or c[1] == 0) if cr else 0
    if step % 2 == 0:
        print(f"  {sl.simTime():8.3f}s  {base.pq[2]:8.4f}  {base.vs[2]:8.2f}  {gc}")

print(f"\n  Final: base_z={base.pq[2]:.4f}")

# ---- MeshCat ----
print("\n  Generating MeshCat animation...")
sl.recordsContactCptInfo()
display_init = sim.displayInitJson()
result = sl.recordsToJson()
n_frames = len(result.get('timeIndex', []))

if n_frames == 0:
    print("  ERROR: No frames recorded!")
else:
    print(f"  {n_frames} frames ready")

    # Print display init geometry pool to check hfield shape_type
    gp = display_init.get('geometry_pool', [])
    for i, g in enumerate(gp):
        if g.get('part_id') == 0:
            print(f"  display geom[{i}]: part_id=0 shape_type={g.get('shape_type')} keys={list(g.keys())}")

    try:
        import meshcat
        vis = meshcat.Visualizer()
        vis.open()
        resource_path = os.path.join(os.path.dirname(os.path.dirname(_TEST_DIR)), 'dogRL')
        sire.robotInit(m.numLinks(), resource_path, display_init, vis)
        print("  Open http://localhost:7000/static/")
        sire.animateRobotByRecords(m.numLinks(), result, 1000, vis)
        input("  Press Enter to close...")
    except ImportError:
        print("  meshcat not installed: pip install meshcat")
