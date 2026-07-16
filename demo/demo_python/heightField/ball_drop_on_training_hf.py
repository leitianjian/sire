"""
Ball drop on HeightField — uses the exact HeightField from training config.
Prints sim_time, ball_z, vz, penetration, contacts at every substep.
"""
import sys, os
import numpy as np

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_TEST_DIR))
sys.path.insert(0, os.path.dirname(os.path.dirname(_TEST_DIR)))

import sire

# Load training config's HeightField terrain from the generated XML
XML_PATH = r"C:\Users\leiti\AppData\Local\Temp\mujoco_rl_terrain\sire_terrain_znf9cxb4.xml"
sim = sire.Simulator()
sire.fromXmlFile(sim, XML_PATH)
sim.init()

m = sim.model()
pe = sim.physicsEngine()
sl = sim.simulationLoop()

# ── Replace robot with a single ball ──────────────────────────────
# Remove all robot parts (keep ground=part[0])
while m.partPool().size() > 1:
    m.partPool().pop_back()

# Deactivate all existing motions/joints (they reference deleted parts)
mpool = m.motionPool()
while mpool.size() > 0:
    mpool.pop_back()
jpool = m.jointPool()
while jpool.size() > 0:
    jpool.pop_back()

# Remove robot collision geoms from physics engine (keep ground=part_id=0)
gp = pe.geometryPool
to_remove = []
for idx in range(gp.size()):
    g = gp[idx]
    if g is not None and g.prtId != 0:
        to_remove.append(idx)
# Remove in reverse order
for idx in sorted(to_remove, reverse=True):
    gp.erase(idx)  # try erase; if not available, use pop

# Add a ball part
ball_name = "ball"
ball_pe = np.array([7.0, 7.0, 0.8, 0.0, 0.0, 0.0])  # above HeightField center
ball_inertia = np.array([1.0, 0.0, 0.0, 0.0, 0.02, 0.02, 0.02, 0.0, 0.0, 0.0])

m.partPool().emplace_back(ball_name, ball_pe, np.zeros(6), np.zeros(6), ball_inertia)

# Add sphere collision geometry for the ball
ball_part_id = m.partPool().size() - 1  # should be 1
ball_radius = 0.05
ball_pm = np.array([1.0,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1])

# Construct a SphereCollisionGeometry
# Use XML-like approach: add to physics engine geometry pool
# Actually, let's use from_xml or manually add
sphere_geom = sire.physics.geometry.SphereCollisionGeometry(
    ball_radius, ball_part_id, True,  # radius, part_id, is_dynamic
    ball_pm, True, "m1", "{k:2.8e8,d:2000}"
)
pe.geometryPool.push_back(sphere_geom)

# Update collision filter for new ball (now 2 parts: ground=0, ball=1)
# Filter: [1,1; 1,1] — all collide
cf = pe.collisionFilter
old_fs = cf.filterState()
n_old = int(np.sqrt(len(old_fs))) if old_fs else 0
n_new = m.partPool().size()
new_fs = list(old_fs[:n_old*n_old]) if old_fs else []
# Extend to new size
fs_2d = np.array(new_fs).reshape(n_old, n_old) if n_old > 0 else np.zeros((0,0))
fs_new = np.ones((n_new, n_new), dtype=np.int32)
if n_old > 0:
    fs_new[:n_old, :n_old] = fs_2d
cf.setFilterState(fs_new.flatten().tolist())

# Add ball to display
import meshcat
vis = meshcat.Visualizer()
vis.open()

# Create ball visual
ball_mesh = meshcat.geometry.Sphere(ball_radius)
ball_mat = meshcat.geometry.MeshLambertMaterial(color=0xff4444)
vis['ball'].set_object(ball_mesh, ball_mat)

# HeightField visual (from existing display)
display_init = sim.displayInitJson()
resource_path = os.path.join(os.path.dirname(os.path.dirname(_TEST_DIR)), 'dogRL')
sire.robotInit(m.numLinks(), resource_path, display_init, vis)

# ── Step physics and track ─────────────────────────────────────────
print(f"{'t(s)':>8s}  {'ball_z':>8s}  {'vz':>8s}  {'penet':>8s}  {'contacts':>8s}")
last_z = ball_pe[2]
for i in range(200):  # 200 ctrl intervals = 4 seconds
    try:
        while not sl.headerIsCtrl():
            sl.handleContact()
        sl.handleContact()
    except RuntimeError as e:
        print(f"  CRASH at t={sl.simTime():.3f}s: {e}")
        break

    ball = m.partPool()[ball_part_id]
    bz = ball.pq[2]
    bvz = ball.vs[2]

    # Penetration info
    cr = sl.lastContactPairResults()
    penet = 0.0
    gc = 0
    for c in cr:
        if (c[0] == 0 and c[1] == ball_part_id) or (c[1] == 0 and c[0] == ball_part_id):
            gc += 1
            # penetration depth (pair format: pa,pb,fx,fy,fz,px,py,pz)
            # Actually check format
            if len(c) >= 6:
                penet = max(penet, abs(c[5]) if len(c) > 5 else 0)

    t = sl.simTime()
    # Update ball mesh position
    vis['ball'].set_transform(meshcat.transformations.translation_matrix(
        [ball.pq[0], ball.pq[1], ball.pq[2]]))

    if i % 10 == 0:
        print(f"{t:8.3f}  {bz:8.4f}  {bvz:8.3f}  {penet:8.4f}  {gc:8d}")

print(f"\nFinal: t={sl.simTime():.3f}s  ball_z={m.partPool()[ball_part_id].pq[2]:.4f}")
print(f"Open http://localhost:7000/static/")
input("Press Enter to close...")
