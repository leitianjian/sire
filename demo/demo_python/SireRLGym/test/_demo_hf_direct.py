"""
Demo: load the training XML directly, position robot ON the HeightField,
step physics, show contacts and MeshCat visualization.
"""
import sys, os
import numpy as np

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_TEST_DIR))
sys.path.insert(0, os.path.dirname(os.path.dirname(_TEST_DIR)))

import sire

XML_PATH = r"D:\code\sire\demo\demo_python\SireRLGym\test\go2_test.xml"

sim = sire.Simulator()
sire.fromXmlFile(sim, XML_PATH)
sim.init()

m = sim.model()
pe = sim.physicsEngine()
sl = sim.simulationLoop()

# ---- HeightField info ----
gp = pe.geometryPool
for idx in range(pe.numGeometries()):
    g = gp[idx]
    if g is not None:
        print(f"geom id={g.id} part_id={g.prtId} type={type(g).__name__}")
        if hasattr(g, 'file'):
            print(f"  HeightField file={g.file} x_dim={g.x_dim} y_dim={g.y_dim} scale_z={g.scale_z} min_h={g.min_height}")
        pm = g.getPm()
        if pm is not None:
            pm4 = np.array(pm).reshape(4,4)
            print(f"  pm pos=({pm4[0,3]:.2f}, {pm4[1,3]:.2f}, {pm4[2,3]:.2f})")

# ---- Move robot to heightfield center ----
# HeightField is at (7,7,0) center, robot needs to be at (7,7,0.44)
base = m.partPool()[1]
# old_z = base.pq[2]
# base.pq = (7.0, 7.0, old_z, base.pq[3], base.pq[4], base.pq[5], base.pq[6])
# base.vs = (0, 0, 0, 0, 0, 0)
m.forwardKinematics()
m.forwardKinematicsVel()

# print(f"\n  Robot moved to: base_z={base.pq[2]:.3f} at (7,7)")

# ---- Step 100 ctrl intervals with ZERO actions to let robot fall onto HeightField ----
print("\n  Falling for 100 ctrl steps...")
for step in range(10000):
    t = sl.simTime()
    # step events
    # while not sl.headerIsCtrl():
    sl.handleContact()
    
    cr = sl.lastContactPairResults()
    gc = sum(1 for c in cr if c[0] == 0 or c[1] == 0) if cr else 0
    
    if step % 10 == 0:
        print(f"    step {step:3d}: base_z={base.pq[2]:.4f}  vz={base.vs[2]:.3f}  contacts={len(cr)}  ground_contacts={gc}")
    
    if t > 0.5:
        break

print(f"\n  FINAL: base_z={base.pq[2]:.4f}  contacts={len(cr)}  ground_contacts={gc}")

# ---- MeshCat ----
print("\n  Processing records for MeshCat...")
sl.recordsContactCptInfo()
display_init = sim.displayInitJson()
result = sl.recordsToJson()
n_frames = len(result.get('timeIndex', []))

if n_frames == 0:
    print("  WARNING: No frames recorded!")
else:
    print(f"  {n_frames} frames, opening MeshCat...")
    try:
        import meshcat
        vis = meshcat.Visualizer()
        vis.open()
        resource_path = os.path.join(os.path.dirname(os.path.dirname(_TEST_DIR)), 'dogRL')
        sire.robotInit(m.numLinks(), resource_path, display_init, vis)
        print("  Open http://localhost:7000/static/ in browser")
        print("  Ground is a HeightField (C++ collision geom, NOT rendered in MeshCat)")
        print("  Robot should fall and land on the invisible HeightField at z~0")
        sire.animateRobotByRecords(m.numLinks(), result, 1000, vis)
        input("  Press Enter to stop...")
    except Exception as e:
        print(f"  MeshCat error: {e}")
        import traceback; traceback.print_exc()
