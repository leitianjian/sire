"""
Visualize the GENERATED trimesh terrain XML directly with MeshCat.
Loads the exact XML used by the training simulator.
"""
import sys, os
import numpy as np

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_TEST_DIR))
sys.path.insert(0, os.path.dirname(os.path.dirname(_TEST_DIR)))

import sire

XML_PATH = r"C:\Users\leiti\AppData\Local\Temp\mujoco_rl_terrain\sire_terrain_znf9cxb4.xml"

# Load simulator from the exact training XML
sim = sire.Simulator()
sire.fromXmlFile(sim, XML_PATH)
sim.init()

m = sim.model()
pe = sim.physicsEngine()
sl = sim.simulationLoop()

print("=" * 70)
print("LOADED XML:", XML_PATH)
print("=" * 70)

# Part info
print(f"\n  Parts ({m.numLinks()}):")
for idx in range(min(m.numLinks(), 15)):
    p = m.partPool()[idx]
    print(f"    [{idx}] name={p.name:>12s}  z={p.pq[2]:.4f}")

# Collision geometries
print(f"\n  Collision geometries ({pe.numGeometries()}):")
for idx in range(pe.numGeometries()):
    g = pe.geometryPool[idx]
    if g is not None:
        pm = g.getPm()
        if pm is not None:
            pm4 = np.array(pm).reshape(4, 4)
            pz = pm4[2, 3]
        else:
            pz = 'N/A'
        print(f"    id={g.id}  part_id={g.prtId:>2d}  type={type(g).__name__:>30s}  z={pz}")

# Collision filter
cf = pe.collisionFilter
fs = cf.filterState if cf else None
if fs is not None:
    n = int(np.sqrt(len(fs)))
    print(f"\n  Collision filter ({n}x{n}):")
    print(f"  Row[0] (ground): {fs[:n]}")
    print(f"  Row[1] (base  ): {fs[n:2*n]}")

# Pre-step contacts (testing without stepping)
print("\n  Pre-init contacts:")
sl.recordsContactCptInfo()
cr = sl.lastContactPairResults()
print(f"    {len(cr)} contact pairs")

# Let gravity run a few steps to see if robot lands
print("\n  Stepping 5 ctrl intervals with zero actions to test contacts...")
for _ in range(5):
    while not sl.headerIsCtrl():
        sl.handleContact()
    sl.handleContact()
    cr = sl.lastContactPairResults()
    gc = sum(1 for c in cr if c[0] == 0 or c[1] == 0) if cr else 0
    print(f"    step: base_z={m.partPool()[1].pq[2]:.4f}  contacts={len(cr)}  ground_contacts={gc}")

# ---- MeshCat ----
print("\n  Processing records for MeshCat...")
sl.recordsContactCptInfo()
display_init = m.displayInitJson()
result = sl.recordsToJson()
n_frames = len(result.get('timeIndex', []))
print(f"  Records: {n_frames} frames")

if n_frames == 0:
    print("  WARNING: No frames recorded!")
else:
    try:
        import meshcat
        vis = meshcat.Visualizer()
        vis.open()
        resource_path = os.path.join(os.path.dirname(os.path.dirname(_TEST_DIR)), 'dogRL')
        n_links = m.numLinks()
        print(f"  Robot has {n_links} links")
        print(f"  Resource path: {resource_path}")
        sire.robotInit(n_links, resource_path, display_init, vis)
        print("  Open http://localhost:7000/static/ in browser")
        print("  Playing animation...")
        sire.animateRobotByRecords(n_links, result, 1000, vis)
        input("Press Enter to stop...")
    except Exception as e:
        print(f"  MeshCat error: {e}")
        import traceback
        traceback.print_exc()
