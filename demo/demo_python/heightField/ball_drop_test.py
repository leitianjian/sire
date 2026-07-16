"""Ball drop on training HeightField — verify contact detection and response."""
import sys, os
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))
sys.path.insert(0, os.path.dirname(os.path.dirname(_HERE)))

import sire

XML = os.path.join(_HERE, "ball_drop_hf.xml")
sim = sire.Simulator()
sire.fromXmlFile(sim, XML)
sim.init()

m = sim.model()
pe = sim.physicsEngine()
sl = sim.simulationLoop()

ball = m.partPool()[1]  # ball is part 1

print(f"Ball init: z={ball.pq[2]:.4f}  vz={ball.vs[2]:.3f}")
print(f"HeightField: part_id=0")
print()
print(f"{'t(s)':>8s}  {'ball_z':>8s}  {'vz':>8s}  {'penet_pair':>12s}  contacts")

def penetration_z(contact_results):
    """Extract max penetration z from contact pair results."""
    pen = 0.0
    for c in contact_results:
        if (c[0] == 0 and c[1] == 1) or (c[1] == 0 and c[0] == 1):
            # pair format: pa, pb, fx, fy, fz, px, py, pz (7 elements from Sire)
            if len(c) >= 7:
                pen = c[6]  # pz = penetration z
    return pen

for i in range(10000):  # max 6 seconds
    sl.handleContact()
    t = sl.simTime()
    bz = ball.pq[2]
    bvz = ball.vs[2]
    cr = sl.lastContactPairResults()
    pz = penetration_z(cr)
    gc = sum(1 for c in cr if (c[0] == 0 and c[1] == 1) or (c[1] == 0 and c[0] == 1))

    if i % 5 == 0:  # every 0.1s
        print(f"{i} {t:8.3f}  {bz:8.4f}  {bvz:8.3f}  {pz:12.6f}  {gc:5d}")

    # Stop if ball fell through and stopped
    if bz < -5.0:
        print(f"  Ball fell through terrain at t={t:.3f}s")
        break
    # if t > 0.221:
    if t > 0.5:
        break

print(f"\nFinal: t={sl.simTime():.3f}s  ball_z={ball.pq[2]:.4f}")

# MeshCat
sl.recordsContactCptInfo()
display_init = sim.displayInitJson()
result = sl.recordsToJson()
n_frames = len(result.get('timeIndex', []))
if n_frames > 0:
    import meshcat
    vis = meshcat.Visualizer()
    vis.open()
    resource_path = os.path.join(os.path.dirname(os.path.dirname(_HERE)), 'dogRL')
    sire.robotInit(m.numLinks(), resource_path, display_init, vis)
    sire.animateRobotByRecords(m.numLinks(), result, 1000, vis)
    input("  Press Enter to close...")
