"""Diagnose solver internals - capture minTime, x0, forces at each frame."""
import sys, os
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
ball = m.partPool()[1]

# Run a short simulation, print key data every ctrl step
fmt = "%6s  %8s  %8s  %8s  %12s  %12s  %12s"
print(fmt % ("i", "t", "bz", "vz", "fx", "fy", "fz"))

for i in range(4000):
    sl.handleContact()
    t = sl.simTime()
    bz = ball.pq[2]
    bvz = ball.vs[2]
    cr = sl.lastContactPairResults()
    if len(cr) > 0:
        c = cr[0]
        print(fmt % (i, "%.4f"%t, "%.4f"%bz, "%.3f"%bvz, 
                     "%.3f"%c[2], "%.3f"%c[3], "%.3f"%c[4]))
    else:
        print(fmt % (i, "%.4f"%t, "%.4f"%bz, "%.3f"%bvz,
                     "---", "---", "---"))
    
    # # Stop early if bouncing
    # if bz > 0.5:
    #     print("*** BOUNCE at i=%d ***" % i)
    #     break
