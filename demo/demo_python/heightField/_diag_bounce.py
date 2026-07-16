"""Diagnose energy injection in contact solver - dense sampling."""
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

fmt = "{:>8s}  {:>8s}  {:>8s}  {:>12s}  {:>12s}  {:>12s}  {:>10s}  {:>10s}  {:>10s}"
print(fmt.format("t", "ball_z", "vz", "fx", "fy", "fz", "px", "py", "pz"))

bounced = False
for i in range(10000):
    while not sl.headerIsCtrl():
        sl.handleContact()
    sl.handleContact()
    t = sl.simTime()
    bz = ball.pq[2]
    bvz = ball.vs[2]
    cr = sl.lastContactPairResults()
    if len(cr) > 0:
        c = cr[0]
        print(f"{t:8.4f}  {bz:8.4f}  {bvz:8.3f}  {c[2]:12.3f}  {c[3]:12.3f}  {c[4]:12.3f}  {c[5]:10.4f}  {c[6]:10.4f}  {c[7]:10.4f}")
    else:
        print(f"{t:8.4f}  {bz:8.4f}  {bvz:8.3f}  {'(no contact)':>12s}")
    
    # Detect bounce
    if bz > 0.8 and not bounced:
        print("*** MAJOR BOUNCE DETECTED ***")
        bounced = True
        # Print 20 more frames for post-bounce analysis
        for j in range(20):
            while not sl.headerIsCtrl():
                sl.handleContact()
            sl.handleContact()
            t = sl.simTime()
            bz = ball.pq[2]
            bvz = ball.vs[2]
            cr = sl.lastContactPairResults()
            if len(cr) > 0:
                c = cr[0]
                print(f"{t:8.4f}  {bz:8.4f}  {bvz:8.3f}  {c[2]:12.3f}  {c[3]:12.3f}  {c[4]:12.3f}  {c[5]:10.4f}  {c[6]:10.4f}  {c[7]:10.4f}")
            else:
                print(f"{t:8.4f}  {bz:8.4f}  {bvz:8.3f}  {'(no contact)':>12s}")
        break
    
    if t > 12:
        break
