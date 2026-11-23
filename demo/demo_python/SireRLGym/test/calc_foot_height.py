import sys, numpy as np
sys.path.insert(0, r'D:\code\sire\demo\demo_python')
import sire

sim = sire.Simulator()
sire.fromXmlFile(sim, r'D:\code\sire\demo\demo_python\sirePaperDogRL\go2.xml')
sim.init()
model = sim.model()

# Test 1: legs FULLY STRAIGHT (all joints = 0)
print("=== Legs FULLY STRAIGHT (all joints=0) ===")
for i in range(12):
    model.motionPool()[i].mp = 0.0
    model.motionPool()[i].mv = 0.0

for bz in [0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.78, 0.80]:
    model.partPool()[1].pq = np.array([0.0, 0.0, bz, 0.0, 0.0, 0.0, 1.0])
    model.forwardKinematics()
    fz = [model.partPool()[pid].getPm()[11] for pid in [4,7,10,13]]
    fb = [z - 0.022 for z in fz]
    min_fb = min(fb)
    status = "PENETRATES!" if min_fb < 0 else "above"
    print(f"  base_z={bz:.2f}: min_foot_bottom={min_fb:.4f}  {status}")

print()

# Test 2: DEFAULT joint angles
defaults = [0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1.0, -1.5, -0.1, 1.0, -1.5]
print("=== DEFAULT joint angles ===")
for i in range(12):
    model.motionPool()[i].mp = float(defaults[i])
    model.motionPool()[i].mv = 0.0

for bz in [0.10, 0.14, 0.16, 0.18, 0.20, 0.25, 0.34]:
    model.partPool()[1].pq = np.array([0.0, 0.0, bz, 0.0, 0.0, 0.0, 1.0])
    model.forwardKinematics()
    fz = [model.partPool()[pid].getPm()[11] for pid in [4,7,10,13]]
    fb = [z - 0.022 for z in fz]
    min_fb = min(fb)
    status = "PENETRATES!" if min_fb < 0 else "above"
    print(f"  base_z={bz:.2f}: min_foot_bottom={min_fb:.4f}  {status}")

# What base_z gives foot_bottom=0 with default angles?
# Linear interpolation: foot_bottom = base_z - 0.34 + 0.18 = base_z - 0.16
# So base_z = 0.16 for foot_bottom=0
print()
print("=== Summary ===")
print("With default joint angles: foot touches ground at base_z ~ 0.16")
print("With fully straight legs:   foot touches ground at base_z ~ 0.78")
print("Current spawn: base_z = 0.34 -> foot is 0.18m ABOVE ground")
print("Leg length from hip to foot (straight) ~ 0.78m total")
