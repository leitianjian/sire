"""Correct foot position: calf world pose × sphere local offset."""
import sys, numpy as np
sys.path.insert(0, r'D:\code\sire\demo\demo_python')
import sire

sim = sire.Simulator()
sire.fromXmlFile(sim, r'D:\code\sire\demo\demo_python\sirePaperDogRL\go2.xml')
sim.init()
m = sim.model()

defaults = [0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1.0, -1.5, -0.1, 1.0, -1.5]
for i in range(12):
    m.motionPool()[i].mp = float(defaults[i])
    m.motionPool()[i].mv = 0.0

# Foot sphere local offset from calf part (from go2.xml)
sphere_offset = np.array([-0.002, 0.0, -0.213, 1.0])
sphere_radius = 0.022

print("Correct foot sphere positions (default joint angles):")
print(f"{'base_z':>8s}  {'foot_center_z':>45s}  {'min_bottom':>10s}  status")
for bz in [0.34, 0.40, 0.445, 0.50, 0.55, 0.60]:
    m.partPool()[1].pq = np.array([0.0, 0.0, bz, 0.0, 0.0, 0.0, 1.0])
    m.forwardKinematics()
    
    foot_centers = []
    for pid in [4, 7, 10, 13]:
        pm = np.array(m.partPool()[pid].getPm()).reshape(4, 4)
        sphere_world = pm @ sphere_offset
        foot_centers.append(sphere_world[2])
    
    foot_bottoms = [c - sphere_radius for c in foot_centers]
    centers_str = " ".join(f"{c:.4f}" for c in foot_centers)
    min_b = min(foot_bottoms)
    status = "PENETRATES!" if min_b < -0.001 else "TOUCHING" if abs(min_b) < 0.005 else "above"
    print(f"{bz:8.3f}  {centers_str:>45s}  {min_b:10.4f}  {status}")

# Also show which base_z puts foot exactly at ground
print("\nLinear interpolation: foot_bottom ≈ base_z - offset")
print("Finding base_z where min_foot_bottom = 0...")
# From data: at bz=0.34, min_b=?, at bz=0.445, min_b=?
m.partPool()[1].pq = np.array([0.0, 0.0, 0.34, 0.0, 0.0, 0.0, 1.0])
m.forwardKinematics()
pm = np.array(m.partPool()[4].getPm()).reshape(4, 4)
fz_034 = (pm @ sphere_offset)[2] - sphere_radius

m.partPool()[1].pq = np.array([0.0, 0.0, 0.445, 0.0, 0.0, 0.0, 1.0])
m.forwardKinematics()
pm = np.array(m.partPool()[4].getPm()).reshape(4, 4)
fz_445 = (pm @ sphere_offset)[2] - sphere_radius

slope = (fz_445 - fz_034) / (0.445 - 0.34)
intercept = fz_034 - slope * 0.34
bz_touch = -intercept / slope if abs(slope) > 1e-9 else 0
print(f"At base_z=0.34: FL_foot_bottom={fz_034:.4f}")
print(f"At base_z=0.445: FL_foot_bottom={fz_445:.4f}")
print(f"Slope={slope:.4f}, intercept={intercept:.4f}")
print(f"Foot touches ground at base_z ≈ {bz_touch:.3f}")
