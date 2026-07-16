"""Step-by-step foot position calculation for base_z=0.34 with default joints."""
import sys, numpy as np
sys.path.insert(0, r'D:\code\sire\demo\demo_python')
import sire

sim = sire.Simulator()
sire.fromXmlFile(sim, r'D:\code\sire\demo\demo_python\sirePaperDogRL\go2.xml')
sim.init()
m = sim.model()

# Set default joint angles
defaults = [0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1.0, -1.5, -0.1, 1.0, -1.5]
for i in range(12):
    m.motionPool()[i].mp = float(defaults[i])
    m.motionPool()[i].mv = 0.0

# Set base at z=0.34
base_z = 0.34
m.partPool()[1].pq = np.array([0.0, 0.0, base_z, 0.0, 0.0, 0.0, 1.0])
m.forwardKinematics()

# Print all part world positions
print("=== Part world-frame positions (getPm translation) ===")
for pid in range(m.nbody):
    pm = np.array(m.partPool()[pid].getPm()).reshape(4, 4)
    name = m.partPool()[pid].name
    print(f"  Part {pid} ({name:>10s}): xyz=({pm[0,3]:.4f}, {pm[1,3]:.4f}, {pm[2,3]:.4f})")

print()

# Ground collision surface
print("=== Ground ===")
pe = sim.physicsEngine()
gp = pe.geometryPool
ground_geom = gp[0]  # first geometry is ground box
print(f"  Ground part pe z: {m.partPool()[0].pq[2]:.4f}")
# Ground BoxCollisionGeometry pm: {1,0,0,0,0,1,0,0,0,0,1,-0.1,0,0,0,1}
# The pm matrix is stored in the geometry. Let me check its actual value.
print(f"  Ground collision geometry type: {type(ground_geom).__name__}")
print(f"  Ground collision geometry part_id: {ground_geom.prtId}")

# The ground collision box has pm with z translation = -0.1
# Ground part at z=0, box pm z offset = -0.1, box half-height = 0.1
# Ground surface = 0 + (-0.1) + 0.1 = 0.0
print(f"  Ground surface z = 0 (part z=0 + pm_z=-0.1 + half_side=0.1)")
print()

# Foot sphere calculation
print("=== Foot sphere positions ===")
sphere_offset = np.array([-0.002, 0.0, -0.213, 1.0])  # from XML SphereCollisionGeometry pm
sphere_radius = 0.022

for pid, leg_name in [(4, 'FL'), (7, 'FR'), (10, 'RL'), (13, 'RR')]:
    pm = np.array(m.partPool()[pid].getPm()).reshape(4, 4)
    calf_xyz = pm[:3, 3]
    
    # Transform sphere offset by calf world pose
    sphere_world = pm @ sphere_offset
    sphere_center = sphere_world[:3]
    sphere_bottom = sphere_center[2] - sphere_radius
    
    print(f"  {leg_name} leg:")
    print(f"    Calf part world xyz: ({calf_xyz[0]:.4f}, {calf_xyz[1]:.4f}, {calf_xyz[2]:.4f})")
    print(f"    Sphere local offset: ({sphere_offset[0]:.4f}, {sphere_offset[1]:.4f}, {sphere_offset[2]:.4f})")
    print(f"    Sphere world center:  ({sphere_center[0]:.4f}, {sphere_center[1]:.4f}, {sphere_center[2]:.4f})")
    print(f"    Sphere bottom z:      {sphere_bottom:.4f}")
    print(f"    Distance to ground:   {sphere_bottom:.4f}  {'PENETRATES!' if sphere_bottom < -0.001 else 'TOUCHING' if abs(sphere_bottom) < 0.005 else 'above'}")

print()
print("=== Summary ===")
print(f"base_z = {base_z}")
print(f"ground surface at z = 0")
all_bottoms = []
for pid in [4, 7, 10, 13]:
    pm = np.array(m.partPool()[pid].getPm()).reshape(4, 4)
    sphere_world = pm @ sphere_offset
    all_bottoms.append(sphere_world[2] - sphere_radius)
print(f"foot bottoms: {[f'{b:.4f}' for b in all_bottoms]}")
print(f"min foot bottom: {min(all_bottoms):.4f}")
print(f"All feet above ground: {all(b > -0.001 for b in all_bottoms)}")
