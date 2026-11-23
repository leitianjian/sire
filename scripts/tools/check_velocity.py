"""Check post-contact velocity consistency across solvers."""
import json
import numpy as np

with open("scripts/tools/result1.json") as f:
    data = json.load(f)

n = data["n"]
h = data["h"]
v0 = np.array(data["v0"])
b = np.array(data["b"])
W = np.array(data["invM"]).reshape(3*n, 3*n)
v_target = np.array(data["v_target"])

# Solver results from the compare output
results = {
    "v2": [0.217, 0.827, 8.433, -6.6e-11, -2.6e-10, 0.0],
    "v3": [0, 0, 0, 2.366, -0.677, 8.203],
    "v4": [0.217, 0.827, 8.442, 0, 0, 0],
    "v5": [-0.744, -0.302, 4.471, 1.098, 0.390, 3.884],
}
results2 = {
    "v5_swapped": [0.390, 1.098, 3.884, 0.744, -0.302, 4.471],
}

print("=== Post-contact velocity v' = v0 + h*b + h*W*f ===\n")

for name, f_vec in {**results, **results2}.items():
    f = np.array(f_vec)
    v_prime = v0 + h * b + h * W @ f
    print(f"--- {name} ---")
    for i in range(n):
        ft = np.linalg.norm(f[3*i:3*i+2])
        fn = f[3*i+2]
        vn = v_prime[3*i+2]
        print(f"  c{i}: f_t={ft:.4f} f_n={fn:.4f}  (mu*f_n={0.3*fn:.4f})")
    print(f"  v'_n = [{v_prime[2]:.6f}, {v_prime[5]:.6f}]")
    
    # Check normal target: v_target is DAE penetration rate
    # In contact frame, we want v'_n to match some target
    # v1 formula: a_target = (v_target + v0_n)/h - b_n
    a_target = np.array([(v_target[i] + v0[3*i+2])/h - b[3*i+2] for i in range(n)])
    print(f"  a_target (v1) = {a_target}")
    print(f"  W·f computed   = {W @ f}")
    print()

# Compare net wrench equivalence
print("=== Net impulse Σ J_i^T * f_i ===")
for name, f_vec in results.items():
    f = np.array(f_vec)
    F_total = np.array([f[0]+f[3], f[1]+f[4], f[2]+f[5]])
    print(f"  {name}: Σf = {F_total}")
print("  (same total normal force = same net vertical impulse)")
