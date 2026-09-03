"""Swap contact order in result1.json to test order-dependence of v2/v4."""
import json
import numpy as np

with open("scripts/tools/result1.json") as f:
    data = json.load(f)

n = data["n"]

def swap3(arr, i, j):
    """Swap 3-element blocks at positions i and j in flat array."""
    tmp = arr[i:i+3]
    arr[i:i+3] = arr[j:j+3]
    arr[j:j+3] = tmp

def swap_matrix_3n(M, n, a, b):
    """Swap contacts a and b in a 3n x 3n row-major matrix."""
    sz = 3 * n
    # Swap rows
    for k in range(3):
        i1, i2 = 3*a + k, 3*b + k
        tmp = M[i1*sz : (i1+1)*sz].copy()
        M[i1*sz : (i1+1)*sz] = M[i2*sz : (i2+1)*sz]
        M[i2*sz : (i2+1)*sz] = tmp
    # Swap cols
    for k in range(3):
        j1, j2 = 3*a + k, 3*b + k
        col1 = [M[r*sz + j1] for r in range(sz)]
        col2 = [M[r*sz + j2] for r in range(sz)]
        for r in range(sz):
            M[r*sz + j1] = col2[r]
            M[r*sz + j2] = col1[r]

# Swap contacts 0 and 1
swap3(data["v0"], 0, 3)
swap3(data["b"], 0, 3)
swap3(data["cResult"], 0, 3)
data["v_target"][0], data["v_target"][1] = data["v_target"][1], data["v_target"][0]
data["fri_coef"][0], data["fri_coef"][1] = data["fri_coef"][1], data["fri_coef"][0]
swap_matrix_3n(data["invM"], n, 0, 1)

with open("scripts/tools/result1_swapped.json", "w") as f:
    json.dump(data, f, indent=2)

print("Swapped JSON written to scripts/tools/result1_swapped.json")
print("Compare with: python scripts/tools/compare_solvers.py --load scripts/tools/result1_swapped.json")
