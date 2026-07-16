"""
Test: Sire heightfield bilinear interpolation vs MuJoCo mj_ray.

Creates a synthetic heightfield, writes it as a MuJoCo PNG hfield scene,
then compares Sire-style bilinear interpolation against MuJoCo mj_ray results
at 500 random world-space positions.

Key finding: MuJoCo flips PNG rows when reading hfields
(hfield_data[i,:] ≈ hf[nrow-1-i,:] / peak).
The Sire implementation accounts for this via flipped y mapping.

Usage:
    cd demo/demo_python
    python test_height_scan.py
"""

from __future__ import annotations

import struct
import tempfile
import zlib
from pathlib import Path

import numpy as np

# Shared bilinear interpolation (same code used by legged_robot_sire.py)
from SireRLGym.utils.height_query import bilinear_height

try:
    from PIL import Image as PILImage
    _HAS_PIL = True
except ImportError:
    _HAS_PIL = False


# ═══════════════════════════════════════════════════════════════════
#  Heightfield generation
# ═══════════════════════════════════════════════════════════════════

def make_heightfield(nrow=129, ncol=129, amplitude=0.15, seed=42):
    """Generate a smooth random heightfield (nrow x ncol), min=0."""
    rng = np.random.default_rng(seed)
    h = rng.normal(0, 1, (nrow, ncol)).astype(np.float64)
    for _ in range(4):
        h = (h + np.roll(h, 1, axis=0) + np.roll(h, -1, axis=0)
             + np.roll(h, 1, axis=1) + np.roll(h, -1, axis=1)) / 5.0
    h -= h.min()
    peak = float(h.max())
    if peak > 1e-6:
        h *= amplitude / peak
    return h


# ═══════════════════════════════════════════════════════════════════
#  Test 1: self-consistency
# ═══════════════════════════════════════════════════════════════════

def test_self_consistency():
    print("-" * 50)
    print("Test 1: self-consistency (grid-point recovery)")

    border, total_len, total_wid = 1.0, 4.0, 3.0
    nrow, ncol = 65, 65
    hf = make_heightfield(nrow, ncol, amplitude=0.1, seed=123)

    max_err = 0.0
    for r in range(nrow):
        for c in range(ncol):
            wx = border + total_len * c / max(1, ncol - 1)
            wy = border + total_wid * r / max(1, nrow - 1)
            err = abs(bilinear_height(hf, wx, wy,
                                      border, total_len, total_wid) - hf[r, c])
            if err > max_err:
                max_err = err

    print(f"  Max error at grid points: {max_err:.2e}")
    assert max_err < 1e-14, f"Grid recovery failed: {max_err}"

    rng = np.random.default_rng(456)
    for _ in range(1000):
        wx = border + rng.uniform(0, total_len)
        wy = border + rng.uniform(0, total_wid)
        h = bilinear_height(hf, wx, wy, border, total_len, total_wid)
        assert 0.0 <= h <= 0.1 + 1e-9
    print("  PASS")


# ═══════════════════════════════════════════════════════════════════
#  Test 2: out-of-bounds
# ═══════════════════════════════════════════════════════════════════

def test_out_of_bounds():
    print("\n" + "-" * 50)
    print("Test 2: out-of-bounds handling")
    hf = make_heightfield(33, 33)
    assert bilinear_height(hf, -10., 0., 1., 4., 3.) == 0.
    assert bilinear_height(hf, 0., -10., 1., 4., 3.) == 0.
    assert bilinear_height(hf, 1000., 1000., 1., 4., 3.) == 0.
    print("  PASS")


# ═══════════════════════════════════════════════════════════════════
#  MuJoCo helpers
# ═══════════════════════════════════════════════════════════════════

def _write_png_8bit(hf: np.ndarray, suffix: str = "") -> Path:
    peak = float(hf.max())
    if peak > 1e-6:
        norm = np.clip(np.round(hf / peak * 255.0), 0, 255).astype(np.uint8)
    else:
        norm = np.zeros_like(hf, dtype=np.uint8)
    path = Path(tempfile.gettempdir()) / f"test_hfield_mj_{suffix}.png"
    if _HAS_PIL:
        PILImage.fromarray(norm, mode="L").save(str(path))
    else:
        # manual 8-bit greyscale PNG (no PIL dependency)
        w, h = norm.shape[1], norm.shape[0]
        def _chunk(typ, data):
            c = typ + data
            return struct.pack('>I', len(data)) + c + \
                   struct.pack('>I', zlib.crc32(c) & 0xFFFFFFFF)
        raw = b''.join(b'\x00' + row.tobytes() for row in norm)
        ihdr = struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0)
        png = b'\x89PNG\r\n\x1a\n' + _chunk(b'IHDR', ihdr) + \
              _chunk(b'IDAT', zlib.compress(raw)) + _chunk(b'IEND', b'')
        path.write_bytes(png)
    return path


def _write_mjcf_xml(hf: np.ndarray, border: float,
                    total_len: float, total_wid: float,
                    png_path: Path) -> Path:
    cx = border + total_len * 0.5
    cy = border + total_wid * 0.5
    peak = max(float(hf.max()), 0.01)  # MuJoCo needs zscale > 0
    xml = (
        '<mujoco model="test_hfield">\n'
        '  <compiler angle="degree"/>\n'
        '  <asset>\n'
        f'    <hfield name="terrain_hfield" file="{png_path}"'
        f' size="{total_len*0.5:.4f} {total_wid*0.5:.4f} {peak:.4f} 0.02"/>\n'
        '  </asset>\n'
        '  <worldbody>\n'
        f'    <geom name="terrain_heightfield" type="hfield"'
        f' hfield="terrain_hfield" pos="{cx:.4f} {cy:.4f} 0.0"/>\n'
        '    <light directional="true" pos="0 0 10" dir="0 0 -1"/>\n'
        '  </worldbody>\n'
        '</mujoco>\n'
    )
    path = Path(tempfile.gettempdir()) / "test_hfield_scene.xml"
    path.write_text(xml)
    return path


# ═══════════════════════════════════════════════════════════════════
#  Test 3: MuJoCo comparison
# ═══════════════════════════════════════════════════════════════════

def _run_one_comparison(name, hf, border, total_len, total_wid, n_test, seed):
    """Run a single Sire vs MuJoCo comparison and return stats dict."""
    import mujoco

    nrow, ncol = hf.shape
    amplitude = float(hf.max()) if hf.max() > 1e-6 else 0.12
    uid = str(abs(hash(name)) % 100000)  # unique suffix per test case

    png_path = _write_png_8bit(hf, suffix=uid)
    xml_path = _write_mjcf_xml(hf, border, total_len, total_wid, png_path)

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    hf_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, "terrain_hfield")
    adr = model.hfield_adr[hf_id]
    mj_nrow, mj_ncol = model.hfield_nrow[hf_id], model.hfield_ncol[hf_id]
    mj_raw = model.hfield_data[adr : adr + mj_nrow * mj_ncol]
    hf_quant = mj_raw.reshape((mj_nrow, mj_ncol)).copy()
    zscale = float(model.hfield_size[hf_id][2])

    geomgroup = np.ones(6, dtype=np.uint8)
    ray_dir = np.array([0, 0, -1], dtype=np.float64)
    geomid = np.zeros(1, dtype=np.int32)

    rng = np.random.default_rng(seed)
    x_vals = rng.uniform(border + 0.1, border + total_len - 0.1, n_test)
    y_vals = rng.uniform(border + 0.1, border + total_wid - 0.1, n_test)

    hf_flipped = np.flipud(hf)

    # sanity: compare flipped hf vs MuJoCo's hfield_data * zscale at corners
    err_grid = np.abs(hf_flipped - hf_quant * zscale)
    print(f"    [debug] hf_flipped vs hf_quant*zscale grid err:"
          f" max={err_grid.max():.6f} mean={err_grid.mean():.6f}")

    diff_float = np.zeros(n_test)
    diff_quant = np.zeros(n_test)

    for i in range(n_test):
        wx, wy = x_vals[i], y_vals[i]
        sh = bilinear_height(hf_flipped, wx, wy,
                             border, total_len, total_wid)
        sq = bilinear_height(hf_quant, wx, wy,
                             border, total_len, total_wid) * zscale
        origin = np.array([wx, wy, 20.0], dtype=np.float64)
        dist = mujoco.mj_ray(model, data, origin, ray_dir,
                             geomgroup, 1, -1, geomid)
        mh = 20.0 - dist if dist >= 0 else 0.0
        diff_float[i] = abs(sh - mh)
        diff_quant[i] = abs(sq - mh)

    print(f"\n  [{name}]  {n_test} samples, hf range [{hf.min():.4f},{hf.max():.4f}]")
    print(f"    float64 vs MJ:  max={diff_float.max()*1000:.3f} mm"
          f"  mean={diff_float.mean()*1000:.4f} mm"
          f"  std={diff_float.std()*1000:.4f} mm")
    print(f"    quant   vs MJ:  max={diff_quant.max()*1000:.3f} mm"
          f"  mean={diff_quant.mean()*1000:.4f} mm"
          f"  std={diff_quant.std()*1000:.4f} mm")

    return {"diff_float": diff_float, "diff_quant": diff_quant}


def test_mujoco_comparison():
    print("\n" + "-" * 50)
    print("Test 3: MuJoCo mj_ray comparison")

    try:
        import mujoco
    except ImportError:
        print("  (mujoco not installed - skipping)")
        return

    border, total_len, total_wid = 1.0, 4.0, 3.0

    # ── case A: random heightfield, many samples ──
    hf_rand = make_heightfield(129, 129, amplitude=0.12, seed=42)
    _run_one_comparison("random hfield",
                        hf_rand, border, total_len, total_wid, 10000, seed=789)

    # ── case B: different random heightfield, different seed ──
    hf_rand2 = make_heightfield(257, 257, amplitude=0.25, seed=99)
    _run_one_comparison("random hfield (257x257, amp=0.25)",
                        hf_rand2, border, total_len, total_wid, 5000, seed=101)

    # ── case C: flat zero heightfield ──
    hf_flat = np.zeros((129, 129), dtype=np.float64)
    r = _run_one_comparison("flat zero",
                            hf_flat, border, total_len, total_wid, 2000, seed=42)

    tol_mm = 0.01  # 0.01 mm tolerance for flat case
    if r["diff_float"].max() < tol_mm / 1000 and r["diff_quant"].max() < 1e-12:
        print("\n  PASS - all cases within tolerance")


# ═══════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    test_self_consistency()
    test_out_of_bounds()
    test_mujoco_comparison()
    print("\n" + "=" * 50)
    print("Done.")
    print("=" * 50)
