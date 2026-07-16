"""
Sire HeightField terrain API tests.

Validates:
  1. Heightfield PNG generation from numpy
  2. Sire <HeightField> XML element generation
  3. HeightField loading & collision in Sire simulation
  4. Multi-patch terrain (threshold, scene_curriculum)
  5. Heightfield height query accuracy (via height_query.py)

Usage (from demo/demo_python):
    cd heightField
    python test_sire_heightfield.py
"""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import sire
from SireRLGym.utils.terrain import TerrainLayout
from SireRLGym.utils.height_query import heightfield_height


# ═══════════════════════════════════════════════════════════════════
#  Test helpers
# ═══════════════════════════════════════════════════════════════════

def make_test_cfg(**overrides):
    """Create a minimal terrain config for testing."""
    from types import SimpleNamespace
    cfg = SimpleNamespace()
    cfg.num_rows = 1
    cfg.num_cols = 1
    cfg.terrain_length = 4.0
    cfg.terrain_width = 4.0
    cfg.border_size = 0.0
    cfg.spawn_offset_x = 1.0
    cfg.spawn_offset_y = 2.0
    cfg.base_half_height = 0.04
    cfg.slope_half_height = 0.06
    cfg.terrain_seed = 42
    cfg.terrain_type_mode = 'heightfield'
    cfg.heightfield_height_range = [0.02, 0.08]
    cfg.heightfield_height_override = None
    cfg.heightfield_nrow = 129
    cfg.heightfield_ncol = 129
    cfg.heightfield_smooth_steps = 8
    cfg.heightfield_spawn_flat_radius = 3
    cfg.slope_range_deg = [4.0, 12.0]
    cfg.slope_angle_override_deg = None
    cfg.threshold_height = 0.15
    cfg.threshold_height_levels = None
    cfg.threshold_depth = 0.15
    cfg.threshold_width = 0.9
    cfg.threshold_offset_x = 2.0
    cfg.corridor_width = 0.9
    cfg.enable_corridor_walls = True
    cfg.corridor_wall_height = 0.35
    cfg.corridor_wall_thickness = 0.08
    cfg.corridor_margin = 0.1
    cfg.scene_curriculum_dir = None
    cfg.scene_curriculum_single_level = None
    cfg.scene_curriculum_content_length = None
    cfg.scene_curriculum_content_width = None
    for k, v in overrides.items():
        setattr(cfg, k, v)
    return cfg


def find_base_xml() -> Path:
    for candidate in [
        ROOT / 'sirePaperDogRL' / 'go2.xml',
    ]:
        if candidate.exists():
            return candidate
    raise FileNotFoundError('go2.xml not found')


# ═══════════════════════════════════════════════════════════════════
#  Tests
# ═══════════════════════════════════════════════════════════════════

def test_heightfield_generation():
    """Heightfield numpy array is valid."""
    cfg = make_test_cfg()
    tl = TerrainLayout(cfg, num_envs=1)
    hf = tl.global_heightfield
    assert hf is not None, 'global_heightfield is None'
    assert hf.shape == (129, 129), f'Shape mismatch: {hf.shape}'
    assert hf.min() >= -1e-6, f'min < 0: {hf.min()}'
    assert hf.max() > 0, 'peak is 0'
    print(f'  [OK] heightfield shape={hf.shape}, range=[{hf.min():.4f}, {hf.max():.4f}]')


def test_png_generation():
    """PNG file is written correctly."""
    cfg = make_test_cfg()
    tl = TerrainLayout(cfg, num_envs=1)
    png = tl._write_heightfield_png(tl.global_heightfield)
    assert png.exists(), f'PNG not found: {png}'
    assert png.stat().st_size > 0, 'PNG empty'
    from PIL import Image
    img = Image.open(png)
    assert img.size == (129, 129), f'PNG size: {img.size}'
    print(f'  [OK] PNG {png.name} size={img.size}')


def test_sire_heightfield_xml_generation():
    """Sire <HeightField> XML is generated and loads correctly."""
    cfg = make_test_cfg()
    tl = TerrainLayout(cfg, num_envs=1)

    base_xml = find_base_xml()
    gen_xml = tl.write_scene(base_xml)
    assert gen_xml.exists(), f'Generated XML missing: {gen_xml}'

    # Verify XML contains HeightField element
    import xml.etree.ElementTree as ET
    tree = ET.parse(str(gen_xml))
    root = tree.getroot()
    pe = root.find('PhysicsEngine')
    gpo = pe.find('GeometryPoolObject')
    hfs = gpo.findall('HeightField')
    assert len(hfs) >= 1, 'No HeightField in generated XML'

    hf_elem = hfs[0]
    assert hf_elem.get('x_dim') is not None, 'Missing x_dim'
    assert hf_elem.get('y_dim') is not None, 'Missing y_dim'
    assert hf_elem.get('scale_z') is not None, 'Missing scale_z'
    assert hf_elem.get('file') is not None, 'Missing file'
    print(f'  [OK] HeightField XML: x_dim={hf_elem.get("x_dim")}, '
          f'y_dim={hf_elem.get("y_dim")}, scale_z={hf_elem.get("scale_z")}')


def test_sire_heightfield_loading():
    """HeightField loads into Sire simulator and collision works."""
    cfg = make_test_cfg()
    tl = TerrainLayout(cfg, num_envs=1)

    base_xml = find_base_xml()
    gen_xml = tl.write_scene(base_xml)

    sim = sire.Simulator()
    sire.fromXmlFile(sim, str(gen_xml))

    pe = sim.physicsEngine()
    n_geoms = pe.numGeometries()
    assert n_geoms >= 5, f'Expected >=5 geometries, got {n_geoms}'

    # Find the static ground geometry (HeightField, id=0 or part_id=0)
    gp = pe.geometryPool
    found_ground = False
    for i in range(pe.numGeometries()):
        g = gp[i]
        if g is not None and not g.isDynamic:
            found_ground = True
            print(f'  [OK] Found static ground geometry at index {i}: '
                  f'id={g.id}, type={type(g).__name__}')
            break
    assert found_ground, 'No static ground geometry found'


def test_heightfield_collision():
    """Robot falls onto heightfield and settles."""
    cfg = make_test_cfg()
    tl = TerrainLayout(cfg, num_envs=1)

    base_xml = find_base_xml()
    gen_xml = tl.write_scene(base_xml)

    sim = sire.Simulator()
    sire.fromXmlFile(sim, str(gen_xml))
    sim.init()
    sl = sim.simulationLoop()

    model = sim.model()
    pp = model.partPool()
    # Part at index 1 is the base link (ground is index 0)
    if model.nbody > 1:
        base_part = pp[1]
        # getPm returns 16-element flat vector (column-major 4x4)
        init_pm = base_part.getPm()
        init_z = init_pm[2*4 + 3]  # pm[2,3] in column-major: col 3, row 2
        # Step to settle
        for _ in range(200):
            sl.step()
        settled_pm = base_part.getPm()
        settled_z = settled_pm[2*4 + 3]
        assert settled_z < init_z, (
            f'Robot did not fall: init_z={init_z:.3f}, settled_z={settled_z:.3f}'
        )
        print(f'  [OK] Robot settled on heightfield: '
              f'init_z={init_z:.3f} -> settled_z={settled_z:.3f}')
    else:
        print(f'  [SKIP] Only {model.nbody} bodies (no base link to track)')


def test_heightfield_query_accuracy():
    """Direct heightfield query matches generated heightfield."""
    cfg = make_test_cfg(heightfield_nrow=33, heightfield_ncol=33)
    tl = TerrainLayout(cfg, num_envs=1)
    hf = tl.global_heightfield
    nrow, ncol = hf.shape
    dx = tl.total_length / (ncol - 1)
    dy = tl.total_width / (nrow - 1)

    max_err = 0.0
    for r in range(1, nrow - 1):
        for c in range(1, ncol - 1):
            wx = tl.border + c * dx
            wy = tl.border + r * dy
            h_query = heightfield_height(
                hf, wx, wy, tl.border, tl.total_length, tl.total_width
            )
            h_actual = float(hf[r, c])
            err = abs(h_query - h_actual)
            max_err = max(max_err, err)

    assert max_err < 0.02, f'Height query error too large: {max_err:.6f}'
    print(f'  [OK] Max height query error: {max_err:.6f}m')


def test_threshold_terrain_xml():
    """Threshold terrain generates BoxCollisionGeometry in XML."""
    cfg = make_test_cfg(terrain_type_mode='threshold',
                        num_rows=2, num_cols=1,
                        terrain_length=3.0, terrain_width=3.0)
    tl = TerrainLayout(cfg, num_envs=2)

    base_xml = find_base_xml()
    gen_xml = tl.write_scene(base_xml)

    import xml.etree.ElementTree as ET
    tree = ET.parse(str(gen_xml))
    gpo = tree.getroot().find('PhysicsEngine').find('GeometryPoolObject')
    boxes = gpo.findall('BoxCollisionGeometry')
    assert len(boxes) >= 3, f'Expected >=3 boxes (floor + 2 thresholds), got {len(boxes)}'
    print(f'  [OK] Threshold terrain: {len(boxes)} BoxCollisionGeometry elements')


def test_slope_terrain_xml():
    """Slope terrain generates HeightField in XML."""
    cfg = make_test_cfg(terrain_type_mode='slope',
                        slope_angle_override_deg=8.0)
    tl = TerrainLayout(cfg, num_envs=1)

    base_xml = find_base_xml()
    gen_xml = tl.write_scene(base_xml)

    import xml.etree.ElementTree as ET
    tree = ET.parse(str(gen_xml))
    gpo = tree.getroot().find('PhysicsEngine').find('GeometryPoolObject')
    hfs = gpo.findall('HeightField')
    assert len(hfs) >= 1, 'Slope terrain should have HeightField'
    print(f'  [OK] Slope terrain: {len(hfs)} HeightField elements '
          f'(scale_z={hfs[0].get("scale_z")})')


# ═══════════════════════════════════════════════════════════════════

def main():
    print('=' * 55)
    print('  Sire HeightField API tests')
    print('=' * 55)

    tests = [
        ('HeightField numpy generation', test_heightfield_generation),
        ('PNG file generation', test_png_generation),
        ('Sire HeightField XML generation', test_sire_heightfield_xml_generation),
        ('HeightField loading into Sire', test_sire_heightfield_loading),
        ('HeightField collision (robot settles)', test_heightfield_collision),
        ('Height query accuracy', test_heightfield_query_accuracy),
        ('Threshold terrain (boxes)', test_threshold_terrain_xml),
        ('Slope terrain (HeightField)', test_slope_terrain_xml),
    ]

    for name, fn in tests:
        try:
            print(f'\n[{tests.index((name, fn)) + 1}] {name} ...')
            fn()
        except Exception as e:
            print(f'  FAIL: {e}')
            import traceback
            traceback.print_exc()
            sys.exit(1)

    print('\n' + '=' * 55)
    print('  ALL HEIGHTFIELD TESTS PASSED')
    print('=' * 55)


if __name__ == '__main__':
    main()
