"""Quick standalone viewer for a reconstructed scene npz (full height, no curriculum clipping).

Usage:
    python3 view_scene_npz.py <path-to-npz> [--out-dir /tmp/scene_viz] [--no-viewer]
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image


def _parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("npz_path", type=Path)
    p.add_argument("--out-dir", type=Path, default=Path("/tmp/scene_viz"))
    p.add_argument("--no-viewer", action="store_true")
    return p.parse_args()


def main():
    args = _parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    d = np.load(args.npz_path, allow_pickle=True)
    hm = d["height_map"].astype(np.float32)
    vm = d["valid_mask"].astype(bool) if "valid_mask" in d.files else np.ones_like(hm, dtype=bool)
    resolution = float(d["resolution"])
    origin = d["origin_xy"]

    ny, nx = hm.shape
    half_x = nx * resolution / 2
    half_y = ny * resolution / 2
    center_x = float(origin[0]) + half_x
    center_y = float(origin[1]) + half_y

    h = np.where(vm, hm, 0.0)
    max_h = float(max(h.max(), 1e-6))
    img = np.clip(h / max_h, 0, 1)
    Image.fromarray((img * 255).astype(np.uint8), mode="L").save(args.out_dir / "scene.png")

    xml = f"""<mujoco model="scene_viz">
  <statistic center="{center_x:.4f} {center_y:.4f} 0.10" extent="2.0"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.2 0.2 0.2" specular="0.8 0.8 0.8"/>
    <rgba haze="0.12 0.14 0.18 1"/>
    <global azimuth="135" elevation="-28" offwidth="1600" offheight="1200"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.17 0.2 0.24" rgb2="0.02 0.03 0.05" width="512" height="3072"/>
    <texture type="2d" name="gp_tex" builtin="checker" mark="edge" rgb1="0.30 0.34 0.30" rgb2="0.20 0.22 0.20" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="gp_mat" texture="gp_tex" texuniform="true" texrepeat="8 8" reflectance="0.08"/>
    <material name="terrain_mat" rgba="0.70 0.74 0.62 1" reflectance="0.10"/>
    <hfield name="terrain_hf" file="scene.png" size="{half_x:.4f} {half_y:.4f} {max_h:.4f} 0.05"/>
  </asset>
  <worldbody>
    <light pos="1 0 4" dir="0 0 -1" directional="true"/>
    <geom name="groundplane" type="plane" material="gp_mat" pos="{center_x:.4f} {center_y:.4f} -0.001" size="4.5 4.5 0.1" friction="1.0 0.1 0.1"/>
    <geom name="terrain" type="hfield" hfield="terrain_hf" material="terrain_mat" pos="{center_x:.4f} {center_y:.4f} 0.0" friction="1.0 0.1 0.1"/>
  </worldbody>
</mujoco>
"""
    xml_path = args.out_dir / "scene.xml"
    xml_path.write_text(xml)

    print(f"npz       : {args.npz_path}")
    print(f"shape     : {hm.shape}  resolution={resolution}m")
    print(f"range     : x=[{origin[0]:.2f}, {origin[0]+2*half_x:.2f}]  y=[{origin[1]:.2f}, {origin[1]+2*half_y:.2f}]")
    print(f"max_h     : {max_h:.4f} m")
    print(f"valid     : {int(vm.sum())}/{vm.size} ({100*vm.mean():.1f}%)")
    print(f"wrote xml : {xml_path}")

    if not args.no_viewer:
        import mujoco
        import mujoco.viewer
        m = mujoco.MjModel.from_xml_path(str(xml_path))
        data = mujoco.MjData(m)
        print("launching viewer (Ctrl-C or close window to exit)...")
        mujoco.viewer.launch(m, data)


if __name__ == "__main__":
    main()
