"""
重新生成 heightfield 地形 PNG 和对应的 Sire XML（ball_drop  demo 专用）。

生成的文件放在当前目录，不会被系统自动清理。

用法：
    python regenerate_terrain.py

输出：
    - terrain_hf.png         灰度高度图 PNG
    - ball_drop_hf_generated.xml  对应的 Sire Simulator XML
"""
import os
import sys
import numpy as np
from pathlib import Path
from PIL import Image

# ─── 可调参数（与 ball_drop_hf.xml 匹配）───────────────────────────
TERRAIN_SIZE_X = 10.0       # 地形 X 方向尺寸 (m)
TERRAIN_SIZE_Y = 10.0       # 地形 Y 方向尺寸 (m)
AMPLITUDE     = 0.2         # 最大高度 (m)，对应 XML scale_z
MIN_HEIGHT    = -0.02       # 最低高度偏移 (m)，对应 XML min_height
HF_NROW       = 257         # 高度图分辨率（行）
HF_NCOL       = 257         # 高度图分辨率（列）
SMOOTH_STEPS  = 8           # 平滑迭代次数（越大越平滑）
SEED          = 1234        # 随机种子（固定可复现）

# 输出目录：当前脚本所在目录（不放在 Temp 下）
OUT_DIR   = Path(__file__).resolve().parent
PNG_PATH  = OUT_DIR / "terrain_hf.png"
XML_PATH  = OUT_DIR / "ball_drop_hf_generated.xml"


def generate_heightfield(amplitude: float, nrow: int, ncol: int,
                         smooth_steps: int, seed: int) -> np.ndarray:
    """用 Gaussian 噪声 + box blur 生成高度场（与 TerrainLayout 一致）。"""
    rng = np.random.default_rng(seed)
    heights = rng.normal(0.0, 1.0, size=(nrow, ncol)).astype(np.float32)

    for _ in range(max(1, smooth_steps)):
        heights = (
            heights
            + np.roll(heights, 1, axis=0)
            + np.roll(heights, -1, axis=0)
            + np.roll(heights, 1, axis=1)
            + np.roll(heights, -1, axis=1)
        ) / 5.0

    heights -= float(np.min(heights))
    peak = float(np.max(heights))
    if peak > 1e-6:
        heights /= peak
    heights *= float(amplitude)
    return heights


def write_heightfield_png(heights: np.ndarray, png_path: Path) -> None:
    """将高度场写入 8-bit 灰度 PNG（与 _write_heightfield_png 逻辑一致）。"""
    peak = float(np.max(heights))
    if peak <= 1e-6:
        normalized = np.zeros_like(heights, dtype=np.uint8)
    else:
        normalized = np.clip(np.round(255.0 * heights / peak), 0, 255).astype(np.uint8)

    Image.fromarray(normalized, mode='L').save(str(png_path))
    print(f"[OK] PNG 已保存: {png_path}")


def generate_ball_drop_xml(png_path: Path, xml_path: Path,
                           x_dim: float, y_dim: float,
                           scale_z: float, min_height: float) -> None:
    """生成包含 HeightField 的 Sire Simulator XML。"""

    # 球放在地形中心上方
    cx = x_dim * 0.5
    cy = y_dim * 0.5
    ball_z = scale_z + 0.3   # 起抛高度：最高地形上方 0.3 m

    xml = f"""<Simulator>
  <Model __prop_name__="model" name="BallDropHf" time="0">
    <Environment __prop_name__="environment" gravity="{{0,0,-9.81,0,0,0}}"/>
    <VariablePoolElement __prop_name__="variable_pool"/>
    <PartPoolElement __prop_name__="part_pool">
      <Part name="ground" active="true" pe="{{0,0,0,-0,0,-0}}" vel="{{0,0,0,0,0,0}}" acc="{{0,0,0,0,0,0}}" inertia="{{1,0,0,0,1,1,1,0,0,0}}">
        <MarkerPoolElement __prop_name__="marker_pool">
          <Marker name="joint_0_k" active="true" pe="{{0,0,0,-0,0,-0}}"/>
        </MarkerPoolElement>
        <GeometryPoolElement __prop_name__="geometry_pool"/>
      </Part>
      <Part name="ball" active="true" pe="{{{cx:.4f},{cy:.4f},{ball_z:.4f},0,0,0}}" vel="{{0,0,-0.5,0,0,0}}" acc="{{0,0,0,0,0,0}}" inertia="{{1,0,0,0,0.1,0.1,0.1,0,0,0}}">
        <MarkerPoolElement __prop_name__="marker_pool">
          <Marker name="sphere_center" active="true" pe="{{0,0,0,0,0,0}}"/>
        </MarkerPoolElement>
        <GeometryPoolElement __prop_name__="geometry_pool"/>
      </Part>
    </PartPoolElement>
    <MotionPoolElement __prop_name__="motion_pool"/>
    <JointPoolElement __prop_name__="joint_pool"/>
    <GeneralMotionPoolElement __prop_name__="general_motion_pool"/>
    <ForcePoolElement __prop_name__="force_pool"/>
    <SolverPoolElement __prop_name__="solver_pool">
      <InverseKinematicSolver max_iter_count="100" max_error="1e-10"/>
      <ForwardKinematicSolver max_iter_count="100" max_error="1e-10"/>
      <InverseDynamicSolver max_iter_count="100" max_error="1e-10"/>
      <ForwardDynamicSolver max_iter_count="100" max_error="1e-10"/>
    </SolverPoolElement>
    <CalibratorPoolElement __prop_name__="calibrator_pool"/>
  </Model>
  <SimulationLoop __prop_name__="simulator" dt="0.001" ctrlt="10" realtime_rate="-1" sim_duration="5" global_variable_pool="{{}}">
    <EventManager __prop_name__="event_manager">
      <EventHandlerPairPool __prop_name__="event_handler_pair_pool">
        <EventHandlerIdPair event_id="0" handler_id="12"/>
        <EventHandlerIdPair event_id="1" handler_id="13"/>
        <EventHandlerIdPair event_id="2" handler_id="14"/>
      </EventHandlerPairPool>
    </EventManager>
    <ZeroPosition __prop_name__="controller"/>
  </SimulationLoop>
  <PhysicsEngine __prop_name__="physics_engine" enable_collision_detection="true" enable_contact_solver="true">
    <CollisionDetection __prop_name__="collision_detection"/>
    <PsVsSolver3 __prop_name__="contact_solver" default_k="200000000" default_cr="0.2">
      <MaterialManager __prop_name__="material_manager" default_prop="{{k:140000000,d:1000,cr:1}}">
        <MaterialPairPropPool __prop_name__="material_pair_prop_pool">
          <MaterialPairProp first_name="m1" second_name="m1" prop="{{k:1e8,d:50000,cr:0.3,cof:0.6,threshold_velocity:0.3}}"/>
        </MaterialPairPropPool>
      </MaterialManager>
    </PsVsSolver3>
    <GeometryPoolObject __prop_name__="dynamic_geometry_pool">
      <SphereCollisionGeometry visible="true" id="0" part_id="1" is_dynamic="true" material="m1" contact_prop="{{k:2.8e8,d:2000}}" radius="0.05" pm="{{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}}"/>
      <HeightField visible="true" id="1" part_id="0" is_dynamic="false" material="m1" file="{png_path.as_posix()}" x_dim="{x_dim:.4f}" y_dim="{y_dim:.4f}" scale_z="{scale_z:.4f}" min_height="{min_height:.4f}" contact_prop="{{k:2.8e8,d:2000}}" pm="{{1,0,0,0,0,1,0,0,0,0,1,0.0000,0,0,0,1}}"/>
    </GeometryPoolObject>
    <CollisionFilter __prop_name__="collision_filter" filter_state="{{1,0, 0,1}}"/>
  </PhysicsEngine>
  <SimulatorModules __prop_name__="simulator_modules">
    <IntegratorPoolObject __prop_name__="integrator_pool">
      <SemiImplicitEulerIntegrator step_size="0.001" data_length="0"/>
    </IntegratorPoolObject>
    <SireSensorPoolObject __prop_name__="sensor_pool"/>
  </SimulatorModules>
</Simulator>
"""
    xml_path.write_text(xml, encoding='utf-8')
    print(f"[OK] XML 已保存: {xml_path}")


def main():
    print("=" * 60)
    print("  生成 ball_drop 高度场地形")
    print("=" * 60)
    print(f"  地形尺寸 : {TERRAIN_SIZE_X:.1f} x {TERRAIN_SIZE_Y:.1f} m")
    print(f"  高度范围 : [{MIN_HEIGHT:.3f}, {AMPLITUDE:.3f}] m")
    print(f"  分辨率   : {HF_NROW} x {HF_NCOL}")
    print(f"  平滑步数 : {SMOOTH_STEPS}")
    print(f"  随机种子 : {SEED}  (可复现)")
    print()

    # 1. 生成高度场
    heights = generate_heightfield(
        amplitude=AMPLITUDE,
        nrow=HF_NROW,
        ncol=HF_NCOL,
        smooth_steps=SMOOTH_STEPS,
        seed=SEED,
    )
    print(f"  实际峰值 : {np.max(heights):.4f} m")
    print(f"  实际谷值 : {np.min(heights):.4f} m")

    # 2. 写 PNG
    write_heightfield_png(heights, PNG_PATH)

    # 3. 写 XML
    generate_ball_drop_xml(
        png_path=PNG_PATH,
        xml_path=XML_PATH,
        x_dim=TERRAIN_SIZE_X,
        y_dim=TERRAIN_SIZE_Y,
        scale_z=AMPLITUDE,
        min_height=MIN_HEIGHT,
    )

    print()
    print("=" * 60)
    print("  完成！现在可以运行：")
    print(f"    python ball_drop_test.py  # 如果测试脚本已更新路径")
    print(f"  或直接加载生成的 XML：")
    print(f"    sire.fromXmlFile(sim, '{XML_PATH}')")
    print("=" * 60)


if __name__ == '__main__':
    main()
