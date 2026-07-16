"""
MuJoCo Simulation: Box Sliding on 15° Inclined Plane
=====================================================
A 1×1×1m box (with 4 corner spheres) slides on a 15° incline.
Initial velocity: 2 m/s along the incline.
Friction coefficient: μ = 0.3.

Run: python mujocoBoxSliding.py
Requires: pip install mujoco matplotlib numpy
"""

import mujoco
import numpy as np
import math
import matplotlib.pyplot as plt
import mujoco.viewer

# ======================== Parameters ========================
angle_deg = 15
angle_rad = math.radians(angle_deg)
g = 9.81
mu = 0.3        # friction coefficient
v0 = 2.0        # initial velocity (m/s) along the incline
dt = 0.001     # simulation timestep (s)

# ======================== Analytical Solution ========================
# Deceleration: a = g*sin(θ) + μ*g*cos(θ)  (gravity + friction both oppose uphill motion)
a_decel = g * math.sin(angle_rad) + mu * g * math.cos(angle_rad)
t_stop = v0 / a_decel
x_stop = v0 * t_stop - 0.5 * a_decel * t_stop**2

print(f"{'='*50}")
print(f"  Box Sliding on {angle_deg}° Incline — MuJoCo Simulation")
print(f"{'='*50}")
print(f"  Gravity:          {g} m/s²")
print(f"  Friction (μ):     {mu}")
print(f"  Initial velocity: {v0} m/s (uphill)")
print(f"  Deceleration:     {a_decel:.4f} m/s²")
print(f"  Analytical stop:  t = {t_stop:.4f} s,  x = {x_stop:.4f} m")
print(f"{'='*50}")

# ======================== Geometry Calculation ========================
# Ramp: box half-extent (3, 3, 0.5), center at z=0.5, rotated -15° about X.
#   Top surface at y=0: z_ramp_top = 0.5 + 0.5*cos(15°) ≈ 0.983
# Cube: half-extent 0.5, also rotated -15° about X.
#   Bottom offset from center = 0.5*cos(15°) ≈ 0.483
#   Cube center z = ramp_top + bottom_offset + tiny_gap ≈ 1.4665
ramp_z = 0.5
cube_z = ramp_z + 1 / math.cos(angle_rad)    # +2mm gap to avoid initial bounce
print(f"  Ramp center z:    {ramp_z:.4f} m")
print(f"  Cube center z:    {cube_z:.4f} m")
print(f"{'='*50}")

# ======================== Build MuJoCo Model ========================
# NOTE: MuJoCo XML euler angles are in DEGREES, not radians!
xml = f"""
<mujoco model="box_sliding">
  <option timestep="{dt}" gravity="0 0 -9.81">
    <flag contact="enable"/>
  </option>
  <worldbody>
    <light pos="0 5 10"/>
    <camera pos="3 -3 3" xyaxes="0.707 0.707 0 -0.408 0.408 0.816"/>

    <!-- Ground reference plane -->
    <geom type="plane" size="10 10 0.1" rgba="0.3 0.3 0.3 1"/>

    <!-- Inclined ramp: box half-extent [3, 3, 0.5], rotated -15° about X (matching sire) -->
    <body pos="0 0 {ramp_z}" euler="-{angle_deg} 0 0">
      <geom type="box" size="3 3 0.5" rgba="0.5 0.5 0.5 1" friction="{mu} 0.01 0.001"/>
    </body>

    <!-- Sliding box: 1×1×1 cube, rotated -15° about X -->
    <body pos="0 0 {cube_z}" euler="-{angle_deg} 0 0">
      <freejoint/>
      <geom type="box" size="0.5 0.5 0.5" rgba="0.2 0.4 0.8 1" mass="1"
            friction="{mu} 0.01 0.001"/>
    </body>
  </worldbody>
</mujoco>
"""

# Load model
mj_model = mujoco.MjModel.from_xml_string(xml)
mj_data = mujoco.MjData(mj_model)

# ======================== Initial State ========================
# Box starts at REST on the incline (velocity set to 0 initially).
# The viewer will let you inspect the scene, then apply velocity on ENTER.
print(f"  Initial velocity:  0 (box at rest, press ENTER in viewer to start)")
print(f"{'='*50}")

# ======================== Visual Check (Interactive Viewer) ========================
# Box starts at REST on the ramp. Press ENTER to apply uphill velocity.
print("\n>>> Launching MuJoCo viewer...")
print("    Box is at REST on the incline — inspect the scene first.")
print("    [Right-drag] rotate  |  [Scroll] zoom  |  [Middle-drag] pan")
print("    Press ENTER in this console to START sliding.\n")

with mujoco.viewer.launch_passive(mj_model, mj_data, show_left_ui=True, show_right_ui=True) as viewer:
    import time

    # Phase 1: render static scene WITHOUT stepping (no gravity creep)
    viewer.sync()
    input(">>> Press ENTER to apply initial velocity (2 m/s uphill) and start...")

    # Apply initial velocity (uphill along incline)
    mj_data.qvel[0] = 0
    mj_data.qvel[1] = -v0 * math.cos(angle_rad)   # -y = uphill
    mj_data.qvel[2] = v0 * math.sin(angle_rad)     # +z = uphill
    print("    Box sliding! Close viewer window when done.\n")

    sim_time_prev = mj_data.time
    while viewer.is_running():
        mujoco.mj_step(mj_model, mj_data)
        viewer.sync()
        # Slow down playback: ~200 fps real-time display (5ms per frame)
        time.sleep(0.005)

print("Viewer closed. Collecting data...\n")

# ======================== Full Simulation (Data Collection) ========================
# Reset
mj_data = mujoco.MjData(mj_model)
mj_data.qvel[0] = 0
mj_data.qvel[1] = -v0 * math.cos(angle_rad)
mj_data.qvel[2] = v0 * math.sin(angle_rad)

sim_duration = 1
dt = mj_model.opt.timestep
n_steps = int(sim_duration / dt)

# Incline direction (downhill unit vector in world frame)
incline_dir = np.array([0, -math.cos(angle_rad), math.sin(angle_rad)])

t_hist = np.zeros(n_steps)
disp_hist = np.zeros(n_steps)   # displacement along incline
vel_hist = np.zeros(n_steps)    # velocity along incline
init_pos = mj_data.qpos[0:3].copy()

for step in range(n_steps):
    t_hist[step] = mj_data.time
    delta = mj_data.qpos[0:3] - init_pos
    disp_hist[step] = np.dot(delta, incline_dir)
    vel_hist[step] = np.dot(mj_data.qvel[0:3], incline_dir)
    mujoco.mj_step(mj_model, mj_data)

print(f"Data collected: {n_steps} steps, dt = {dt:.6f}s")

# ======================== Analytical Curves ========================
t_ana = np.linspace(0, sim_duration, 10000)
disp_ana = np.piecewise(t_ana,
                        [t_ana <= t_stop, t_ana > t_stop],
                        [lambda t: v0*t - 0.5*a_decel*t**2,
                         lambda t: x_stop])
vel_ana = np.piecewise(t_ana,
                       [t_ana < t_stop, t_ana >= t_stop],
                       [lambda t: v0 - a_decel*t,
                        lambda t: 0])

# ======================== Plot: MuJoCo vs Analytical ========================
fig, axes = plt.subplots(2, 1, figsize=(8, 7), sharex=True)

# Displacement
axes[0].plot(t_ana, disp_ana, 'r-', linewidth=1.5, label='Analytical')
axes[0].plot(t_hist, disp_hist, 'b--', linewidth=1.0, label='MuJoCo')
axes[0].set_ylabel("Displacement (m)")
axes[0].legend()
axes[0].set_title(f"Box Sliding on {angle_deg}° Incline: MuJoCo vs Analytical")
axes[0].grid(True, alpha=0.3)

# Velocity
axes[1].plot(t_ana, vel_ana, 'r-', linewidth=1.5, label='Analytical')
axes[1].plot(t_hist, vel_hist, 'b--', linewidth=1.0, label='MuJoCo')
axes[1].set_xlabel("Time (s)")
axes[1].set_ylabel("Velocity (m/s)")
axes[1].legend()
axes[1].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# ======================== Quantitative Comparison ========================

# Interpolate analytical solution onto MuJoCo time points for pointwise comparison
disp_ana_at_mj = np.interp(t_hist, t_ana, disp_ana)
vel_ana_at_mj = np.interp(t_hist, t_ana, vel_ana)

# --- Full trajectory errors ---
disp_error = disp_hist - disp_ana_at_mj
vel_error = vel_hist - vel_ana_at_mj

disp_rmse_full = np.sqrt(np.mean(disp_error**2))
vel_rmse_full = np.sqrt(np.mean(vel_error**2))
disp_mae_full = np.mean(np.abs(disp_error))
vel_mae_full = np.mean(np.abs(vel_error))
disp_max_err = np.max(np.abs(disp_error))
vel_max_err = np.max(np.abs(vel_error))

# R² scores
ss_res_disp = np.sum(disp_error**2)
ss_tot_disp = np.sum((disp_hist - np.mean(disp_hist))**2)
r2_disp = 1 - ss_res_disp / ss_tot_disp if ss_tot_disp > 1e-15 else 1.0

ss_res_vel = np.sum(vel_error**2)
ss_tot_vel = np.sum((vel_hist - np.mean(vel_hist))**2)
r2_vel = 1 - ss_res_vel / ss_tot_vel if ss_tot_vel > 1e-15 else 1.0

# --- Stable phase errors ---
stable_mask = t_hist >= t_stop
mj_stable_disp = disp_hist[stable_mask]
mj_stable_vel = vel_hist[stable_mask]
mj_disp_mean = np.mean(mj_stable_disp)
mj_disp_std = np.std(mj_stable_disp)
mj_vel_std = np.std(mj_stable_vel)
mj_disp_abs_diff = abs(mj_disp_mean - x_stop)
mj_disp_rmse = np.sqrt(np.mean((mj_stable_disp - x_stop)**2))
mj_vel_rmse = np.sqrt(np.mean(mj_stable_vel**2))

# --- Print results ---
print(f"\n{'='*62}")
print(f"  Accuracy Metrics")
print(f"{'='*62}")

traj_label = f"— Full Trajectory (0 — {sim_duration:.1f}s) —"
print(f"\n  {traj_label:>62}")
print(f"  {'':>30} {'Displacement':>15} {'Velocity':>15}")
print(f"  {'-'*62}")
print(f"  {'RMSE:':<30} {disp_rmse_full:>15.6f} {vel_rmse_full:>15.6f}")
print(f"  {'MAE:':<30} {disp_mae_full:>15.6f} {vel_mae_full:>15.6f}")
print(f"  {'Max absolute error:':<30} {disp_max_err:>15.6f} {vel_max_err:>15.6f}")
print(f"  {'R² score:':<30} {r2_disp:>15.6f} {r2_vel:>15.6f}")

stable_label = f"— Stable Phase (t >= {t_stop:.4f}s) —"
print(f"\n  {stable_label:>62}")
print(f"  {'Analytical final displacement:':<30} {x_stop:>15.6f} m")
print(f"  {'MuJoCo mean displacement:':<30} {mj_disp_mean:>15.6f} m")
print(f"  {'MuJoCo displacement std:':<30} {mj_disp_std:>15.6f} m")
print(f"  {'MuJoCo velocity std:':<30} {mj_vel_std:>15.6f} m/s")
print(f"  {'Displacement abs diff:':<30} {mj_disp_abs_diff:>15.6f} m")
print(f"  {'Displacement rel diff:':<30} {mj_disp_abs_diff/x_stop*100:>15.4f} %")
print(f"  {'Displacement RMSE:':<30} {mj_disp_rmse:>15.6f} m")
print(f"  {'Velocity RMSE:':<30} {mj_vel_rmse:>15.6f} m/s")
print(f"{'='*62}")

# --- Error-over-time plots (left: signed, right: absolute) ---
fig, axes = plt.subplots(2, 2, figsize=(12, 5), sharex=True)

# Signed displacement error
axes[0, 0].plot(t_hist, disp_error * 1000, 'b-', linewidth=0.8)
axes[0, 0].axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
axes[0, 0].axvline(x=t_stop, color='r', linestyle=':', linewidth=1.0, label=f't_stop={t_stop:.3f}s')
axes[0, 0].set_ylabel("Δ Disp (mm)")
axes[0, 0].set_title("Signed Error: MuJoCo − Analytical  (<0 = shorter slide)")
axes[0, 0].legend(fontsize=7)
axes[0, 0].grid(True, alpha=0.3)

# Absolute displacement error
axes[0, 1].plot(t_hist, np.abs(disp_error) * 1000, 'b-', linewidth=0.8)
axes[0, 1].axvline(x=t_stop, color='r', linestyle=':', linewidth=1.0, label=f't_stop={t_stop:.3f}s')
axes[0, 1].set_ylabel("|Δ Disp| (mm)")
axes[0, 1].set_title("Absolute Error: |MuJoCo − Analytical|")
axes[0, 1].legend(fontsize=7)
axes[0, 1].grid(True, alpha=0.3)

# Signed velocity error
axes[1, 0].plot(t_hist, vel_error, 'r-', linewidth=0.8)
axes[1, 0].axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
axes[1, 0].axvline(x=t_stop, color='b', linestyle=':', linewidth=1.0, label=f't_stop={t_stop:.3f}s')
axes[1, 0].set_xlabel("Time (s)")
axes[1, 0].set_ylabel("Δ Vel (m/s)")
axes[1, 0].legend(fontsize=7)
axes[1, 0].grid(True, alpha=0.3)

# Absolute velocity error
axes[1, 1].plot(t_hist, np.abs(vel_error), 'r-', linewidth=0.8)
axes[1, 1].axvline(x=t_stop, color='b', linestyle=':', linewidth=1.0, label=f't_stop={t_stop:.3f}s')
axes[1, 1].set_xlabel("Time (s)")
axes[1, 1].set_ylabel("|Δ Vel| (m/s)")
axes[1, 1].legend(fontsize=7)
axes[1, 1].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

plt.tight_layout()
plt.show()