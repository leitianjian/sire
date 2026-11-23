import mujoco
import numpy as np
import matplotlib.pyplot as plt
import mujoco.viewer
import time
# 指定支持扩展字符的字体
plt.rcParams['font.sans-serif'] = 'Microsoft YaHei'  # 或 'Source Han Sans', 'BabelStone Han'

    
    # <!-- 墙壁 -->
    # <geom name="wall1" type="box" size="0.1 10 1" pos="-10 0 1" rgba="0.8 0.8 0.9 1"/>
    # <geom name="wall2" type="box" size="0.1 10 1" pos="10 0 1" rgba="0.8 0.8 0.9 1"/>
    # <geom name="wall3" type="box" size="10 0.1 1" pos="0 -10 1" rgba="0.8 0.8 0.9 1"/>
    # <geom name="wall4" type="box" size="10 0.1 1" pos="0 10 1" rgba="0.8 0.8 0.9 1"/>
# MuJoCo模型XML字符串
MUJOCO_MODEL_XML = """
<mujoco>
  <option timestep="0.001"/>

  <default>
    <geom solref="-1000000 0"/>
  </default>
  
  <worldbody>
    <camera name="fixed" pos="0 -15 5" xyaxes="1 0 0 0 0.7 1" mode="fixed"/>
    <!-- 地面 -->
    <geom name="ground" type="plane" size="10 10 0.1" pos="0 0 0" rgba="0.8 0.9 0.8 1"/>
    
    <!-- 方块 -->
    <body name="ball" pos="0 0 10">
      <freejoint/>
      <geom name="box_geom" type="box" size="1 1 1" rgba="0.9 0.2 0.2 1" mass="1"/>
    </body>
  </worldbody>
  
  <!-- 重力设置 -->
  <option gravity="0 0 -9.81"/>
</mujoco>
"""

# 创建模型和仿真环境
model = mujoco.MjModel.from_xml_string(MUJOCO_MODEL_XML)
data = mujoco.MjData(model)

# 设置仿真时长
simulation_time = 30.0  # 仿真时间（秒）
num_steps = int(simulation_time / model.opt.timestep)

# 存储时间和高度数据
time_points = []
height_points = []
print(num_steps)
i = 0
with mujoco.viewer.launch_passive(model, data) as viewer:
  # Close the viewer automatically after simulation_duration wall-seconds.
        # 设置摄像头参数
  viewer.cam.distance = 15.0      # 摄像头距离
  viewer.cam.azimuth = 90.0       # 水平角度
  viewer.cam.elevation = -20.0    # 俯仰角度
  viewer.cam.lookat = np.array([0.0, 0.0, 5.0])  # 注视点
  start = time.time()
  while viewer.is_running() and data.time < simulation_time:

    mujoco.mj_step(model, data)
    
    # 记录时间和高度
    time_points.append(data.time)
    height_points.append(data.qpos[2])  # z坐标
    i += 1
    viewer.sync()    # 新的API
    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = data.time + model.opt.timestep - (time.time() - start)
    if time_until_next_step > 0:
        time.sleep(time_until_next_step)
# for i in range(num_steps):
#     # 执行仿真步骤
#     mujoco.mj_step(model, data)
    
#     # 记录时间和高度
#     time_points[i] = data.time
#     height_points[i] = data.qpos[2]  # z坐标
#     viewer.sync()    # 新的API
# 绘制高度-时间曲线
plt.figure(figsize=(10, 6))
plt.plot(time_points[:], height_points[:], 'b-', lw=2)
plt.xlabel('时间 (秒)', fontsize=12)
plt.ylabel('小球高度 (米)', fontsize=12)
plt.title('小球下落高度随时间变化', fontsize=14)
plt.grid(True, linestyle='--', alpha=0.7)
# plt.xlim(0, simulation_time)
# plt.ylim(0, 5.5)
plt.tight_layout()

# 标记关键点
# 找到第一次反弹的位置
# for i in range(1, num_steps):
#     if height_points[i] < height_points[i-1] and height_points[i] < 0.1:
#         plt.plot(time_points[i], height_points[i], 'ro', markersize=8)
#         plt.text(time_points[i]+0.1, height_points[i]+0.1, 
#                  f'第一次反弹: {time_points[i]:.2f}s', fontsize=10)
#         break

# 添加理论曲线对比
# 理论自由落体高度公式: h = h0 - 0.5*g*t^2
# g = 9.81
# h0 = 5
# theoretical_time = np.linspace(0, np.sqrt(2*h0/g), 100)
# theoretical_height = h0 - 0.5 * g * theoretical_time**2
# plt.plot(theoretical_time, theoretical_height, 'r--', lw=1.5, alpha=0.7, label='理论自由落体')

# # 添加图例
# plt.legend(['仿真结果', '理论自由落体'], loc='upper right')

# # 保存图表
# plt.savefig('ball_drop_height.png', dpi=300)
plt.show()

# # 保存仿真数据
# np.savez('ball_drop_data.npz', time=time_points, height=height_points)

print("仿真完成！")
print(f"仿真时长: {simulation_time}秒")
print(f"仿真步数: {num_steps}")
print(f"最终高度: {height_points[-1]:.4f}米")
print(f"数据已保存到 ball_drop_data.npz")
print(f"图表已保存为 ball_drop_height.png")