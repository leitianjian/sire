import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, pi

def jointTrajectory(time, TotalTime, start_q, target_q):
    # if time > TotalTime + 1e-1:
    #     return target_q
    size = len(target_q)
    current_q = np.zeros(3 * size)
    for i in range(size):
        current_q[3 * i] = (target_q[i] - start_q[i]) * (sin(pi * time / TotalTime - pi / 2.0) + 1) / 2.0 + start_q[i]
        current_q[3 * i + 1] = (target_q[i] - start_q[i]) * (pi / TotalTime) * cos(pi * time / TotalTime - pi / 2.0) / 2.0
        current_q[3 * i + 2] = - (target_q[i] - start_q[i]) * (pi / TotalTime) * (pi / TotalTime) * sin(pi * time / TotalTime - pi / 2.0) / 2.0
    return current_q

import math
# 参数设置
TotalTime = 6.0  # 总时间
# disable motor of bb joints
state_2_init = [0, 0, 0, 0, 0, 0, 0]
# enable motor of bb joints
state_1_init = [math.pi, math.pi, 0, 0, 0, 0, 0]
# enable motor of bs joint
state_3_init = [0, 0, 2.92, 2.92, -1.6, -1.6, -1.6]
# enable motor of ss joint
state_4_init = [math.pi, math.pi, math.pi * 2 / 3.0, math.pi * 2 / 3.0, -math.pi / 3.0, -math.pi / 3.0, - math.pi / 3.0]

trajectory_path = [state_3_init, state_2_init, state_1_init, state_4_init, state_1_init, state_2_init]

num_points = 6000  # 时间点数

# 创建时间数组
time_points = np.linspace(0, TotalTime, num_points)

# 存储结果
positions = []
velocities = []
accelerations = []

# 计算轨迹
for sim_time in time_points:
  if sim_time <= 1:
    currentJoint = jointTrajectory(sim_time, 1, state_2_init, trajectory_path[0])
  elif sim_time > 1 and sim_time <= 2:
    currentJoint = jointTrajectory(sim_time - 1, 1, trajectory_path[0], trajectory_path[1])
  elif sim_time > 2 and sim_time <= 3:
    currentJoint = jointTrajectory(sim_time - 2, 1, trajectory_path[1], trajectory_path[2])
  elif sim_time > 3 and sim_time <= 4: 
    currentJoint = jointTrajectory(sim_time - 3, 1, trajectory_path[2], trajectory_path[3])
  elif sim_time > 4 and sim_time <= 5: 
    currentJoint = jointTrajectory(sim_time - 4, 1, trajectory_path[3], trajectory_path[4])
  elif sim_time > 5 and sim_time <= 6: 
    currentJoint = jointTrajectory(sim_time - 5, 1, trajectory_path[4], trajectory_path[5])
  positions.append(currentJoint[0])
  velocities.append(currentJoint[1])
  accelerations.append(currentJoint[2])

# 创建图表
plt.figure(figsize=(12, 8))

# 位置图
plt.subplot(3, 1, 1)
plt.plot(time_points, positions, 'b-', linewidth=2)
plt.axvline(x=TotalTime, color='r', linestyle='--', alpha=0.5)
plt.title('Joint Position vs Time')
plt.ylabel('Position (rad)')
plt.grid(True)

# 速度图
plt.subplot(3, 1, 2)
plt.plot(time_points, velocities, 'g-', linewidth=2)
plt.axvline(x=TotalTime, color='r', linestyle='--', alpha=0.5)
plt.title('Joint Velocity vs Time')
plt.ylabel('Velocity (rad/s)')
plt.grid(True)

# 加速度图
plt.subplot(3, 1, 3)
plt.plot(time_points, accelerations, 'r-', linewidth=2)
plt.axvline(x=TotalTime, color='r', linestyle='--', alpha=0.5)
plt.title('Joint Acceleration vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (rad/s²)')
plt.grid(True)

# 添加总标题
plt.suptitle('Joint Trajectory Profile', fontsize=16)

# 调整布局
plt.tight_layout()
plt.subplots_adjust(top=0.92)

# 显示图表
plt.show()