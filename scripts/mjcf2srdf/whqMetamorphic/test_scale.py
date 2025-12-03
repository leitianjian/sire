import numpy as np
import meshcat
from meshcat.visualizer import Visualizer
import time

# 创建可视化器
vis = Visualizer().open()

# 创建几何体
box = meshcat.geometry.Box([1, 1, 1])
vis["scaling_object"].set_object(box)
vis["scaling_object"].set_property("color", [0, 1, 0, 1])  # 绿色

# 设置初始位置
initial_position = np.array([0, 0, 0])
T_initial = np.eye(4)
T_initial[:3, 3] = initial_position
vis["scaling_object"].set_transform(T_initial)

# 动画参数
duration = 10.0  # 总运行时间
start_time = time.time()

print("缩放动画运行中...按Ctrl+C停止")

try:
    while True:
        # 计算经过的时间
        elapsed = time.time() - start_time
        if elapsed > duration:
            break
            
        # 计算当前缩放因子（每2秒一个周期）
        t = elapsed % 2.0
        scale_factor = 1.0 + 0.5 * np.sin(np.pi * t)
        
        # 创建缩放矩阵
        scale_matrix = np.eye(4)
        scale_matrix[0, 0] = scale_factor
        scale_matrix[1, 1] = scale_factor
        scale_matrix[2, 2] = scale_factor
        
        # 应用缩放
        vis["scaling_object"].set_transform(scale_matrix)
        
        # 控制刷新率
        time.sleep(0.05)
        
except KeyboardInterrupt:
    print("动画已停止")

print("程序结束")