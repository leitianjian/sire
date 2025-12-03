import mujoco
import numpy as np
import matplotlib.pyplot as plt
import os
import time

# 物理参数
class Params:
    g = -9.81  # 重力加速度 (m/s²)
    m = 1.0    # 小球质量 (kg)
    M = 10.0   # 盒子质量 (kg)
    n = 25     # 小球数量 (5x5)
    F = 150.0  # 施加的力 (N)
    raiGroundMu = 0.4  # 地面摩擦系数
    raiBoxMu = 1.0     # 盒子摩擦系数
    raiBallMu = 0.8    # 小球摩擦系数
    T = 1.0    # 仿真时间 (s)

class Options:
    dt = 0.001  # 时间步长 (s)
    forceDirection = "XY"  # 力方向

# 计算解析解
def compute_analytical_sol(t, is_ball):
    # 计算摩擦系数
    mu1 = Params.raiGroundMu * Params.raiBoxMu
    mu2 = Params.raiBoxMu * Params.raiBallMu
    
    # 计算摩擦力
    f1 = mu1 * (Params.M + Params.n * Params.m) * abs(Params.g)
    f2 = 1 / Params.M * (Params.F - f1) / (3.5 / Params.m + 25 / Params.M)
    
    # 计算加速度
    a1 = (Params.F - f1 - Params.n * f2) / Params.M
    a2 = f2 / Params.m
    
    # 计算速度
    v = a2 * t if is_ball else a1 * t
    
    # 根据力方向返回速度向量
    if Options.forceDirection == "XY":
        return np.array([v * 0.5, v * 0.866025403784439, 0])
    else:
        return np.array([0, v, 0])

# 创建 MuJoCo 模型
def create_model():
    model_xml = """
    <mujoco>
        <option cone="elliptic"/>
        <option timestep="{dt}"/>
        <option gravity="0 0 {g}"/>
        
        <worldbody>
            <!-- 地面 -->
            <geom name="ground" type="plane" size="100 100 5" rgba=".9 0 0 1" friction="0.4 0 0"/>
            
            <!-- 盒子 -->
            <body name="box" pos="0 0 0.499998">
                <inertial pos="0 0 0" mass="10" diaginertia="334.16666666666669 334.16666666666669 666.66666666666674"/>
                <joint type="free"/>
                <geom name="box_geom" type="box" size="10.0 10.0 0.5" rgba="0 .9 0 1" friction="0.4 0 0"/>
            </body>
    """.format(dt=Options.dt, g=Params.g)
    
    # 添加小球 (5x5 排列)
    positions = [
        (-4.0, -4.0), (-4.0, -2.0), (-4.0, 0.0), (-4.0, 2.0), (-4.0, 4.0),
        (-2.0, -4.0), (-2.0, -2.0), (-2.0, 0.0), (-2.0, 2.0), (-2.0, 4.0),
        (0.0, -4.0), (0.0, -2.0), (0.0, 0.0), (0.0, 2.0), (0.0, 4.0),
        (2.0, -4.0), (2.0, -2.0), (2.0, 0.0), (2.0, 2.0), (2.0, 4.0),
        (4.0, -4.0), (4.0, -2.0), (4.0, 0.0), (4.0, 2.0), (4.0, 4.0)
    ]
    
    for i, (x, y) in enumerate(positions):
        model_xml += f"""
            <body name="ball_{i}" pos="{x} {y} 1.499998">
                <inertial pos="0 0 0" mass="1" diaginertia=".1 .1 .1"/>
                <joint type="free"/>
                <geom name="ball_{i}_geom" type="sphere" size="0.5" rgba="0 .9 0 1" friction="0.8 0 0"/>
            </body>
        """
    
    model_xml += """
        </worldbody>
    </mujoco>
    """
    
    return mujoco.MjModel.from_xml_string(model_xml)

# 计算误差
def compute_error(ball_vel, box_vel):
    n = len(box_vel)
    vel_error_sq = np.zeros((n, 3))
    
    for i in range(n):
        t = i * Options.dt
        ball_vec = compute_analytical_sol(t, True)
        box_vec = compute_analytical_sol(t, False)
        
        # 计算每个方向的误差平方
        print(len(ball_vel[i]))
        for v in ball_vel[i]:
          print(v) 
        for j in range(3):
            # 计算所有小球速度的平均值
            avg_ball_vel = np.mean([v[j] for v in ball_vel[i]], axis=0)
            
            ball_err = avg_ball_vel - ball_vec[j]
            box_err = box_vel[i][j] - box_vec[j]
            vel_error_sq[i, j] = ball_err**2 + box_err**2
    
    # 计算总误差
    total_error = np.mean(np.sum(vel_error_sq, axis=1))
    
    # 绘图
    tdata = np.arange(n) * Options.dt
    xdata = vel_error_sq[:, 0]
    ydata = vel_error_sq[:, 1]
    zdata = vel_error_sq[:, 2]
    sumdata = np.sum(vel_error_sq, axis=1)
    
    plt.figure(figsize=(12, 8))
    plt.plot(tdata, xdata, label='X Error Sq')
    plt.plot(tdata, ydata, label='Y Error Sq')
    plt.plot(tdata, zdata, label='Z Error Sq')
    plt.plot(tdata, sumdata, label='Total Error', linewidth=2)
    
    plt.xlabel('Time (s)')
    plt.ylabel('Squared Velocity Error')
    plt.title('Velocity Error Over Time')
    plt.legend()
    plt.grid(True)
    plt.savefig('velocity_error.png', dpi=300)
    plt.show()
    
    return total_error

# 主仿真函数
def run_simulation():
    # 创建模型和数据
    model = create_model()
    data = mujoco.MjData(model)
    
    # 设置仿真参数
    simulation_time = Params.T
    num_steps = int(simulation_time / Options.dt)
    
    # 存储数据
    ball_vel = []
    ball_pos = []
    box_vel = []
    box_pos = []
    
    # 施加力
    force_direction = np.array([0.5, 0.866025403784439, 0]) if Options.forceDirection == "XY" else np.array([0, 1, 0])
    force_vector = force_direction * Params.F
    
    # 初始化仿真
    mujoco.mj_resetData(model, data)
    
    # 运行仿真
    for i in range(num_steps):
        # 施加力到盒子
        data.xfrc_applied[model.body("box").id] = [*force_vector, 0, 0, 0]
        
        # 执行仿真步骤
        mujoco.mj_step(model, data)
        
        # 记录盒子的速度和位置
        box_id = model.body("box").id
        box_vel.append(data.qvel[6*box_id:6*box_id+3].copy())  # 线性速度
        box_pos.append(data.qpos[7*box_id:7*box_id+3].copy())  # 位置
        
        # 记录所有小球的速度和位置
        frame_ball_vel = []
        frame_ball_pos = []
        for j in range(Params.n):
            ball_id = model.body(f"ball_{j}").id
            frame_ball_vel.append(data.qvel[6*ball_id:6*ball_id+3].copy())
            frame_ball_pos.append(data.qpos[7*ball_id:7*ball_id+3].copy())
        
        ball_vel.append(frame_ball_vel)
        ball_pos.append(frame_ball_pos)
    
    # 计算误差
    error = compute_error(ball_vel, box_vel)
    print(f"Total Error: {error:.6f}")
    
    return ball_vel, ball_pos, box_vel, box_pos, error

# 运行仿真
if __name__ == "__main__":
    # 确保 mujoco 已正确加载
    
    print("Starting rolling test simulation...")
    start_time = time.time()
    
    ball_vel, ball_pos, box_vel, box_pos, error = run_simulation()
    
    end_time = time.time()
    print(f"Simulation completed in {end_time - start_time:.2f} seconds")
    print(f"Total error: {error:.6f}")
    
    # 保存结果
    np.savez('rolling_test_results.npz', 
             ball_vel=ball_vel, 
             ball_pos=ball_pos, 
             box_vel=box_vel, 
             box_pos=box_pos,
             error=error)
    
    print("Results saved to rolling_test_results.npz")