import sire
from os.path import abspath
import numpy as np
import os
import matplotlib.pyplot as plt

cs = sire.ControlServer.instance()
# parameters = [[2e3, 0], [2e5, 0], [3e6, 0], [2e8, 0]]
parameters = [[2e8, 0]]
sim_duration = 0.1
force_duration = 0.4
dt = 0.001
ctrlt = 10

force_range = [9.1]  # 9~12 N，31个点
# force_range = np.linspace(9, 12, 31)  # 9~12 N，31个点

heights = np.zeros([len(parameters), len(force_range)])
for i in range(len(parameters)):
  print(f"Testing with stiffness {parameters[i][0]} and damping {parameters[i][1]}")
  for k in range(len(force_range)):
    f = force_range[k]
    sire.fromXmlFile(cs, r"D:\code\sire\demo\demo_python\sire\ballFreefall\sire_ball_with_up_force.xml")
    cs.init()
    simulator = sire.simulator(cs)
    pe = cs.physicsEngine()
    simulator.simDuration = sim_duration
    simulator.deltaT = dt
    simulator.ctrlT = ctrlt
    contactPropStr = f"{{k:{parameters[i][0]},d:{parameters[i][1]},cof:0,threshold_velocity:1e-4}}"
    pe.contactPositionForceSolver().addMaterialPair("steel", "copper", contactPropStr)
    # print(sire.toXmlString(cs))
    while(not simulator.isTimeout() and not simulator.isEventListEmpty()):
      model = cs.model()
      isCtrl = simulator.integrate()
      if simulator.simTime() <= force_duration:
        if isinstance(model.forcePool()[0], sire.GeneralForce):
          model.forcePool()[0].fce = [0, 0, f, 0, 0, 0]
      else:
        if isinstance(model.forcePool()[0], sire.GeneralForce):
          model.forcePool()[0].fce = [0, 0, 0, 0, 0, 0]
      simulator.handleContact()
      # simulator.step(1, False)
    displayInitJson = model.displayInitJson()
    sim_result = simulator.recordsToJson()
    time_force_end = force_duration
    timeIndices = sim_result['timeIndex']
    contactInfo = sim_result["contactInfo"]
    partpq = sim_result["partPq"]
    max_height = -1  # 初始高度
    for j in range(len(timeIndices)-1):
      # print(i, timeIndices[i], dts[i], partpq[i][1], partvs[i][1], partas[i][1], contactInfo[i])
      # print(f"{j}: time: {timeIndices[j]}")
      # print(f"Current ps: {partpq[j][0]}")
      current_height = partpq[j][1][2] - 0.2  # z position from transformation matrix
      if current_height > max_height:
        max_height = current_height

    # for step in range(int(sim_time / dt)):
    #     t = step * dt
    #     if t < time_force_end:
    #         # 在球心施加向上的力
    #         p.applyExternalForce(sphere_id, -1, forceObj=[0, 0, force],
    #                              posObj=[0, 0, 0.2], flags=p.LINK_FRAME)
    #     p.stepSimulation()
    #     pos, _ = p.getBasePositionAndOrientation(sphere_id)
    #     current_height = pos[2] - radius  # 球心高度减去半径得到球底高度
    #     if current_height > max_height:
    #         max_height = current_height
    
    # h = run_simulation(f, stiffness_high, damping_high, force_duration=0.4)
    heights[i, k] = max_height
    print(f"力 {f:4.1f} N -> 最大高度 {max_height} m")

# -------------------- 绘制结果 --------------------
# for i in range(len(parameters)):
#   plt.plot(force_range, heights[i], 'o', label=f'k = {parameters[i][0]}, d = {parameters[i][1]}')
# plt.xlabel('normal force (N)')
# plt.ylabel('max height (m)')
# # plt.title('接触刚度对推力-高度曲线平滑性的影响')
# plt.title('contact stiffness effect on force-height curve smoothness')
# plt.legend()
# plt.grid(True)
# plt.show()