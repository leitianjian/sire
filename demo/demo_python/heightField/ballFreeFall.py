from os.path import abspath
import os
import sire
import sys

sim = sire.Simulator()
print(abspath(os.getcwd()) + "/sire_ball_free_fall.xml")
sire.fromXmlFile(sim, abspath(os.getcwd()) + "/sire_ball_free_fall.xml")
sim.init()
simulator = sim.simulationLoop()
simulator.simDuration = 10
while(not simulator.isTimeout() and not simulator.isEventListEmpty()):
  simulator.step(1, False)

model = sim.model()
displayInitJson = sim.displayInitJson()
result = simulator.recordsToJson()

print("Simulation finished, records loaded")
import meshcat
vis = meshcat.Visualizer()
resourcePath = "D:/code/sire/web_interface/public"
sire.robotInit(model.numLinks(), resourcePath, displayInitJson, vis)
sire.animateRobotByRecords(model.numLinks(), result, 1000, vis)
print("Simulation time", simulator.simTime())


import matplotlib.pyplot as plt
import scienceplots
import numpy as np

def simple_sci_format(number):
    """最简单的科学计数法格式化"""
    sci_str = f"{number:.0e}"
    return sci_str.replace('+0', '').replace('+', '')

with plt.style.context(['science','ieee']):
  legendStrs = []
  linestyles = ["-","--", "-.", ":"]
  scientific_colors = [
        '#2E86AB', '#A23B72', '#F18F01', '#C73E1D', '#3B1F2B',
        '#6B8E23', '#8B4513', '#4682B4', '#D2691E', '#2F4F4F'
  ]
  parameters = [[1, 1]]

  pltResult = result
  timeIndices = pltResult['timeIndex']
  pp = pltResult["penetrationPairs"]  # contactInfo is renamed to penetrationPairs in the latest code
  partpq = pltResult["partPq"]
  partvs = pltResult["partVs"]
  partas = pltResult["partAs"]
  gf = pltResult["generalForces"]
  dts = pltResult["dts"]
  x = []
  y = []
  yv = []
  # print(result[i][2])
  
  lowerBound = sire.binarySearch(timeIndices, 0.0)
  upperBound = sire.binarySearch(timeIndices, 1.0)
  print(lowerBound, upperBound)
  # with open('hfield_data.txt', 'w') as f:          # 'w' 覆盖写，'a' 追加写
  # with open('box_data.txt', 'w') as f:          # 'w' 覆盖写，'a' 追加写
  # for j in range(lowerBound, upperBound):
      # print(j, f"{timeIndices[j]:.4f}", f"{dts[j]:.4f}", f"{partpq[j][1][2]:.6f}", f"{partvs[j][1][2]:.6f}", f"{partas[j][1][2]:.6f}", pp[j], gf[j], file=f)
    # print(j, f"{timeIndices[j]:.4f}", f"{dts[j]:.4f}", f"{partpq[j][1][2]:.6f}", f"{partvs[j][1][2]:.6f}", f"{partas[j][1][2]:.6f}", pp[j], gf[j])
  #   print(j, timeIndices[j], dts[j], partpq[j][1][2], partvs[j][1][2], partas[j][1][2], pp[j], gf[j])
  #   x.append(timeIndices[j])
  #   y.append(sire.pq2tfmatrix(partpq[j][1])[2, 3]-0.5)  # z position from transformation matrix
  #   yv.append(partvs[j][1][2])  # z velocity
  # # .append(f"k: {parameters[i][0]:.1e},d: {parameters[i][1]}")
  # # plt.plot(x, y, linewidth=1, label=f"k: {parameters[i][0]:.1e},d: {parameters[i][1]}")
  # plt.plot(x, y, marker='o', markersize=2, linewidth=1)
  
  # plt.legend(loc="best", fontsize=8)
  # plt.ylabel(r'Height (m)')
  # plt.xlabel(r'Time (s)')
  # plt.title('Bouncing ball by proposed method')
  # plt.grid(True, linestyle='--', alpha=0.3)
  # plt.savefig("D:/papers/sire_contact_model/res/sphere_free_fall_height_data_sire.pdf", format="pdf")
  # plt.show()
input("Press Enter to exit...")
