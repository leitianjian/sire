import sys
# sys.path.append("D:/code/sire/install/python/release")
import sire

from os.path import abspath
import os
cs = sire.ControlServer.instance()
# current python path
print(abspath(os.path.dirname(__file__)) + "/sire_ball_free_fall.xml")
sire.fromXmlFile(cs, abspath(os.path.dirname(__file__)) + "/sire_ball_free_fall.xml")
cs.init()
simulator = sire.simulator(cs)
simulator.simDuration = 0.906
while(not simulator.isTimeout() and not simulator.isEventListEmpty()):
  simulator.step(1, False)

model = cs.model()
displayInitJson = model.displayInitJson()
result = simulator.recordsToJson()

import matplotlib.pyplot as plt
import numpy as np
timeIndices = result['timeIndex']
contactInfo = result["contactInfo"]
partpq = result["partPq"]
partvs = result["partVs"]
partas = result["partAs"]
dts = result["dts"]
x = []
y = []
yv = []

lowerBound = sire.binarySearch(timeIndices, 0.902)
upperBound = sire.binarySearch(timeIndices, 0.905)
print("lowerBound:", lowerBound, "upperBound:", upperBound)
for i in range(lowerBound, upperBound):
  print(i, timeIndices[i], dts[i], partpq[i][1], partvs[i][1], partas[i][1], contactInfo[i])
  x.append(timeIndices[i])
  y.append(sire.pq2tfmatrix(partpq[i][1])[2, 3]-0.5)  # z position from transformation matrix
  print(y[-1])
  yv.append(partvs[i][1][2])  # z velocity
plt.plot(x, y, marker='o', label='height (m)')
plt.grid(True, linestyle='--', alpha=0.7)
# plt.savefig("D:/papers/sire_contact_model/res/sphere_free_fall_height_data-1.pdf", format="pdf")
plt.show()