import math
import numpy as np

def calculate_inclined_positions(angle_deg=10):
    """
    计算小方块在10度斜面上的位置
    
    参数:
    angle_deg: 斜面角度(度)
    """
    # 角度转换为弧度
    angle_rad = math.radians(angle_deg)
    
    # 当前参数
    ground_height = 0.5  # 地面几何中心z坐标
    ground_thickness = 1  # 地面厚度
    cube_size = 1         # 小方块尺寸
    sphere_radius = 0.1   # 小球半径
    
    # 计算斜面位置
    # 斜面绕x轴旋转10度，保持地面顶部与原点相切
    ground_pe = [0, 0, ground_height, 0, -angle_rad, 0]  # [x,y,z,rx,ry,rz]
    
    # 计算小方块位置
    # 小方块应该放在斜面上，底部四个小球与斜面接触
    cube_height = ground_height + (cube_size + ground_thickness) * 0.5 / math.cos(angle_rad)
    # cube_z_offset = sphere_radius + (cube_size/2 - 0.4)  # 考虑小球半径和几何偏移
    # cube_height = cube_z_offset + (cube_size/2) * math.sin(angle_rad)
    
    # 小方块的位置和姿态
    cube_pe = [0, 0, cube_height, 0, -angle_rad, 0]
    
    return ground_pe, cube_pe

  # import sys
  # sys.path.append("D:/code/sire/install/python/debug")
import sire
cs = sire.ControlServer.instance()
model = cs.model()
model.addSolvers()
middleware = cs.addSireMiddleware()
simulator = middleware.simulationLoop()
simulator.simDuration = 0.6
simulator.setEventHandlerMap({0:6, 1:7, 2:8})
physicsEngine = middleware.physicsEngine()
contactSolver = physicsEngine.addContactPositionForceSolver()
physicsEngine.collisionDetectionFlag = True
physicsEngine.contactSolverFlag = True
contactSolver.setDefaultProp("{k:1.4e8,d:10000,cr:0.2}")
# contactSolver.setDefaultProp("{k:2e7,d:10000,cr:0.2}")
contactSolver.addMaterialPair("m1", "m1", "{k:2e8,d:10000,cr:0,cof:0.3,threshold_velocity:2e-3}")
# contactSolver.addMaterialPair("m1", "m2", "{k:2e8,d:150000,cr:0.3,cof:0.5,threshold_velocity:1e-4}")
model.setGravity([0, 0, -9.81, 0, 0, 0])
model.ground().addMarker("joint_0_k")
model.ground().addMarker("ground_marker")
ground_pe, cube_pe = calculate_inclined_positions(15)
ground_pm = sire.pe3132tfmatrix(ground_pe).tolist()
model.ground().addBoxGeometry(0, 100, 100, 1, [1,0,0,0,0,1,0,0,0,0,1,-0.5,0,0,0,1])
model.ground().addBoxGeometry(0, 5, 5, 1, ground_pm)

physicsEngine.addBoxGeometry(100, 100, 1, 0, False, [1,0,0,0,0,1,0,0,0,0,1,-0.5,0,0,0,1])
physicsEngine.addBoxGeometry(5, 5, 1, 0, False, ground_pm)
# sliderPrt = model.addPartByPe(ground_pe, "313", [10, 0, 0, 0, 0.1, 0.1, 0.1, 0, 0, 0])
# model.link(1).addMarker("slider_center")
boxPrt = model.addPartByPe(cube_pe, "313", [1, 0, 0, 0, 0.1, 0.1, 0.1, 0, 0, 0])
model.link(1).addMarker("box_center")
model.init()
boxPrt.addBoxGeometry(boxPrt.id, 1.0, 1.0, 1.0, prt_pm=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1])
boxPrt.cptGeometryInertial2Part(1)
boxPrt.vs = sire.vp2vs(cube_pe[:3], [0, -2 * math.cos(math.radians(15)), 2 * math.sin(math.radians(15))])

physicsEngine.addBoxGeometry(1, 1, 1, boxPrt.id, True)
physicsEngine.addSphereGeometry(0.1, boxPrt.id, True, [1,0,0,0.5,0,1,0,0.5,0,0,1,-0.4,0,0,0,1], material="m2")
physicsEngine.addSphereGeometry(0.1, boxPrt.id, True, [1,0,0,0.5,0,1,0,-0.5,0,0,1,-0.4,0,0,0,1], material="m2")
physicsEngine.addSphereGeometry(0.1, boxPrt.id, True, [1,0,0,-0.5,0,1,0,0.5,0,0,1,-0.4,0,0,0,1], material="m2")
physicsEngine.addSphereGeometry(0.1, boxPrt.id, True, [1,0,0,-0.5,0,1,0,-0.5,0,0,1,-0.4,0,0,0,1], material="m2")
print(sire.toXmlString(cs))
cs.init()
for i in range(4):
  physicsEngine.collisionFilter().enableCollisionPair(1, i + 3)

physicsEngine.collisionFilter().saveMatConfig()

count = 0
while(not simulator.isTimeout() and not simulator.isEventListEmpty()):
  sim_time = simulator.simTime()
  if abs(sim_time - 0.37) < 1e-8 and count == 0:
    print(boxPrt.getAs())
    count += 1
  simulator.step(1, False)

simulator.recordsContactCptInfo()
displayInitJson = model.displayInitJson()
result = simulator.recordsToJson()
print("Simulation finished, records loaded")
import meshcat
vis = meshcat.Visualizer()
resourcePath = "D:/code/sire/web_interface/public"
sire.robotInit(model.numLinks(), resourcePath, displayInitJson, vis)
sire.animateRobotByRecords(model.numLinks(), result, 1000, vis)
input("按 Enter 键退出程序...")

timeIndices = result['timeIndex']
contactInfo = result["contactInfo"]
partpq = result["partPq"]
partvs = result["partVs"]
partas = result["partAs"]
dts = result["dts"]
x = []
y = []
yv = []
ya = []

lowerBound = sire.binarySearch(timeIndices, 0.37)
upperBound = sire.binarySearch(timeIndices, 0.6)
print(timeIndices[lowerBound], timeIndices[lowerBound + 1])
for i in range(lowerBound, upperBound + 1):
  # print(i, timeIndices[i], dts[i], partpq[i][1], partvs[i][1], partas[i][1], contactInfo[i])
  pm = sire.pq2tfmatrix(partpq[i][1])
  yOffset = pm[1, 3]
  zOffset = (pm[2, 3] - cube_pe[2])
  vp = sire.vs2vp(partpq[i][1], partvs[i][1], [0, 0, 0])
  v = math.sqrt(vp[2] ** 2 + vp[1] ** 2)
  ap = sire.as2ap(partpq[i][1], partvs[i][1], partas[i][1], [0, 0, 0])
  a = math.sqrt(ap[2] ** 2 + ap[1] ** 2)
  print(timeIndices[i], ap, partas[i][1])
  print(timeIndices[i], vp)
  # boxPrt = model.addPartByPe([0,0,0.5,0,0,0], 10)
  # boxPrt.add
  # sire.fromXmlFile(cs, 'D:/code/sire/demo/demo_python/sire_ball_rotate.xml')
  
  # simulator = sire.simulator(cs)
  # while(not simulator.isTimeout() and not simulator.isEventListEmpty()):
  #   simulator.step(1, False)
  
  # model = cs.model()
  # displayInitJson = model.displayInitJson()
  # result = simulator.recordsToJson()
  
  # import meshcat
  # displayInitJson
  # vis = meshcat.Visualizer()
  # resourcePath = "D:/code/sire/web_interface/public"
  # robotInit(model.numLinks(), resourcePath, displayInitJson, vis)
  # animateRobotByRecords(model.numLinks(), result, 1000, vis)
  
  # input("按 Enter 键退出程序...")
  # import json
  # with open("result.json", "w", encoding="utf-8") as f:
  #     json.dump(result, f, ensure_ascii=False, indent=2)