# def pq2tfmatrix(pq):
#   import meshcat.transformations as tf
#   import numpy as np
#   """
#   Convert a pq to a transformation matrix
#   :param pq: [position, quaternion]
#   :return: transformation matrix
#   """
#   p = pq[:3]
#   q = pq[3:]
#   q.insert(0, q.pop())
#   return tf.quaternion_matrix(q) + np.array([[0, 0, 0, p[0]],
#                                              [0, 0, 0, p[1]],
#                                              [0, 0, 0, p[2]],
#                                              [0, 0, 0, 0]])
    
# def robotInit(numLinks, resource_path, displayInitJson, vis):
#   import meshcat.geometry as g
#   import numpy as np
#   robot = vis['robot']
#   partInitConfig = displayInitJson['part_init_config']
#   for i in range(numLinks):
#     robot[str(i)].set_transform(pq2tfmatrix(partInitConfig[i]))

#   geometryPool = displayInitJson['geometry_pool']
#   for i in range(len(geometryPool)):
#     geometry = geometryPool[i]
#     meshcatGeo = robot[str(geometry['part_id'])][str(geometry['geometry_id'])]
#     meshcatGeo.set_transform(np.array(geometry['init_pm']).reshape(4, 4))
#     if i % 2 == 1: 
#       material = g.MeshPhongMaterial(color=0x0660FF, specular=0xffffff, shininess=50)
#     else:
#       material = g.MeshPhongMaterial(color=0xD4D4D4, specular=0xffffff, shininess=50)
#     if i == 0:
#       material = g.MeshPhongMaterial(color=0x755338, specular=0xffffff, shininess=50)
#     if(geometry['shape_type'] == 'box'):
#       meshcatGeo.set_object(g.Box([geometry['length'], geometry['width'], geometry['height']]), material=material)
#     elif(geometry['shape_type'] == 'capsule'):
#       meshcatGeo.set_object(g.Cylinder(geometry['size'][0], geometry['size'][1]), material=material)
#     elif(geometry['shape_type'] == 'sphere'):
#       meshcatGeo.set_object(g.Sphere(geometry['radius']), material=material)
#     elif(geometry['shape_type'] == 'mesh'):
#       ext = geometry['resource_path'].split('.')[-1]
#       if ext == 'stl':
#         meshcatGeo.set_object(g.StlMeshGeometry.from_file(
#           resource_path, geometry['resource_path']), material=material)
#       elif ext == 'obj':
#         meshcatGeo.set_object(g.ObjMeshGeometry.from_file(
#           resource_path + geometry['resource_path']), material=material)
#       else:
#         print("Unknown mesh file type", ext)

# def setRobotPq(numLinks, frame, pqs):
#   robot = frame['robot']
#   for i in range(numLinks):
#     robot[str(i)].set_transform(pq2tfmatrix(pqs[i]))

# def binarySearch(timeIndices, time):
#   import math
#   """
#   Binary search to find the index of the closest time
#   :param timeIndices: list of time indices
#   :param time: target time
#   :return: index of the closest time index
#   """
#   low = 0
#   high = len(timeIndices) - 1
#   while low <= high:
#     mid = (low + high) // 2
#     if math.isclose(timeIndices[mid], time):
#       return mid
#     if timeIndices[mid] < time:
#       low = mid + 1
#     else:
#       high = mid - 1

#   return min(int(low), len(timeIndices) - 1)

# def animateRobotByRecords(numLinks, records, frameRate, vis):
#   from meshcat.animation import Animation
#   partpq = records['partPq']
#   timeIndices = records['timeIndex']
#   anim = Animation()
#   anim.default_framerate = frameRate

#   minTime = 0
#   maxTime = timeIndices[-1]
#   totalFrameNumber = int((maxTime - minTime) * anim.default_framerate)

#   for i in range(totalFrameNumber):
#     currentTime = minTime + i / anim.default_framerate
#     currentIdx = binarySearch(timeIndices, currentTime)
#     with anim.at_frame(vis, i) as frame:
#       setRobotPq(numLinks, frame, partpq[currentIdx])

#   vis.set_animation(anim)

# 分布m * n 个物体在一个平面上
def distributeObjectOnPlane(m, n, mInterval, nInterval, height):
  if m < 1 or n < 1:
    return []
  distPos = []
  mLen = (m - 1) * mInterval
  nLen = (n - 1) * nInterval
  for i in range(m):
    for j in range(n):
      distPos.append([-mLen / 2 + i * mInterval, -nLen / 2 + j * nInterval, height, 0, 0, 0])
  return distPos

if __name__ == "__main__":
  # import sys
  # sys.path.append("D:/code/sire/install/python/debug")
  import sire
  m = 1
  n = 1
  numBalls = m * n
  cs = sire.ControlServer.instance()
  model = cs.model()
  model.addSolvers()
  middleware = cs.addSireMiddleware()
  simulator = middleware.simulationLoop()
  simulator.simDuration = 1.55
  simulator.setEventHandlerMap({0:6, 1:7, 2:8})
  physicsEngine = middleware.physicsEngine()
  contactSolver = physicsEngine.addContactPositionForceSolver()
  physicsEngine.collisionDetectionFlag = True
  physicsEngine.contactSolverFlag = True
  contactSolver.setDefaultProp("{k:1.4e8,d:1500,cr:0.2}")
  # contactSolver.setDefaultProp("{k:2e7,d:10000,cr:0.2}")
  contactSolver.addMaterialPair("m1", "m1", "{k:2e8,d:150000,cr:0.3,cof:0.5,threshold_velocity:1e-4}")
  # contactSolver.addMaterialPair("m1", "m2", "{k:2e8,d:150000,cr:0.3,cof:0.5,threshold_velocity:1e-4}")
  model.setGravity([0, 0, -9.8, 0, 0, 0])
  model.ground().addMarker("joint_0_k")
  model.ground().addMarker("ground_marker")
  # model.ground().addBoxGeometry(0, 100, 100, 1, [1,0,0,0,0,1,0,0,0,0,1,-0.5,0,0,0,1])
  physicsEngine.addBoxGeometry(100, 100, 1, 0, False, [1,0,0,0,0,1,0,0,0,0,1,-0.5,0,0,0,1])
  boxPrt = model.addPartByPe([0,0,0.5,0,0,0], "313", [10, 0, 0, 0, 0.1, 0.1, 0.1, 0, 0, 0])
  model.link(1).addMarker("box_center")
  spherePos = distributeObjectOnPlane(m, n, 1, 1, 1.15)
  for i in range(numBalls):
    spherePrt = model.addPartByPe(spherePos[i], "313", [1, 0, 0, 0, 0.1, 0.1, 0.1, 0, 0, 0])
    model.link(2 + i).addMarker("sphere" + str(2 + i) +"_center")
  model.init()
  # print(sire.toXmlString(model))
  boxPrt.addBoxGeometry(boxPrt.id, 10.0, 10.0, 1.0, prt_pm=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1])
  boxPrt.cptGeometryInertial2Part(10)
  physicsEngine.addBoxGeometry(10, 10, 1, boxPrt.id, True)
  physicsEngine.addSphereGeometry(0.5, boxPrt.id, True, [1,0,0,5,0,1,0,5,0,0,1,0,0,0,0,1], material="m2")
  physicsEngine.addSphereGeometry(0.5, boxPrt.id, True, [1,0,0,5,0,1,0,-5,0,0,1,0,0,0,0,1], material="m2")
  physicsEngine.addSphereGeometry(0.5, boxPrt.id, True, [1,0,0,-5,0,1,0,5,0,0,1,0,0,0,0,1], material="m2")
  physicsEngine.addSphereGeometry(0.5, boxPrt.id, True, [1,0,0,-5,0,1,0,-5,0,0,1,0,0,0,0,1], material="m2")
  # spherePos = distributeObjectOnPlane(3, 3, 4, 4, 1)
  for i in range(numBalls):
    spherePrt = model.link(2 + i)
    spherePrt.addSphereGeometry(spherePrt.id, 0.1, prt_pm=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1])
    spherePrt.cptGeometryInertial2Part(1)
    physicsEngine.addSphereGeometry(0.1, spherePrt.id)
  
  cs.init()
  for i in range(4):
    physicsEngine.collisionFilter().enableCollisionPair(0, i + 2)
  for i in range(numBalls):
    physicsEngine.collisionFilter().enableCollisionPair(1, i + 6)

  physicsEngine.collisionFilter().saveMatConfig()
  print(sire.toXmlString(cs))
  # print(sire.fpm2fs([0,100,0], [1,0,0,0,0,1,0,0,0,0,1,0.5,0,0,0,1]))
  while(not simulator.isTimeout() and not simulator.isEventListEmpty()):
    sim_time = simulator.simTime()
    if (sim_time > 0.5):
      generalFce = model.forcePool()[1]
      # slideFce = sire.fpm2fs([0,0,0], boxPrt.markerPm(0))
      if isinstance(generalFce, sire.GeneralForce):
        # print(generalFce.fce, sire.fpm2fs([0,100,0], boxPrt.markerPm(0)))
        boxCenterPm = boxPrt.markerPm(0)
        pm = [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]
        pm[3] = boxCenterPm[3]
        pm[7] = boxCenterPm[7] 
        pm[11] = boxCenterPm[11]
        generalFce.fce = list(map(lambda a, b: a + b, generalFce.fce, sire.fpm2fs([0,100,0], pm)))
        # print(generalFce.fce)

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