def pq2tfmatrix(pq):
  import meshcat.transformations as tf
  import numpy as np
  """
  Convert a pq to a transformation matrix
  :param pq: [position, quaternion]
  :return: transformation matrix
  """
  p = pq[:3]
  q = pq[3:]
  q.insert(0, q.pop())
  return tf.quaternion_matrix(q) + np.array([[0, 0, 0, p[0]],
                                             [0, 0, 0, p[1]],
                                             [0, 0, 0, p[2]],
                                             [0, 0, 0, 0]])
    
def robotInit(numLinks, resource_path, displayInitJson, vis):
  import meshcat.geometry as g
  import numpy as np
  robot = vis['robot']
  partInitConfig = displayInitJson['part_init_config']
  for i in range(numLinks):
    robot[str(i)].set_transform(pq2tfmatrix(partInitConfig[i]))

  geometryPool = displayInitJson['geometry_pool']
  for i in range(len(geometryPool)):
    geometry = geometryPool[i]
    meshcatGeo = robot[str(geometry['part_id'])][str(geometry['geometry_id'])]
    meshcatGeo.set_transform(np.array(geometry['init_pm']).reshape(4, 4))
    if i % 2 == 1: 
      material = g.MeshPhongMaterial(color=0x0660FF)
    else:
      material = g.MeshPhongMaterial(color=0xD4D4D4)
    if i == 0:
      material = g.MeshPhongMaterial(color=0x755338)
    if(geometry['shape_type'] == 'box'):
      meshcatGeo.set_object(g.Box([geometry['length'], geometry['width'], geometry['height']]), material=material)
    elif(geometry['shape_type'] == 'capsule'):
      meshcatGeo.set_object(g.Cylinder(geometry['size'][0], geometry['size'][1]), material=material)
    elif(geometry['shape_type'] == 'sphere'):
      meshcatGeo.set_object(g.Sphere(geometry['radius']), material=material)
    elif(geometry['shape_type'] == 'mesh'):
      ext = geometry['resource_path'].split('.')[-1]
      if ext == 'stl':
        meshcatGeo.set_object(g.StlMeshGeometry.from_file(
          resource_path, geometry['resource_path']), material=material)
      elif ext == 'obj':
        meshcatGeo.set_object(g.ObjMeshGeometry.from_file(
          resource_path + geometry['resource_path']), material=material)
      else:
        print("Unknown mesh file type", ext)

def setRobotPq(numLinks, frame, pqs):
  robot = frame['robot']
  for i in range(numLinks):
    robot[str(i)].set_transform(pq2tfmatrix(pqs[i]))

def binarySearch(timeIndices, time):
  import math
  """
  Binary search to find the index of the closest time
  :param timeIndices: list of time indices
  :param time: target time
  :return: index of the closest time index
  """
  low = 0
  high = len(timeIndices) - 1
  while low <= high:
    mid = (low + high) // 2
    if math.isclose(timeIndices[mid], time):
      return mid
    if timeIndices[mid] < time:
      low = mid + 1
    else:
      high = mid - 1

  return min(int(low), len(timeIndices) - 1)

def animateRobotByRecords(numLinks, records, frameRate, vis):
  from meshcat.animation import Animation
  partpq = records['partPq']
  timeIndices = records['timeIndex']
  anim = Animation()
  anim.default_framerate = frameRate

  minTime = 0
  maxTime = timeIndices[-1]
  totalFrameNumber = int((maxTime - minTime) * anim.default_framerate)

  for i in range(totalFrameNumber):
    currentTime = minTime + i / anim.default_framerate
    currentIdx = binarySearch(timeIndices, currentTime)
    with anim.at_frame(vis, i) as frame:
      setRobotPq(numLinks, frame, partpq[currentIdx])

  vis.set_animation(anim)

if __name__ == "__main__":
  import sys
  sys.path.append("D:/code/sire/install/python/release")
  import sire
  from os.path import abspath
  import os
  cs = sire.ControlServer.instance()
  sire.fromXmlFile(cs, 'D:/code/sire/demo/demo_python/box.xml')
  cs.init()
  
  simulator = sire.simulator(cs)
  while(not simulator.isTimeout() and not simulator.isEventListEmpty()):
    simulator.step(1, False)
  
  model = cs.model()
  displayInitJson = model.displayInitJson()
  result = simulator.recordsToJson()
  
  import meshcat
  displayInitJson
  vis = meshcat.Visualizer()
  resourcePath = "D:/code/sire/web_interface/public"
  robotInit(model.numLinks(), resourcePath, displayInitJson, vis)
  animateRobotByRecords(model.numLinks(), result, 1000, vis)
  
  input("按 Enter 键退出程序...")
  import json
  with open("result.json", "w", encoding="utf-8") as f:
      json.dump(result, f, ensure_ascii=False, indent=2)