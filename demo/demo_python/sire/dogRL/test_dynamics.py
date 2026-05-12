import sys
import numpy as np
from math import sin, cos

FRAME_AXIS_POSITIONS = (
    np.array([[0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 1, 0], [0, 0, 0], [0, 0, 1]])
    .astype(np.float32)
    .T
)
FRAME_AXIS_COLORS = (
    np.array([[1, 0, 0], [1, 0.6, 0], [0, 1, 0], [0.6, 1, 0], [0, 0, 1], [0, 0.6, 1]])
    .astype(np.float32)
    .T
)


def visualize_frame(
    visualizer, name, tform, line_length=0.2, line_width=3, line_color=None
):
    import meshcat.geometry as mg
    import numpy as np
    """
    Visualizes a coordinate frame as an axis triad at a specified pose.

    Parameters
    ----------
        visualizer : `pinocchio.visualize.meshcat_visualizer.MeshcatVisualizer`
            The visualizer instance.
        name : str
            The name of the MeshCat component to add.
        tform : `pinocchio.SE3`
            The transform at which to display the frame.
        line_length : float, optional
            The length of the axes in the triad.
        line_width : float, optional
            The width of the axes in the triad.
        line_color : array-like, optional
            The line colors to use. If None, chooses default axes colors.
    """
    if line_color:
        color = np.array([line_color] * 6).T
    else:
        color = FRAME_AXIS_COLORS

    visualizer.viewer[name].set_object(
        mg.LineSegments(
            mg.PointsGeometry(
                position=line_length * FRAME_AXIS_POSITIONS,
                color=color,
            ),
            mg.LineBasicMaterial(
                linewidth=line_width,
                vertexColors=True,
            ),
        )
    )
    visualizer.viewer[name].set_transform(tform.homogeneous)


def visualize_frames(
    visualizer, prefix_name, tforms, line_length=0.2, line_width=3, line_color=None
):
    """
    Visualizes a set of coordinate frames as axis triads at specified poses.

    Parameters
    ----------
        visualizer : `pinocchio.visualize.meshcat_visualizer.MeshcatVisualizer`
            The visualizer instance.
        prefix_name : str
            The name of the MeshCat component to add.
        tforms : list[`pinocchio.SE3`]
            A list of transforms at which to display frames.
        line_length : float, optional
            The length of the axes in the triad.
        line_width : float, optional
            The width of the axes in the triad.
        line_color : array-like, optional
            The line colors to use. If None, chooses default axes colors.
    """
    visualizer.viewer[prefix_name].delete()
    for idx, tform in enumerate(tforms):
        visualize_frame(
            visualizer,
            f"{prefix_name}/frame{idx}",
            tform,
            line_length=line_length,
            line_width=line_width,
            line_color=line_color,
        )


def visualize_path(visualizer, name, tforms, line_width=3, line_color=[0.0, 0.0, 0.0]):
    """
    Visualizes a path of poses as lines containing only the translation component.

    Parameters
    ----------
        visualizer : `pinocchio.visualize.meshcat_visualizer.MeshcatVisualizer`
            The visualizer instance.
        name : str
            The name of the MeshCat component to add.
        tforms : list[`pinocchio.SE3`]
            A list of transforms representing the vertices of the path.
        line_width : float, optional
            The width of the axes in the triad.
        line_color : array-like, optional
            The line color to use.
    """
    visualize_paths(
        visualizer, name, [tforms], line_width=line_width, line_color=line_color
    )

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
    
def robotInit(numLinks, resourcePath: str, displayInitJson, vis):
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
      if ext == 'stl' or ext == 'STL':
        meshcatGeo.set_object(g.StlMeshGeometry.from_file(
          resourcePath + geometry['resource_path']), material=material)
      elif ext == 'obj':
        meshcatGeo.set_object(g.ObjMeshGeometry.from_file(
          resourcePath + geometry['resource_path']), material=material)
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
      try:
        setRobotPq(numLinks, frame, partpq[currentIdx])
      except Exception as e:
        print("Error setting robot pq at time", currentTime, ":", e)

  vis.set_animation(anim)

def jointTrajectory(time, TotalTime, start_q, target_q):
   if (time >= TotalTime):
      return target_q
   size = len(target_q)
   current_q = np.zeros(size)
   for i in range(size):
      current_q[i] = (target_q[i] - start_q[i]) * sin(np.pi * time / (2 * TotalTime)) + start_q[i]
   return current_q

def main():
  import pathlib
  currentDir = pathlib.Path(__file__).parent.resolve()
  # pyEnvPath = str((currentDir / "python/debug").resolve())
  # sys.path.append(pyEnvPath)
  sys.path.append("D:/code/sire/install/python/debug")
  import sire
  cs = sire.ControlServer.instance()
  configPath = str((currentDir / "a1_modified.xml").resolve())
  # configPath = str((currentDir / "sire_edge_simple.xml").resolve())
  sire.fromXmlFile(cs, configPath)
  cs.init()
  simulator = sire.simulationLoop(cs)
  model = cs.model()
  for i in range(12):
    if (isinstance(model.force(0), sire.SingleComponentForce)):
      model.force(0).fce = 1
  print("model joint constraints force before forward dynamics")
  for i in range(model.numJoints()):
    print("joint", i, "cf =", model.joint(i).cf())
  if model.forwardDynamics():
    print("Model forward dynamics failed, please check the model configuration.")
    return
  print("model joint constraints force after forward dynamics")
  for i in range(model.numJoints()):
    print("joint", i, "cf =", model.joint(i).cf())
  print("After forward dynamics and before update motion accel")
  for i in range(model.numMotions()):
    print("motion", i, "accel =", model.motion(i).ma)
  for i in range(model.numMotions()):
    model.motion(i).updA()
  print("After update motion accel")
  for i in range(model.numMotions()):
    print("motion", i, "accel =", model.motion(i).ma)
  
  for i in range(model.numLinks()):
    print("Link", i, "pq", model.link(i).getPq())
    print("Link", i, "as", model.link(i).getAs())
    print("Link", i, "vs", model.link(i).getVs())

if __name__ == "__main__":
  main()