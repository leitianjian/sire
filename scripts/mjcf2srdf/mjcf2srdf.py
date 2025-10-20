import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import RigidTransform as TF
from dm_control import mjcf
import sys
sys.path.append("D:/code/sire/install/python/debug")
import sire

def quat_from_vectors(source_vector, target_vector):
    """
    Generates a rotation matrix that aligns source_vector with target_vector.

    Args:
        source_vector (np.array): The initial vector (e.g., [1, 0, 0]).
        target_vector (np.array): The vector to align with (e.g., [0, 1, 0]).

    Returns:
        np.array: A 3x3 rotation matrix.
    """
    # Normalize the input vectors
    source_vector = source_vector / np.linalg.norm(source_vector)
    target_vector = target_vector / np.linalg.norm(target_vector)

    # Use scipy's align_vectors to find the rotation
    rotation = R.align_vectors(target_vector.reshape(1, -1), source_vector.reshape(1, -1))[0]

    # Convert the Rotation object to a 3x3 rotation matrix
    quat = rotation.as_quat(scalar_first=True)
    return quat

class MJCFtoSIREConverter:
    def __init__(self, mjcf_path):
        self.mjcf_path = mjcf_path
        self.sire_cs = sire.ControlServer.instance()
        self.sire_model = self.sire_cs.model()
        self.parts = {}
        self.partId = 1
        self.model = None
        self.sire_structure = {
            'parts': [],
            'joints': [],
            'connects': []
        }
        self.body_abs_poses = {}  # 存储杆件在世界坐标系中的绝对位姿
        self.joint_abs_poses = {}  # 存储关节在世界坐标系中的绝对位姿
        self.markers = {}  # 存储标记点信息

    def load_and_parse_mjcf(self):
        """使用 dm_control 加载和解析 MJCF 文件"""
        self.model = mjcf.from_path(self.mjcf_path)
        print(f"成功加载 MJCF 模型: {self.model.model}")
        
        # 递归计算所有杆件的绝对位姿
        for i in range(len(self.model.worldbody.body)):
            self._compute_body_absolute_poses(self.model.worldbody.body[i], 
                                         np.zeros(3), 
                                         np.array([1, 0, 0, 0]))
# 
    
    def _compute_body_absolute_poses(self, body, parent_pos, parent_quat):
        """递归计算杆件在世界坐标系中的绝对位姿"""
        # 获取相对位姿
        print(body)
        rel_pos = body.pos if body.pos is not None else np.zeros(3)
        rel_quat = body.quat if body.quat is not None else np.array([1, 0, 0, 0])
        
        # 计算绝对位姿
        prt_abs_pos, prt_abs_quat = self._compute_absolute_pose(parent_pos, parent_quat, rel_pos, rel_quat)
        
        # 存储杆件绝对位姿
        self.body_abs_poses[body.name] = {
            'pos': prt_abs_pos,
            'quat': prt_abs_quat
        }
        print(f"杆件 '{body.name}' 绝对位置: {prt_abs_pos}, 姿态: {prt_abs_quat}")
        
        # 处理当前杆件的关节
        for joint in body.joint:
          rel_pos = joint.pos if joint.pos is not None else np.zeros(3)
          jnt_axis = joint.axis if joint.axis is not None else np.array([0, 0, 1])
          rel_quat = quat_from_vectors(np.array([0, 0, 1]), jnt_axis)
          jnt_abs_pos, jnt_abs_quat = self._compute_absolute_pose(prt_abs_pos, prt_abs_quat, rel_pos, rel_quat)
          jnt_type = "revolute"  # 默认关节类型
          if joint.type is None: # 旋转关节
            jnt_type = "revolute"
          elif joint.type == 'slide': # 滑动关节
            jnt_type = "slide"
          print(f"  关节 '{joint.name}': 类型={joint.type}, 位置={joint.pos}, 轴={joint.axis}")
          self.joint_abs_poses[joint.name] = {
                'type': jnt_type,
                'pos': jnt_abs_pos,
                'quat': jnt_abs_quat
            }
        
        # 递归处理子杆件
        for child in body.body:
            self._compute_body_absolute_poses(child, prt_abs_pos, prt_abs_quat)
    
    def _compute_absolute_pose(self, parent_pos, parent_quat, rel_pos, rel_quat):
        """计算杆件在世界坐标系中的绝对位置和姿态"""
        # 处理四元数顺序 (dm_control 使用 [w, x, y, z])
        parent_rot = R.from_quat(parent_quat, scalar_first=True)
        rel_rot = R.from_quat(rel_quat, scalar_first=True)
        
        # 计算绝对旋转
        abs_rot = parent_rot * rel_rot
        abs_quat = abs_rot.as_quat(scalar_first=True)  # [x, y, z, w]
        # abs_quat = [abs_quat[3], abs_quat[0], abs_quat[1], abs_quat[2]]  # 转换为 [w, x, y, z]
        
        # 计算绝对位置
        rotated_rel_pos = parent_rot.apply(rel_pos)
        abs_pos = parent_pos + rotated_rel_pos
        
        return abs_pos, np.array(abs_quat)
    
    def convert_to_sire(self, gravity=[0, 0, -9.81, 0, 0, 0]):
        self.sire_model.ground().addMarker("joint_0_k")
        self.sire_model.ground().addMarker("ground_marker")
        self.sire_model.setGravity(gravity)
        """将 MJCF 结构转换为 SIRE 格式"""
        if not self.model:
            raise RuntimeError("请先加载 MJCF 文件")
        # 转换杆件
        print(" ----- add part to aris model ------ ")
        for body in self.model.find_all('body'):
            sire_part = self._convert_body_to_sire(body)
            # self.sire_structure['parts'].append(sire_part)
        
        print(" ----- add joint to aris model ------ ")
        # 转换关节
        for joint in self.model.find_all('joint'):
            sire_joint = self._convert_joint_to_sire(joint)
            # if sire_joint:
            #     self.sire_structure['joints'].append(sire_joint)
        
        # 转换连接 (equality/connect)
        print(" ----- add connect joint to aris model ------ ")
        equality = self.model.find_all('equality')
        if equality:
            for connect in equality:
                sire_connect = self._convert_connect_to_sire(connect)
        self.sire_model.addSolvers()
                # if sire_connect:
                #     self.sire_structure['connects'].append(sire_connect)

                #  import sire
#   cs = sire.ControlServer.instance()
#   model = cs.model()
#   middleware = cs.addSireMiddleware()
#   simulator = middleware.simulationLoop()
#   simulator.setEventHandlerMap({0:6, 1:7, 2:8})
#   # simulator.addEventHandlerRule(0, 6)
#   # simulator.addEventHandlerRule(1, 7)
#   # simulator.addEventHandlerRule(2, 8)
#   physicsEngine = middleware.physicsEngine()
#   contactSolver = physicsEngine.addContactPositionForceSolver()
#   contactSolver.setDefaultProp("{k:1.4e8,d:1500,cr:0.2}")
#   contactSolver.addMaterialPair("m1", "m1", "{k:2e8,d:10000,cr:0.3,cof:0.8,threshold_velocity:1e-4}")
#   model.ground().addMarker("joint_0_k")
#   model.ground().addMarker("ground_marker")
#   boxPrt = model.addPartByPe([0,0,0.5,0,0,0], "313", [1, 0, 0, 0, 0.1, 0.1, 0.1, 0, 0, 0])
#   model.link(1).addMarker("box_center")
#   spherePos = distributeObjectOnPlane(3, 3, 4, 4, 1)
#   for i in range(9):
#     spherePrt = model.addPartByPe(spherePos[i], "313", [1, 0, 0, 0, 0.1, 0.1, 0.1, 0, 0, 0])
#     model.link(2 + i).addMarker("sphere" + str(2 + i) +"_center")
#   model.init()
#   # print(sire.toXmlString(model))
#   boxPrt.addBoxGeometry(boxPrt.id, 10, 10, 1)
#   boxPrt.cptGeometryInertial2Part()
#   physicsEngine.addBoxGeometry(10, 10, 1, boxPrt.id)
#   spherePos = distributeObjectOnPlane(3, 3, 4, 4, 1)
#   for i in range(9):
#     spherePrt = model.link(2 + i)
#     spherePrt.addSphereGeometry(spherePrt.id, 1)
#     spherePrt.cptGeometryInertial2Part()
#     physicsEngine.addSphereGeometry(1, spherePrt.id)
    
    def _convert_body_to_sire(self, body):
        """转换单个杆件到 SIRE 格式"""
        # 计算inertial
        link_iv = [0.1,0,0,0,1e-3,1e-3,1e-3,0,0,0]
        partIv = [0.1,0,0,0,1e-3,1e-3,1e-3,0,0,0]
        inertial = body.inertial if body.inertial else None
        if inertial:
            link_iv[0] = inertial.mass
            i_pos = inertial.pos if inertial.pos is not None else np.zeros(3)
            i_quat = inertial.quat if inertial.quat is not None else np.array([1, 0, 0, 0])
            inertialTF = TF.from_components(
                translation=i_pos, 
                rotation=R.from_quat(i_quat, scalar_first=True))
            if inertial.diaginertia is not None: # diagonal inertia
                link_iv[4:7] = [float(x) for x in inertial.diaginertia]
            else: # fullinertia
                link_iv[4:10] = [float(x) for x in inertial.fullinertia]
            partIv = sire.iv2iv(inertialTF.as_matrix().flatten(), link_iv)
        abs_pose = self.body_abs_poses.get(body.name, {'pos': np.zeros(3), 'quat': np.array([1, 0, 0, 0])})
        # 转换位姿为 SIRE 格式 (位置 + 欧拉角)
        # quat 全部是零转换成313的欧拉角会报如下错误：
        # UserWarning: Gimbal lock detected. Setting third angle to zero since it is not possible to uniquely determine all angles.
        sire_pose = self._format_sire_pose(abs_pose['pos'], abs_pose['quat'])

        part = self.sire_model.addPartByPe(sire_pose, "313", partIv)
        part.id = self.partId
        self.partId += 1
        self.parts[body.name] = part
        part.name = body.name
        for geom in body.geom:
            quat = geom.quat if geom.quat is not None else np.array([1, 0, 0, 0])
            pos = geom.pos if geom.pos is not None else np.zeros(3)
            geomPM = TF.from_components(
                translation=pos,
                rotation=R.from_quat(quat, scalar_first=True)).as_matrix().flatten().tolist()
            print(body.name, len(geomPM), geomPM, geom)
            if geom.type == 'mesh' or geom.mesh is not None:
                part.addMeshGeometry(part.id, "/meshes/" + str(geom.mesh.name) + ".obj", prt_pm=geomPM)
            elif geom.type == 'box':
                part.addBoxGeometry(part.id, geom.size[0]*2, geom.size[1]*2, geom.size[2]*2, prt_pm=geomPM)
            elif geom.type == 'sphere' or (geom.type is None and geom.mesh is None):
                part.addSphereGeometry(part.id, geom.size[0], prt_pm=geomPM)
            elif geom.type == 'cylinder':
                part.addCylinderGeometry(part.id, geom.size[0], geom.size[1]*2, prt_pm=geomPM)
            elif geom.type == 'capsule':
                part.addCapsuleGeometry(part.id, geom.size[0], geom.size[1]*2, prt_pm=geomPM)
        print(f"转换杆件: {body.name} -> {sire_pose}")

        # # 创建 SIRE 杆件
        # sire_part = {
        #     'name': body.name,
        #     'active': 'true',
        #     'pe': sire_pose,
        #     'inertia': '{1,0,0,0,1,1,1,0,0,0}',  # 默认惯性参数
        #     'markers': [],
        #     'geoms': []
        # }
        
        # # 添加几何信息 (简化处理)
        # for geom in body.geom:
        #     geom_info = {
        #         'type': geom.type,
        #         'mesh': getattr(geom, 'mesh', None),
        #         'size': getattr(geom, 'size', None),
        #         'rgba': getattr(geom, 'rgba', None)
        #     }
        #     sire_part['geoms'].append(geom_info)
        
    
    def _convert_joint_to_sire(self, joint):
        """转换单个关节到 SIRE 格式"""
        # 获取关节所在的杆件
        body = joint.parent
        if not body or not hasattr(body, 'name'):
            print(f"警告: 关节 '{joint.name}' 没有父杆件，跳过")
            return None
        jnt_pose = self.joint_abs_poses.get(joint.name, {'type': 'revolute', 'pos': np.zeros(3), 'quat': np.array([1, 0, 0, 0])})
        rotation = R.from_quat(jnt_pose['quat'], scalar_first=True)
        
        # 获取父杆件 (MJCF 中关节的父杆件是当前杆件的父级)
        parent_body = body.parent
        if not parent_body or not hasattr(parent_body, 'name'):
            print(f"警告: 关节 '{joint.name}' 的父杆件无效，跳过")
            return None
        print(f"转换关节: {joint.name}, 类型={jnt_pose['type']}, 位置={jnt_pose['pos']}, 父杆件={parent_body.name if parent_body else '无'}")
        if jnt_pose['type'] == 'revolute':
          sireJoint = self.sire_model.addRevoluteJoint(self.parts[body.name], self.parts[parent_body.name], jnt_pose['pos'], rotation.apply([0, 0, 1]))
        elif jnt_pose['type'] == 'slide':
          sireJoint = self.sire_model.addPrismaticJoint(self.parts[body.name], self.parts[parent_body.name], jnt_pose['pos'], rotation.apply([0, 0, 1]))
        sireJoint.name = joint.name
        sire.ActuatorSISO.add2Model(self.sire_model, sireJoint)
        # # 创建标记点名称
        # parent_marker_name = f"marker_{parent_body.name}_{body.name}_p"
        # child_marker_name = f"marker_{parent_body.name}_{body.name}_c"
        
        # # 计算标记点在各自杆件局部坐标系中的位置
        # # 子杆件上的标记点位置就是关节的相对位置
        # child_marker_pos = joint.pos if joint.pos is not None else np.zeros(3)
        
        # # 父杆件上的标记点位置需要转换到父杆件坐标系
        # joint_world_pos = self._transform_point_to_world(
        #     self.body_abs_poses[body.name]['pos'],
        #     self.body_abs_poses[body.name]['quat'],
        #     joint.pos if joint.pos is not None else np.zeros(3)
        # )
        
        # parent_marker_pos = self._transform_point_to_local(
        #     self.body_abs_poses[parent_body.name]['pos'],
        #     self.body_abs_poses[parent_body.name]['quat'],
        #     joint_world_pos
        # )
        
        # # 添加标记点到杆件
        # self._add_marker_to_body(parent_body.name, parent_marker_name, parent_marker_pos)
        # self._add_marker_to_body(body.name, child_marker_name, child_marker_pos)
        
        # # 映射关节类型
        # sire_joint_type = self._map_joint_type(joint.type)
        
        # # 创建 SIRE 关节
        # sire_joint = {
        #     'name': joint.name,
        #     'active': 'true',
        #     'type': sire_joint_type,
        #     'prt_m': body.name,  # 子杆件
        #     'prt_n': parent_body.name,  # 父杆件
        #     'mak_i': child_marker_name,  # 子杆件上的标记点
        #     'mak_j': parent_marker_name  # 父杆件上的标记点
        # }
        
        # print(f"转换关节: {joint.name} ({sire_joint_type}) "
        #       f"连接 {parent_body.name} -> {body.name}")
        # return sire_joint
    
    def _convert_connect_to_sire(self, connect):
        """转换连接(connect)到 SIRE 格式"""
        body1 = connect.body1
        body2 = connect.body2
        body1_name = body1.name
        body2_name = body2.name
        
        if not body1 or not body2:
            print(f"警告: 连接 {body1_name}-{body2_name} 的杆件未找到，跳过")
            return None
        anchor = np.array([float(x) for x in connect.anchor])
        
        body_pos = self.body_abs_poses.get(body1.name, {'pos': np.zeros(3), 'quat': np.array([1, 0, 0, 0])})
        body_tf = TF.from_components(
            translation=body_pos['pos'], 
            rotation=R.from_quat(body_pos['quat'], scalar_first=True))
        anchor_tf = TF.from_translation(anchor)
        jnt_pos = (body_tf * anchor_tf).translation.reshape(3)
        # self.sire_model.addSphericalJoint(self.parts[body1.name], self.parts[body2.name], jnt_pos)
        # # 解析锚点位置
        
        # # 创建标记点名称
        # marker1_name = f"connect_{body1_name}_{body2_name}_1"
        # marker2_name = f"connect_{body1_name}_{body2_name}_2"
        
        # # 计算标记点在各自杆件局部坐标系中的位置
        # marker1_pos = self._transform_point_to_local(
        #     self.body_abs_poses[body1_name]['pos'],
        #     self.body_abs_poses[body1_name]['quat'],
        #     anchor
        # )
        
        # marker2_pos = self._transform_point_to_local(
        #     self.body_abs_poses[body2_name]['pos'],
        #     self.body_abs_poses[body2_name]['quat'],
        #     anchor
        # )
        
        # # 添加标记点到杆件
        # self._add_marker_to_body(body1_name, marker1_name, marker1_pos)
        # self._add_marker_to_body(body2_name, marker2_name, marker2_pos)
        
        # # 创建 SIRE 连接 (使用球关节)
        # sire_connect = {
        #     'name': f"connect_{body1_name}_{body2_name}",
        #     'active': 'true',
        #     'type': 'SphericalJoint',
        #     'prt_m': body1_name,
        #     'prt_n': body2_name,
        #     'mak_i': marker1_name,
        #     'mak_j': marker2_name
        # }
        
        # print(f"转换连接: {body1_name} <-> {body2_name} 于 {anchor}")
        # return sire_connect
    
    def _add_marker_to_body(self, body_name, marker_name, position):
        """为杆件添加标记点"""
        # 查找杆件
        for part in self.sire_structure['parts']:
            if part['name'] == body_name:
                # 创建标记点 (姿态默认为0)
                marker_pose = self._format_sire_pose(position, np.array([1, 0, 0, 0]))
                part['markers'].append({
                    'name': marker_name,
                    'active': 'true',
                    'pe': marker_pose
                })
                print(f"  添加标记点 '{marker_name}' 到杆件 '{body_name}': {position}")
                return
        
        print(f"警告: 找不到杆件 '{body_name}' 来添加标记点 '{marker_name}'")
    
    def _transform_point_to_world(self, body_pos, body_quat, point):
        """将杆件局部坐标系中的点转换到世界坐标系"""
        body_rot = R.from_quat(body_quat, scalar_first=True)
        rotated_point = body_rot.apply(point)
        return body_pos + rotated_point
    
    def _transform_point_to_local(self, body_pos, body_quat, world_point):
        """将世界坐标系中的点转换到杆件局部坐标系"""
        body_rot = R.from_quat(body_quat, scalar_first=True)
        inv_rot = body_rot.inv()
        local_point = inv_rot.apply(world_point - body_pos)
        return local_point
    
    def _format_sire_pose(self, pos, quat):
        """格式化SIRE中的位姿表示：位置 + 欧拉角 (ZYX顺序)"""
        # 将四元数转换为欧拉角
        r = R.from_quat(quat, scalar_first=True)
        euler = r.as_euler('ZXZ', degrees=False)
        return [pos[0], pos[1], pos[2], euler[0], euler[1], euler[2]];
    
    def _map_joint_type(self, mjcf_joint_type):
        """映射MJCF关节类型到SIRE关节类型"""
        mapping = {
            'hinge': 'RevoluteJoint',
            'slide': 'PrismaticJoint',
            'ball': 'SphericalJoint',
            'free': 'FreeJoint'
        }
        return mapping.get(mjcf_joint_type, 'RevoluteJoint')
    
    def generate_sire_xml(self, output_path):
        """生成SIRE格式的XML文件 (概念性实现)"""
        # 在实际实现中，这里需要构建完整的XML树
        # 以下是一个简化的概念实现
        
        from xml.etree.ElementTree import Element, SubElement, tostring
        from xml.dom import minidom
        
        # 创建根元素
        root = Element('ControlServer')
        
        # 创建模型部分
        model = SubElement(root, 'Model', {
            '__prop_name__': 'model',
            'name': 'converted_model',
            'time': '0'
        })
        
        # 环境设置
        environment = SubElement(model, 'Environment', {
            '__prop_name__': 'environment',
            'gravity': "{0,0,0,0,0,0}"
        })
        
        # 变量池
        variable_pool = SubElement(model, 'VariablePoolElement', {
            '__prop_name__': 'variable_pool'
        })
        
        # 零件池
        part_pool = SubElement(model, 'PartPoolElement', {
            '__prop_name__': 'part_pool'
        })
        
        # 添加所有零件
        for part in self.sire_structure['parts']:
            part_elem = SubElement(part_pool, 'Part', {
                'name': part['name'],
                'active': part['active'],
                'pe': part['pe'],
                'inertia': part['inertia']
            })
            
            # 添加标记点池
            marker_pool = SubElement(part_elem, 'MarkerPoolElement', {
                '__prop_name__': 'marker_pool'
            })
            
            for marker in part['markers']:
                SubElement(marker_pool, 'Marker', {
                    'name': marker['name'],
                    'active': marker['active'],
                    'pe': marker['pe']
                })
            
            # 添加几何池 (简化)
            geom_pool = SubElement(part_elem, 'GeometryPoolElement', {
                '__prop_name__': 'geometry_pool'
            })
            if part['geoms']:
                # 实际实现中需要添加几何细节
                SubElement(geom_pool, 'MeshGeometry', {
                    'id': '0',
                    'part_id': '0',
                    'is_dynamic': 'true',
                    'resource_path': "/meshes/placeholder.STL",
                    'pm': "{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}"
                })
        
        # 添加关节池
        joint_pool = SubElement(model, 'JointPoolElement', {
            '__prop_name__': 'joint_pool'
        })
        
        # 添加所有关节
        for joint in self.sire_structure['joints']:
            SubElement(joint_pool, joint['type'], {
                'name': joint['name'],
                'active': joint['active'],
                'prt_m': joint['prt_m'],
                'prt_n': joint['prt_n'],
                'mak_i': joint['mak_i'],
                'mak_j': joint['mak_j']
            })
        
        # 添加所有连接 (作为球关节)
        for connect in self.sire_structure['connects']:
            SubElement(joint_pool, connect['type'], {
                'name': connect['name'],
                'active': connect['active'],
                'prt_m': connect['prt_m'],
                'prt_n': connect['prt_n'],
                'mak_i': connect['mak_i'],
                'mak_j': connect['mak_j']
            })
        
        # 添加其他必要部分 (简化)
        SubElement(model, 'MotionPoolElement', {'__prop_name__': 'motion_pool'})
        SubElement(model, 'GeneralMotionPoolElement', {'__prop_name__': 'general_motion_pool'})
        SubElement(model, 'ForcePoolElement', {'__prop_name__': 'force_pool'})
        
        # 美化XML并写入文件
        rough_string = tostring(root, 'utf-8')
        reparsed = minidom.parseString(rough_string)
        pretty_xml = reparsed.toprettyxml(indent="  ")
        
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(pretty_xml)
        
        print(f"已生成 SIRE XML 文件: {output_path}")

    def to_sire_xml(self, output_path):
        """将转换后的结构保存为 SIRE XML 文件"""
        with open(output_path, 'w+', encoding='utf-8') as f:
            f.write(sire.toXmlString(self.sire_model))
        print(f"SIRE XML 文件已保存到: {output_path}")
    
    def print_sire_xml(self):
        """打印转换后的 SIRE XML 字符串 (用于调试)"""
        xml_str = sire.toXmlString(self.sire_model)
        print(xml_str)

# 使用示例
if __name__ == "__main__":
    # 创建转换器实例
    converter = MJCFtoSIREConverter("D:/code/sire/scripts/mjcf2srdf/go2/go2.xml")
    
    # 加载和解析 MJCF 文件
    print("加载和解析 MJCF 文件...")
    converter.load_and_parse_mjcf()
    
    # 转换为 SIRE 格式
    print("\n转换为 SIRE 格式...")
    gravity = [0, 0, 0, 0, 0, 0]
    converter.convert_to_sire(gravity)
    
    # 生成 SIRE XML 文件
    print("\n生成 SIRE XML 文件...")
    converter.print_sire_xml()
    # converter.to_sire_xml("D:/code/sire/scripts/mjcf2srdf/whqMetamorphic/metamophicRobot.xml")
    
    print("\n转换完成!")