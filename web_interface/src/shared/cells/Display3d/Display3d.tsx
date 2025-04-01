import React, {
  Suspense,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
} from "react";
import {
  Canvas,
  useLoader,
  useThree,
  Vector3 as Vector3Type,
  Quaternion as Vector4Type,
} from "@react-three/fiber";
import {
  ConfigConsumerProps,
  ConfigContext,
} from "antd/lib/config-provider/context";
import { RedoOutlined } from "@ant-design/icons";
import {
  CameraControls,
  PerspectiveCamera,
  View,
  Billboard,
  Text,
} from "@react-three/drei";

import { STLLoader } from "three/examples/jsm/loaders/STLLoader";
import { FBXLoader } from "three/examples/jsm/loaders/FBXLoader";
// import { BackSide, PerspectiveCamera, Plane, Vector3 } from 'three';
import {
  getGeometryPm,
  getGeometryPool,
  getViewerConfig,
} from "../../../state/selectors";
import {
  display3dInitRequest,
  sendCmdSilence,
  // deleteResultByChannel
} from "../../../state/actions";
import { shallowEqual, useDispatch, useSelector } from "react-redux";
import { RootState } from "~/state/reducers";
import { Display3dConfig } from "./types";
import sizeMe, { withSize } from "react-sizeme";
import {
  BufferGeometry,
  Group,
  Material,
  Matrix4,
  Mesh,
  MeshStandardMaterial,
  Quaternion,
  PerspectiveCamera as pc,
  Vector3,
  Color,
  ArrowHelper,
  Euler,
} from "three";

import {
  Matrix4Array,
  PositionType,
  QuaternionType,
  SireBoxGeometry,
  SireGeometry,
  SireMeshGeometry,
  SireSphereGeometry,
} from "~/types/lib";

interface Screw {
  force: Vector3;  // 前三维：力向量 (Fx, Fy, Fz)
  moment: Vector3; // 后三维：力矩向量 (Mx, My, Mz)
  position: Vector3; // 前三维：位置 (X, Y, Z)
  orientation: Quaternion; // 后三维转换为四元数的方向
}

interface ContactForce {
  contact_force: number[];
  contact_point_pe: number[];
}

interface ContactForcesProps {
  forces?: Screw;
  forceScale?: number;
  momentScale?: number;
}

interface CellProps {
  prefixCls: string;
  size: { width: number; height: number };
}
const default_material: MeshStandardMaterial = new MeshStandardMaterial({
  color: "rgb(255,255,255)",
});
const ground_material: MeshStandardMaterial = new MeshStandardMaterial({
  color: "rgb(117, 83, 56)",
});

const material_odd_part: MeshStandardMaterial = new MeshStandardMaterial({
  color: "rgb(6,96,255)",
});

const material_even_part: MeshStandardMaterial = new MeshStandardMaterial({
  color: "rgb(212,212,212)",
});

const default_scale: number = 1;

const default_position: PositionType = [0, 0, 0];
const default_quaternion: QuaternionType = [0, 0, 0, 1];
console.log("default_position", default_position);
interface ModelProps {
  path: string;
  material?: Material;
  scale?: number;
  position?: Vector3Type;
  quaternion?: Vector4Type;
}

// 欧拉角转四元数工具函数
const eulerToQuaternion = (rx: number, ry: number, rz: number) => {
  const q = new Quaternion();
  q.setFromEuler(new Euler(rx, ry, rz));
  return q;
};

// 解析6维旋量数据
const parseScrewData = (data: any = []): Screw[] => {
  // 直接展开数据（兼容 data 是 ContactForce[] 或 ContactForce[][] 的情况）
  const flatData = Array.isArray(data[0]?.contact_force_vector) ? data : data.flat();

  return flatData.flatMap((d: { contact_force_vector: (number | undefined)[]; contact_point_pe: number[]; }) => {
    // 保留原有的空值检查
    if (!d || !d.contact_force_vector || !d.contact_point_pe) {
      // console.warn("Null or invalid contact force data:", d);
      return {
        force: new Vector3(0, 0, 0),
        moment: new Vector3(0, 0, 0),
        position: new Vector3(0, 0, 0),
        orientation: new Quaternion(),
      };
    }
    // 强制解析（假设 contact_force 和 contact_point_pe 一定是 6 元素数组）
    return {
      force: new Vector3(d.contact_force_vector[0], d.contact_force_vector[1], d.contact_force_vector[2]),
      // moment: new Vector3(d.contact_force[3], d.contact_force[4], d.contact_force[5]),
      moment: new Vector3(0, 0, 0), // TODO: 这里的 moment 数据暂时不需要
      position: new Vector3(d.contact_point_pe[0], d.contact_point_pe[1], d.contact_point_pe[2]),
      orientation: eulerToQuaternion(
        d.contact_point_pe[3], 
        d.contact_point_pe[4], 
        d.contact_point_pe[5]
      ),
    };
  });
};

const ContactForces = ({ 
  forces, 
  forceScale = 0.001,
  // momentScale = 0.01 
}: ContactForcesProps) => {
  if (!forces) return null;

  // 力可视化参数
  const forceMagnitude = forces.force.length();
  const forceDirection = forces.force.clone().normalize();
  
  // 力矩可视化参数
  // const momentMagnitude = forces.moment.length();
  // const momentDirection = forces.moment.clone().normalize();

  return (
    <group 
    >
      {/* 力箭头 (红色) */}
      {forceMagnitude > 0.1 && (
        <primitive
          object={new ArrowHelper(
            forceDirection,
            forces.position,
            forceMagnitude * forceScale,
            0xff0000, // 红色
            forceMagnitude * forceScale * 0.2,
            forceMagnitude * forceScale * 0.1
          )}
        />
      )}

      {/* 力矩箭头 (蓝色) */}
      {/* {momentMagnitude > 0.1 && (
        <primitive
          object={new ArrowHelper(
            momentDirection,
            forces.position,
            momentMagnitude * momentScale,
            0x0000ff, // 蓝色
            momentMagnitude * momentScale * 0.2,
            momentMagnitude * momentScale * 0.1
          )}
        />
      )} */}

      {/* 接触点标记 (绿色球体) */}
      {forces.position.lengthSq() !== 0 && (
        <mesh position={forces.position}>
          <sphereGeometry args={[0.02, 16, 16]} />
          <meshBasicMaterial color={0x00ff00} />
        </mesh>
      )}
    </group>
  );
};


const FBXModel = ({
  path,
  material = default_material,
  scale = default_scale,
  position = default_position,
  quaternion = default_quaternion,
}: ModelProps) => {
  const fbx: Group = useMemo(() => useLoader(FBXLoader, path), [path]);
  fbx.children.forEach((mesh) => {
    (mesh as Mesh).material = material;
    (mesh as Mesh).castShadow = true;
  });

  return (
    <mesh position={position} quaternion={quaternion} scale={scale}>
      <primitive object={fbx} attach="geometry" />
    </mesh>
  );
};

const STLModel = ({
  path,
  material = default_material,
  scale = default_scale,
  position = default_position,
  quaternion = default_quaternion,
}: ModelProps) => {
  const stl: BufferGeometry = useMemo(() => useLoader(STLLoader, path), [path]);
  return (
    <mesh
      material={material}
      position={position}
      quaternion={quaternion}
      scale={scale}
      castShadow={true}
    >
      <primitive object={stl} attach="geometry" />
    </mesh>
  );
};

interface RobotMeshProps {
  poses?: number[][];
  scales?: number[];
}

// TODO: init_pm没有任何作用，建议放在内部处理，感觉发送display3d_init没有必要保存状态在全局，放在Display3d component中就可以了
function RobotMesh({ poses, scales }: RobotMeshProps) {
  const geometry_pool = useSelector<RootState, SireGeometry[] | null>(
    getGeometryPool,
    shallowEqual
  );
  const geometry_pm_data = useSelector<RootState, Matrix4Array[] | null>(
    getGeometryPm,
    shallowEqual
  );
  const evaluate_pose = useCallback(
    (
      geometry: SireGeometry,
      pose?: number[],
      geometry_pm?: Matrix4Array
    ): [PositionType | Vector3, Quaternion | QuaternionType] => {
      if (!pose) {
        return [default_position, default_quaternion];
      }
      if (!geometry.relative_to_part || !geometry_pm) {
        // directly update
        return [
          [pose[0], pose[1], pose[2]],
          [pose[3], pose[4], pose[5], pose[6]],
        ];
      } else {
        let current_position = new Vector3(pose[0], pose[1], pose[2]);
        let current_rotation = new Quaternion(
          pose[3],
          pose[4],
          pose[5],
          pose[6]
        );
        let current_scale = new Vector3(1, 1, 1);
        let current_pm = new Matrix4().compose(
          current_position,
          current_rotation,
          current_scale
        );
        let geo_pm = new Matrix4().set(...geometry_pm);
        current_pm.multiply(geo_pm);
        current_position.setFromMatrixPosition(current_pm);
        current_rotation.setFromRotationMatrix(current_pm);
        current_scale.setFromMatrixScale(current_pm);
        return [current_position, current_rotation];
      }
    },
    []
  );

  const processed_pose = useMemo(
    () =>
      geometry_pool?.map((geometry: SireGeometry, i) => {
        return evaluate_pose(geometry, poses?.at(geometry.part_id), geometry_pm_data?.at(i));
      }),
    [poses]
  );
  return (
    <>
      {geometry_pool?.map((geometry, i) => {
        let [position, quaternion]: [
          PositionType | Vector3Type,
          QuaternionType | Quaternion
        ] = [default_position, default_quaternion];
        if (processed_pose?.at(i)) {
          [position, quaternion] = processed_pose[i];
        }
        let material: MeshStandardMaterial =
          i % 2 ? material_odd_part : material_even_part;
        if (i == 0) material = ground_material;
        switch (geometry.shape_type) {
          case "sphere":
            return (
              <mesh
                material={material}
                position={position}
                quaternion={quaternion}
                castShadow={true}
                receiveShadow={i == 0}
              >
                <sphereGeometry
                  args={[(geometry as SireSphereGeometry).radius]}
                />
              </mesh>
            );
          case "box":
            return (
              <mesh
                material={material}
                position={position}
                quaternion={quaternion}
                castShadow={true}
                receiveShadow={i == 0}
              >
                <boxGeometry
                  args={[
                    (geometry as SireBoxGeometry).length,
                    (geometry as SireBoxGeometry).width,
                    (geometry as SireBoxGeometry).height,
                  ]}
                />
              </mesh>
            );
          case "mesh":
            return (
              <FBXModel
                path={(geometry as SireMeshGeometry).resource_path}
                position={position}
                quaternion={quaternion}
                material={material}
                scale={scales?.at(i)}
              />
            );
          default:
            console.log("unrecoginized geometry!");
            break;
        }
        return;
      })}
    </>
  );
}

const DefaultConfig: Display3dConfig = {
  frame_rate: 30,
};

function Loading({ prefixCls }: { prefixCls: string }) {
  return <div className={`${prefixCls}-loading`}>载入中...</div>;
}

const makeLight = () => {
  return (
    <>
      <hemisphereLight
        position={[0, 0, 200]}
        color={0x808080}
        groundColor={0x444444}
        intensity={8}
      />
      <pointLight
        position={[10, 10, 20]}
        color={0xffffff}
        castShadow={true}
        intensity={1000}
      />
      <ambientLight color={0x808080} intensity={9} />
    </>
  );
};

const makeBackground = () => {
  return (
    <>
      <color attach="background" args={[0x000000]} />
      {/* <fog attach="fog" color={0x000000} near={2} far={10} /> */}
    </>
  );
};

const Display3d = (props: CellProps) => {
  const dispatch = useDispatch();
  const config = useSelector<RootState, Display3dConfig>(
    (state) => getViewerConfig(state, DefaultConfig),
    shallowEqual
  );

  const [location_update_per_second, set_LUPS] = useState<number>(0);
  const update_location_count = useRef<number>(0);

  const prev_update_location_time = useRef<number>(performance.now());
  const [pose, setPose] = useState<number[][]>();

  const camera_control_robot = useRef<CameraControls | null>(null);
  const ctrlRef = useRef<CameraControls | null>(null);
  const cameraRef = useRef<pc | null>(null);
  const [camPos, setCamPos] = useState<Vector3>(new Vector3());
  const [ctrlTarget, setCtrlTarget] = useState<Vector3>(new Vector3());

  const [timeIndices, setTimeIndices] = useState<number[]>([]);
  const [totalDuration, setTotalDuration] = useState(0);
  const [uploadedData, setUploadedData] = useState<number[][][]>([]);
  const [currentFrameIndex, setCurrentFrameIndex] = useState(0);
  const [isPlaying, setIsPlaying] = useState(false);
  const playbackInterval = useRef<number | null>(null);

  const [recording, setRecording] = useState(false);
  const mediaRecorderRef = useRef<MediaRecorder | null>(null);
  const recordedChunksRef = useRef<Blob[]>([]);
  const [canvasElement, setCanvasElement] = useState<HTMLCanvasElement | null>(null);
  const [contactForces, setContactForces] = useState<ContactForce[]>([]);
  const screws = useMemo(() => parseScrewData(contactForces), [contactForces]);
  const CanvasCapturer = () => {
    const { gl } = useThree();
    useEffect(() => {
      // 通过状态更新传递canvas引用
      setCanvasElement(gl.domElement);
    }, [gl.domElement]);
    return null;
  };

  const handleRecord = async () => {
    if (!canvasElement) {
      console.error("Canvas元素未就绪");
      return;
    }

    if (!recording) {
      try {
        recordedChunksRef.current = [];
        const stream = canvasElement.captureStream(30);
        
        mediaRecorderRef.current = new MediaRecorder(stream, {
          mimeType: 'video/webm;codecs=vp9',
          videoBitsPerSecond: 2_500_000
        });

        mediaRecorderRef.current.ondataavailable = (e) => {
          if (e.data.size > 0) recordedChunksRef.current.push(e.data);
        };

        mediaRecorderRef.current.onstop = () => {
          const blob = new Blob(recordedChunksRef.current, { type: 'video/webm' });
          const url = URL.createObjectURL(blob);
          const a = document.createElement('a');
          a.href = url;
          a.download = `recording-${Date.now()}.webm`;
          a.click();
          URL.revokeObjectURL(url);
          recordedChunksRef.current = [];
        };

        mediaRecorderRef.current.start(100);
        setRecording(true);
      } catch (error) {
        console.error("录屏启动失败:", error);
        setRecording(false);
      }
    } else {
      mediaRecorderRef.current?.stop();
      setRecording(false);
    }
  };

  const [fileName, setFileName] = useState<string>("📁 上传JSON文件");

  // 文件上传处理
  const handleFileUpload = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = (event) => {
      try {
        const data = JSON.parse(event.target?.result as string);
        if (!data.partpq || !data.timeindex) {
          alert("Invalid JSON format!");
          // throw new Error("Invalid JSON format");
          return;
        }
        setFileName(file.name);
        // 提取 partpq 和 timeindex
        setUploadedData(data.partpq);
        setTimeIndices(data.timeindex);
        setTotalDuration(data.timeindex[data.timeindex.length - 1] || 0);

        setCurrentFrameIndex(0);
        setIsPlaying(false);
        setContactForces(data.contact_info);
        if (playbackInterval.current) {
          clearInterval(playbackInterval.current);
        }
      } catch (error) {
        console.error("Invalid JSON file");
      }
    };
    reader.readAsText(file);
  };

  const handleFileFromDrop = (file: File) => {
    setFileName(file.name);
    const reader = new FileReader();
    reader.onload = (event) => {
      try {
        const data = JSON.parse(event.target?.result as string);
        if (!data.partpq || !data.timeindex) {
          throw new Error("Invalid JSON format");
        }
  
        setUploadedData(data.partpq);
        setTimeIndices(data.timeindex);
        setTotalDuration(data.timeindex[data.timeindex.length - 1] || 0);
        setCurrentFrameIndex(0);
        setIsPlaying(false);
        setContactForces(data.contact_info);
        if (playbackInterval.current) clearInterval(playbackInterval.current);
      } catch (error) {
        console.error("Invalid JSON file");
      }
    };
    reader.readAsText(file);
  };

  // 播放控制
  const togglePlayback = () => {
    setIsPlaying(!isPlaying);
  };

  const binarySearch = (arr: number[], target: number): number => {
    let low = 0;
    let high = arr.length - 1;

    while (low <= high) {
      const mid = Math.floor((low + high) / 2);
      if (arr[mid] === target) return mid;
      arr[mid] < target ? (low = mid + 1) : (high = mid - 1);
    }

    return Math.min(low, arr.length - 1);
  };

  // 播放逻辑
  useEffect(() => {
    if (!isPlaying || uploadedData.length === 0) return;

    let startTime = performance.now();
    let currentTime = 0;

    const animate = () => {
      const elapsed = performance.now() - startTime;
      currentTime = Math.min(elapsed / 1000, totalDuration);

      // 查找当前时间对应的帧索引
      const newIndex = binarySearch(timeIndices, currentTime);
      setCurrentFrameIndex(newIndex);

      if (currentTime < totalDuration) {
        animationFrameId = requestAnimationFrame(animate);
      } else {
        setIsPlaying(false);
      }
    };

    let animationFrameId = requestAnimationFrame(animate);

    return () => cancelAnimationFrame(animationFrameId);
  }, [isPlaying, uploadedData, totalDuration, timeIndices]);

  // 原有数据获取逻辑（添加上传数据判断）
  useEffect(() => {
    if (uploadedData.length === 0) {
      console.log("send display3d init");
      dispatch(display3dInitRequest());

      const timer = setInterval(() => {
        dispatch(
          sendCmdSilence("get --part_pq", (msg: any) => {
            if (msg?.jsData?.return_code === 0) {
              setPose(msg.jsData.part_pq);
              set_LUPS((prev) => {
                const now = performance.now();
                if (now >= prev_update_location_time.current + 500) {
                  const value = Math.round(
                    (update_location_count.current * 1000) /
                      (now - prev_update_location_time.current)
                  );
                  prev_update_location_time.current = now;
                  update_location_count.current = 0;
                  return value;
                }
                update_location_count.current++;
                return prev;
              });
            }
          })
        );
      }, 1000 / config.frame_rate);

      return () => clearInterval(timer);
    }
  }, [uploadedData.length]);

  const { getPrefixCls, rootPrefixCls } =
    useContext<ConfigConsumerProps>(ConfigContext);
  const prefixCls = getPrefixCls("display3d", rootPrefixCls);
  const eventSrcRef = useRef<any>();
  return (
    <div className={`${prefixCls}`}>
      <Suspense fallback={<Loading prefixCls={prefixCls} />}>
        <div ref={eventSrcRef} className={`${prefixCls}-three`}>
          {/* 新增控制面板 */}
          <div className={`${prefixCls}-controls`}>
            <div className={`${prefixCls}-upload-wrapper`}>
              <input
                type="file"
                id="fileUpload"
                accept=".json"
                onChange={handleFileUpload}
                style={{ display: "none" }}
              />
              <label
                htmlFor="fileUpload"
                className={`${prefixCls}-upload-button`}
                onDragOver={(e) => e.preventDefault()}
                onDrop={(e) => {
                  e.preventDefault();
                  const file = e.dataTransfer.files?.[0];
                  if (file && file.type === "application/json") {
                    handleFileFromDrop(file);  // 调用统一处理逻辑
                  } else {
                    alert("请上传 .json 文件");
                  }
                }}
              >
                {fileName}
              </label>
              {/* {fileName && <span className={`${prefixCls}-upload-filename`}>{fileName}</span>} */}
            </div>
            <div className={`${prefixCls}-progress-wrapper`}>
              <div className={`${prefixCls}-progress-main`}>
                {/* 播放按钮 */}
                <button 
                  className={`${prefixCls}-play-button`} 
                  onClick={togglePlayback}
                >
                  {isPlaying ? "暂停" : "播放"}
                </button>

                <div className={`${prefixCls}-progress-group`}>
                  {/* 帧进度条 + 信息 */}
                  <input 
                    type="range"
                    className={`${prefixCls}-progress-frame`}
                    min="0"
                    max={uploadedData.length - 1}
                    value={currentFrameIndex}
                    onChange={(e) => {
                      const index = parseInt(e.target.value);
                      setCurrentFrameIndex(isNaN(index) ? 0 : index);
                    }}
                    disabled={!uploadedData.length}
                  />
                  <div className={`${prefixCls}-progress-info`}>
                    帧: {currentFrameIndex + 1}/{uploadedData.length}
                  </div>

                  {/* 时间进度条 + 信息 */}
                  <input
                    type="range"
                    className={`${prefixCls}-progress-time`}
                    min="0"
                    max={totalDuration * 1000}
                    value={timeIndices[currentFrameIndex] * 1000 || 0}
                    onChange={(e) => {
                      const targetTime = parseInt(e.target.value) / 1000;
                      const newIndex = binarySearch(timeIndices, targetTime);
                      setCurrentFrameIndex(newIndex);
                    }}
                    disabled={!uploadedData.length}
                  />
                  <div className={`${prefixCls}-progress-info`}>
                    时间: {timeIndices[currentFrameIndex]?.toFixed(3) || 0}s / {totalDuration.toFixed(3)}s
                  </div>
                </div>
              </div>
            </div>
            <button 
              onClick={handleRecord}
              className={`
                ${prefixCls}-record-button
                ${recording ? `${prefixCls}-recording` : ''}
              `}
            >
              {recording ? '停止录制' : '开始录制'}
            </button>
          </div>
          <RedoOutlined
            onClick={() => {
              camera_control_robot.current?.reset(true);
            }}
          />
          <div className={`${prefixCls}-three-info`}>
            {/* <div className={`${prefixCls}-three-info-fps`}>fps: {this.state.fps}</div> */}
            <div className={`${prefixCls}-three-info-lups`}>
              LUPS: {location_update_per_second}
            </div>
          </div>
          <View index={1} className={`${prefixCls}-three-robot`}>
            <PerspectiveCamera
              ref={cameraRef}
              position={[-2.5, 0, 5]}
              makeDefault
              fov={50}
              near={0.1}
            />
            <CameraControls
              ref={ctrlRef}
              onChange={() => {
                let camPosition = new Vector3();
                let targetPosition = new Vector3();
                // console.log(cameraRef.current?.position);
                ctrlRef.current?.getPosition(camPosition);
                // let camPosition = cameraRef.current?.position || new Vector3;
                // cameraRef.current?.get(camPosition);
                ctrlRef.current?.getTarget(targetPosition);
                setCamPos(camPosition);
                setCtrlTarget(targetPosition);
                // console.log(camPosition);
                // console.log(targetPosition);
              }}
            />
            {makeLight()}
            {makeBackground()}
            {/* <gridHelper rotation={[Math.PI / 2, 0, 0]} scale={}>
              <lineBasicMaterial
                opacity={0.3}
                depthWrite={false}
                transparent={true}
              />
            </gridHelper> */}
            <RobotMesh
              poses={
                uploadedData.length > 0 ? uploadedData[currentFrameIndex] : pose
              }
            />
            <ContactForces 
              forces={screws[currentFrameIndex]}
              // frameIndex={currentFrameIndex}
              forceScale={0.0005} 
              momentScale={0.0002}
            />
          </View>
          <View index={2} className={`${prefixCls}-three-axes`}>
            <Billboard
              position={[1.2, 0, 0]}
              follow={true}
              lockX={false}
              lockY={false}
              lockZ={false} // Lock the rotation on the z axis (default=false)
            >
              <Text fontSize={0.5}>x</Text>
            </Billboard>
            <Billboard
              position={[0, 1.2, 0]}
              follow={true}
              lockX={false}
              lockY={false}
              lockZ={false} // Lock the rotation on the z axis (default=false)
            >
              <Text fontSize={0.5}>y</Text>
            </Billboard>
            <Billboard
              position={[0, 0, 1.2]}
              follow={true}
              lockX={false}
              lockY={false}
              lockZ={false} // Lock the rotation on the z axis (default=false)
            >
              <Text fontSize={0.5}>z</Text>
            </Billboard>
            <axesHelper scale={1} />
            <PerspectiveCamera
              makeDefault
              position={camPos.sub(ctrlTarget).setLength(5)}
              onUpdate={(self) => {
                self.lookAt(0, 0, 0);
              }}
            />
            {makeLight()}
            {makeBackground()}
          </View>
          <Canvas
            eventSource={eventSrcRef}
            camera={{
              fov: 50,
              near: 0.1
            }}
            gl={{
              antialias: true,
              autoClearColor: false,
              alpha: false,
              powerPreference: "high-performance",
            }}
            shadows={"soft"}
          >
            <CanvasCapturer />
            <CameraControls ref={camera_control_robot} />
            <View.Port />
          </Canvas>
        </div>
      </Suspense>
    </div>
  );
};

const sizedDisplay = withSize({ monitorHeight: true, refreshRate: 30 })(
  Display3d
) as React.ComponentType<Omit<CellProps, "size"> & sizeMe.WithSizeProps> & {
  NAME: string;
};
sizedDisplay.NAME = "仿真可视化";
export default sizedDisplay;
