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
  Vector3 as Vector3Type,
  Quaternion as Vector4Type,
} from "@react-three/fiber";
import {
  ConfigConsumerProps,
  ConfigContext,
} from "antd/lib/config-provider/context";
import { RedoOutlined } from "@ant-design/icons";
import { CameraControls } from "@react-three/drei";

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
import sizeMe, { withSize } from "react-sizeme";
import {
  BufferGeometry,
  Group,
  LoadingManager,
  Material,
  Matrix4,
  Mesh,
  MeshStandardMaterial,
  Quaternion,
  Vector3,
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
import { useMeshcat, testMeshcatCommand } from "./useMeshcat";
import { Viewer } from "./meshcat";

interface CellProps {
  prefixCls: string;
  size: { width: number; height: number };
}
const default_material: MeshStandardMaterial = new MeshStandardMaterial({
  color: "rgb(255,255,255)",
  roughness: 0.1,
  metalness: 0.1,
});

const material_odd_part: MeshStandardMaterial = new MeshStandardMaterial({
  color: "rgb(6,96,255)",
  roughness: 0.1,
  metalness: 0.1,
});

const material_even_part: MeshStandardMaterial = new MeshStandardMaterial({
  color: "rgb(212,212,212)",
  roughness: 0.1,
  metalness: 0.1,
});

const default_scale: number = 1;

const default_position: PositionType = [0, 0, 0];
const default_quaternion: QuaternionType = [0, 0, 0, 1];

interface ModelProps {
  path: string;
  material?: Material;
  scale?: number;
  position?: Vector3Type;
  quaternion?: Vector4Type;
}

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
        return evaluate_pose(geometry, poses?.at(i), geometry_pm_data?.at(i));
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
        const material: MeshStandardMaterial =
          i % 2 ? material_odd_part : material_even_part;
        switch (geometry.shape_type) {
          case "sphere":
            return (
              <mesh
                material={material}
                position={position}
                quaternion={quaternion}
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

function Loading({ prefixCls }: { prefixCls: string }) {
  return <div className={`${prefixCls}-loading`}>载入中...</div>;
}

const MeshcatDisplay = (props: CellProps) => {
  const { getPrefixCls, rootPrefixCls } =
    useContext<ConfigConsumerProps>(ConfigContext);
  const prefixCls = getPrefixCls("meshcatDisplay", rootPrefixCls);
  const ref = useRef<HTMLCanvasElement>(null);
  let viewer = useRef<Viewer | null>(null);
  useEffect(() => {
    if (!ref.current) return;
    viewer.current = useMeshcat(ref.current);
    viewer.current = testMeshcatCommand(ref.current);
    return () => {};
  }, []);
  useEffect(() => {
    if (!viewer.current) return;
    const width = props.size.width;
    const height = props.size.height;
    viewer.current.set_3d_pane_size(width, height);
  }, [props.size.width, props.size.height]);

  return (
    <div className={`${prefixCls}`}>
      <Suspense fallback={<Loading prefixCls={prefixCls} />}>
        <div className={`${prefixCls}-three`}>
          <canvas
            ref={ref}
            id={"meshcat-pane"}
            className={`${prefixCls}-three`}
          ></canvas>
        </div>
      </Suspense>
    </div>
  );
};

const meshcatDisplay = withSize({ monitorHeight: true, refreshRate: 30 })(
  MeshcatDisplay
) as React.ComponentType<Omit<CellProps, "size"> & sizeMe.WithSizeProps> & {
  NAME: string;
};
meshcatDisplay.NAME = "Meshcat仿真可视化";
export default meshcatDisplay;
