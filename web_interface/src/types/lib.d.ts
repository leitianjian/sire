export interface CellConfig {
  name: string;
  i: string;
  x: number;
  y: number;
  width: number;
  height: number;
  options: string;
  optionsJson: Object;
}

interface DashboardConfig {
  i: string;
  editable: boolean;
  cells: CellConfig[];
}

interface FetchError {
  error: string;
}

interface FetchResponse<T> {
  response: T;
}

interface SireGeometryBase {
  geometry_id: number;
}

interface SireGeometryOnPart extends SireGeometryBase {
  part_id: number;
  relative_to_part: boolean;
}

export interface SireSphereGeometry extends SireGeometryOnPart{
  shape_type: string;
  radius: number;
}

export interface SireBoxGeometry extends SireGeometryOnPart{
  shape_type: string;
  length: number;
  width: number;
  height: number;
}

export interface SireCapsuleGeometry extends SireGeometryOnPart{
  shape_type: string;
  radius: number;
  length: number;
}

export interface SireMeshGeometry extends SireGeometryOnPart{
  shape_type: string;
  resource_path: string;
}

export type SireGeometry = SireBoxGeometry | SireSphereGeometry | SireMeshGeometry;

type PositionType = [ x: number, y: number, z: number ];

type QuaternionType = [i: number, j: number, k: number, w: number];

type Matrix4Array = [
  n11: number,
  n12: number,
  n13: number,
  n14: number,
  n21: number,
  n22: number,
  n23: number,
  n24: number,
  n31: number,
  n32: number,
  n33: number,
  n34: number,
  n41: number,
  n42: number,
  n43: number,
  n44: number,
];
// declare global {
//   interface window { __REDUX_DEVTOOLS_EXTENSION_COMPOSE__: any }
// }
