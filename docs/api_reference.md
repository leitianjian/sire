# 场景资产与 RGB-D 相机 API

## `sire.scene.SceneAsset`

`SceneAsset` 从一份 JSON 读取静态场景的外观、深度和碰撞资产。资产共享一套局部坐标系，使用同一个位姿和等比缩放变换到仿真世界坐标系。

```python
SceneAsset.load(path: str | Path) -> SceneAsset
SceneAsset.asset_to_world: np.ndarray
SceneAsset.replace_collision(simulator, geometry_id: int) -> MeshCollisionGeometry
```

| 参数或字段 | 默认值与含义 |
| --- | --- |
| `path` | 场景 JSON 路径。JSON 内的相对路径以该文件所在目录为基准。 |
| `depth_mesh` | 必填。用于生成深度的三角网格，支持 PLY 或 OBJ。 |
| `collision_mesh` | 可选。用于碰撞的网格，支持 PLY、OBJ 或 STL。省略时使用 `depth_mesh`。 |
| `gaussians` | 可选。用于渲染外观的 3DGS PLY。启用 3DGS 渲染时必填。 |
| `position` | 默认 `[0, 0, 0]`。资产原点在世界坐标系中的位置，单位为米。 |
| `rotation_wxyz` | 默认 `[1, 0, 0, 0]`。资产坐标系到世界坐标系的旋转四元数，按 WXYZ 排列。 |
| `scale` | 默认 `1`。正数，表示每个资产单位对应的米数。 |
| `sh_degree` | 可选。渲染使用的最高球谐阶数，省略时由资产推断。 |
| `radius_clip` | 默认 `0`。3DGS 渲染的屏幕空间半径阈值，单位为像素。 |
| `simulator` | 已加载模型、尚未调用 `Simulator.init()` 的实例。 |
| `geometry_id` | 要替换的世界坐标系静态碰撞体 ID。 |

`load` 返回解析后的 `SceneAsset`，其中资产路径为绝对路径。`asset_to_world` 返回 `[4, 4]` NumPy 矩阵，左上角为 `scale * R`，右上角为平移向量。`replace_collision` 返回替换后的 `MeshCollisionGeometry`，保留原碰撞体的 ID 和接触材料设置。

```python
import sire
from sire.scene import SceneAsset

asset = SceneAsset.load("room.json")
sim = sire.Simulator()
sire.fromXmlFile(sim, "robot.xml")
asset.replace_collision(sim, geometry_id=0)  # 本例中地面碰撞体的 ID
sim.init()
```

## `sire.scene_import.import_gs_real2sim`

将 GS-Real2Sim 的静态对象导出登记为 Sire 场景。接口引用原始文件，通过调用方给出的标定位姿对齐对象。

```python
import_gs_real2sim(
    output_dir: str | Path,
    object_name: str,
    destination: str | Path,
    *,
    position=(0, 0, 0),
    rotation_wxyz=(1, 0, 0, 0),
    scale=1,
) -> Path
```

| 参数 | 默认值与含义 |
| --- | --- |
| `output_dir` | GS-Real2Sim 导出根目录，其中应有 `3d_assets/`。 |
| `object_name` | 对象名，匹配 `3d_assets/<object_name>.obj` 和 `3d_assets/<object_name>.ply`。 |
| `destination` | 要写入的场景 JSON 路径。 |
| `position` | 默认 `(0, 0, 0)`。标定后的世界坐标位置，单位为米。 |
| `rotation_wxyz` | 默认 `(1, 0, 0, 0)`。标定后的资产到世界旋转，按 WXYZ 排列。 |
| `scale` | 默认 `1`。每个资产单位对应的米数。 |

返回场景 JSON 的绝对 `Path`。`gaussians` 相对引用原始 PLY，`depth_mesh` 和 `collision_mesh` 相对引用同一个 OBJ。

```python
from sire.scene_import import import_gs_real2sim

scene_path = import_gs_real2sim(
    "exports", "chair", "scenes/chair.json", position=(1.2, 0.4, 0.0)
)
```

生成导出后，在 Sire 的 Python 环境运行。[GS-Real2Sim 导出说明](https://github.com/discoverse-dev/gs-real2sim)

```bash
python -m sire.scene_import gs-real2sim exports chair scenes/chair.json --position 1.2 0.4 0
```

## `sire.scene_import.fuse_rgbd`

使用 Open3D 的 TSDF 将已标定的 RGB-D 帧融合成世界坐标系中的三角网格。可将外部拟合的 3DGS PLY 一并登记到场景，并要求 PLY 已与网格处于同一米制世界坐标系。

```python
fuse_rgbd(
    capture_path: str | Path,
    destination: str | Path,
    *,
    gaussians: str | Path | None = None,
    voxel_size: float = 0.02,
    sdf_trunc: float = 0.06,
    depth_trunc: float = 5.0,
) -> Path
```

| 参数 | 默认值与含义 |
| --- | --- |
| `capture_path` | 符合下表格式的 NPZ 路径。 |
| `destination` | 要写入的场景 JSON 路径。 |
| `gaussians` | 默认 `None`。已对齐的外部 3DGS PLY 路径。 |
| `voxel_size` | 默认 `0.02` 米。TSDF 体素边长。 |
| `sdf_trunc` | 默认 `0.06` 米。符号距离的截断阈值。 |
| `depth_trunc` | 默认 `5.0` 米。仅融合小于此值的有效深度。 |

| NPZ 数组 | 格式 |
| --- | --- |
| `depth` | `[T, H, W]` 浮点数组，沿光轴的深度，单位为米。非有限值、零和负值均视为无效。 |
| `camera_to_world` | `[T, 4, 4]` 刚性变换矩阵。输入相机采用 OpenCV 坐标约定，+X 向右、+Y 向下、+Z 向前。 |
| `intrinsics` | `[4]` 或 `[T, 4]`，按 `(fx, fy, cx, cy)` 排列，单位为像素。 |
| `rgb` | 可选的 `[T, H, W, 3]` `uint8` RGB 数组。提供时融合顶点颜色。 |

返回场景 JSON 的绝对 `Path`。输出网格与 JSON 同目录，命名为 `<JSON 主文件名>.mesh.ply`。JSON 的 `depth_mesh` 和 `collision_mesh` 均指向该网格，资产位姿默认为单位变换。

```python
from sire.scene_import import fuse_rgbd

scene_path = fuse_rgbd("captures/room.npz", "scenes/room.json", voxel_size=0.03)
```

```bash
python -m pip install open3d
python -m sire.scene_import fuse-rgbd captures/room.npz scenes/room.json --voxel-size 0.03
```

## `sire.scene_import.simplify_collision`

按目标面数简化碰撞网格，保留资产原坐标系。输出文件可填入场景的 `collision_mesh`。

```python
simplify_collision(mesh_path: str | Path, destination: str | Path, *, target_triangles: int) -> Path
```

| 参数 | 含义 |
| --- | --- |
| `mesh_path` | Open3D 可读的原始三角网格。 |
| `destination` | 独立的 PLY 输出路径。 |
| `target_triangles` | 正整数，简化后的目标三角面数量。 |

返回生成的碰撞网格的绝对 `Path`。

```bash
python -m sire.scene_import simplify-collision room.mesh.ply room.collision.ply --target-triangles 150000
```

## `sire.sensor.CameraConfig` 与 `Camera`

`CameraConfig` 定义光学参数和相机安装位姿。`Camera` 通过渲染回调采集单个相机，支持按仿真时间采样或立即采集。相机局部坐标采用 +X 向右、+Y 向上、−Z 向前，图像以左上角为原点，像素中心位于半整数坐标。

```python
CameraConfig(
    width=640, height=480, fps=30.0, fov_y=60.0, intrinsics=None,
    near=0.05, far=20.0, parent=None, local_pose=...,
)
Camera(config, render, *, deferred_render=None)
Camera.capture(sim_time, body_poses=None) -> CameraFrame
Camera.update(sim_time, body_poses=None) -> CameraFrame | None
Camera.update_deferred(sim_time, body_poses=None) -> bool
Camera.read() -> CameraFrame | None
Camera.reset() -> None
```

| 参数或属性 | 默认值与含义 |
| --- | --- |
| `width`、`height` | 默认 `640`、`480` 像素。 |
| `fps` | 默认 `30.0` Hz，按仿真时间采样。 |
| `fov_y`、`intrinsics` | 默认垂直视野 `60.0` 度。显式内参 `(fx, fy, cx, cy)` 优先，单位为像素。`config.pinhole` 返回最终使用的内参。 |
| `near`、`far` | 默认 `0.05`、`20.0` 米，沿光轴的裁剪距离。 |
| `parent`、`local_pose` | 默认 `None` 和 `np.eye(4)`。指定刚体名时，`local_pose` 为相机到刚体的 `[4, 4]` 位姿。否则为相机到世界的位姿。 |
| `render` | 接收 `(config, world_pose)` 并返回 `(rgb, depth, valid)` 的函数。 |
| `deferred_render` | 默认 `None`。接收相同参数，返回由 `read()` 调用的无参读回函数。`update_deferred()` 要求提供此回调。 |
| `sim_time`、`body_poses` | 仿真时间（秒）和刚体名到 `[4, 4]` 世界位姿的映射。配置了 `parent` 时须提供该刚体的位姿。 |

`CameraFrame` 包含 `rgb`（`uint8 [H, W, 3]`）、`depth`（`float32 [H, W]`，沿光轴的米制深度）、`valid`（布尔型 `[H, W]`）、`sim_time` 和 `frame_id`。`capture()` 立即返回一帧。`update()` 在采样到期时返回一帧，否则返回 `None`。`update_deferred()` 返回本次是否采样。`read()` 返回最新一帧，尚无帧时返回 `None`。`reset()` 清空缓存并重置采样时钟和从零开始的帧编号。

```python
from sire.sensor import Camera, CameraConfig

config = CameraConfig(320, 240, fps=15, parent="base")
camera = Camera(config, lambda optics, pose: renderer.capture(body_poses, optics, pose))
frame = camera.capture(sim_time, body_poses)
depth_m = frame.depth[frame.valid]
```

## `sire.sensor.BatchCamera`

`BatchCamera` 在 CUDA 上为每个环境采集一份 RGB-D 图像。同一个实例的所有环境共享场景拓扑和相机配置。多组相机可分别创建实例，各自指定内参或安装位姿，并从同一个批量位姿张量采集。

```python
BatchCamera(scene, config, num_envs, *, device="cuda:0", chunk_size=64, scene_asset=None)
BatchCamera.capture(poses) -> BatchCameraFrame
BatchCamera.close() -> None
```

| 参数 | 默认值与含义 |
| --- | --- |
| `scene` | `SireScene`，提供所有环境共享的渲染网格和刚体顺序。环境之间可有不同刚体位姿。 |
| `config` | 共享的 `CameraConfig`。调用方决定 `capture()` 的时间，`fps` 用于单相机的采样时钟。 |
| `num_envs` | 固定批大小 `N`，为正整数。 |
| `device`、`chunk_size` | 默认 `"cuda:0"` 和 `64`，指定 CUDA 设备及每批渲染分块的环境数上限。 |
| `scene_asset` | 默认 `None`。接受 `SceneAsset` 或场景 JSON 路径。添加 `depth_mesh` 并在提供 `gaussians` 时合成静态 3DGS 外观。所有环境共享资产和变换。同设备、同资产路径的相机组共享高斯数据。碰撞由仿真环境配置决定。 |
| `poses` | 同一 CUDA 设备上连续存储的 `float64 [N, B, 3, 4]` 位姿张量，`B` 为刚体数。矩阵按行存储，刚体顺序与模型一致。 |

`capture()` 返回 `BatchCameraFrame`，其中 `rgb` 为 `uint8 [N, H, W, 3]`，`depth` 为 `float32 [N, H, W]`，`valid` 为布尔型 `[N, H, W]`，三个张量均在 CUDA 上。深度沿光轴测量，单位为米。采集使用当前 PyTorch CUDA 流，跨流使用时须同步。每次采集复用同一组张量，需要保留帧时应在下次采集前克隆。可使用 `with` 自动调用 `close()` 释放渲染器。

```python
from sire.sensor import BatchCamera, CameraConfig

front_config = CameraConfig(320, 240, parent="base", local_pose=front_mount)
wide_config = CameraConfig(320, 240, fov_y=100, parent="base", local_pose=wide_mount)
with BatchCamera(scene, front_config, poses.shape[0], scene_asset="room.json") as front:
    with BatchCamera(scene, wide_config, poses.shape[0], scene_asset="room.json") as wide:
        front_rgb = front.capture(poses).rgb.clone()
        wide_depth = wide.capture(poses).depth.clone()
```

## `sire.viewer.photoreal.PhotorealRenderer`

在 GLFW 或 EGL 的当前 OpenGL 上下文中渲染单个真实感 RGB-D 视图。可与 `Viewer` 或 `Camera` 配合。

```python
PhotorealRenderer(items, scene_path, *, device="cuda:0")
PhotorealRenderer.capture(poses, config, world_pose) -> tuple
PhotorealRenderer.close() -> None
```

| 参数 | 默认值与含义 |
| --- | --- |
| `items` | `SireScene.items` 或一组 `RenderItem`。已有场景深度网格由 `scene_path` 加载。 |
| `scene_path` | 含 `gaussians` 和 `depth_mesh` 的场景 JSON 路径。 |
| `device` | 默认 `"cuda:0"`，用于 3DGS 光栅化。 |
| `poses` | 刚体名到世界位姿 `[4, 4]` 的映射。 |
| `config`、`world_pose` | `CameraConfig` 和相机到世界坐标系的 `[4, 4]` 位姿。 |

`capture()` 返回 RGB、米制光轴深度和有效像素掩码的 NumPy 数组。退出图形上下文前调用 `close()`。

```python
from sire.viewer import graphics_context
from sire.viewer.photoreal import PhotorealRenderer

with graphics_context(headless=True):
    renderer = PhotorealRenderer(scene.items, "room.json")
    try:
        rgb, depth, valid = renderer.capture(scene.poses(), optics, camera_to_world)
    finally:
        renderer.close()
```
