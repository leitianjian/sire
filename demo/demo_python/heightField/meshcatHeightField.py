import numpy as np
from PIL import Image
from meshcat.geometry import TriangularMeshGeometry, MeshPhongMaterial, Mesh

def buildHFieldMeshHeights(width, depth, nrow, ncol, heights):
    np.set_printoptions(precision=5, linewidth=np.inf, suppress=True, threshold=np.inf)
    print(nrow, ncol, heights)
    # 生成顶点
    x_vals = np.linspace(-width / 2, width / 2, ncol)
    y_vals = np.linspace(depth/2, -depth/2, nrow)
    xx, yy = np.meshgrid(x_vals, y_vals)
    zz = np.array(heights).reshape(nrow, ncol)

    vertices = np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=1).astype(np.float32)

    # 生成三角面索引
    faces = []
    for r in range(nrow - 1):
        for c in range(ncol - 1):
            a = r * ncol + c
            b = a + 1
            c_idx = a + ncol
            d = c_idx + 1
            faces.append([a, b, d])
            faces.append([a, d, c_idx])
    faces = np.array(faces, dtype=np.uint32)

    geom = TriangularMeshGeometry(vertices, faces)

    # 反射材质（可选，你可以自由调整）
    material = MeshPhongMaterial(
        color=0xd4af37,
        reflectivity=0.7,
        specular=0x111111,
        shininess=30,
    )
    return Mesh(geom, material)

def buildHFieldMeshPNG(png_path, width, depth, scale_z):
    """
    按照 MuJoCo 的 hfield 归一化方式从 PNG 生成三角网格。

    png_path  : MuJoCo 输出的高度场 PNG
    width     : 地形全长（X 方向，米）
    depth     : 地形全长（Y 方向，米）
    scale_z   : 高度缩放（对应 MuJoCo geom.size[2]）
    pos_z     : 垂直偏移（对应 MuJoCo geom.pos.z）
    """
    # 1. 读取 PNG 并转换为 [0, 1]（MuJoCo 用红色通道或灰度，256 或 65535）
    img = Image.open(png_path)
    if img.mode in ('I', 'I;16'):
        # 16 位灰度
        pixels = np.array(img, dtype=np.float64) / 65535.0
    elif img.mode == 'L':
        pixels = np.array(img, dtype=np.float64) / 255.0
    else:
        # RGB/RGBA -> 取红色通道
        pixels = np.array(img.convert('RGB'))[:, :, 0].astype(np.float64)

    # 2. MuJoCo 风格的 min‑max 归一化到 [0, 1]
    emin = pixels.min()
    emax = pixels.max()
    if emin > emax:
        raise ValueError("Invalid height field data: min > max")
    pixels -= emin
    if emax - emin > 1e-10:  # MuJoCo 中用的是 mjEPS ≈ 1e-10
        pixels /= (emax - emin)
    print(emax, emin)
    rows, cols = pixels.shape

    # 3. 生成顶点（局部坐标，中心在 origin）
    x_vals = np.linspace(-width / 2, width / 2, cols)
    # y_vals = np.linspace(-depth / 2, depth / 2, rows)
    y_vals = np.linspace(depth/2, -depth/2, rows)    # 修正后
    xx, yy = np.meshgrid(x_vals, y_vals)
    zz = pixels * scale_z

    return buildHFieldMeshHeights(width, depth, rows, cols, zz)

import meshcat
viewer = meshcat.Visualizer().open()
terrain_mesh = buildHFieldMeshPNG("hfield.png", 
                                  width=0.5, depth=0.5, 
                                  scale_z=0.05)
viewer["terrain"].set_object(terrain_mesh)
input("Press Enter to exit...")