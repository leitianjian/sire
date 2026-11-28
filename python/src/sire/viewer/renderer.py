"""OpenGL rendering of Sire scene geometry and RGB-D cameras."""

from __future__ import annotations

from contextlib import contextmanager
from ctypes import c_void_p

import numpy as np


_SCENE_VERTEX = """
#version 330 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
uniform mat4 model;
uniform mat4 view_projection;
out vec3 world_position;
out vec3 world_normal;

void main() {
    vec4 world = model * vec4(position, 1.0);
    world_position = world.xyz;
    world_normal = transpose(inverse(mat3(model))) * normal;
    gl_Position = view_projection * world;
}
"""

_SCENE_FRAGMENT = """
#version 330 core
in vec3 world_position;
in vec3 world_normal;
uniform vec3 color;
uniform bool checker;
out vec4 pixel;

void main() {
    vec3 surface = color;
    if (checker) {
        vec2 tile = floor(world_position.xy * 2.0);
        float checker_value = mod(tile.x + tile.y, 2.0);
        surface = mix(surface, surface * 0.75, checker_value);
    }
    vec3 light = normalize(vec3(-0.4, -0.6, 1.0));
    float diffuse = max(dot(normalize(world_normal), light), 0.0);
    pixel = vec4(surface * (0.55 + 0.45 * diffuse), 1.0);
}
"""

_OVERLAY_VERTEX = """
#version 330 core
out vec2 uv;

void main() {
    const vec2 corners[4] = vec2[4](
        vec2(0, 0), vec2(1, 0), vec2(0, 1), vec2(1, 1)
    );
    vec2 corner = corners[gl_VertexID];
    gl_Position = vec4(2.0 * corner - 1.0, 0.0, 1.0);
    uv = vec2(corner.x, 1.0 - corner.y);
}
"""

_OVERLAY_FRAGMENT = """
#version 330 core
in vec2 uv;
uniform sampler2D preview;
uniform int preview_mode;
uniform vec2 clip_planes;
out vec4 pixel;

void main() {
    vec2 sample_uv = preview_mode == 0 ? uv : vec2(uv.x, 1.0 - uv.y);
    vec3 color = texture(preview, sample_uv).rgb;
    if (preview_mode == 2) {
        float near_plane = clip_planes.x;
        float far_plane = clip_planes.y;
        float depth = near_plane * far_plane / (near_plane + color.r * (far_plane - near_plane));
        color = vec3(color.r > 0.0 ? 1.0 - clamp(depth / 4.0, 0.0, 1.0) : 0.0);
    }
    pixel = vec4(color, 1.0);
}
"""


def _load_obj(path):
    positions, normals = [], []
    triangles = []
    with path.open(encoding="utf-8") as source:
        for line in source:
            fields = line.split("#", 1)[0].split()
            if not fields:
                continue
            kind, *values = fields
            if kind == "v":
                positions.append(tuple(map(float, values[:3])))
            elif kind == "vn":
                normals.append(tuple(map(float, values[:3])))
            elif kind == "f":
                face = []
                for value in values:
                    indices = value.split("/")
                    position = int(indices[0])
                    position = position - 1 if position > 0 else len(positions) + position
                    normal = None
                    if len(indices) > 2 and indices[2]:
                        normal = int(indices[2])
                        normal = normal - 1 if normal > 0 else len(normals) + normal
                    face.append((position, normal))
                for index in range(1, len(face) - 1):
                    triangles.append((face[0], face[index], face[index + 1]))

    if not triangles:
        raise ValueError(f"Go2 mesh {path} contains no faces")
    faces = np.asarray([[(p, -1 if n is None else n) for p, n in face]
                        for face in triangles], dtype=np.intp)
    points = np.asarray(positions, dtype=np.float32)[faces[..., 0]]
    normal_indices = faces[..., 1]
    vertex_normals = np.empty_like(points)
    supplied = normal_indices >= 0
    if supplied.any():
        vertex_normals[supplied] = np.asarray(normals, dtype=np.float32)[normal_indices[supplied]]
    if not supplied.all():
        flat = np.cross(points[:, 1] - points[:, 0], points[:, 2] - points[:, 0])
        vertex_normals[~supplied] = np.broadcast_to(flat[:, None, :], points.shape)[~supplied]
    return np.ascontiguousarray(np.concatenate((points, vertex_normals), axis=-1).reshape(-1, 6))


def _load_depth_mesh(path):
    if path.suffix.lower() != ".ply":
        return _load_obj(path)

    from plyfile import PlyData, PlyParseError

    # Binary triangle PLY can be mapped directly instead of reading each face separately.
    try:
        ply = PlyData.read(str(path), known_list_len={"face": {"vertex_indices": 3, "vertex_index": 3}})
    except PlyParseError as error:
        raise ValueError(f"Could not read triangle depth mesh {path}: {error}") from error
    if "vertex" not in ply or "face" not in ply or not len(ply["face"].data):
        raise ValueError(f"Depth mesh {path} contains no faces")
    vertices = ply["vertex"].data
    faces = ply["face"].data
    fields = faces.dtype.names
    index_field = next((name for name in ("vertex_indices", "vertex_index") if name in fields), None)
    if index_field is None:
        raise ValueError(f"Depth mesh {path} must contain triangle faces")
    indices = faces[index_field]
    if indices.dtype == object:
        if any(len(face) != 3 for face in indices):
            raise ValueError(f"Depth mesh {path} must contain triangle faces")
        indices = np.stack(indices)
    if np.any(indices < 0) or np.any(indices >= len(vertices)):
        raise ValueError(f"Depth mesh {path} contains an invalid vertex index")
    points = np.column_stack([vertices[axis] for axis in "xyz"]).astype(np.float32)[indices]
    if all(name in vertices.dtype.names for name in ("nx", "ny", "nz")):
        normals = np.column_stack([vertices[name] for name in ("nx", "ny", "nz")]).astype(np.float32)[indices]
    else:
        flat = np.cross(points[:, 1] - points[:, 0], points[:, 2] - points[:, 0])
        normals = np.broadcast_to(flat[:, None, :], points.shape)
    return np.ascontiguousarray(np.concatenate((points, normals), axis=-1).reshape(-1, 6))


def _box_vertices(size):
    x, y, z = np.asarray(size) / 2
    corners = np.array([[-x, -y, -z], [x, -y, -z], [x, y, -z], [-x, y, -z],
                        [-x, -y, z], [x, -y, z], [x, y, z], [-x, y, z]])
    sides = [
        ((0, 3, 2, 1), (0, 0, -1)),
        ((4, 5, 6, 7), (0, 0, 1)),
        ((0, 1, 5, 4), (0, -1, 0)),
        ((3, 7, 6, 2), (0, 1, 0)),
        ((0, 4, 7, 3), (-1, 0, 0)),
        ((1, 2, 6, 5), (1, 0, 0)),
    ]
    # A zero-thickness ground box only needs its upward face.
    if z == 0:
        sides = [sides[1]]
    return np.asarray([
        (*corners[indices[index]], *normal)
        for indices, normal in sides for index in (0, 1, 2, 0, 2, 3)
    ], dtype=np.float32)


def _projection(config):
    fx, fy, cx, cy = config.pinhole
    width, height = config.width, config.height
    near, far = config.near, config.far
    return np.array([
        [2 * fx / width, 0, 1 - 2 * cx / width, 0],
        [0, 2 * fy / height, 2 * cy / height - 1, 0],
        [0, 0, -(far + near) / (far - near), -2 * far * near / (far - near)],
        [0, 0, -1, 0],
    ])


class MeshRenderer:
    """Render scene items in the current OpenGL context."""

    scene_fragment = _SCENE_FRAGMENT
    clear_color = (0.18, 0.23, 0.28, 1.0)
    _load_mesh = staticmethod(_load_depth_mesh)

    def __init__(self, items):
        # The context owner selects the PyOpenGL platform before constructing us.
        from OpenGL import GL as gl
        from OpenGL.GL import shaders

        self._gl = gl
        self._programs = []
        self._meshes = {}
        self._items = []
        self._framebuffer = self._color = self._depth = 0
        self._capture_size = None
        self._capture_sample = None
        self._overlay_vao = self._overlay_texture = 0
        self._overlay_source = None
        self._overlay_size = None

        try:
            self._scene_program = shaders.compileProgram(
                shaders.compileShader(_SCENE_VERTEX, gl.GL_VERTEX_SHADER),
                shaders.compileShader(self.scene_fragment, gl.GL_FRAGMENT_SHADER),
            )
            self._programs.append(self._scene_program)
            self._scene_uniforms = {
                name: gl.glGetUniformLocation(self._scene_program, name)
                for name in ("model", "view_projection", "color", "checker")
            }
            self._overlay_program = shaders.compileProgram(
                shaders.compileShader(_OVERLAY_VERTEX, gl.GL_VERTEX_SHADER),
                shaders.compileShader(_OVERLAY_FRAGMENT, gl.GL_FRAGMENT_SHADER),
            )
            self._programs.append(self._overlay_program)
            self._overlay_sampler = gl.glGetUniformLocation(self._overlay_program, "preview")
            self._overlay_mode = gl.glGetUniformLocation(self._overlay_program, "preview_mode")
            self._overlay_clip = gl.glGetUniformLocation(self._overlay_program, "clip_planes")
            self._overlay_vao = gl.glGenVertexArrays(1)
            self._overlay_texture = gl.glGenTextures(1)
            self._framebuffer = gl.glGenFramebuffers(1)
            self._color = gl.glGenTextures(1)
            self._depth = gl.glGenTextures(1)
            for item in items:
                key = item.mesh if item.mesh is not None else tuple(item.size)
                if key not in self._meshes:
                    vertices = self._load_mesh(item.mesh) if item.mesh is not None else _box_vertices(item.size)
                    self._upload(key, vertices)
                self._items.append((item, self._meshes[key]))
        except Exception:
            self.close()
            raise

    def _upload(self, key, vertices):
        gl = self._gl
        vao = gl.glGenVertexArrays(1)
        vbo = gl.glGenBuffers(1)
        self._meshes[key] = vao, vbo, len(vertices)
        gl.glBindVertexArray(vao)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, vertices.nbytes, vertices, gl.GL_STATIC_DRAW)
        for index in range(2):
            gl.glEnableVertexAttribArray(index)
            gl.glVertexAttribPointer(index, 3, gl.GL_FLOAT, gl.GL_FALSE, 24, c_void_p(12 * index))
        gl.glBindVertexArray(0)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

    def _set_scene_uniforms(self, world_pose):
        """Set extra uniforms for the active scene program."""

    def _set_item_uniforms(self, item):
        """Set extra uniforms for a single draw in the scene pass."""

    def _render(self, poses, config, world_pose, width, height):
        gl = self._gl
        uniforms = self._scene_uniforms
        view_projection = _projection(config) @ np.linalg.inv(world_pose)
        gl.glViewport(0, 0, width, height)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glDepthMask(gl.GL_TRUE)
        gl.glDepthRange(1, 0)
        gl.glDepthFunc(gl.GL_GREATER)
        gl.glClearDepth(0)
        gl.glClearColor(*self.clear_color)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glUseProgram(self._scene_program)
        gl.glUniformMatrix4fv(uniforms["view_projection"], 1, gl.GL_FALSE,
                              np.ascontiguousarray(view_projection.T, dtype=np.float32))
        self._set_scene_uniforms(world_pose)
        for item, (vao, _, count) in self._items:
            model = item.local_pose if item.body is None else poses[item.body] @ item.local_pose
            gl.glUniformMatrix4fv(uniforms["model"], 1, gl.GL_FALSE,
                                  np.ascontiguousarray(model.T, dtype=np.float32))
            gl.glUniform3f(uniforms["color"], *item.color)
            gl.glUniform1i(uniforms["checker"], item.checker)
            self._set_item_uniforms(item)
            gl.glBindVertexArray(vao)
            gl.glDrawArrays(gl.GL_TRIANGLES, 0, count)
        gl.glBindVertexArray(0)
        gl.glUseProgram(0)

    def draw(self, poses, config, world_pose, width, height):
        """Draw the scene to the default framebuffer."""
        gl = self._gl
        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, 0)
        self._render(poses, config, world_pose, width, height)

    def _resize_capture(self, width, height):
        gl = self._gl
        if self._capture_size == (width, height):
            return
        self._capture_size = None
        gl.glActiveTexture(gl.GL_TEXTURE0)
        try:
            for texture, storage, attachment, fmt, pixel_type, filter_mode in (
                (self._color, gl.GL_RGB8, gl.GL_COLOR_ATTACHMENT0, gl.GL_RGB, gl.GL_UNSIGNED_BYTE, gl.GL_LINEAR),
                (self._depth, gl.GL_DEPTH_COMPONENT32F, gl.GL_DEPTH_ATTACHMENT,
                 gl.GL_DEPTH_COMPONENT, gl.GL_FLOAT, gl.GL_NEAREST),
            ):
                gl.glBindTexture(gl.GL_TEXTURE_2D, texture)
                gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, storage, width, height, 0, fmt, pixel_type, None)
                gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, filter_mode)
                gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, filter_mode)
                gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_CLAMP_TO_EDGE)
                gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_CLAMP_TO_EDGE)
                gl.glFramebufferTexture2D(gl.GL_FRAMEBUFFER, attachment, gl.GL_TEXTURE_2D, texture, 0)
        finally:
            gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
        if gl.glCheckFramebufferStatus(gl.GL_FRAMEBUFFER) != gl.GL_FRAMEBUFFER_COMPLETE:
            raise RuntimeError(f"OpenGL RGB-D framebuffer is incomplete at {width}x{height}")
        self._capture_size = width, height

    def capture(self, poses, config, world_pose):
        """Return aligned top-down RGB, optical-axis depth, and validity arrays."""
        return self.prepare_capture(poses, config, world_pose)()

    def prepare_capture(self, poses, config, world_pose):
        """Render to textures and return a readback callable for this sample.

        Call it with the graphics context current, before another capture or close.
        """
        if not self._framebuffer:
            raise RuntimeError("Cannot capture after the renderer has been closed")
        gl = self._gl
        width, height = config.width, config.height
        sample = object()
        self._capture_sample = None
        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, self._framebuffer)
        try:
            self._resize_capture(width, height)
            self._render(poses, config, world_pose, width, height)
        finally:
            gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, 0)
        self._capture_sample = (sample, config)

        def readback():
            if self._capture_sample is None or self._capture_sample[0] is not sample:
                raise RuntimeError("The renderer no longer holds this camera sample")
            return self._read_capture(config)

        return readback

    def _read_capture(self, config):
        gl = self._gl
        width, height = config.width, config.height
        rgb = np.empty((height, width, 3), dtype=np.uint8)
        buffer = np.empty((height, width), dtype=np.float32)
        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, self._framebuffer)
        try:
            gl.glPixelStorei(gl.GL_PACK_ALIGNMENT, 1)
            gl.glReadBuffer(gl.GL_COLOR_ATTACHMENT0)
            gl.glReadPixels(0, 0, width, height, gl.GL_RGB, gl.GL_UNSIGNED_BYTE, rgb)
            gl.glReadPixels(0, 0, width, height, gl.GL_DEPTH_COMPONENT, gl.GL_FLOAT, buffer)
        finally:
            gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, 0)

        # Invert the perspective mapping after the reversed depth range.
        valid = buffer > 0
        near, far = config.near, config.far
        depth = near * far / (near + buffer * (far - near))
        depth[~valid] = np.nan
        return rgb[::-1].copy(), depth[::-1].copy(), valid[::-1].copy()

    def draw_overlay(self, frame, width, height):
        """Draw RGB and depth previews over the main viewport."""
        if frame is None or min(width, height) < 64:
            return

        gl = self._gl
        height_pixels, width_pixels = frame.depth.shape
        preview_width, preview_height, stride = self._preview_size(width_pixels, height_pixels, width, height)
        with self._overlay(width, height):
            gl.glUniform1i(self._overlay_mode, 0)
            gl.glBindTexture(gl.GL_TEXTURE_2D, self._overlay_texture)
            texture_size = preview_width, 2 * preview_height
            if self._overlay_source is not frame or self._overlay_size != texture_size:
                rgb = frame.rgb[::stride, ::stride]
                depth = frame.depth[::stride, ::stride]
                valid = frame.valid[::stride, ::stride]
                grey = np.where(valid, 255 * (1 - np.clip(depth / 4, 0, 1)), 0).astype(np.uint8)
                images = np.concatenate((rgb, np.repeat(grey[..., None], 3, axis=2)))
                gl.glPixelStorei(gl.GL_UNPACK_ALIGNMENT, 1)
                if self._overlay_size != texture_size:
                    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR)
                    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)
                    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_CLAMP_TO_EDGE)
                    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_CLAMP_TO_EDGE)
                    gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_RGB8, *texture_size,
                                    0, gl.GL_RGB, gl.GL_UNSIGNED_BYTE, images)
                else:
                    gl.glTexSubImage2D(gl.GL_TEXTURE_2D, 0, 0, 0, *texture_size,
                                       gl.GL_RGB, gl.GL_UNSIGNED_BYTE, images)
                self._overlay_source = frame
                self._overlay_size = texture_size
            gl.glViewport(width - preview_width - 10, 10, preview_width, 2 * preview_height)
            gl.glDrawArrays(gl.GL_TRIANGLE_STRIP, 0, 4)

    def draw_capture_overlay(self, width, height):
        """Display the latest capture's color and depth textures directly."""
        if self._capture_sample is None or min(width, height) < 64:
            return
        _, config = self._capture_sample
        preview_width, preview_height, _ = self._preview_size(config.width, config.height, width, height)
        x = width - preview_width - 10
        gl = self._gl
        with self._overlay(width, height):
            gl.glUniform2f(self._overlay_clip, config.near, config.far)
            for mode, texture, y in ((1, self._color, 10 + preview_height), (2, self._depth, 10)):
                gl.glUniform1i(self._overlay_mode, mode)
                gl.glBindTexture(gl.GL_TEXTURE_2D, texture)
                gl.glViewport(x, y, preview_width, preview_height)
                gl.glDrawArrays(gl.GL_TRIANGLE_STRIP, 0, 4)

    @staticmethod
    def _preview_size(image_width, image_height, width, height):
        stride = max(1, int(np.ceil(4 * max(image_width / width, image_height / height))))
        return (image_width + stride - 1) // stride, (image_height + stride - 1) // stride, stride

    @contextmanager
    def _overlay(self, width, height):
        gl = self._gl
        gl.glDisable(gl.GL_DEPTH_TEST)
        gl.glDepthMask(gl.GL_FALSE)
        gl.glUseProgram(self._overlay_program)
        gl.glUniform1i(self._overlay_sampler, 0)
        gl.glBindVertexArray(self._overlay_vao)
        gl.glActiveTexture(gl.GL_TEXTURE0)
        try:
            yield
        finally:
            gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
            gl.glBindVertexArray(0)
            gl.glUseProgram(0)
            gl.glDepthMask(gl.GL_TRUE)
            gl.glEnable(gl.GL_DEPTH_TEST)
            gl.glViewport(0, 0, width, height)

    def close(self):
        """Release this renderer's resources with its context current."""
        gl = self._gl
        for vao, vbo, _ in self._meshes.values():
            gl.glDeleteVertexArrays(1, [vao])
            gl.glDeleteBuffers(1, [vbo])
        self._meshes.clear()
        for program in self._programs:
            gl.glDeleteProgram(program)
        self._programs.clear()
        if self._overlay_vao:
            gl.glDeleteVertexArrays(1, [self._overlay_vao])
            self._overlay_vao = 0
        if self._overlay_texture:
            gl.glDeleteTextures(1, [self._overlay_texture])
            self._overlay_texture = 0
        if self._framebuffer:
            gl.glDeleteFramebuffers(1, [self._framebuffer])
            gl.glDeleteTextures(2, [self._color, self._depth])
            self._framebuffer = self._color = self._depth = 0
        self._capture_sample = None
        self._overlay_source = None
