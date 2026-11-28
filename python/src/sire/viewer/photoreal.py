"""Compose a captured Gaussian scene with Sire's live rigid-body meshes."""

from dataclasses import dataclass

import sire

from ._gaussian_rasterizer import _GaussianRasterizer
from ._pbr import PBR_FRAGMENT
from .renderer import MeshRenderer
from .scene import RenderItem


_COMPOSITE_VERTEX = """
#version 330 core
void main() {
    const vec2 corners[3] = vec2[3](vec2(-1,-1), vec2(3,-1), vec2(-1,3));
    gl_Position = vec4(corners[gl_VertexID], 0, 1);
}
"""

_COMPOSITE_FRAGMENT = """
#version 330 core
uniform sampler2D mesh_color;
uniform sampler2D mesh_depth;
uniform sampler2D splat_color;
out vec4 pixel;
void main() {
    ivec2 p = ivec2(gl_FragCoord.xy);
    vec4 mesh = texelFetch(mesh_color, p, 0);
    vec4 splat = texelFetch(splat_color, ivec2(p.x, textureSize(splat_color, 0).y - 1 - p.y), 0);
    // The nearest mesh marks whether this pixel belongs to a live object.
    // The GS color is already premultiplied by alpha.
    vec3 color = mesh.a > 0.5 ? mesh.rgb : splat.rgb + (1.0 - splat.a) * mesh.rgb;
    pixel = vec4(color, 1.0);
    gl_FragDepth = texelFetch(mesh_depth, p, 0).r;
}
"""


@dataclass
class _Target:
    framebuffer: int
    mesh_color: int
    mesh_depth: int
    splat_color: int
    size: tuple
    cuda_texture: object = None


class PhotorealRenderer(MeshRenderer):
    """Render an aligned Gaussian PLY and depth mesh with moving Sire meshes.

    Use in a current OpenGL context, through the same draw and capture interface
    as MeshRenderer. Scene paths and the shared asset-to-world transform are
    read from the supplied JSON file. The depth mesh defines sensor geometry.
    """

    scene_fragment = PBR_FRAGMENT
    clear_color = (0.16, 0.19, 0.22, 0.0)

    def __init__(self, items, scene_path, *, device="cuda:0"):
        self._rasterizer = _GaussianRasterizer.for_scene(scene_path, device)
        self._targets = {}
        self._composite_vao = 0
        self._room = RenderItem(
            None, self._rasterizer.asset_to_world, (0.40, 0.38, 0.35), mesh=self._rasterizer.depth_mesh,
        )
        super().__init__([self._room, *items])

        from OpenGL.GL import shaders

        gl = self._gl
        try:
            self._material = gl.glGetUniformLocation(self._scene_program, "material")
            self._proxy = gl.glGetUniformLocation(self._scene_program, "room_proxy")
            self._camera = gl.glGetUniformLocation(self._scene_program, "camera_position")
            self._composite = shaders.compileProgram(
                shaders.compileShader(_COMPOSITE_VERTEX, gl.GL_VERTEX_SHADER),
                shaders.compileShader(_COMPOSITE_FRAGMENT, gl.GL_FRAGMENT_SHADER),
            )
            self._programs.append(self._composite)
            self._composite_vao = gl.glGenVertexArrays(1)
            gl.glUseProgram(self._composite)
            for slot, name in enumerate(("mesh_color", "mesh_depth", "splat_color")):
                gl.glUniform1i(gl.glGetUniformLocation(self._composite, name), slot)
            gl.glUseProgram(0)
        except Exception:
            self.close()
            raise

    def _set_scene_uniforms(self, world_pose):
        self._gl.glUniform3f(self._camera, *world_pose[:3, 3])

    def _set_item_uniforms(self, item):
        gl = self._gl
        gl.glUniform1i(self._proxy, item is self._room)
        # Light shells and dark rubber share the classic geometry and style.
        metallic = 0.35 if item.mesh is not None and min(item.color) > 0.4 else 0.0
        roughness = 0.38 if metallic else 0.72
        gl.glUniform2f(self._material, metallic, roughness)

    def _target(self, output, width, height):
        gl = self._gl
        target = self._targets.get(output)
        if target and target.size == (width, height):
            gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, target.framebuffer)
            return target
        if target:
            self._delete_target(self._targets.pop(output))

        framebuffer = gl.glGenFramebuffers(1)
        mesh_color, mesh_depth, splat_color = map(int, gl.glGenTextures(3))
        target = _Target(framebuffer, mesh_color, mesh_depth, splat_color, (width, height))
        self._targets[output] = target
        gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, framebuffer)
        gl.glActiveTexture(gl.GL_TEXTURE0)
        for texture, storage, pixel_format, pixel_type, attachment in (
            (mesh_color, gl.GL_RGBA8, gl.GL_RGBA, gl.GL_UNSIGNED_BYTE, gl.GL_COLOR_ATTACHMENT0),
            (mesh_depth, gl.GL_DEPTH_COMPONENT32F, gl.GL_DEPTH_COMPONENT, gl.GL_FLOAT, gl.GL_DEPTH_ATTACHMENT),
            (splat_color, gl.GL_RGBA8, gl.GL_RGBA, gl.GL_UNSIGNED_BYTE, None),
        ):
            gl.glBindTexture(gl.GL_TEXTURE_2D, texture)
            gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, storage, width, height, 0,
                            pixel_format, pixel_type, None)
            gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_NEAREST)
            gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_NEAREST)
            if attachment is not None:
                gl.glFramebufferTexture2D(gl.GL_FRAMEBUFFER, attachment, gl.GL_TEXTURE_2D, texture, 0)
        gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
        if gl.glCheckFramebufferStatus(gl.GL_FRAMEBUFFER) != gl.GL_FRAMEBUFFER_COMPLETE:
            raise RuntimeError(f"Photoreal mesh framebuffer is incomplete at {width}x{height}")
        texture_type = getattr(sire.sire, "_SireCudaGlTexture", None)
        if texture_type is not None:
            try:
                target.cuda_texture = texture_type(
                    splat_color, self._rasterizer.device_index, width, height)
            except RuntimeError:
                # A non-NVIDIA OpenGL context cannot share this CUDA texture.
                pass
        return target

    @property
    def splat_transfer(self):
        """Report the transfer path actually used by the active targets."""
        if all(target.cuda_texture is not None for target in self._targets.values()):
            return "cuda-gl"
        return "host"

    def _render(self, poses, config, world_pose, width, height):
        gl = self._gl
        output = int(gl.glGetIntegerv(gl.GL_DRAW_FRAMEBUFFER_BINDING))
        try:
            target = self._target(output, width, height)
            super()._render(poses, config, world_pose, width, height)
            rgba = self._rasterizer.capture(config, world_pose)
        finally:
            gl.glBindFramebuffer(gl.GL_FRAMEBUFFER, output)

        gl.glActiveTexture(gl.GL_TEXTURE2)
        gl.glBindTexture(gl.GL_TEXTURE_2D, target.splat_color)
        if target.cuda_texture is not None:
            target.cuda_texture.upload(rgba, self._rasterizer.current_stream)
        else:
            gl.glPixelStorei(gl.GL_UNPACK_ALIGNMENT, 1)
            gl.glTexSubImage2D(gl.GL_TEXTURE_2D, 0, 0, 0, width, height, gl.GL_RGBA,
                               gl.GL_UNSIGNED_BYTE, rgba.cpu().numpy())
        gl.glActiveTexture(gl.GL_TEXTURE1)
        gl.glBindTexture(gl.GL_TEXTURE_2D, target.mesh_depth)
        gl.glActiveTexture(gl.GL_TEXTURE0)
        gl.glBindTexture(gl.GL_TEXTURE_2D, target.mesh_color)
        gl.glUseProgram(self._composite)
        gl.glBindVertexArray(self._composite_vao)
        gl.glDepthFunc(gl.GL_ALWAYS)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, 3)
        gl.glDepthFunc(gl.GL_GREATER)
        gl.glBindVertexArray(0)
        gl.glUseProgram(0)
        for slot in (gl.GL_TEXTURE2, gl.GL_TEXTURE1, gl.GL_TEXTURE0):
            gl.glActiveTexture(slot)
            gl.glBindTexture(gl.GL_TEXTURE_2D, 0)

    def _delete_target(self, target):
        if target.cuda_texture is not None:
            target.cuda_texture.close()
        gl = self._gl
        gl.glDeleteFramebuffers(1, [target.framebuffer])
        gl.glDeleteTextures(3, [target.mesh_color, target.mesh_depth, target.splat_color])

    def close(self):
        self._rasterizer = None
        for target in self._targets.values():
            self._delete_target(target)
        self._targets.clear()
        if self._composite_vao:
            self._gl.glDeleteVertexArrays(1, [self._composite_vao])
            self._composite_vao = 0
        super().close()
