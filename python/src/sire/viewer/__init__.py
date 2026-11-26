"""Display or capture a Sire scene with OpenGL."""

import math

import numpy as np

from ..sensor import CameraConfig
from .context import graphics_context
from .renderer import MeshRenderer
from .scene import RenderItem, SireScene

__all__ = ["Viewer", "graphics_context", "MeshRenderer", "RenderItem", "SireScene"]


class Viewer:
    """Present a Sire scene with an independent orbit camera."""

    def __init__(self, window, renderer):
        import glfw

        self._glfw = glfw
        self._window = window
        self._renderer = renderer
        self._distance = 2.6
        self._azimuth = math.radians(-135)
        self._elevation = math.radians(25)
        self._offset = np.array([0., 0., 0.20])
        self._cursor = glfw.get_cursor_pos(window)
        self._projection = None
        glfw.set_cursor_pos_callback(window, self._drag)
        glfw.set_scroll_callback(window, self._scroll)
        glfw.set_key_callback(window, self._key)

    def _pose(self, target):
        c = math.cos(self._elevation)
        back = np.array([c * math.cos(self._azimuth), c * math.sin(self._azimuth), math.sin(self._elevation)])
        right = np.array([-math.sin(self._azimuth), math.cos(self._azimuth), 0.])
        up = np.cross(back, right)
        pose = np.eye(4)
        pose[:3, :3] = np.column_stack((right, up, back))
        pose[:3, 3] = target + self._offset + self._distance * back
        return pose

    def _key(self, window, key, _scancode, action, _mods):
        if key == self._glfw.KEY_ESCAPE and action == self._glfw.PRESS:
            self._glfw.set_window_should_close(window, True)

    def _drag(self, window, x, y):
        glfw = self._glfw
        old_x, old_y = self._cursor
        self._cursor = (x, y)
        height = max(1, glfw.get_window_size(window)[1])
        dx, dy = (x - old_x) / height, (y - old_y) / height
        if glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS:
            self._azimuth -= 4 * dx
            self._elevation = float(np.clip(self._elevation + 4 * dy, -1.55, 1.55))
        elif glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS:
            axes = self._pose(np.zeros(3))[:3, :3]
            self._offset += self._distance * (-dx * axes[:, 0] + dy * axes[:, 1])

    def _scroll(self, _window, _x, y):
        self._distance = float(np.clip(self._distance * math.exp(-0.12 * y), 0.2, 100.))

    def sync(self, poses, target, preview=None, *, capture_preview=False):
        """Draw the scene with a supplied frame or the renderer's latest capture."""
        glfw = self._glfw
        glfw.poll_events()
        if glfw.window_should_close(self._window):
            return False

        width, height = glfw.get_framebuffer_size(self._window)
        if width and height:
            if self._projection is None or (width, height) != (self._projection.width, self._projection.height):
                self._projection = CameraConfig(width, height, fov_y=45., near=0.03, far=150.)
            self._renderer.draw(poses, self._projection, self._pose(target), width, height)
            if preview is not None:
                self._renderer.draw_overlay(preview, width, height)
            elif capture_preview:
                self._renderer.draw_capture_overlay(width, height)
            glfw.swap_buffers(self._window)
        return True
