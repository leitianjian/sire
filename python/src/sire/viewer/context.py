"""Desktop GLFW and headless EGL graphics contexts."""

from contextlib import contextmanager
import os

@contextmanager
def _egl_context():
    # Select EGL before PyOpenGL loads its platform bindings.
    os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
    from OpenGL import EGL
    from OpenGL.EGL.EXT.device_enumeration import eglQueryDevicesEXT
    from OpenGL.EGL.EXT.platform_base import eglGetPlatformDisplayEXT
    from OpenGL.error import GLError

    def displays():
        if eglQueryDevicesEXT and eglGetPlatformDisplayEXT:
            devices = (EGL.EGLDeviceEXT * 16)()
            count = EGL.EGLint()
            try:
                eglQueryDevicesEXT(len(devices), devices, count)
            except GLError:
                count.value = 0
            for device in devices[:count.value]:
                yield lambda: eglGetPlatformDisplayEXT(0x313F, device, None)  # EGL_PLATFORM_DEVICE_EXT
        if eglGetPlatformDisplayEXT:
            yield lambda: eglGetPlatformDisplayEXT(0x31DD, None, None)  # EGL_PLATFORM_SURFACELESS_MESA
        yield lambda: EGL.eglGetDisplay(EGL.EGL_DEFAULT_DISPLAY)

    for get_display in displays():
        try:
            display = get_display()
            if display and EGL.eglInitialize(display, None, None):
                break
        except GLError:
            continue
    else:
        raise RuntimeError("Cannot initialize an EGL display for headless Sire rendering")

    surface = EGL.EGL_NO_SURFACE
    context = EGL.EGL_NO_CONTEXT
    try:
        EGL.eglBindAPI(EGL.EGL_OPENGL_API)
        attrs = (EGL.EGLint * 7)(
            EGL.EGL_SURFACE_TYPE, EGL.EGL_PBUFFER_BIT,
            EGL.EGL_RENDERABLE_TYPE, EGL.EGL_OPENGL_BIT,
            EGL.EGL_RED_SIZE, 8, EGL.EGL_NONE,
        )
        configs = (EGL.EGLConfig * 1)()
        count = EGL.EGLint()
        EGL.eglChooseConfig(display, attrs, configs, 1, count)
        if not count.value:
            raise RuntimeError("EGL has no desktop OpenGL pbuffer configuration")
        config = configs[0]
        pbuffer = (EGL.EGLint * 5)(EGL.EGL_WIDTH, 1, EGL.EGL_HEIGHT, 1, EGL.EGL_NONE)
        surface = EGL.eglCreatePbufferSurface(display, config, pbuffer)
        version = (EGL.EGLint * 7)(
            EGL.EGL_CONTEXT_MAJOR_VERSION, 3, EGL.EGL_CONTEXT_MINOR_VERSION, 3,
            EGL.EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL.EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT,
            EGL.EGL_NONE,
        )
        context = EGL.eglCreateContext(display, config, EGL.EGL_NO_CONTEXT, version)
        EGL.eglMakeCurrent(display, surface, surface, context)
        yield
    finally:
        EGL.eglMakeCurrent(display, EGL.EGL_NO_SURFACE, EGL.EGL_NO_SURFACE, EGL.EGL_NO_CONTEXT)
        if context:
            EGL.eglDestroyContext(display, context)
        if surface:
            EGL.eglDestroySurface(display, surface)
        EGL.eglTerminate(display)


@contextmanager
def graphics_context(headless=False, *, width=1200, height=800, title="Sire"):
    """Yield a GLFW window, or a current EGL context with no window."""
    if headless:
        with _egl_context():
            yield None
        return

    import glfw

    if not glfw.init():
        raise RuntimeError("Cannot open a GLFW display. Use headless=True for offscreen rendering.")
    try:
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
        window = glfw.create_window(width, height, title, None, None)
        if window is None:
            raise RuntimeError("Could not create the Sire OpenGL window")
        glfw.make_context_current(window)
        glfw.swap_interval(0)
        yield window
    finally:
        glfw.terminate()
