#include "sire/sensor/cuda_batch_camera.hpp"

#include "cuda_batch_camera_kernels.hpp"

#include <EGL/egl.h>
#include <EGL/eglext.h>
#define GL_GLEXT_PROTOTYPES
#include <GL/glcorearb.h>
#include <GL/glx.h>
#include <cuda_gl_interop.h>
#include <cuda_runtime.h>
#include <dlfcn.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace sire::sensor {
namespace {

constexpr const char* vertex_source = R"GLSL(
#version 430 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(std430, binding = 0) readonly buffer BodyRows {
    vec4 body_rows[];
};
layout(std430, binding = 1) readonly buffer ViewProjectionRows {
    vec4 view_projection_rows[];
};

uniform mat4 local_pose;
uniform int bodies;
uniform int body;
uniform int first;
uniform ivec2 atlas_tiles;

out vec3 world_position;
out vec3 world_normal;
out float optical_depth;
out gl_PerVertex {
    vec4 gl_Position;
    float gl_PointSize;
    float gl_ClipDistance[4];
};

void main() {
    int environment = first + gl_InstanceID;
    mat4 model = local_pose;
    if (body >= 0) {
        int offset = 3 * (environment * bodies + body);
        mat4 body_pose = transpose(mat4(body_rows[offset], body_rows[offset + 1],
                                       body_rows[offset + 2], vec4(0, 0, 0, 1)));
        model = body_pose * local_pose;
    }

    vec4 world = model * vec4(position, 1.0);
    world_position = world.xyz;
    world_normal = transpose(inverse(mat3(model))) * normal;
    int offset = 4 * environment;
    vec4 clip = vec4(dot(view_projection_rows[offset], world),
                     dot(view_projection_rows[offset + 1], world),
                     dot(view_projection_rows[offset + 2], world),
                     dot(view_projection_rows[offset + 3], world));
    optical_depth = clip.w;

    // Clip to the camera frustum before rasterizing in this instance's atlas tile.
    gl_ClipDistance[0] = clip.w + clip.x;
    gl_ClipDistance[1] = clip.w - clip.x;
    gl_ClipDistance[2] = clip.w + clip.y;
    gl_ClipDistance[3] = clip.w - clip.y;
    ivec2 tile = ivec2(gl_InstanceID % atlas_tiles.x, gl_InstanceID / atlas_tiles.x);
    clip.xy = (clip.xy + (2.0 * vec2(tile) + 1.0 - vec2(atlas_tiles)) * clip.w)
              / vec2(atlas_tiles);
    gl_Position = clip;
}
)GLSL";

constexpr const char* fragment_source = R"GLSL(
#version 430 core
in vec3 world_position;
in vec3 world_normal;
in float optical_depth;
uniform vec3 color;
uniform bool checker;
layout(location = 0) out vec4 pixel;
layout(location = 1) out float depth;

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
    depth = optical_depth;
}
)GLSL";

void checkEgl(EGLBoolean status, const char* action) {
    if (!status) {
        throw std::runtime_error(std::string(action) + " failed with EGL error " +
                                 std::to_string(eglGetError()));
    }
}

void checkGl(const char* action) {
    if (const GLenum error = glGetError(); error != GL_NO_ERROR) {
        throw std::runtime_error(std::string(action) + " failed with OpenGL error " +
                                 std::to_string(error));
    }
}

bool hasExtension(const char* extensions, std::string_view extension) {
    std::string_view words = extensions ? extensions : "";
    while (!words.empty()) {
        const auto end = words.find(' ');
        if (words.substr(0, end) == extension) {
            return true;
        }
        if (end == std::string_view::npos) {
            break;
        }
        words.remove_prefix(end + 1);
    }
    return false;
}

class CurrentCudaDevice {
public:
    explicit CurrentCudaDevice(int device) {
        checkCudaCamera(cudaGetDevice(&previous_), "Get current CUDA device");
        if (previous_ != device) {
            checkCudaCamera(cudaSetDevice(device), "Select CUDA camera device");
            changed_ = true;
        }
    }

    ~CurrentCudaDevice() {
        if (changed_) {
            cudaSetDevice(previous_);
        }
    }

private:
    int previous_ = -1;
    bool changed_ = false;
};

class CurrentEglContext {
public:
    CurrentEglContext(EGLDisplay display, EGLSurface surface, EGLContext context)
        : display_(display) {
        if (previous_context_ == EGL_NO_CONTEXT) {
            rememberGlx();
        }
        try {
            checkEgl(eglBindAPI(EGL_OPENGL_API), "Select EGL OpenGL API");
            checkEgl(eglMakeCurrent(display, surface, surface, context), "Activate camera EGL context");
        } catch (...) {
            restore(false);
            throw;
        }
    }

    ~CurrentEglContext() { restore(true); }

private:
    void rememberGlx() {
        glx_library_ = dlopen("libGLX.so.0", RTLD_LAZY | RTLD_NOLOAD);
        if (!glx_library_) {
            return;
        }
        const auto current_context = reinterpret_cast<decltype(&glXGetCurrentContext)>(
            dlsym(glx_library_, "glXGetCurrentContext"));
        const auto current_display = reinterpret_cast<decltype(&glXGetCurrentDisplay)>(
            dlsym(glx_library_, "glXGetCurrentDisplay"));
        const auto current_draw = reinterpret_cast<decltype(&glXGetCurrentDrawable)>(
            dlsym(glx_library_, "glXGetCurrentDrawable"));
        const auto current_read = reinterpret_cast<decltype(&glXGetCurrentReadDrawable)>(
            dlsym(glx_library_, "glXGetCurrentReadDrawable"));
        make_glx_current_ = reinterpret_cast<decltype(&glXMakeContextCurrent)>(
            dlsym(glx_library_, "glXMakeContextCurrent"));
        if (current_context && current_display && current_draw && current_read && make_glx_current_) {
            glx_context_ = current_context();
            glx_display_ = current_display();
            glx_draw_ = current_draw();
            glx_read_ = current_read();
            if (glx_context_) {
                make_glx_current_(glx_display_, 0, 0, nullptr);
            }
        }
    }

    void restore(bool activated) {
        if (activated && (previous_context_ == EGL_NO_CONTEXT || previous_api_ != EGL_OPENGL_API)) {
            eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        }
        eglBindAPI(previous_api_);
        if (previous_context_ != EGL_NO_CONTEXT) {
            eglMakeCurrent(previous_display_, previous_draw_, previous_read_, previous_context_);
        }
        if (glx_context_) {
            make_glx_current_(glx_display_, glx_draw_, glx_read_, glx_context_);
        }
        if (glx_library_) {
            dlclose(glx_library_);
        }
    }

    EGLDisplay display_;
    EGLenum previous_api_ = eglQueryAPI();
    EGLDisplay previous_display_ = eglGetCurrentDisplay();
    EGLSurface previous_draw_ = eglGetCurrentSurface(EGL_DRAW);
    EGLSurface previous_read_ = eglGetCurrentSurface(EGL_READ);
    EGLContext previous_context_ = eglGetCurrentContext();
    void* glx_library_ = nullptr;
    decltype(&glXMakeContextCurrent) make_glx_current_ = nullptr;
    Display* glx_display_ = nullptr;
    GLXDrawable glx_draw_ = 0;
    GLXDrawable glx_read_ = 0;
    GLXContext glx_context_ = nullptr;
};

GLuint compileShader(GLenum type, const char* source) {
    const GLuint shader = glCreateShader(type);
    if (!shader) {
        throw std::runtime_error("Cannot allocate CUDA camera shader");
    }
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);
    GLint status = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    if (status != GL_TRUE) {
        GLint size = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &size);
        std::string log(std::max(size, 1), '\0');
        glGetShaderInfoLog(shader, size, nullptr, log.data());
        glDeleteShader(shader);
        throw std::runtime_error("Cannot compile CUDA camera shader: " + log);
    }
    return shader;
}

GLuint linkProgram() {
    const GLuint vertex = compileShader(GL_VERTEX_SHADER, vertex_source);
    GLuint fragment = 0;
    GLuint program = 0;
    try {
        fragment = compileShader(GL_FRAGMENT_SHADER, fragment_source);
        program = glCreateProgram();
        glAttachShader(program, vertex);
        glAttachShader(program, fragment);
        glLinkProgram(program);
        GLint status = 0;
        glGetProgramiv(program, GL_LINK_STATUS, &status);
        if (status != GL_TRUE) {
            GLint size = 0;
            glGetProgramiv(program, GL_INFO_LOG_LENGTH, &size);
            std::string log(std::max(size, 1), '\0');
            glGetProgramInfoLog(program, size, nullptr, log.data());
            throw std::runtime_error("Cannot link CUDA camera program: " + log);
        }
    } catch (...) {
        glDeleteProgram(program);
        glDeleteShader(fragment);
        glDeleteShader(vertex);
        throw;
    }
    glDeleteShader(fragment);
    glDeleteShader(vertex);
    return program;
}

class MappedResources {
public:
    MappedResources(std::array<cudaGraphicsResource*, 2>& resources, cudaStream_t stream)
        : resources_(resources), stream_(stream) {
        checkCudaCamera(cudaGraphicsMapResources(resources_.size(), resources_.data(), stream_),
                        "Map CUDA camera graphics resources");
    }

    ~MappedResources() {
        if (mapped_) {
            cudaGraphicsUnmapResources(resources_.size(), resources_.data(), stream_);
        }
    }

    void unmap() {
        const cudaError_t result =
            cudaGraphicsUnmapResources(resources_.size(), resources_.data(), stream_);
        mapped_ = false;
        checkCudaCamera(result, "Unmap CUDA camera graphics resources");
    }

    float* buffer(int index) const {
        void* pointer = nullptr;
        size_t size = 0;
        checkCudaCamera(cudaGraphicsResourceGetMappedPointer(&pointer, &size, resources_[index]),
                        "Access mapped CUDA camera transform buffer");
        return static_cast<float*>(pointer);
    }

    cudaArray_t image(int index) const {
        cudaArray_t array = nullptr;
        checkCudaCamera(cudaGraphicsSubResourceGetMappedArray(&array, resources_[index], 0, 0),
                        "Access mapped CUDA camera atlas");
        return array;
    }

private:
    std::array<cudaGraphicsResource*, 2>& resources_;
    cudaStream_t stream_;
    bool mapped_ = true;
};

}  // namespace

struct CudaBatchCamera::Impl {
    struct Mesh {
        GLuint vao = 0;
        GLuint vbo = 0;
        GLsizei count = 0;
    };

    explicit Impl(CudaCameraConfig config, const std::vector<CudaCameraItem>& items)
        : config(config), items(items) {}

    ~Impl() {
        try {
            const CurrentCudaDevice current_device(config.device);
            if (context != EGL_NO_CONTEXT) {
                const CurrentEglContext current_context(display, surface, context);
                for (auto* resource : input_resources) {
                    if (resource) {
                        cudaGraphicsUnregisterResource(resource);
                    }
                }
                for (auto* resource : output_resources) {
                    if (resource) {
                        cudaGraphicsUnregisterResource(resource);
                    }
                }
            }
            cudaFree(rgba_atlas);
            cudaFree(depth_atlas);
        } catch (...) {
            // Destruction cannot report failures during graphics or CUDA shutdown.
        }
        destroyContext();
    }

    void destroyContext() {
        if (context != EGL_NO_CONTEXT) {
            eglDestroyContext(display, context);
        }
        if (surface != EGL_NO_SURFACE) {
            eglDestroySurface(display, surface);
        }
        if (tracked_display && initialized_display) {
            eglTerminate(display);
        }
        context = EGL_NO_CONTEXT;
        surface = EGL_NO_SURFACE;
        display = EGL_NO_DISPLAY;
        tracked_display = false;
        initialized_display = false;
    }

    void createContext() {
        const auto query_devices = reinterpret_cast<PFNEGLQUERYDEVICESEXTPROC>(
            eglGetProcAddress("eglQueryDevicesEXT"));
        const auto get_display = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(
            eglGetProcAddress("eglGetPlatformDisplayEXT"));
        const auto query_device = reinterpret_cast<PFNEGLQUERYDEVICEATTRIBEXTPROC>(
            eglGetProcAddress("eglQueryDeviceAttribEXT"));
        EGLint count = 0;
        if (!query_devices || !get_display || !query_devices(0, nullptr, &count) || count <= 0) {
            throw std::runtime_error("CUDA camera requires an EGL hardware device");
        }
        std::vector<EGLDeviceEXT> devices(count);
        checkEgl(query_devices(count, devices.data(), &count), "Enumerate EGL devices");
        devices.resize(count);
        if (query_device) {
            std::stable_partition(devices.begin(), devices.end(), [&](EGLDeviceEXT device) {
                EGLAttrib ordinal = -1;
                return query_device(device, EGL_CUDA_DEVICE_NV, &ordinal) && ordinal == config.device;
            });
        }

        const EGLenum previous_api = eglQueryAPI();
        const bool can_track = hasExtension(eglQueryString(EGL_NO_DISPLAY, EGL_EXTENSIONS),
                                            "EGL_KHR_display_reference");
        std::string failure;
        for (EGLDeviceEXT device : devices) {
            // A tracked EGL display has an independent, balanced initialization lifetime.
            // An untracked display may be shared with another graphics client.
            const EGLint track[] = {EGL_TRACK_REFERENCES_KHR, EGL_TRUE, EGL_NONE};
            display = can_track ? get_display(EGL_PLATFORM_DEVICE_EXT, device, track) : EGL_NO_DISPLAY;
            tracked_display = display != EGL_NO_DISPLAY;
            if (!tracked_display) {
                display = get_display(EGL_PLATFORM_DEVICE_EXT, device, nullptr);
            }
            if (display == EGL_NO_DISPLAY) {
                continue;
            }
            try {
                checkEgl(eglInitialize(display, nullptr, nullptr), "Initialize camera EGL display");
                initialized_display = true;
                checkEgl(eglBindAPI(EGL_OPENGL_API), "Select EGL OpenGL API");
                const EGLint config_attributes[] = {
                    EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
                    EGL_RED_SIZE, 8, EGL_NONE,
                };
                EGLConfig egl_config{};
                EGLint configs = 0;
                checkEgl(eglChooseConfig(display, config_attributes, &egl_config, 1, &configs),
                         "Choose camera EGL configuration");
                if (!configs) {
                    throw std::runtime_error("No desktop OpenGL EGL pbuffer configuration");
                }
                const EGLint surface_attributes[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
                surface = eglCreatePbufferSurface(display, egl_config, surface_attributes);
                checkEgl(surface != EGL_NO_SURFACE, "Create camera EGL pbuffer");
                const EGLint context_attributes[] = {
                    EGL_CONTEXT_MAJOR_VERSION, 4, EGL_CONTEXT_MINOR_VERSION, 3,
                    EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT, EGL_NONE,
                };
                context = eglCreateContext(display, egl_config, EGL_NO_CONTEXT, context_attributes);
                checkEgl(context != EGL_NO_CONTEXT, "Create camera OpenGL 4.3 context");
                checkEgl(eglBindAPI(previous_api), "Restore EGL client API");
                const CurrentEglContext current(display, surface, context);
                unsigned int mapped_count = 0;
                int mapped_device = -1;
                checkCudaCamera(cudaGLGetDevices(&mapped_count, &mapped_device, 1, cudaGLDeviceListAll),
                                "Identify CUDA device of camera OpenGL context");
                if (mapped_count == 1 && mapped_device == config.device) {
                    return;
                }
                failure = "EGL context is attached to a different CUDA device";
            } catch (const std::runtime_error& error) {
                failure = error.what();
            }
            eglBindAPI(previous_api);
            destroyContext();
        }
        throw std::runtime_error("No OpenGL 4.3 EGL context for CUDA device " +
                                 std::to_string(config.device) + (failure.empty() ? "" : ": " + failure));
    }

    void configureAtlas() {
        GLint max_texture = 0;
        GLint max_renderbuffer = 0;
        GLint max_viewport[2]{};
        glGetIntegerv(GL_MAX_TEXTURE_SIZE, &max_texture);
        glGetIntegerv(GL_MAX_RENDERBUFFER_SIZE, &max_renderbuffer);
        glGetIntegerv(GL_MAX_VIEWPORT_DIMS, max_viewport);
        const int max_columns = std::min({max_texture, max_renderbuffer, max_viewport[0]}) / config.width;
        const int max_rows = std::min({max_texture, max_renderbuffer, max_viewport[1]}) / config.height;
        if (max_columns < 1 || max_rows < 1) {
            throw std::invalid_argument("CUDA camera image exceeds OpenGL atlas limits");
        }
        const int requested = std::min(config.environments, config.chunk_size);
        const auto max_tiles = std::int64_t(max_columns) * max_rows;
        capacity = static_cast<int>(std::min(std::int64_t(requested), max_tiles));
        const double aspect = static_cast<double>(config.height) / config.width;
        const int square_columns = static_cast<int>(std::ceil(std::sqrt(capacity * aspect)));
        const int min_columns = 1 + (capacity - 1) / max_rows;
        columns = std::clamp(square_columns, min_columns, std::min(max_columns, capacity));
        rows = 1 + (capacity - 1) / columns;
        atlas_width = columns * config.width;
        atlas_height = rows * config.height;

        glGenFramebuffers(1, &framebuffer);
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
        glGenTextures(textures.size(), textures.data());
        constexpr std::array<GLenum, 2> formats = {GL_RGBA8, GL_R32F};
        constexpr std::array<GLenum, 2> attachments = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
        for (size_t index = 0; index < textures.size(); ++index) {
            glBindTexture(GL_TEXTURE_2D, textures[index]);
            glTexStorage2D(GL_TEXTURE_2D, 1, formats[index], atlas_width, atlas_height);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glFramebufferTexture2D(GL_FRAMEBUFFER, attachments[index], GL_TEXTURE_2D, textures[index], 0);
        }
        GLuint depth_buffer = 0;
        glGenRenderbuffers(1, &depth_buffer);
        glBindRenderbuffer(GL_RENDERBUFFER, depth_buffer);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32F, atlas_width, atlas_height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_buffer);
        glDrawBuffers(attachments.size(), attachments.data());
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            throw std::runtime_error("CUDA camera OpenGL framebuffer is incomplete");
        }

        for (size_t index = 0; index < textures.size(); ++index) {
            checkCudaCamera(cudaGraphicsGLRegisterImage(&output_resources[index], textures[index],
                                                        GL_TEXTURE_2D, cudaGraphicsRegisterFlagsReadOnly),
                            "Register CUDA camera OpenGL atlas");
        }
        const size_t pixels = size_t(atlas_width) * atlas_height;
        checkCudaCamera(cudaMalloc(&rgba_atlas, pixels * 4), "Allocate CUDA camera color atlas");
        checkCudaCamera(cudaMalloc(&depth_atlas, pixels * sizeof(float)), "Allocate CUDA camera depth atlas");
    }

    void uploadMeshes(const std::vector<CudaCameraMesh>& geometry) {
        meshes.resize(geometry.size());
        for (size_t index = 0; index < geometry.size(); ++index) {
            const auto& vertices = geometry[index].vertices;
            if (vertices.empty() || vertices.size() % 18 != 0 ||
                vertices.size() / 6 > size_t(std::numeric_limits<GLsizei>::max())) {
                throw std::invalid_argument("Invalid CUDA camera triangle mesh");
            }
            auto& mesh = meshes[index];
            mesh.count = static_cast<GLsizei>(vertices.size() / 6);
            glGenVertexArrays(1, &mesh.vao);
            glGenBuffers(1, &mesh.vbo);
            glBindVertexArray(mesh.vao);
            glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
            glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                                  reinterpret_cast<const void*>(3 * sizeof(float)));
        }
    }

    void initialize(const std::vector<CudaCameraMesh>& geometry) {
        if (config.device < 0 || config.environments < 1 || config.bodies < 1 ||
            config.width < 1 || config.height < 1 || config.chunk_size < 1 ||
            config.parent < -1 || config.parent >= config.bodies) {
            throw std::invalid_argument("Invalid CUDA camera dimensions, chunk size, or parent");
        }
        for (const auto& item : items) {
            if (item.mesh < 0 || size_t(item.mesh) >= geometry.size() ||
                item.body < -1 || item.body >= config.bodies) {
                throw std::invalid_argument("Invalid CUDA camera render item");
            }
        }
        const CurrentCudaDevice current_device(config.device);
        createContext();
        const CurrentEglContext current_context(display, surface, context);
        program = linkProgram();
        uploadMeshes(geometry);

        GLint64 max_ssbo = 0;
        glGetInteger64v(GL_MAX_SHADER_STORAGE_BLOCK_SIZE, &max_ssbo);
        const auto body_count = std::uint64_t(config.environments) * config.bodies;
        const auto storage_limit = std::min(
            std::uint64_t(max_ssbo), std::uint64_t(std::numeric_limits<GLint>::max()) * sizeof(float));
        if (body_count > storage_limit / (12 * sizeof(float)) ||
            std::uint64_t(config.environments) > storage_limit / (16 * sizeof(float))) {
            throw std::invalid_argument("CUDA camera transforms exceed OpenGL storage limits");
        }
        const std::array<std::uint64_t, 2> buffer_sizes = {
            body_count * 12 * sizeof(float), std::uint64_t(config.environments) * 16 * sizeof(float),
        };
        glGenBuffers(transform_buffers.size(), transform_buffers.data());
        for (size_t index = 0; index < transform_buffers.size(); ++index) {
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, transform_buffers[index]);
            glBufferData(GL_SHADER_STORAGE_BUFFER, buffer_sizes[index], nullptr, GL_DYNAMIC_DRAW);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, index, transform_buffers[index]);
            checkGl("Allocate CUDA camera OpenGL transform buffer");
            checkCudaCamera(cudaGraphicsGLRegisterBuffer(&input_resources[index], transform_buffers[index],
                                                         cudaGraphicsRegisterFlagsWriteDiscard),
                            "Register CUDA camera OpenGL transform buffer");
        }
        configureAtlas();

        glUseProgram(program);
        glUniform1i(glGetUniformLocation(program, "bodies"), config.bodies);
        glUniform2i(glGetUniformLocation(program, "atlas_tiles"), columns, rows);
        first_uniform = glGetUniformLocation(program, "first");
        body_uniform = glGetUniformLocation(program, "body");
        local_pose_uniform = glGetUniformLocation(program, "local_pose");
        color_uniform = glGetUniformLocation(program, "color");
        checker_uniform = glGetUniformLocation(program, "checker");
        glViewport(0, 0, atlas_width, atlas_height);
        glEnable(GL_DEPTH_TEST);
        glDepthRange(1.0, 0.0);
        glDepthFunc(GL_GREATER);
        for (int edge = 0; edge < 4; ++edge) {
            glEnable(GL_CLIP_DISTANCE0 + edge);
        }
        checkGl("Configure CUDA camera OpenGL renderer");
    }

    void draw(int first, int count) const {
        constexpr float background[] = {0.18f, 0.23f, 0.28f, 1.0f};
        constexpr float zero[] = {0.0f, 0.0f, 0.0f, 0.0f};
        glClearBufferfv(GL_COLOR, 0, background);
        glClearBufferfv(GL_COLOR, 1, zero);
        glClearBufferfv(GL_DEPTH, 0, zero);
        glUniform1i(first_uniform, first);
        for (const auto& item : items) {
            const auto& mesh = meshes[item.mesh];
            glUniform1i(body_uniform, item.body);
            glUniformMatrix4fv(local_pose_uniform, 1, GL_TRUE, item.local_pose.data());
            glUniform3fv(color_uniform, 1, item.color.data());
            glUniform1i(checker_uniform, item.checker);
            glBindVertexArray(mesh.vao);
            glDrawArraysInstanced(GL_TRIANGLES, 0, mesh.count, count);
        }
        checkGl("Render CUDA camera OpenGL atlas");
    }

    void capture(const double* poses, unsigned char* rgb, float* depth, bool* valid, cudaStream_t stream) {
        const std::lock_guard lock(mutex);
        const CurrentCudaDevice current_device(config.device);
        const CurrentEglContext current_context(display, surface, context);
        {
            MappedResources inputs(input_resources, stream);
            launchCameraTransforms(poses, inputs.buffer(0), inputs.buffer(1), config, stream);
            inputs.unmap();
        }
        for (int first = 0; first < config.environments;) {
            const int count = std::min(capacity, config.environments - first);
            draw(first, count);
            MappedResources outputs(output_resources, stream);
            const size_t active_height = size_t(1 + (count - 1) / columns) * config.height;
            const size_t color_pitch = size_t(atlas_width) * 4;
            const size_t depth_pitch = size_t(atlas_width) * sizeof(float);
            checkCudaCamera(cudaMemcpy2DFromArrayAsync(rgba_atlas, color_pitch, outputs.image(0), 0, 0,
                                                      color_pitch, active_height, cudaMemcpyDeviceToDevice, stream),
                            "Copy CUDA camera color atlas");
            checkCudaCamera(cudaMemcpy2DFromArrayAsync(depth_atlas, depth_pitch, outputs.image(1), 0, 0,
                                                      depth_pitch, active_height, cudaMemcpyDeviceToDevice, stream),
                            "Copy CUDA camera depth atlas");
            launchCameraPack(rgba_atlas, depth_atlas, rgb, depth, valid,
                             config.width, config.height, columns, first, count, stream);
            outputs.unmap();
            first += count;
        }
    }

    CudaCameraConfig config;
    const std::vector<CudaCameraItem> items;
    std::vector<Mesh> meshes;
    std::mutex mutex;
    EGLDisplay display = EGL_NO_DISPLAY;
    EGLContext context = EGL_NO_CONTEXT;
    EGLSurface surface = EGL_NO_SURFACE;
    bool tracked_display = false;
    bool initialized_display = false;
    GLuint program = 0;
    GLuint framebuffer = 0;
    std::array<GLuint, 2> transform_buffers{};
    std::array<GLuint, 2> textures{};
    std::array<cudaGraphicsResource*, 2> input_resources{};
    std::array<cudaGraphicsResource*, 2> output_resources{};
    unsigned char* rgba_atlas = nullptr;
    float* depth_atlas = nullptr;
    int capacity = 0;
    int columns = 0;
    int rows = 0;
    int atlas_width = 0;
    int atlas_height = 0;
    GLint first_uniform = -1;
    GLint body_uniform = -1;
    GLint local_pose_uniform = -1;
    GLint color_uniform = -1;
    GLint checker_uniform = -1;
};

CudaBatchCamera::CudaBatchCamera(CudaCameraConfig config, const std::vector<CudaCameraMesh>& meshes,
                                 const std::vector<CudaCameraItem>& items)
    : impl_(std::make_unique<Impl>(config, items)) {
    impl_->initialize(meshes);
}

CudaBatchCamera::~CudaBatchCamera() = default;

void CudaBatchCamera::capture(const double* poses, unsigned char* rgb, float* depth, bool* valid,
                             cudaStream_t stream) {
    impl_->capture(poses, rgb, depth, valid, stream);
}

const CudaCameraConfig& CudaBatchCamera::config() const {
    return impl_->config;
}

}  // namespace sire::sensor
