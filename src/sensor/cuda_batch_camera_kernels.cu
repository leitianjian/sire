#include "cuda_batch_camera_kernels.hpp"

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>

#include <cuda_runtime.h>

namespace sire::sensor {
namespace {

constexpr int kThreads = 256;

struct CameraMatrices {
  float mount[16];
  float projection[16];
};

__global__ void castBodyRows(const double* poses, float* rows, std::size_t size) {
  const std::size_t i = blockIdx.x * static_cast<std::size_t>(blockDim.x) + threadIdx.x;
  if (i < size) rows[i] = static_cast<float>(poses[i]);
}

__global__ void cameraViewProjections(const float* body_rows, float* view_projections,
                                      int environments, int bodies, int parent,
                                      CameraMatrices matrices) {
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= environments) return;

  float camera[12];
  if (parent >= 0) {
    const float* body = body_rows + (static_cast<std::size_t>(index) * bodies + parent) * 12;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 4; ++col) {
        camera[row * 4 + col] = body[row * 4] * matrices.mount[col]
            + body[row * 4 + 1] * matrices.mount[4 + col]
            + body[row * 4 + 2] * matrices.mount[8 + col];
        if (col == 3) camera[row * 4 + col] += body[row * 4 + 3];
      }
    }
  } else {
    for (int i = 0; i < 12; ++i) camera[i] = matrices.mount[i];
  }

  // The inverse of a rigid transform is [R^T, -R^T t]. Both the input
  // transform and the shader's output matrix use row-major storage.
  float view[16]{};
  view[15] = 1.0f;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      view[row * 4 + col] = camera[col * 4 + row];
      view[row * 4 + 3] -= view[row * 4 + col] * camera[col * 4 + 3];
    }
  }

  float* destination = view_projections + static_cast<std::size_t>(index) * 16;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      float value = 0.0f;
      for (int inner = 0; inner < 4; ++inner)
        value += matrices.projection[row * 4 + inner] * view[inner * 4 + col];
      destination[row * 4 + col] = value;
    }
  }
}

__global__ void packImages(const uchar4* rgba_atlas, const float* depth_atlas,
                           unsigned char* rgb, float* depth, bool* valid,
                           int width, int height, int columns, int first,
                           std::size_t count) {
  const std::size_t i = blockIdx.x * static_cast<std::size_t>(blockDim.x) + threadIdx.x;
  if (i >= count) return;

  const std::size_t pixels = static_cast<std::size_t>(width) * height;
  const std::size_t tile = i / pixels;
  const std::size_t x = (tile % columns) * width + i % width;
  const std::size_t y = (tile / columns) * height + height - 1 - (i % pixels) / width;
  const std::size_t source = y * (static_cast<std::size_t>(width) * columns) + x;
  const std::size_t output = static_cast<std::size_t>(first) * pixels + i;
  const uchar4 color = rgba_atlas[source];
  rgb[output * 3] = color.x;
  rgb[output * 3 + 1] = color.y;
  rgb[output * 3 + 2] = color.z;
  const float z = depth_atlas[source];
  valid[output] = z > 0.0f;
  depth[output] = valid[output] ? z : __int_as_float(0x7fc00000);
}

unsigned int blocks(std::size_t count) {
  return static_cast<unsigned int>((count + kThreads - 1) / kThreads);
}

}  // namespace

void checkCudaCamera(cudaError_t result, const char* operation) {
  if (result != cudaSuccess)
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(result));
}

void launchCameraTransforms(const double* poses, float* body_rows,
                            float* view_projections,
                            const CudaCameraConfig& config, cudaStream_t stream) {
  const float near = config.near_plane;
  const float far = config.far_plane;
  CameraMatrices matrices{
      {}, {2.0f * config.fx / config.width, 0.0f, 1.0f - 2.0f * config.cx / config.width, 0.0f,
           0.0f, 2.0f * config.fy / config.height, 2.0f * config.cy / config.height - 1.0f, 0.0f,
           0.0f, 0.0f, -(far + near) / (far - near), -2.0f * far * near / (far - near),
           0.0f, 0.0f, -1.0f, 0.0f}};
  std::copy(config.mount.begin(), config.mount.end(), matrices.mount);
  const auto count = static_cast<std::size_t>(config.environments) * config.bodies * 12;
  castBodyRows<<<blocks(count), kThreads, 0, stream>>>(poses, body_rows, count);
  checkCudaCamera(cudaGetLastError(), "launch camera pose conversion");
  cameraViewProjections<<<blocks(config.environments), kThreads, 0, stream>>>(
      body_rows, view_projections, config.environments, config.bodies,
      config.parent, matrices);
  checkCudaCamera(cudaGetLastError(), "launch camera view matrices");
}

void launchCameraPack(const unsigned char* rgba_atlas, const float* depth_atlas,
                      unsigned char* rgb, float* depth, bool* valid, int width,
                      int height, int columns, int first, int count,
                      cudaStream_t stream) {
  const auto pixels = static_cast<std::size_t>(count) * width * height;
  packImages<<<blocks(pixels), kThreads, 0, stream>>>(
      reinterpret_cast<const uchar4*>(rgba_atlas), depth_atlas, rgb, depth,
      valid, width, height, columns, first, pixels);
  checkCudaCamera(cudaGetLastError(), "launch RGB-D packing");
}

}  // namespace sire::sensor
