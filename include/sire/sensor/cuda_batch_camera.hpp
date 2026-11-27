#ifndef SIRE_SENSOR_CUDA_BATCH_CAMERA_HPP_
#define SIRE_SENSOR_CUDA_BATCH_CAMERA_HPP_

#include <array>
#include <memory>
#include <vector>

#include <cuda_runtime_api.h>

namespace sire::sensor {

struct CudaCameraConfig {
  int device, environments, bodies, parent, width, height, chunk_size;
  float fx, fy, cx, cy, near_plane, far_plane;
  std::array<float, 16> mount;
};

struct CudaCameraMesh {
  std::vector<float> vertices;  // Interleaved positions and normals.
};

struct CudaCameraItem {
  int mesh, body;
  std::array<float, 16> local_pose;
  std::array<float, 3> color;
  bool checker;
};

class CudaBatchCamera {
 public:
  CudaBatchCamera(CudaCameraConfig config,
                  const std::vector<CudaCameraMesh>& meshes,
                  const std::vector<CudaCameraItem>& items);
  ~CudaBatchCamera();

  // All pointers refer to contiguous storage on config().device. Outputs are
  // ordered by environment with the top image row first. Work uses stream.
  void capture(const double* poses, unsigned char* rgb, float* depth,
               bool* valid, cudaStream_t stream);
  const CudaCameraConfig& config() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace sire::sensor

#endif
