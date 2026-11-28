#ifndef SIRE_SENSOR_CUDA_BATCH_CAMERA_KERNELS_HPP_
#define SIRE_SENSOR_CUDA_BATCH_CAMERA_KERNELS_HPP_

#include "sire/sensor/cuda_batch_camera.hpp"

namespace sire::sensor {

void checkCudaCamera(cudaError_t result, const char* operation);
void launchCameraTransforms(const double* poses, float* body_rows,
                            float* view_projections,
                            const CudaCameraConfig& config, cudaStream_t stream);
void launchCameraPack(const unsigned char* rgba_atlas, const float* depth_atlas,
                      unsigned char* rgb, float* depth, bool* valid, bool* foreground, int width,
                      int height, int columns, int first, int count,
                      cudaStream_t stream);

}  // namespace sire::sensor

#endif
