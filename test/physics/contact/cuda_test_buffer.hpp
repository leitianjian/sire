#pragma once

#include <cstddef>
#include <stdexcept>
#include <vector>

#include <cuda_runtime_api.h>

namespace sire::test {

inline void checkCuda(cudaError_t error) {
  if (error != cudaSuccess) throw std::runtime_error(cudaGetErrorString(error));
}

template <class T>
class DeviceBuffer {
 public:
  explicit DeviceBuffer(std::size_t size) : size_(size) {
    checkCuda(cudaMalloc(reinterpret_cast<void**>(&data_), size_ * sizeof(T)));
  }
  ~DeviceBuffer() { cudaFree(data_); }
  DeviceBuffer(const DeviceBuffer&) = delete;
  DeviceBuffer& operator=(const DeviceBuffer&) = delete;

  auto data() const -> T* { return data_; }
  void upload(const std::vector<T>& data) {
    if (data.size() != size_) throw std::runtime_error("Wrong upload size");
    checkCuda(cudaMemcpy(data_, data.data(), size_ * sizeof(T), cudaMemcpyHostToDevice));
    // Pageable host copies can return after staging. Tests consume the data
    // on nonblocking streams, which do not wait on the default stream.
    checkCuda(cudaStreamSynchronize(nullptr));
  }
  auto download() const -> std::vector<T> {
    std::vector<T> data(size_);
    checkCuda(cudaMemcpy(data.data(), data_, size_ * sizeof(T), cudaMemcpyDeviceToHost));
    return data;
  }

 private:
  std::size_t size_;
  T* data_{nullptr};
};

}  // namespace sire::test
