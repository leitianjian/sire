#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sire/sensor/cuda_batch_camera.hpp"

namespace py = pybind11;
using namespace pybind11::literals;

namespace {

using sire::sensor::CudaBatchCamera;
using sire::sensor::CudaCameraConfig;
using sire::sensor::CudaCameraItem;
using sire::sensor::CudaCameraMesh;
using sire::sensor::CudaGlTexture;

using MeshVertices = py::array_t<float, py::array::c_style>;

[[noreturn]] void argumentError(PyObject* type, const std::string& message) {
  // Preserve argument errors through the module's general C++ exception translator.
  PyErr_SetString(type, message.c_str());
  throw py::error_already_set();
}

std::unique_ptr<CudaBatchCamera> makeCamera(
    int device, int environments, int bodies, int parent, int width, int height, int chunk_size,
    const std::array<float, 4>& intrinsics, float near_plane, float far_plane,
    const std::array<float, 16>& mount, const std::vector<MeshVertices>& meshes,
    const std::vector<py::sequence>& items) {
  const auto& [fx, fy, cx, cy] = intrinsics;
  const CudaCameraConfig config{device, environments, bodies, parent, width, height, chunk_size,
                                fx, fy, cx, cy, near_plane, far_plane, mount};
  std::vector<CudaCameraMesh> native_meshes;
  native_meshes.reserve(meshes.size());
  for (const auto& mesh : meshes) {
    if (mesh.ndim() != 2 || mesh.shape(1) != 6 || mesh.size() == 0 || mesh.shape(0) % 3 != 0) {
      argumentError(PyExc_ValueError, "Camera mesh must contain triangle vertices with "
                                      "interleaved positions and normals [vertices, 6]");
    }
    native_meshes.push_back({std::vector<float>(mesh.data(), mesh.data() + mesh.size())});
  }

  std::vector<CudaCameraItem> native_items;
  native_items.reserve(items.size());
  for (const auto& item : items) {
    const auto size = py::len(item);
    if (size != 5 && size != 6) {
      argumentError(PyExc_ValueError, "Camera items must have five or six fields");
    }
    native_items.push_back({py::cast<int>(item[0]), py::cast<int>(item[1]),
                            py::cast<std::array<float, 16>>(item[2]),
                            py::cast<std::array<float, 3>>(item[3]), py::cast<bool>(item[4]),
                            size == 6 && py::cast<bool>(item[5])});
  }

  py::gil_scoped_release release;
  return std::make_unique<CudaBatchCamera>(config, native_meshes, native_items);
}

template <typename T>
T* tensorPointer(const py::object& tensor, const py::module_& torch, const char* dtype,
                 int device, const std::vector<int>& shape, const char* name) {
  if (!py::isinstance(tensor, torch.attr("Tensor"))) {
    argumentError(PyExc_TypeError, std::string(name) + " must be a torch.Tensor");
  }
  const auto tensor_device = tensor.attr("device");
  if (py::cast<std::string>(tensor_device.attr("type")) != "cuda" ||
      py::cast<int>(tensor_device.attr("index")) != device ||
      !tensor.attr("dtype").is(torch.attr(dtype)) ||
      !py::cast<bool>(tensor.attr("is_contiguous")()) ||
      py::cast<std::vector<int>>(tensor.attr("shape")) != shape) {
    argumentError(PyExc_ValueError, std::string(name) +
                                       " must match the expected contiguous CUDA tensor shape, "
                                       "dtype, and device");
  }
  return reinterpret_cast<T*>(py::cast<std::uintptr_t>(tensor.attr("data_ptr")()));
}

void capture(CudaBatchCamera& camera, const py::object& poses, const py::object& rgb,
             const py::object& depth, const py::object& valid, std::uintptr_t stream,
             const py::object& foreground) {
  static_assert(sizeof(bool) == 1 && sizeof(float) == 4 && sizeof(double) == 8);
  const auto torch = py::module_::import("torch");
  const auto& config = camera.config();
  const int device = config.device;
  const int environments = config.environments;
  const int height = config.height;
  const int width = config.width;
  const auto* poses_ptr = tensorPointer<const double>(
      poses, torch, "float64", device, {environments, config.bodies, 3, 4}, "poses");
  auto* rgb_ptr = tensorPointer<unsigned char>(
      rgb, torch, "uint8", device, {environments, height, width, 3}, "rgb");
  auto* depth_ptr = tensorPointer<float>(
      depth, torch, "float32", device, {environments, height, width}, "depth");
  auto* valid_ptr = tensorPointer<bool>(
      valid, torch, "bool", device, {environments, height, width}, "valid");
  auto* foreground_ptr = foreground.is_none() ? nullptr : tensorPointer<bool>(
      foreground, torch, "bool", device, {environments, height, width}, "foreground");

  py::gil_scoped_release release;
  camera.capture(poses_ptr, rgb_ptr, depth_ptr, valid_ptr, reinterpret_cast<cudaStream_t>(stream),
                 foreground_ptr);
}

}  // namespace

void init_cuda_camera(py::module& m) {
  py::class_<CudaBatchCamera>(m, "_SireCudaBatchCamera")
      .def(py::init(&makeCamera), "device"_a, "environments"_a, "bodies"_a, "parent"_a,
           "width"_a, "height"_a, "chunk_size"_a, "intrinsics"_a, "near_plane"_a,
           "far_plane"_a, "mount"_a, "meshes"_a, "items"_a)
      .def("capture", &capture, "poses"_a, "rgb"_a, "depth"_a, "valid"_a, "stream"_a,
           "foreground"_a = py::none());

  py::class_<CudaGlTexture>(m, "_SireCudaGlTexture")
      .def(py::init<unsigned int, int, int, int>(), "texture_id"_a, "device"_a,
           "width"_a, "height"_a)
      .def("upload", [](CudaGlTexture& texture, const py::object& rgba, std::uintptr_t stream) {
        const auto torch = py::module_::import("torch");
        const auto* pixels = tensorPointer<const unsigned char>(
            rgba, torch, "uint8", texture.device(), {texture.height(), texture.width(), 4}, "rgba");
        texture.upload(pixels, reinterpret_cast<cudaStream_t>(stream));
      }, "rgba"_a, "stream"_a)
      .def("close", &CudaGlTexture::close);
}
