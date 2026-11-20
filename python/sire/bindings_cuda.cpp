#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <aris/dynamic/model.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "rigid_body_batch.hpp"
#include "sire/simulator/simulator.hpp"

namespace py = pybind11;
using namespace pybind11::literals;

namespace {

using sire::physics::contact::cuda::RigidBodyBatch;
using sire::physics::contact::cuda::RigidDynamicsStatus;
using sire::physics::contact::cuda::RigidEventClock;
using sire::physics::contact::cuda::RigidJointControl;

void checkCuda(cudaError_t status, const char* operation) {
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(status));
  }
}

class CudaDeviceGuard {
 public:
  explicit CudaDeviceGuard(int device) {
    checkCuda(cudaGetDevice(&previous_), "get current CUDA device");
    changed_ = device != previous_;
    if (changed_) checkCuda(cudaSetDevice(device), "select CUDA device");
  }

  ~CudaDeviceGuard() {
    if (changed_) (void)cudaSetDevice(previous_);
  }

  CudaDeviceGuard(const CudaDeviceGuard&) = delete;
  CudaDeviceGuard& operator=(const CudaDeviceGuard&) = delete;

 private:
  int previous_{0};
  bool changed_{false};
};

struct BatchDeleter {
  int device;

  void operator()(RigidBodyBatch* batch) const noexcept {
    int previous = device;
    const bool restore = cudaGetDevice(&previous) == cudaSuccess && previous != device;
    (void)cudaSetDevice(device);
    delete batch;
    if (restore) (void)cudaSetDevice(previous);
  }
};

class CudaArray {
 public:
  CudaArray(std::shared_ptr<RigidBodyBatch> owner, const void* data,
            std::vector<std::size_t> shape, const char* typestr,
            std::size_t byte_stride = 0)
      : owner_(std::move(owner)),
        data_(reinterpret_cast<std::uintptr_t>(data)),
        shape_(std::move(shape)),
        typestr_(typestr),
        byte_stride_(byte_stride) {}

  auto arrayInterface() const -> py::dict {
    py::dict info;
    info["version"] = 3;
    info["shape"] = py::tuple(py::cast(shape_));
    info["typestr"] = typestr_;
    // PyTorch requires a writable descriptor. These are views of simulator storage.
    info["data"] = py::make_tuple(data_, false);
    if (byte_stride_ != 0) info["strides"] = py::make_tuple(byte_stride_);
    return info;
  }

 private:
  std::shared_ptr<RigidBodyBatch> owner_;
  std::uintptr_t data_;
  std::vector<std::size_t> shape_;
  const char* typestr_;
  std::size_t byte_stride_;
};

template <typename T>
auto tensorPointer(const py::object& tensor, const py::module_& torch, const char* dtype,
                   int device, std::vector<std::size_t> shape, const char* name) -> const T* {
  if (tensor.is_none()) return nullptr;
  if (!py::isinstance(tensor, torch.attr("Tensor"))) {
    throw py::type_error(std::string(name) + " must be a torch.Tensor or None");
  }
  const auto tensor_device = tensor.attr("device");
  if (py::cast<std::string>(tensor_device.attr("type")) != "cuda" ||
      py::cast<int>(tensor_device.attr("index")) != device ||
      !tensor.attr("dtype").is(torch.attr(dtype)) ||
      !py::cast<bool>(tensor.attr("is_contiguous")()) ||
      py::cast<std::vector<std::size_t>>(tensor.attr("shape")) != shape) {
    throw py::value_error(std::string(name) + " must match the expected contiguous CUDA tensor shape, dtype, and device");
  }
  return reinterpret_cast<const T*>(py::cast<std::uintptr_t>(tensor.attr("data_ptr")()));
}

class SireCudaBatchStepper {
 public:
  SireCudaBatchStepper(
      sire::simulator::Simulator& simulator, std::size_t num_envs, double physics_dt,
      double control_dt, const std::vector<std::size_t>& motion_indices,
      const std::vector<double>& p_gains, const std::vector<double>& d_gains,
      const std::vector<double>& default_positions, double action_scale, int device)
      : device_(device) {
    auto& engine = simulator.physicsEngine();
    auto& model = simulator.model();
    if (engine.currentModel() != &model) {
      throw py::value_error("Call Simulator.init() before creating the CUDA stepper");
    }
    const auto count = model.motionPool().size();
    if (motion_indices.size() != count || p_gains.size() != count ||
        d_gains.size() != count || default_positions.size() != count ||
        !std::isfinite(action_scale)) {
      throw py::value_error("Control parameters must cover the model's motion pool");
    }
    std::vector<RigidJointControl> control(count);
    std::vector<bool> seen(count);
    for (std::size_t action = 0; action < count; ++action) {
      const auto motion = motion_indices[action];
      if (motion >= count || seen[motion] || !std::isfinite(p_gains[action]) ||
          !std::isfinite(d_gains[action]) || !std::isfinite(default_positions[action])) {
        throw py::value_error("Motion indices must be a permutation and PD parameters must be finite");
      }
      seen[motion] = true;
      control[motion] = {action, p_gains[action], d_gains[action], default_positions[action],
                         action_scale};
    }
    if (device < 0) throw py::value_error("CUDA device must be a nonnegative index");
    CudaDeviceGuard guard(device_);
    batch_ = std::shared_ptr<RigidBodyBatch>(
        new RigidBodyBatch(engine, num_envs, physics_dt, control_dt, control),
        BatchDeleter{device_});
  }

  auto numEnvs() const -> std::size_t { return batch_->batchSize(); }
  auto bodyCount() const -> std::size_t { return batch_->bodyCount(); }
  auto jointCount() const -> std::size_t { return batch_->jointCount(); }
  auto rootBody() const -> std::size_t { return batch_->rootBody(); }
  auto device() const -> int { return device_; }

  void step(const py::object& actions, std::uintptr_t stream) {
    const float* input = nullptr;
    if (!actions.is_none()) {
      input = tensorPointer<float>(actions, py::module_::import("torch"), "float32", device_,
                                   {numEnvs(), jointCount()}, "actions");
    }
    CudaDeviceGuard guard(device_);
    batch_->step(input, reinterpret_cast<cudaStream_t>(stream));
  }

  void reset(const py::object& mask, const py::object& base_poses,
             const py::object& base_velocities, const py::object& joint_positions,
             const py::object& joint_rates, std::uintptr_t stream) {
    py::module_ torch;
    if (!mask.is_none() || !base_poses.is_none() || !base_velocities.is_none() ||
        !joint_positions.is_none() || !joint_rates.is_none()) {
      torch = py::module_::import("torch");
    }
    const auto* selected = tensorPointer<bool>(mask, torch, "bool", device_, {numEnvs()}, "mask");
    const auto* poses = tensorPointer<double>(base_poses, torch, "float64", device_,
                                              {numEnvs(), 3, 4}, "base_poses");
    const auto* velocities = tensorPointer<double>(base_velocities, torch, "float64", device_,
                                                   {numEnvs(), 6}, "base_velocities");
    const auto* positions = tensorPointer<double>(joint_positions, torch, "float64", device_,
                                                  {numEnvs(), jointCount()}, "joint_positions");
    const auto* rates = tensorPointer<double>(joint_rates, torch, "float64", device_,
                                              {numEnvs(), jointCount()}, "joint_rates");
    CudaDeviceGuard guard(device_);
    batch_->reset(selected, poses, velocities, positions, rates,
                  reinterpret_cast<cudaStream_t>(stream));
  }

  auto outputs() const -> py::dict {
    static_assert(sizeof(double) == 8 && sizeof(bool) == 1);
    static_assert(std::is_same_v<std::underlying_type_t<RigidDynamicsStatus>, std::int32_t>);
    static_assert(std::is_standard_layout_v<RigidEventClock>);
    using EventCount = decltype(RigidEventClock::event_count);
    static_assert(sizeof(EventCount) == 8 || sizeof(EventCount) == 4);
    constexpr auto event_count_dtype = sizeof(EventCount) == 8 ? "<i8" : "<i4";
    const auto state = batch_->state();
    const auto envs = numEnvs(), bodies = bodyCount(), joints = jointCount();
    const auto* clocks = reinterpret_cast<const char*>(state.clocks);
    const auto* time = clocks + offsetof(RigidEventClock, time);
    const auto* event_count = clocks + offsetof(RigidEventClock, event_count);
    py::dict result;
    result["poses"] = CudaArray(batch_, state.poses, {envs, bodies, 3, 4}, "<f8");
    result["velocities"] = CudaArray(batch_, state.velocities, {envs, bodies, 6}, "<f8");
    result["positions"] = CudaArray(batch_, state.positions, {envs, joints}, "<f8");
    result["rates"] = CudaArray(batch_, state.rates, {envs, joints}, "<f8");
    result["motor_torques"] = CudaArray(batch_, state.motor_torques, {envs, joints}, "<f8");
    result["contact_wrenches"] = CudaArray(batch_, state.contact_wrenches, {envs, bodies, 6}, "<f8");
    result["ground_contacts"] = CudaArray(batch_, state.ground_contacts, {envs, bodies}, "|b1");
    result["status"] = CudaArray(batch_, state.status, {envs}, "<i4");
    result["time"] = CudaArray(batch_, time, {envs}, "<f8", sizeof(RigidEventClock));
    result["event_count"] =
        CudaArray(batch_, event_count, {envs}, event_count_dtype, sizeof(RigidEventClock));
    return result;
  }

 private:
  int device_;
  std::shared_ptr<RigidBodyBatch> batch_;
};

}  // namespace

void init_cuda_rl(py::module& m) {
  py::class_<CudaArray>(m, "_SireCudaArray")
      .def_property_readonly("__cuda_array_interface__", &CudaArray::arrayInterface);
  py::class_<SireCudaBatchStepper>(m, "SireCudaBatchStepper")
      .def(py::init<sire::simulator::Simulator&, std::size_t, double, double,
                    const std::vector<std::size_t>&, const std::vector<double>&,
                    const std::vector<double>&, const std::vector<double>&, double, int>(),
           "simulator"_a, "num_envs"_a, "physics_dt"_a, "control_dt"_a, "motion_indices"_a,
           "p_gains"_a, "d_gains"_a, "default_positions"_a, "action_scale"_a, "device"_a = 0)
      .def("step", &SireCudaBatchStepper::step, "actions"_a = py::none(), "stream"_a = 0,
           "Enqueue a control interval. Keep the actions alive until the stream finishes using them.")
      .def("reset", &SireCudaBatchStepper::reset, "mask"_a = py::none(),
           "base_poses"_a = py::none(), "base_velocities"_a = py::none(),
           "joint_positions"_a = py::none(), "joint_rates"_a = py::none(), "stream"_a = 0,
           "Enqueue resets. Keep reset inputs alive until the stream finishes using them.")
      .def("outputs", &SireCudaBatchStepper::outputs,
           "Live views of simulator storage. Treat them as read-only and order access on the "
           "step/reset stream. Poses are [batch, bodies, 3, 4] in row-major order. "
           "Time and event count are [batch] with the native clock record stride.")
      .def_property_readonly("numEnvs", &SireCudaBatchStepper::numEnvs)
      .def_property_readonly("bodyCount", &SireCudaBatchStepper::bodyCount)
      .def_property_readonly("jointCount", &SireCudaBatchStepper::jointCount)
      .def_property_readonly("rootBody", &SireCudaBatchStepper::rootBody)
      .def_property_readonly("device", &SireCudaBatchStepper::device);
}
