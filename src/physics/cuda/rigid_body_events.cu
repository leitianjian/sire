#include "rigid_body_events.hpp"

#include <cmath>
#include <limits>

namespace sire::physics::contact::cuda {
namespace {

constexpr unsigned block_size = 128;
constexpr std::size_t max_events = 100000;

auto validIntervals(double physics_dt, double control_dt) -> bool {
  return std::isfinite(physics_dt) && physics_dt > 0.0 &&
         std::isfinite(control_dt) && control_dt > 0.0;
}

auto blockCount(std::size_t batch) -> unsigned {
  const auto count = (batch - 1) / block_size + 1;
  constexpr auto max_grid = static_cast<std::size_t>(std::numeric_limits<int>::max());
  return count <= max_grid ? static_cast<unsigned>(count) : 0;
}

__device__ auto environmentIndex() -> std::size_t {
  return static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
}

__device__ auto controlEventFirst(double next_control, double next_physics,
                                 double physics_dt, double control_dt) -> bool {
  const double tolerance = ::fmin(1e-6, 1e-3 * ::fmin(physics_dt, control_dt));
  return next_control < next_physics || ::fabs(next_control - next_physics) < tolerance;
}

__global__ void beginControlBatch(std::size_t batch, RigidEventClock* clocks,
                                  const RigidDynamicsStatus* statuses) {
  const auto env = environmentIndex();
  if (env >= batch) return;
  auto& clock = clocks[env];
  clock.done = statuses[env] != RigidDynamicsStatus::Success;
  if (!clock.done) clock.event_count = 0;
}

__global__ void prepareEventBatch(std::size_t batch, const RigidEventClock* clocks,
                                  const RigidDynamicsStatus* statuses, double physics_dt,
                                  double control_dt, ContactHistoryConfig* configs) {
  const auto env = environmentIndex();
  if (env >= batch) return;
  const auto& clock = clocks[env];
  auto& config = configs[env];
  config.suspended = clock.done || statuses[env] != RigidDynamicsStatus::Success;
  config.horizon = 0.0;
  if (config.suspended) return;
  const double next_control = clock.previous_control_time + control_dt;
  const double next_physics = clock.time + physics_dt;
  const double target = controlEventFirst(next_control, next_physics, physics_dt, control_dt)
                            ? next_control : next_physics;
  config.horizon = target - clock.time;
}

__device__ auto validStep(double step, double horizon) -> bool {
  return step > 0.0 && ::isfinite(step) && ::isfinite(horizon) && step <= horizon;
}

__global__ void selectEventStepBatch(
    std::size_t batch, const ContactHistoryConfig* configs,
    const GroundCollisionStatus* collision, const NormalContactResult* normal,
    const AdmmResult* admm, RigidDynamicsStatus* statuses, double* steps) {
  const auto env = environmentIndex();
  if (env >= batch) return;
  steps[env] = 0.0;
  const auto& config = configs[env];
  if (config.suspended || statuses[env] != RigidDynamicsStatus::Success) return;
  const auto step = normal[env].accepted_step;
  const auto result = admm[env].status;
  if (collision[env] != GroundCollisionStatus::Success ||
      normal[env].status != TrajectoryStatus::Success ||
      (result != AdmmStatus::Converged && result != AdmmStatus::IterationLimit) ||
      !validStep(step, config.horizon)) {
    statuses[env] = RigidDynamicsStatus::NumericalFailure;
    return;
  }
  steps[env] = step;
}

__global__ void finishEventBatch(
    std::size_t batch, const ContactHistoryConfig* configs, const double* steps,
    double physics_dt, double control_dt, RigidEventClock* clocks,
    RigidDynamicsStatus* statuses) {
  const auto env = environmentIndex();
  if (env >= batch) return;
  auto& clock = clocks[env];
  auto& status = statuses[env];
  if (status != RigidDynamicsStatus::Success) {
    clock.done = true;
    return;
  }
  const auto& config = configs[env];
  if (config.suspended) return;
  const double step = steps[env];
  const double next_time = clock.time + step;
  if (!validStep(step, config.horizon) || !::isfinite(next_time) || !(next_time > clock.time)) {
    status = RigidDynamicsStatus::NumericalFailure;
    clock.done = true;
    return;
  }
  const double next_control = clock.previous_control_time + control_dt;
  const double next_physics = clock.time + physics_dt;
  clock.time = next_time;
  ++clock.event_count;
  if (step >= config.horizon &&
      controlEventFirst(next_control, next_physics, physics_dt, control_dt)) {
    clock.previous_control_time = clock.time;
    clock.done = true;
  } else if (clock.event_count >= max_events) {
    status = RigidDynamicsStatus::NumericalFailure;
    clock.done = true;
  }
}

#if CUDART_VERSION >= 12030
__global__ void setEventGraphCondition(std::size_t batch, const RigidEventClock* clocks,
                                       cudaGraphConditionalHandle handle) {
  bool pending = false;
  for (auto env = static_cast<std::size_t>(threadIdx.x); env < batch; env += blockDim.x) {
    if (!clocks[env].done) {
      pending = true;
      break;
    }
  }
  const auto any_pending = __syncthreads_or(pending);
  if (threadIdx.x == 0) cudaGraphSetConditional(handle, any_pending != 0);
}
#endif

}  // namespace

auto launchBeginRigidControl(std::size_t batch, RigidEventClock* clocks,
                            RigidDynamicsStatus* statuses, cudaStream_t stream) -> cudaError_t {
  if (batch == 0) return cudaSuccess;
  const auto blocks = blockCount(batch);
  if (!blocks || !clocks || !statuses) return cudaErrorInvalidValue;
  beginControlBatch<<<blocks, block_size, 0, stream>>>(batch, clocks, statuses);
  return cudaGetLastError();
}

auto launchPrepareRigidEvent(std::size_t batch, const RigidEventClock* clocks,
                            const RigidDynamicsStatus* statuses, double physics_dt,
                            double control_dt, ContactHistoryConfig* configs,
                            cudaStream_t stream) -> cudaError_t {
  if (batch == 0) return cudaSuccess;
  const auto blocks = blockCount(batch);
  if (!blocks || !clocks || !statuses || !configs || !validIntervals(physics_dt, control_dt)) {
    return cudaErrorInvalidValue;
  }
  prepareEventBatch<<<blocks, block_size, 0, stream>>>(
      batch, clocks, statuses, physics_dt, control_dt, configs);
  return cudaGetLastError();
}

auto launchSelectRigidEventStep(
    std::size_t batch, const ContactHistoryConfig* configs,
    const GroundCollisionStatus* collision, const NormalContactResult* normal,
    const AdmmResult* admm, RigidDynamicsStatus* statuses, double* steps,
    cudaStream_t stream) -> cudaError_t {
  if (batch == 0) return cudaSuccess;
  const auto blocks = blockCount(batch);
  if (!blocks || !configs || !collision || !normal || !admm || !statuses || !steps) {
    return cudaErrorInvalidValue;
  }
  selectEventStepBatch<<<blocks, block_size, 0, stream>>>(
      batch, configs, collision, normal, admm, statuses, steps);
  return cudaGetLastError();
}

auto launchFinishRigidEvent(
    std::size_t batch, const ContactHistoryConfig* configs, const double* steps,
    double physics_dt, double control_dt, RigidEventClock* clocks,
    RigidDynamicsStatus* statuses, cudaStream_t stream) -> cudaError_t {
  if (batch == 0) return cudaSuccess;
  const auto blocks = blockCount(batch);
  if (!blocks || !configs || !steps || !clocks || !statuses ||
      !validIntervals(physics_dt, control_dt)) return cudaErrorInvalidValue;
  finishEventBatch<<<blocks, block_size, 0, stream>>>(
      batch, configs, steps, physics_dt, control_dt, clocks, statuses);
  return cudaGetLastError();
}

#if CUDART_VERSION >= 12030
auto launchSetRigidEventGraphCondition(std::size_t batch, const RigidEventClock* clocks,
                                      cudaGraphConditionalHandle handle,
                                      cudaStream_t stream) -> cudaError_t {
  if (!handle || (batch != 0 && !clocks) ||
      batch > std::numeric_limits<std::size_t>::max() / sizeof(RigidEventClock)) {
    return cudaErrorInvalidValue;
  }
  setEventGraphCondition<<<1, block_size, 0, stream>>>(batch, clocks, handle);
  return cudaGetLastError();
}
#endif

}  // namespace sire::physics::contact::cuda
