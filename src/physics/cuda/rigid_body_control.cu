#include "rigid_body_control.hpp"

#include <cmath>
#include <limits>

namespace sire::physics::contact::cuda {
namespace {

constexpr int block_size = 128;

__device__ bool validBounds(double lower, double upper) {
  return lower <= upper && lower < INFINITY && upper > -INFINITY;
}

__device__ double desiredTorque(
    const RigidJointControl& control, double action, double position, double rate) {
  const double scaled_action = action * control.action_scale;
  return control.stiffness * (scaled_action + control.default_position - position) -
         control.damping * rate;
}

__device__ bool projectJoint(
    const RigidBodyJointLimits& limits, double tolerance, double& position, double& rate) {
  const bool at_lower = tolerance == 0.0 ? position <= limits.min_position
                                          : position < limits.min_position - tolerance;
  const bool at_upper = tolerance == 0.0 ? position >= limits.max_position
                                          : position > limits.max_position + tolerance;
  bool changed = false;
  if (at_lower) {
    if (position < limits.min_position) {
      position = limits.min_position;
      changed = true;
    }
    if (rate < 0.0) {
      rate = 0.0;
      changed = true;
    }
  } else if (at_upper) {
    if (position > limits.max_position) {
      position = limits.max_position;
      changed = true;
    }
    if (rate > 0.0) {
      rate = 0.0;
      changed = true;
    }
  }
  return changed;
}

template <bool PrepareTorques>
__global__ void prepareJointControls(
    std::size_t batch, std::size_t joint_count, const RigidBodyJointLimits* limits,
    RigidBodyLimitSettings settings, const ContactHistoryConfig* configs,
    const float* actions, const RigidJointControl* control, double* positions,
    double* rates, double* motor_torques, bool* rebuild, RigidDynamicsStatus* status) {
  const auto env = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (env >= batch) return;
  rebuild[env] = false;
  if (status[env] != RigidDynamicsStatus::Success || joint_count == 0) return;
  if constexpr (PrepareTorques) {
    const double horizon = configs[env].horizon;
    if (horizon <= 0.0) return;
    if (!isfinite(horizon)) {
      status[env] = RigidDynamicsStatus::InvalidInput;
      return;
    }
  }

  const bool apply_limits = settings.method != RigidBodyLimitMethod::Disabled;
  if (!PrepareTorques && !apply_limits) return;
  const double tolerance = settings.method == RigidBodyLimitMethod::ShiftedNcp
                               ? settings.emergency_tolerance
                               : 0.0;
  const auto offset = env * joint_count;
  positions += offset;
  rates += offset;
  if constexpr (PrepareTorques) {
    motor_torques += offset;
    if (actions) actions += offset;
  }

  for (std::size_t i = 0; i < joint_count; ++i) {
    if (!isfinite(positions[i]) || !isfinite(rates[i])) {
      status[env] = RigidDynamicsStatus::NumericalFailure;
      return;
    }
    if (apply_limits && !validBounds(limits[i].min_position, limits[i].max_position)) {
      status[env] = RigidDynamicsStatus::InvalidInput;
      return;
    }
    if constexpr (PrepareTorques) {
      const auto& pd = control[i];
      if (pd.action >= joint_count || !isfinite(pd.stiffness) || !isfinite(pd.damping) ||
          !isfinite(pd.action_scale) || !isfinite(pd.default_position) ||
          !validBounds(limits[i].min_force, limits[i].max_force)) {
        status[env] = RigidDynamicsStatus::InvalidInput;
        return;
      }
      const double action = actions ? static_cast<double>(actions[pd.action]) : 0.0;
      if (!isfinite(action) || !isfinite(desiredTorque(pd, action, positions[i], rates[i]))) {
        status[env] = RigidDynamicsStatus::NumericalFailure;
        return;
      }
    }
  }

  bool changed = false;
  for (std::size_t i = 0; i < joint_count; ++i) {
    if constexpr (PrepareTorques) {
      const auto& pd = control[i];
      const double action = actions ? static_cast<double>(actions[pd.action]) : 0.0;
      const double desired = desiredTorque(pd, action, positions[i], rates[i]);
      motor_torques[i] = fmin(fmax(desired, limits[i].min_force), limits[i].max_force);
    }
    if (apply_limits) changed |= projectJoint(limits[i], tolerance, positions[i], rates[i]);
  }
  rebuild[env] = changed;
}

auto validArguments(std::size_t batch, std::size_t joints, const RigidBodyJointLimits* limits,
                    RigidBodyLimitSettings settings, const double* positions, const double* rates,
                    const bool* rebuild, const RigidDynamicsStatus* status) -> bool {
  const bool valid_method = settings.method == RigidBodyLimitMethod::Disabled ||
      settings.method == RigidBodyLimitMethod::Projection ||
      (settings.method == RigidBodyLimitMethod::ShiftedNcp &&
       std::isfinite(settings.emergency_tolerance) && settings.emergency_tolerance >= 0.0);
  constexpr auto max_elements = std::numeric_limits<std::size_t>::max() / sizeof(double);
  return valid_method && batch <= std::numeric_limits<int>::max() &&
         joints <= max_elements / batch && rebuild && status &&
         (joints == 0 || (limits && positions && rates));
}

}  // namespace

auto launchPrepareRigidJointControls(
    std::size_t batch, std::size_t joint_count, const RigidBodyJointLimits* limits,
    RigidBodyLimitSettings settings, const ContactHistoryConfig* configs,
    const float* actions, const RigidJointControl* control, double* positions,
    double* rates, double* motor_torques, bool* rebuild, RigidDynamicsStatus* status,
    cudaStream_t stream) -> cudaError_t {
  if (batch == 0) return cudaSuccess;
  if (!validArguments(batch, joint_count, limits, settings, positions, rates, rebuild, status) ||
      !configs || (joint_count != 0 && (!control || !motor_torques))) {
    return cudaErrorInvalidValue;
  }
  const auto blocks = static_cast<unsigned>((batch + block_size - 1) / block_size);
  prepareJointControls<true><<<blocks, block_size, 0, stream>>>(
      batch, joint_count, limits, settings, configs, actions, control, positions,
      rates, motor_torques, rebuild, status);
  return cudaGetLastError();
}

auto launchProjectRigidJointLimits(
    std::size_t batch, std::size_t joint_count, const RigidBodyJointLimits* limits,
    RigidBodyLimitSettings settings, double* positions, double* rates, bool* rebuild,
    RigidDynamicsStatus* status, cudaStream_t stream) -> cudaError_t {
  if (batch == 0) return cudaSuccess;
  if (!validArguments(batch, joint_count, limits, settings, positions, rates, rebuild, status)) {
    return cudaErrorInvalidValue;
  }
  const auto blocks = static_cast<unsigned>((batch + block_size - 1) / block_size);
  prepareJointControls<false><<<blocks, block_size, 0, stream>>>(
      batch, joint_count, limits, settings, nullptr, nullptr, nullptr, positions,
      rates, nullptr, rebuild, status);
  return cudaGetLastError();
}

}  // namespace sire::physics::contact::cuda
