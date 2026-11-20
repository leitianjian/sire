#include "rigid_body_limits.hpp"

#include <cfloat>
#include <cmath>
#include <limits>

namespace sire::physics::contact::cuda {
namespace {

constexpr unsigned threads_per_block = 128;
constexpr std::size_t floating_dof = 6;

// Workspace per environment: weighted joint rows of length dof, their Gram
// matrix, and the offset and reaction for each possible lower and upper stop.
auto constexpr workspaceStride(std::size_t joints) -> std::size_t {
  return joints * (2 * joints + floating_dof + 4);
}

__device__ auto setOffset(double position, double boundary, double sign,
                          double free_velocity, double dt, double recovery_horizon,
                          double recovery_factor) -> double {
  const double distance = sign * (position - boundary);
  const double target_velocity =
      distance >= 0.0 ? -distance / dt : -recovery_factor * distance / recovery_horizon;
  return sign * free_velocity - target_velocity;
}

// L^-1 times a unit joint row. Components before its diagonal are zero.
__device__ auto weightJointRow(const double* lower, std::size_t dof, std::size_t coordinate,
                               double* row) -> bool {
  for (std::size_t i = 0; i < coordinate; ++i) row[i] = 0.0;
  for (std::size_t i = coordinate; i < dof; ++i) {
    double value = i == coordinate ? 1.0 : 0.0;
    for (std::size_t j = coordinate; j < i; ++j) value -= lower[i * dof + j] * row[j];
    const double diagonal = lower[i * dof + i];
    if (!(diagonal > 0.0) || !::isfinite(diagonal)) return false;
    row[i] = value / diagonal;
    if (!::isfinite(row[i])) return false;
  }
  return true;
}

__device__ auto activeJoint(const double* offsets, std::size_t joint) -> bool {
  return ::isfinite(offsets[2 * joint]) || ::isfinite(offsets[2 * joint + 1]);
}

__device__ auto solveReactions(
    std::size_t joints, const RigidBodyJointLimits* limits, RigidBodyLimitSettings settings,
    double dt, double nominal_dt, const double* positions, const double* rates,
    const double* acceleration, const double* lower, double* reactions, double* scratch)
    -> RigidDynamicsStatus {
  const auto dof = floating_dof + joints;
  auto* offsets = scratch;
  auto* magnitudes = offsets + 2 * joints;
  auto* weighted_rows = magnitudes + 2 * joints;
  auto* inverse_mass = weighted_rows + joints * dof;
  const double recovery_horizon =
      ::isfinite(nominal_dt) && nominal_dt > 0.0 ? ::fmax(dt, nominal_dt) : dt;

  bool any_active = false;
  for (std::size_t i = 0; i < joints; ++i) {
    const double q = positions[i];
    const double v = rates[i];
    if (!::isfinite(q) || !::isfinite(v)) return RigidDynamicsStatus::InvalidInput;
    const double predicted = q + dt * v;
    const auto& bound = limits[i];
    const double min_threshold = bound.min_position + settings.activation_margin;
    const double max_threshold = bound.max_position - settings.activation_margin;
    const bool at_min = ::isfinite(bound.min_position) && (q <= min_threshold || predicted <= min_threshold);
    const bool at_max = ::isfinite(bound.max_position) && (q >= max_threshold || predicted >= max_threshold);
    offsets[2 * i] = offsets[2 * i + 1] = INFINITY;
    if (!at_min && !at_max) continue;
    any_active = true;

    const double free_velocity = v + dt * acceleration[i];
    if (at_min) {
      offsets[2 * i] = setOffset(q, bound.min_position, 1.0, free_velocity, dt,
                                 recovery_horizon, settings.recovery_factor);
      if (!::isfinite(offsets[2 * i])) return RigidDynamicsStatus::NumericalFailure;
    }
    if (at_max) {
      offsets[2 * i + 1] = setOffset(q, bound.max_position, -1.0, free_velocity, dt,
                                     recovery_horizon, settings.recovery_factor);
      if (!::isfinite(offsets[2 * i + 1])) return RigidDynamicsStatus::NumericalFailure;
    }
  }
  if (!any_active) return RigidDynamicsStatus::Success;

  double mass_scale = 0.0;
  for (std::size_t i = 0; i < joints; ++i) {
    if (!activeJoint(offsets, i)) continue;
    auto* row = weighted_rows + i * dof;
    if (!weightJointRow(lower, dof, floating_dof + i, row)) {
      return RigidDynamicsStatus::NumericalFailure;
    }
    for (std::size_t j = 0; j <= i; ++j) {
      if (!activeJoint(offsets, j)) continue;
      const auto* column = weighted_rows + j * dof;
      double value = 0.0;
      for (std::size_t k = floating_dof + i; k < dof; ++k) value += row[k] * column[k];
      if (!::isfinite(value)) return RigidDynamicsStatus::NumericalFailure;
      inverse_mass[i * joints + j] = inverse_mass[j * joints + i] = value;
    }
    mass_scale = ::fmax(mass_scale, ::fabs(inverse_mass[i * joints + i]));
  }

  for (std::size_t i = 0; i < 2 * joints; ++i) magnitudes[i] = 0.0;
  const double regularization = 64 * DBL_EPSILON * ::fmax(1.0, mass_scale);
  for (std::size_t iteration = 0; iteration < settings.max_iterations; ++iteration) {
    double residual = 0.0;
    for (std::size_t i = 0; i < 2 * joints; ++i) {
      if (!::isfinite(offsets[i])) continue;
      const auto joint = i / 2;
      const auto* row = inverse_mass + joint * joints;
      double velocity_error = offsets[i];
      for (std::size_t j = 0; j < 2 * joints; ++j) {
        if (!::isfinite(offsets[j])) continue;
        const double signed_mass = (i & 1) == (j & 1) ? row[j / 2] : -row[j / 2];
        velocity_error += dt * signed_mass * magnitudes[j];
      }
      const double diagonal = dt * ::fmax(row[joint], regularization);
      if (!(diagonal > 0.0) || !::isfinite(diagonal) || !::isfinite(velocity_error)) {
        return RigidDynamicsStatus::NumericalFailure;
      }
      const double previous = magnitudes[i];
      magnitudes[i] = ::fmin(settings.max_force, ::fmax(0.0, previous - velocity_error / diagonal));
      residual = ::fmax(residual, ::fabs(magnitudes[i] - previous) * diagonal);
    }
    if (residual <= settings.tolerance) break;
  }
  for (std::size_t i = 0; i < joints; ++i) reactions[i] = magnitudes[2 * i] - magnitudes[2 * i + 1];
  return RigidDynamicsStatus::Success;
}

__global__ void solveBatch(
    std::size_t batch, std::size_t joints, std::size_t workspace_stride,
    const RigidBodyJointLimits* limits, RigidBodyLimitSettings settings, double nominal_dt,
    const ContactHistoryConfig* configs, const double* positions, const double* rates,
    const double* acceleration, const double* lower, RigidDynamicsStatus* status,
    double* reactions, double* scratch) {
  const auto env = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (env >= batch) return;
  const auto offset = env * joints;
  for (std::size_t i = 0; i < joints; ++i) reactions[offset + i] = 0.0;
  if (status[env] != RigidDynamicsStatus::Success) return;
  const double dt = configs[env].horizon;
  if (!(dt > 0.0) || !::isfinite(dt)) return;
  const auto dof = floating_dof + joints;
  status[env] = solveReactions(joints, limits, settings, dt, nominal_dt, positions + offset,
                               rates + offset, acceleration + offset, lower + env * dof * dof,
                               reactions + offset, scratch + env * workspace_stride);
}

}  // namespace

auto jointLimitReactionWorkspaceSize(std::size_t batch, std::size_t joint_count) -> std::size_t {
  constexpr auto max_elements = std::numeric_limits<std::size_t>::max() / sizeof(double);
  if (batch == 0 || joint_count == 0 || joint_count > (max_elements - floating_dof - 4) / 2 ||
      joint_count > max_elements / batch / (2 * joint_count + floating_dof + 4)) {
    return 0;
  }
  return batch * workspaceStride(joint_count);
}

auto launchJointLimitReactions(
    std::size_t batch, std::size_t joint_count, const RigidBodyJointLimits* limits,
    RigidBodyLimitSettings settings, double nominal_dt, const ContactHistoryConfig* configs,
    const double* positions, const double* rates, const double* detached_joint_acceleration,
    const double* mass_factor, RigidDynamicsStatus* status, double* reactions, double* scratch,
    cudaStream_t stream) -> cudaError_t {
  if (batch == 0 || joint_count == 0) return cudaSuccess;
  constexpr auto max_elements = std::numeric_limits<std::size_t>::max() / sizeof(double);
  if (joint_count > max_elements / batch || !reactions) return cudaErrorInvalidValue;
  if (settings.method == RigidBodyLimitMethod::Disabled ||
      settings.method == RigidBodyLimitMethod::Projection) {
    return cudaMemsetAsync(reactions, 0, batch * joint_count * sizeof(double), stream);
  }
  if (settings.method != RigidBodyLimitMethod::ShiftedNcp ||
      batch > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      jointLimitReactionWorkspaceSize(batch, joint_count) == 0 ||
      !limits || !configs || !positions || !rates || !detached_joint_acceleration || !mass_factor ||
      !status || !scratch || !std::isfinite(settings.activation_margin) ||
      settings.activation_margin < 0.0 || !(settings.recovery_factor > 0.0) ||
      settings.recovery_factor > 1.0 || !std::isfinite(settings.max_force) ||
      !(settings.max_force > 0.0) || settings.max_iterations == 0 ||
      !std::isfinite(settings.tolerance) || !(settings.tolerance > 0.0)) {
    return cudaErrorInvalidValue;
  }
  const auto dof = floating_dof + joint_count;
  if (dof > max_elements / batch / dof) return cudaErrorInvalidValue;
  const auto blocks = static_cast<unsigned>((batch + threads_per_block - 1) / threads_per_block);
  solveBatch<<<blocks, threads_per_block, 0, stream>>>(
      batch, joint_count, workspaceStride(joint_count), limits, settings, nominal_dt, configs,
      positions, rates, detached_joint_acceleration, mass_factor, status, reactions, scratch);
  return cudaGetLastError();
}

}  // namespace sire::physics::contact::cuda
