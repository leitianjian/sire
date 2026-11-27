#include "rigid_body_reset.hpp"

#include <limits>

#include "rigid_body_math.hpp"

namespace sire::physics::contact::cuda {
namespace {

constexpr unsigned block_size = 128;

__global__ void resetRigidBodyBatch(
    std::size_t batch, std::size_t body_count, std::size_t joint_count,
    std::size_t root, RigidBodyResetSource source, RigidBodyResetState state,
    ContactHistory history) {
  const auto env = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (env >= batch || (source.mask && !source.mask[env])) return;

  state.status[env] = RigidDynamicsStatus::Success;
  state.clocks[env] = RigidEventClock{};
  auto& config = state.configs[env];
  config.detected = 0;
  config.horizon = 0.0;
  config.reset = false;
  config.suspended = false;
  // Zero counts invalidate the cached entries and any associated ADMM duals.
  history.penetration_counts[env] = 0;
  history.force_counts[env] = 0;

  const auto body_offset = env * body_count;
  for (std::size_t body = 0; body < body_count; ++body) {
    const auto index = body_offset + body;
    const auto* pose = body == root && source.base_poses
                           ? source.base_poses + 12 * env : source.poses + 12 * body;
    const auto* velocity = body == root && source.base_velocities
                               ? source.base_velocities + 6 * env : source.velocities + 6 * body;
    for (int i = 0; i < 12; ++i) state.poses[12 * index + i] = pose[i];
    for (int i = 0; i < 6; ++i) {
      state.velocities[6 * index + i] = velocity[i];
      state.contact_wrenches[6 * index + i] = 0.0;
    }
    state.ground_contacts[index] = false;
  }
  const auto joint_offset = env * joint_count;
  for (std::size_t joint = 0; joint < joint_count; ++joint) {
    const auto index = joint_offset + joint;
    state.positions[index] = source.joint_positions ? source.joint_positions[index]
                                                    : source.positions[joint];
    state.rates[index] = source.joint_rates ? source.joint_rates[index] : source.rates[joint];
    state.motor_torques[index] = 0.0;
  }
}

__global__ void setDomainParameters(
    std::size_t batch, const double* friction, const double* base_mass_offsets,
    const bool* mask, const RigidBody* nominal_base, double* friction_overrides,
    double* base_inertias) {
  const auto env = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (env >= batch || (mask && !mask[env])) return;
  if (friction) friction_overrides[env] = friction[env];
  if (base_mass_offsets) {
    const auto* nominal = nominal_base->inertia;
    auto* inertia = base_inertias + 10 * env;
    const double mass = nominal[0] + base_mass_offsets[env];
    if (mass == nominal[0]) {
      for (int component = 0; component < 10; ++component) inertia[component] = nominal[component];
      return;
    }
    const double delta = mass - nominal[0];
    const double cx = nominal[1] / nominal[0];
    const double cy = nominal[2] / nominal[0];
    const double cz = nominal[3] / nominal[0];
    inertia[0] = mass;
    inertia[1] = mass * cx;
    inertia[2] = mass * cy;
    inertia[3] = mass * cz;
    inertia[4] = nominal[4] + delta * (cy * cy + cz * cz);
    inertia[5] = nominal[5] + delta * (cx * cx + cz * cz);
    inertia[6] = nominal[6] + delta * (cx * cx + cy * cy);
    inertia[7] = nominal[7] - delta * cx * cy;
    inertia[8] = nominal[8] - delta * cx * cz;
    inertia[9] = nominal[9] - delta * cy * cz;
  }
}

__global__ void pushBase(
    std::size_t batch, std::size_t body_count, std::size_t root, std::size_t ground,
    const double* xy_velocity, const double* poses, double* velocities,
    RigidDynamicsStatus* status) {
  const auto env = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (env >= batch || status[env] != RigidDynamicsStatus::Success) return;
  const auto* target = xy_velocity + 2 * env;
  if (!::isfinite(target[0]) || !::isfinite(target[1])) {
    status[env] = RigidDynamicsStatus::InvalidInput;
    return;
  }
  const auto* base_pose = poses + 12 * (env * body_count + root);
  auto* twists = velocities + 6 * env * body_count;
  const double point[] = {base_pose[3], base_pose[7], base_pose[11]};
  double angular_velocity[3];
  detail::rigidCross3(twists + 6 * root + 3, point, angular_velocity);
  const double base_linear[] = {target[0] - angular_velocity[0],
                                target[1] - angular_velocity[1], -angular_velocity[2]};
  double shift[3];
  for (int axis = 0; axis < 3; ++axis) shift[axis] = base_linear[axis] - twists[6 * root + axis];
  for (std::size_t body = 0; body < body_count; ++body) {
    if (body == ground) continue;
    for (int axis = 0; axis < 3; ++axis) {
      twists[6 * body + axis] = body == root ? base_linear[axis] : twists[6 * body + axis] + shift[axis];
    }
  }
}

}  // namespace

auto launchResetRigidBodyBatch(
    std::size_t batch, std::size_t body_count, std::size_t joint_count,
    std::size_t root, RigidBodyResetSource source, RigidBodyResetState state,
    ContactHistory history, cudaStream_t stream) -> cudaError_t {
  if (batch == 0) return cudaSuccess;
  constexpr auto max_doubles = std::numeric_limits<std::size_t>::max() / sizeof(double);
  const auto blocks = (batch - 1) / block_size + 1;
  if (blocks > std::numeric_limits<int>::max() || body_count < 2 ||
      body_count > max_doubles / batch / 12 || root >= body_count ||
      joint_count > body_count - 2 || !source.poses || !source.velocities ||
      !state.poses || !state.velocities || !state.contact_wrenches ||
      !state.ground_contacts || !state.status || !state.clocks || !state.configs ||
      !history.penetration_counts || !history.force_counts ||
      (joint_count != 0 && ((!source.positions && !source.joint_positions) ||
                           (!source.rates && !source.joint_rates) ||
                           !state.positions || !state.rates || !state.motor_torques))) {
    return cudaErrorInvalidValue;
  }
  resetRigidBodyBatch<<<static_cast<unsigned>(blocks), block_size, 0, stream>>>(
      batch, body_count, joint_count, root, source, state, history);
  return cudaGetLastError();
}

auto launchSetRigidBodyDomainParameters(
    std::size_t batch, const double* friction, const double* base_mass_offsets,
    const bool* mask, const RigidBody* nominal_base, double* friction_overrides,
    double* base_inertias, cudaStream_t stream) -> cudaError_t {
  if (batch == 0 || (!friction && !base_mass_offsets)) return cudaSuccess;
  const auto blocks = (batch - 1) / block_size + 1;
  if (blocks > std::numeric_limits<int>::max() || (friction && !friction_overrides) ||
      (base_mass_offsets && (!base_inertias || !nominal_base))) return cudaErrorInvalidValue;
  setDomainParameters<<<static_cast<unsigned>(blocks), block_size, 0, stream>>>(
      batch, friction, base_mass_offsets, mask, nominal_base, friction_overrides, base_inertias);
  return cudaGetLastError();
}

auto launchPushRigidBodyBase(
    std::size_t batch, std::size_t body_count, std::size_t root, std::size_t ground,
    const double* xy_velocity, const double* poses, double* velocities,
    RigidDynamicsStatus* status, cudaStream_t stream) -> cudaError_t {
  if (batch == 0) return cudaSuccess;
  const auto blocks = (batch - 1) / block_size + 1;
  if (blocks > std::numeric_limits<int>::max() || root >= body_count || ground >= body_count ||
      root == ground || !xy_velocity || !poses || !velocities || !status) return cudaErrorInvalidValue;
  pushBase<<<static_cast<unsigned>(blocks), block_size, 0, stream>>>(
      batch, body_count, root, ground, xy_velocity, poses, velocities, status);
  return cudaGetLastError();
}

}  // namespace sire::physics::contact::cuda
