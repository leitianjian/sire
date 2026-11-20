#include "rigid_body_reset.hpp"

#include <limits>

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

}  // namespace sire::physics::contact::cuda
