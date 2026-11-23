#pragma once

#include "rigid_body_events.hpp"

namespace sire::physics::contact::cuda {

struct RigidBodyResetSource {
  // Single scene prototype in body and motion order.
  const double* poses;
  const double* velocities;
  const double* positions;
  const double* rates;
  // Optional per-environment overrides. A null mask selects the whole batch.
  const bool* mask;
  const double* base_poses;
  const double* base_velocities;
  const double* joint_positions;
  const double* joint_rates;
};

struct RigidBodyResetState {
  double* poses;
  double* velocities;
  double* positions;
  double* rates;
  double* motor_torques;
  double* contact_wrenches;
  bool* ground_contacts;
  RigidDynamicsStatus* status;
  RigidEventClock* clocks;
  ContactHistoryConfig* configs;
};

/// Reset the selected environments before rebuilding with the same mask.
/// Body poses are [batch, body_count, 12]. Velocities and wrenches are
/// [batch, body_count, 6], contacts [batch, body_count], and motion buffers
/// [batch, joint_count]. Base overrides are [batch, 12] or [batch, 6].
/// Buffers reside on the device. Joint overrides may reuse their corresponding
/// outputs. Prototype and base buffers are otherwise separate from outputs.
auto launchResetRigidBodyBatch(
    std::size_t batch, std::size_t body_count, std::size_t joint_count,
    std::size_t root, RigidBodyResetSource source, RigidBodyResetState state,
    ContactHistory history, cudaStream_t stream) -> cudaError_t;

}  // namespace sire::physics::contact::cuda
