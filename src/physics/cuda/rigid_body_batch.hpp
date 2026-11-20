#pragma once

#include <memory>
#include <vector>

#include "rigid_body_control.hpp"
#include "rigid_body_events.hpp"

namespace sire::physics {
class PhysicsEngine;

namespace contact::cuda {

struct RigidBodyBatchState {
  const double* poses;
  const double* velocities;
  const double* positions;
  const double* rates;
  const double* motor_torques;
  const double* contact_wrenches;
  const bool* ground_contacts;
  const RigidDynamicsStatus* status;
  const RigidEventClock* clocks;
};

/// An independent batch of flat Sire scenes. Initialization reads model
/// metadata from the initialized CPU engine once. Simulation buffers are
/// owned by this object and stay on the current CUDA device. Submit steps,
/// resets, and reads on one stream, or explicitly order the caller's streams.
class RigidBodyBatch {
 public:
  RigidBodyBatch(PhysicsEngine& engine, std::size_t batch, double physics_dt,
                 double control_dt, const std::vector<RigidJointControl>& control);
  ~RigidBodyBatch();
  RigidBodyBatch(const RigidBodyBatch&) = delete;
  auto operator=(const RigidBodyBatch&) -> RigidBodyBatch& = delete;

  auto batchSize() const -> std::size_t;
  auto bodyCount() const -> std::size_t;
  auto jointCount() const -> std::size_t;
  auto rootBody() const -> std::size_t;
  auto state() const -> RigidBodyBatchState;

  /// Advance each successful environment to its next control boundary.
  /// Actions are device [batch, joint_count] in policy order. Null means zero
  /// actions. Execution is asynchronous, including contact-refined events.
  void step(const float* actions, cudaStream_t stream);

  /// Reset selected environments from the initialized scene or optional
  /// device values. Null mask selects all. Optional arrays are [batch, 12]
  /// base poses, [batch, 6] base world twists, and [batch, joint_count]
  /// motion-ordered positions and rates, respectively.
  void reset(const bool* mask, const double* base_poses, const double* base_velocities,
             const double* joint_positions, const double* joint_rates, cudaStream_t stream);

 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;
};

}  // namespace contact::cuda
}  // namespace sire::physics
