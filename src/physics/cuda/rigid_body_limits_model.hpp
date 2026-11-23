#pragma once

#include <cstddef>
#include <vector>

namespace sire::physics {
class PhysicsEngine;
}

namespace sire::physics::contact::cuda {

enum class RigidBodyLimitMethod { Disabled, Projection, ShiftedNcp };

struct RigidBodyJointLimits {
  double min_position;
  double max_position;
  double min_force;
  double max_force;
};

struct RigidBodyLimitSettings {
  RigidBodyLimitMethod method;
  double activation_margin;
  double recovery_factor;
  double emergency_tolerance;
  double max_force;
  std::size_t max_iterations;
  double tolerance;
};

struct RigidBodyLimitsModel {
  RigidBodyLimitSettings settings;
  // Motion pool order, shared with the dynamics and actuator command buffers.
  std::vector<RigidBodyJointLimits> joints;
};

auto loadRigidBodyLimitsModel(PhysicsEngine& engine) -> RigidBodyLimitsModel;

}  // namespace sire::physics::contact::cuda
