#include "rigid_body_limits_model.hpp"

#include <limits>
#include <stdexcept>

#include "sire/actuator/actuator.hpp"
#include "sire/physics/physics_engine.hpp"

namespace sire::physics::contact::cuda {
namespace {

auto loadLimitMethod(const PhysicsEngine& engine) -> RigidBodyLimitMethod {
  const auto method = engine.jointLimitMethod();
  if (method == "shifted_ncp") return RigidBodyLimitMethod::ShiftedNcp;
  if (method == "projection") return RigidBodyLimitMethod::Projection;
  if (method == "disabled") return RigidBodyLimitMethod::Disabled;
  throw std::invalid_argument("Unsupported CUDA joint-limit method");
}

auto validBounds(double lower, double upper) -> bool {
  constexpr double infinity = std::numeric_limits<double>::infinity();
  return lower <= upper && lower < infinity && upper > -infinity;
}

}  // namespace

auto loadRigidBodyLimitsModel(PhysicsEngine& engine) -> RigidBodyLimitsModel {
  auto* model = engine.currentModel();
  if (!model) {
    throw std::invalid_argument("Initialize Sire before loading the CUDA actuator limits");
  }
  RigidBodyLimitsModel result{};
  result.settings = {
      loadLimitMethod(engine),
      engine.jointLimitActivationMargin(),
      engine.jointLimitRecoveryFactor(),
      engine.jointLimitEmergencyTolerance(),
      engine.jointLimitMaxForce(),
      engine.jointLimitMaxIterations(),
      engine.jointLimitTolerance(),
  };
  auto& motions = model->motionPool();
  result.joints.reserve(motions.size());
  for (auto& motion : motions) {
    auto* actuator = dynamic_cast<actuator::ActuatorSISO*>(&motion);
    if (!actuator || !motion.active() || motion.rotateRange() != 0) {
      throw std::invalid_argument("CUDA actuator limits require active SISO actuators with zero rotate_range");
    }
    const RigidBodyJointLimits limits{
        actuator->minPosition(), actuator->maxPosition(),
        actuator->minForce(), actuator->maxForce(),
    };
    if (!validBounds(limits.min_position, limits.max_position) ||
        !validBounds(limits.min_force, limits.max_force)) {
      throw std::invalid_argument("Invalid position or effort bounds for CUDA actuator");
    }
    result.joints.push_back(limits);
  }
  return result;
}

}  // namespace sire::physics::contact::cuda
