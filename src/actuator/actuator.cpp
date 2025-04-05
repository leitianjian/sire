#include "sire/actuator/actuator.hpp"

#include <aris/core/serialization.hpp>
#include <aris/dynamic/model.hpp>

#include "sire/core/sire_assert.hpp"

namespace sire::actuator {
ActuatorSISO::ActuatorSISO(const std::string& name, aris::dynamic::Marker* makI,
                           aris::dynamic::Marker* makJ, Size component_axis,
                           const double* frc_coe, double mp_offset,
                           double mp_factor, bool active)
    : ActuatorTemplate<1, 1>(name, makI, makJ, component_axis, frc_coe,
                             mp_offset, mp_factor, active) {};
auto ActuatorSISO::forward(double input) -> double {
  // force = kp * input + kv * dot_input + bias;
  return 0;
};
ARIS_DEFINE_BIG_FOUR_CPP(ActuatorSISO);

ARIS_REGISTRATION {
  aris::core::class_<ActuatorSISO>("Actuator").inherit<aris::dynamic::Motion>();
}
}  // namespace sire::actuator