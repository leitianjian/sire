#include "sire/actuator/actuator.hpp"

#include <aris/core/serialization.hpp>
#include <aris/dynamic/model.hpp>

#include "sire/core/sire_assert.hpp"

namespace sire::actuator {
struct ActuatorSISO::Imp {
  ControlTarget ctrlTar_;
  double desiredValue_;
};
ActuatorSISO::ActuatorSISO(const std::string& name, aris::dynamic::Marker* makI,
                           aris::dynamic::Marker* makJ, Size component_axis,
                           const double* frc_coe, double mp_offset,
                           double mp_factor, bool active)
    : ActuatorTemplate<1, 1>(name, makI, makJ, component_axis, frc_coe,
                             mp_offset, mp_factor, active), imp_(std::make_unique<Imp>()) {};
ActuatorSISO::~ActuatorSISO() = default;
auto ActuatorSISO::forward() -> void {
  SIRE_ASSERT(fcePtr() != nullptr);
  // if (fcePtr() != nullptr) {
  if (auto* fce = dynamic_cast<aris::dynamic::SingleComponentForce*>(fcePtr());
      fce != nullptr) {
    fce->setFce(cptOutput(imp_->desiredValue_ - mp()));
  }
  // }
  // force = kp * input + kv * dot_input + bias;
};
auto ActuatorSISO::setDesiredValue(double dv) -> void {
  imp_->desiredValue_ = dv;
}
auto ActuatorSISO::cptOutput(double input) -> double {
  // double force = -500 * (mp()) - 10 * mv();
  double force = 100 * (input) - 2 * mv();
  // force = kp * input + kv * dot_input + bias;
  // std::cout << force << " " << input << " " << imp_->desiredValue_ << " " << mp() << " " << mv() << std::endl;
  return force;
};
// ARIS_DEFINE_BIG_FOUR_CPP(ActuatorSISO);

ARIS_REGISTRATION {
  aris::core::class_<ControlTarget>("GENERAL_CONTROL_TARGET")
      .textMethod(
          [](ControlTarget* target) -> std::string {
            switch (*target) {
              case ControlTarget::Acceleration:
                return "accel";
              case ControlTarget::Velocity:
                return "vel";
              case ControlTarget::Position:
                return "pos";
              default:
                return "pos";
            }
          },
          [](ControlTarget* type, std::string_view name) -> void {
            if (name == "accel") *type = ControlTarget::Acceleration;
            if (name == "acceleration") *type = ControlTarget::Acceleration;
            if (name == "vel") *type = ControlTarget::Velocity;
            if (name == "velocity") *type = ControlTarget::Velocity;
            if (name == "pos") *type = ControlTarget::Position;
            if (name == "position") *type = ControlTarget::Position;
          });

  aris::core::class_<ActuatorSISO>("Actuator").inherit<aris::dynamic::Motion>();
}
}  // namespace sire::actuator