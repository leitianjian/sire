#include "sire/actuator/actuator.hpp"

#include <aris/core/serialization.hpp>
#include <aris/dynamic/model.hpp>

#include "sire/core/sire_assert.hpp"

#include "log/easyloggingConfig.hpp"

namespace sire::actuator {
struct ActuatorSISO::Imp {
  ControlTarget ctrlTar_;
  double desiredValue_{0};
  double kp_{50.0}, kd_{2.0};  // Proportional and derivative gains
};
ActuatorSISO::ActuatorSISO(const std::string& name, aris::dynamic::Marker* makI,
                           aris::dynamic::Marker* makJ, Size component_axis,
                           const double* frc_coe, double mp_offset,
                           double mp_factor, bool active, double kp, double kd)
    : ActuatorTemplate<1, 1>(name, makI, makJ, component_axis, frc_coe,
                             mp_offset, mp_factor, active),
      imp_(std::make_unique<Imp>()) {
  imp_->kp_ = kp;
  imp_->kd_ = kd;
};
ActuatorSISO::~ActuatorSISO() = default;
auto ActuatorSISO::forward() -> void {
  SIRE_ASSERT(fcePtr() != nullptr);
  // if (fcePtr() != nullptr) {
  if (auto* fce = dynamic_cast<aris::dynamic::SingleComponentForce*>(fcePtr());
      fce != nullptr) {
    fce->setFce(cptOutput(imp_->desiredValue_ - mp()));
    // DLOG(DEBUG) << "ActuatorSISO::forward() called, "
    //             << "desiredValue: " << imp_->desiredValue_ << ", "
    //             << "mp: " << mp() << ", "
    //             << "mv: " << mv() << ", "
    //             << "force: " << cptOutput(imp_->desiredValue_ - mp());
  }
};
auto ActuatorSISO::setKp(double kp) -> void { imp_->kp_ = kp; }
auto ActuatorSISO::kp() -> double { return imp_->kp_; }
auto ActuatorSISO::setKd(double kd) -> void { imp_->kd_ = kd; }
auto ActuatorSISO::kd() -> double { return imp_->kd_; }
auto ActuatorSISO::setDesiredValue(double dv) -> void {
  imp_->desiredValue_ = dv;
}
auto ActuatorSISO::desiredValue() -> double {
  return imp_->desiredValue_;
}
auto ActuatorSISO::cptOutput(double input) -> double {
  // double force = -500 * (mp()) - 10 * mv();
  double force = imp_->kp_ * input - imp_->kd_ * mv();
  // force = kp * input + kv * dot_input + bias;
  // std::cout << force << " " << input << " " << imp_->desiredValue_ << " " <<
  // mp() << " " << mv() << std::endl;
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

  aris::core::class_<ActuatorSISO>("Actuator")
      .inherit<aris::dynamic::Motion>()
      .prop("kp", &ActuatorSISO::setKp, &ActuatorSISO::kp)
      .prop("kd", &ActuatorSISO::setKd, &ActuatorSISO::kd);
}
}  // namespace sire::actuator