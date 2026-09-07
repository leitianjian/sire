#include "sire/actuator/actuator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <aris/core/serialization.hpp>
#include <aris/dynamic/model.hpp>

#include "sire/core/sire_assert.hpp"

#include "log/easyloggingConfig.hpp"

namespace sire::actuator {
struct ActuatorSISO::Imp {
  ControlTarget ctrlTar_;
  double desiredValue_{0};
  double appliedValue_{0};
  double minForce_{-std::numeric_limits<double>::infinity()};
  double maxForce_{std::numeric_limits<double>::infinity()};
  double minPosition_{-std::numeric_limits<double>::infinity()};
  double maxPosition_{std::numeric_limits<double>::infinity()};
  // Retained only so older XML files containing kp/kd keep round-tripping.
  // forward() always interprets desiredValue_ as a generalized force.
  double kp_{50.0}, kd_{2.0};
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
  const double applied_value = limitedDesiredValue();
  imp_->appliedValue_ = applied_value;

  if (auto* fce = dynamic_cast<aris::dynamic::SingleComponentForce*>(fcePtr());
      fce != nullptr) {
    fce->setFce(applied_value);
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
  if (!std::isfinite(dv)) {
    throw std::invalid_argument("Actuator desired force must be finite");
  }
  imp_->desiredValue_ = dv;
}
auto ActuatorSISO::desiredValue() -> double {
  return imp_->desiredValue_;
}
auto ActuatorSISO::setMinForce(double value) -> void {
  if (std::isnan(value) || value > imp_->maxForce_) {
    throw std::invalid_argument("Actuator min_force exceeds max_force");
  }
  imp_->minForce_ = value;
}
auto ActuatorSISO::minForce() -> double { return imp_->minForce_; }
auto ActuatorSISO::setMaxForce(double value) -> void {
  if (std::isnan(value) || value < imp_->minForce_) {
    throw std::invalid_argument("Actuator max_force is below min_force");
  }
  imp_->maxForce_ = value;
}
auto ActuatorSISO::maxForce() -> double { return imp_->maxForce_; }
auto ActuatorSISO::limitedDesiredValue() -> double {
  return std::clamp(imp_->desiredValue_, imp_->minForce_, imp_->maxForce_);
}
auto ActuatorSISO::appliedValue() -> double { return imp_->appliedValue_; }
auto ActuatorSISO::setMinPosition(double value) -> void {
  if (std::isnan(value) || value > imp_->maxPosition_) {
    throw std::invalid_argument("Actuator min_position exceeds max_position");
  }
  imp_->minPosition_ = value;
}
auto ActuatorSISO::minPosition() -> double { return imp_->minPosition_; }
auto ActuatorSISO::setMaxPosition(double value) -> void {
  if (std::isnan(value) || value < imp_->minPosition_) {
    throw std::invalid_argument("Actuator max_position is below min_position");
  }
  imp_->maxPosition_ = value;
}
auto ActuatorSISO::maxPosition() -> double { return imp_->maxPosition_; }
auto ActuatorSISO::enforcePositionLimits(double tolerance) -> bool {
  if (!std::isfinite(tolerance) || tolerance < 0.0) {
    throw std::invalid_argument(
        "Actuator position-limit tolerance must be finite and nonnegative");
  }
  double position = mp();
  double velocity = mv();
  if (!std::isfinite(position) || !std::isfinite(velocity)) {
    throw std::runtime_error("Actuator joint state contains NaN or Inf");
  }
  bool changed = false;
  const bool at_or_below_min =
      tolerance == 0.0 ? position <= imp_->minPosition_
                       : position < imp_->minPosition_ - tolerance;
  const bool at_or_above_max =
      tolerance == 0.0 ? position >= imp_->maxPosition_
                       : position > imp_->maxPosition_ + tolerance;
  if (at_or_below_min) {
    if (position < imp_->minPosition_) {
      setMp(imp_->minPosition_);
      changed = true;
    }
    if (velocity < 0.0) {
      setMv(0.0);
      changed = true;
    }
  } else if (at_or_above_max) {
    if (position > imp_->maxPosition_) {
      setMp(imp_->maxPosition_);
      changed = true;
    }
    if (velocity > 0.0) {
      setMv(0.0);
      changed = true;
    }
  }
  return changed;
}
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
      .prop("kd", &ActuatorSISO::setKd, &ActuatorSISO::kd)
      .prop("min_force", &ActuatorSISO::setMinForce, &ActuatorSISO::minForce)
      .prop("max_force", &ActuatorSISO::setMaxForce, &ActuatorSISO::maxForce)
      .prop("min_position", &ActuatorSISO::setMinPosition,
            &ActuatorSISO::minPosition)
      .prop("max_position", &ActuatorSISO::setMaxPosition,
            &ActuatorSISO::maxPosition);
}
}  // namespace sire::actuator
