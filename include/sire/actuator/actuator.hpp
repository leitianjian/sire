#ifndef SIRE_ACTUATOR_HPP_
#define SIRE_ACTUATOR_HPP_

#include <sire_lib_export.h>

#include <aris/core/object.hpp>
#include <aris/dynamic/math_matrix.hpp>
#include <aris/dynamic/model.hpp>
#include <aris/dynamic/model_force.hpp>
#include <aris/dynamic/model_joint.hpp>
#include <aris/dynamic/model_motion.hpp>

#include "sire/core/constants.hpp"
#include "sire/core/sire_decl_def_macro.hpp"

namespace sire::actuator {
enum class ControlTarget { Position, Velocity, Acceleration };
template <int IN_SIZE, int OUT_SIZE>
class SIRE_API ActuatorTemplate : public aris::dynamic::Motion {
 public:
  auto virtual dim() const noexcept -> Size override { return OUT_SIZE; }
  auto virtual pSize() const noexcept -> Size override { return IN_SIZE; }
  auto virtual p() const noexcept -> const double* override { return p_; }
  auto virtual setP(const double* p) noexcept -> void override {
    aris::dynamic::s_vc(pSize(), p, this->p_);
  }
  auto virtual getP(double* p) const noexcept -> void override {
    aris::dynamic::s_vc(pSize(), this->p_, p);
  }
  auto virtual vSize() const noexcept -> Size override { return IN_SIZE; }
  auto virtual v() const noexcept -> const double* override { return v_; }
  auto virtual setV(const double* v) noexcept -> void override {
    aris::dynamic::s_vc(vSize(), v, this->v_);
  }
  auto virtual getV(double* v) const noexcept -> void override {
    aris::dynamic::s_vc(vSize(), this->v_, v);
  }
  auto virtual aSize() const noexcept -> Size override { return IN_SIZE; }
  auto virtual a() const noexcept -> const double* override { return a_; }
  auto virtual setA(const double* a) noexcept -> void override {
    aris::dynamic::s_vc(aSize(), a, this->a_);
  }
  auto virtual getA(double* a) const noexcept -> void override {
    aris::dynamic::s_vc(aSize(), this->a_, a);
  }

  auto virtual fSize() const noexcept -> Size override { return dim(); }
  auto virtual f() const noexcept -> const double* override { return f_; }
  auto virtual setF(const double* f) noexcept -> void override {
    aris::dynamic::s_vc(fSize(), f, this->f_);
  }
  auto virtual getF(double* f) const noexcept -> void override {
    aris::dynamic::s_vc(fSize(), this->f(), f);
  }
  auto virtual jointPtr() -> aris::dynamic::Joint* { return jointPtr_; };
  auto virtual setjointPtr(aris::dynamic::Joint* jointPtr) -> void {
    jointPtr_ = jointPtr;
  };
  auto virtual fcePtr() -> aris::dynamic::Force* { return fcePtr_; };
  auto virtual setFcePtr(aris::dynamic::Force* fcePtr) -> void {
    fcePtr_ = fcePtr;
  };

  virtual ~ActuatorTemplate() = default;
  explicit ActuatorTemplate(const std::string& name = "actuator_template",
                            aris::dynamic::Marker* makI = nullptr,
                            aris::dynamic::Marker* makJ = nullptr,
                            Size component_axis = 2,
                            const double* frc_coe = nullptr,
                            double mp_offset = 0.0, double mp_factor = 1.0,
                            bool active = true)
      : Motion(name, makI, makJ, component_axis, frc_coe, mp_offset, mp_factor,
               active) {}

 protected:
  aris::dynamic::Joint* jointPtr_{nullptr};
  aris::dynamic::Force* fcePtr_{nullptr};
  double p_[IN_SIZE]{0.0}, v_[IN_SIZE]{0.0}, a_[IN_SIZE]{0.0},
      f_[OUT_SIZE]{0.0};
};
class SIRE_API ActuatorSISO : public ActuatorTemplate<1, 1> {
 public:
  static auto add2Model(aris::dynamic::Model& model,
                        aris::dynamic::Joint& joint) -> aris::dynamic::Motion& {
    Size dim;
    double pitch{0.0};

    if (dynamic_cast<aris::dynamic::RevoluteJoint*>(&joint)) {
      dim = 5;
    } else if (dynamic_cast<aris::dynamic::ScrewJoint*>(&joint)) {
      dim = 5;
      pitch = dynamic_cast<aris::dynamic::ScrewJoint*>(&joint)->pitch();
    } else if (dynamic_cast<aris::dynamic::PrismaticJoint*>(&joint)) {
      dim = 2;
    } else {
      THROW_FILE_LINE(
          "unsupport joint when ActuatorSISO::add2Model(model, joint)");
    }

    auto& ret = model.motionPool().add<ActuatorSISO>(
        "actuator_" + std::to_string(model.motionPool().size()), joint.makI(),
        joint.makJ(), dim);
    ret.setPitch(pitch);
    return ret;
  }
  auto virtual forward() -> void;
  auto virtual cptOutput(double input) -> double;
  auto virtual fcePtr() -> aris::dynamic::SingleComponentForce* override {
    return dynamic_cast<aris::dynamic::SingleComponentForce*>(this->fcePtr_);
  };
  auto setDesiredValue(double dv) -> void;
  auto setKp(double kp) -> void;
  auto kp() -> double;
  auto setKd(double kd) -> void;
  auto kd() -> double;

  virtual ~ActuatorSISO();
  explicit ActuatorSISO(const std::string& name = "actuator_siso",
                        aris::dynamic::Marker* makI = nullptr,
                        aris::dynamic::Marker* makJ = nullptr,
                        Size component_axis = 2,
                        const double* frc_coe = nullptr, double mp_offset = 0.0,
                        double mp_factor = 1.0, bool active = true,
                        double kp = 50.0, double kd = 2.0);
  ARIS_DECLARE_BIG_FOUR(ActuatorSISO);

 private:
  struct Imp;
  std::unique_ptr<Imp> imp_;
};
}  // namespace sire::actuator
#endif