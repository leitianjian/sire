#ifndef SIRE_DYNAMIC_MODEL_FORCE_HPP_
#define SIRE_DYNAMIC_MODEL_FORCE_HPP_

#include <cmath>

#include <sire_lib_export.h>

#include <aris/dynamic/math_matrix.hpp>
#include <aris/dynamic/model_interaction.hpp>

#include "sire/core/constants.hpp"

namespace sire::physics {
/// @defgroup dynamic_model_group 动力学建模模块
/// @{
///
class SIRE_API Force : public aris::dynamic::Interaction {
 public:
  auto virtual cptGlbFs(double* fsI, double* fsJ) const noexcept -> void = 0;

  virtual ~Force() = default;
  explicit Force(const std::string& name = "force",
                 aris::dynamic::Marker* makI = nullptr,
                 aris::dynamic::Marker* makJ = nullptr, bool active = true)
      : Interaction(name, makI, makJ, active) {}
  ARIS_DEFINE_BIG_FOUR(Force);
  auto setFs(double* value) noexcept -> void {
    std::copy(value, value + 6, fce_screw_);
  }
  auto fs() const noexcept -> const double* { return fce_screw_; }
  auto fs() noexcept -> double* { return fce_screw_; }

 private:
  double fce_screw_[6]{0};
};

class SIRE_API GeneralForce final : public Force {
 public:
  auto virtual cptGlbFs(double* fsI, double* fsJ) const noexcept
      -> void override {
    aris::dynamic::s_vc(6, fs(), fsI);
    aris::dynamic::s_vi(6, fs(), fsJ);
  }
  auto setFce(const double* value) noexcept -> void {
    std::copy(value, value + 6, fs());
  }
  auto fce() const noexcept -> const double* { return fs(); }

  virtual ~GeneralForce() = default;
  explicit GeneralForce(const std::string& name = "general_force",
                        aris::dynamic::Marker* makI = nullptr,
                        aris::dynamic::Marker* makJ = nullptr)
      : Force(name, makI, makJ){};
  ARIS_DEFINE_BIG_FOUR(GeneralForce);
};

class SIRE_API SingleComponentForce final : public Force {
 public:
  auto virtual cptGlbFs(double* fsI, double* fsJ) const noexcept
      -> void override;
  auto setComponentAxis(sire::Size id) noexcept -> void {
    component_axis_ = id;
  }
  auto componentAxis() const noexcept -> sire::Size { return component_axis_; }
  auto setFce(double value) noexcept -> void {
    std::fill_n(fs(), 6, 0);
    fs()[component_axis_] = value;
  }
  auto setFce(double value, sire::Size componentID) noexcept -> void {
    this->component_axis_ = componentID;
    setFce(value);
  }
  auto fce() const noexcept -> const double { return fs()[component_axis_]; }

  virtual ~SingleComponentForce() = default;
  explicit SingleComponentForce(
      const std::string& name = "single_component_force",
      aris::dynamic::Marker* makI = nullptr,
      aris::dynamic::Marker* makJ = nullptr, sire::Size componentID = 0);
  ARIS_DEFINE_BIG_FOUR(SingleComponentForce);

 private:
  sire::Size component_axis_;
};

/// @}
}  // namespace sire::physics

#endif
