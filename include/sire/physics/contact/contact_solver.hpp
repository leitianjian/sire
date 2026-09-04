#ifndef SIRE_CONTACT_SOLVER_HPP_
#define SIRE_CONTACT_SOLVER_HPP_

#include <string>

#include <aris/core/expression_calculator.hpp>
#include <aris/core/object.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/material_manager.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"
#include "sire/physics/contact/contact_solver_result.hpp"

namespace sire::physics {
class PhysicsEngine;
namespace contact {
/* contact-based implementation
 */
class SIRE_API ContactSolver {
 public:
  ContactSolver() = default;
  virtual ~ContactSolver() = default;
  ARIS_DECLARE_BIG_FOUR(ContactSolver);

  auto init(physics::PhysicsEngine* engine_ptr) -> void;
  virtual auto doInit(physics::PhysicsEngine* engine_ptr) -> void{};

  auto physicsEnginePtr() -> physics::PhysicsEngine* { return engine_ptr_; };

  /// V3/V5 contact pipeline: "height_field" preserves the existing behavior;
  /// "single_point" records one initial depth per geometry pair and computes
  /// contact end times, without carrying predicted contact states across steps.
  auto setContactModelMode(const std::string& mode) -> void;
  auto contactModelMode() const -> std::string { return contact_model_mode_; }
  auto setContactTimeMethod(const std::string& method) -> void;
  auto contactTimeMethod() const -> std::string { return contact_time_method_; }
  auto singlePointContactMode() const -> bool {
    return contact_model_mode_ == "single_point";
  }

  virtual auto cptContactSolverResult(
      const aris::dynamic::Model* current_state,
      std::vector<common::PenetrationAsPointPair>& penetration_pairs,
      std::vector<std::array<double, 16>>& T_C_vec,
      ContactSolverResult& result) -> void = 0;
  virtual auto debugByRecords() -> nlohmann::json {
    return nlohmann::json();
    // Default implementation does nothing.
    // Derived classes can override this method to implement debugging behavior.
  }

 protected:
  virtual auto supportsSinglePointContactMode() const -> bool { return false; }
  /// Called once per collision batch by V3/V5, including empty batches.
  auto prepareSinglePointContacts(
      std::vector<common::PenetrationAsPointPair>& pairs,
      std::vector<std::array<double, 16>>& frames,
      std::vector<sire::Size>& preserved) -> void;

 private:
  physics::PhysicsEngine* engine_ptr_{nullptr};
  std::string contact_model_mode_{"height_field"};
  std::string contact_time_method_{"exponential"};
};
}  // namespace contact
}  // namespace sire::physics
#endif
