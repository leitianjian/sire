#ifndef SIRE_CONTACT_SOLVER_HPP_
#define SIRE_CONTACT_SOLVER_HPP_

#include <string>

#include <aris/core/expression_calculator.hpp>
#include <aris/core/object.hpp>
#include <aris/server/control_server.hpp>

#include "sire/core/material_manager.hpp"
#include "sire/physics/common/penetration_as_point_pair.hpp"
#include <unordered_map>
#include <cstdint>
#include "sire/physics/contact/contact_solver_result.hpp"

namespace sire::physics {
class PhysicsEngine;
namespace contact {
enum class InitialPenetrationPolicy {
  Eliminate,
  RetainBounded,
};

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
  /// "height_field_patch" records initial depth per persistent contact ID and
  /// computes coupled contact end times; "single_point" records one initial
  /// depth per geometry pair. "height_field_prism" uses the same patch solver
  /// with one raw candidate per triangular prism. No predicted state crosses steps.
  auto setContactModelMode(const std::string& mode) -> void;
  auto contactModelMode() const -> std::string { return contact_model_mode_; }
  auto setHeightFieldPrismMaxContacts(int maximum) -> void;
  auto heightFieldPrismMaxContacts() const -> int {
    return height_field_prism_max_contacts_;
  }
  auto setContactTimeMethod(const std::string& method) -> void;
  auto contactTimeMethod() const -> std::string { return contact_time_method_; }
  /// Policy for penetration already present when a geometry pair first
  /// appears: "retain_bounded" preserves the legacy bounded overlap, whereas
  /// "eliminate" stores the complete detected depth as the pair baseline.
  auto setInitialPenetrationPolicy(const std::string& policy) -> void;
  auto initialPenetrationPolicy() const -> std::string;
  auto singlePointContactMode() const -> bool {
    return contact_model_mode_ == "single_point";
  }
  auto patchContactMode() const -> bool {
    return contact_model_mode_ == "height_field_patch" ||
           contact_model_mode_ == "height_field_prism";
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
  virtual auto supportsPatchContactMode() const -> bool { return false; }
  /// Called once per collision batch by V3/V5, including empty batches.
  auto prepareSinglePointContacts(
      std::vector<common::PenetrationAsPointPair>& pairs,
      std::vector<std::array<double, 16>>& frames,
      std::vector<sire::Size>& preserved) -> void;
  auto preparePatchContacts(
      std::vector<common::PenetrationAsPointPair>& pairs,
      std::vector<std::array<double, 16>>& frames,
      std::vector<sire::Size>& preserved) -> void;
  /// Applies and maintains the per-contact initial-depth baseline used by
  /// patch mode. Kept separate from frame/velocity filtering so its behavior
  /// can be regression-tested for large-stiffness contact models.
  auto updatePatchInitialDepths(
      std::vector<common::PenetrationAsPointPair>& pairs) -> void;

 private:
  physics::PhysicsEngine* engine_ptr_{nullptr};
  std::string contact_model_mode_{"height_field"};
  int height_field_prism_max_contacts_{50};
  std::string contact_time_method_{"exponential"};
  InitialPenetrationPolicy initial_penetration_policy_{
      InitialPenetrationPolicy::RetainBounded};
  std::unordered_map<std::uint64_t, double> patch_initial_depth_;
};
}  // namespace contact
}  // namespace sire::physics
#endif
