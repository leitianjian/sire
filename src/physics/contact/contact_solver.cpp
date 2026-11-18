#include "sire/physics/contact/contact_solver.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

#include <aris/core/reflection.hpp>

#include "sire/physics/collision/collision_detection.hpp"
#include "sire/physics/collision/collision_exists_callback.hpp"
#include "sire/physics/physics_engine.hpp"
#include "sire/physics/utils.hpp"
#include "sire/simulator/simulation_loop.hpp"

namespace sire::physics::contact {
ARIS_DEFINE_BIG_FOUR_CPP(ContactSolver);

auto ContactSolver::init(physics::PhysicsEngine* engine_ptr) -> void {
  engine_ptr_ = engine_ptr;
  patch_initial_depth_.clear();
  if ((singlePointContactMode() || patchContactMode()) && engine_ptr_ &&
      engine_ptr_->simLoopPtr()) {
    engine_ptr_->simLoopPtr()->contactPairManager()->clear();
  }
  doInit(engine_ptr);
}

auto ContactSolver::setContactModelMode(const std::string& mode) -> void {
  if (mode != "height_field" && mode != "single_point" &&
      mode != "height_field_patch" && mode != "height_field_prism") {
    throw std::invalid_argument(
        "contact_model_mode must be height_field, height_field_patch, "
        "height_field_prism, or single_point");
  }
  if (mode == "single_point" && !supportsSinglePointContactMode()) {
    throw std::invalid_argument(
        "single_point is supported by PsVsSolver3 and V5-derived solvers");
  }
  if ((mode == "height_field_patch" || mode == "height_field_prism") &&
      !supportsPatchContactMode()) {
    throw std::invalid_argument(
        "HeightField patch/prism mode requires the shared-friction SimpleAdmm "
        "or ShiftedSpectralAdmm solver");
  }
  if (mode == contact_model_mode_) return;
  contact_model_mode_ = mode;
  patch_initial_depth_.clear();
  if (engine_ptr_ && engine_ptr_->simLoopPtr()) {
    engine_ptr_->simLoopPtr()->contactPairManager()->clear();
  }
}

auto ContactSolver::setHeightFieldPrismMaxContacts(int maximum) -> void {
  if (maximum <= 0)
    throw std::invalid_argument("height_field_prism_max_contacts must be positive");
  height_field_prism_max_contacts_ = maximum;
}

auto ContactSolver::setContactTimeMethod(const std::string& method) -> void {
  if (method != "exponential" && method != "polynomial")
    throw std::invalid_argument("contact_time_method must be exponential or polynomial");
  contact_time_method_ = method;
}

auto ContactSolver::setInitialPenetrationPolicy(const std::string& policy)
    -> void {
  InitialPenetrationPolicy requested;
  if (policy == "retain_bounded") {
    requested = InitialPenetrationPolicy::RetainBounded;
  } else if (policy == "eliminate") {
    requested = InitialPenetrationPolicy::Eliminate;
  } else {
    throw std::invalid_argument(
        "initial_penetration_policy must be retain_bounded or eliminate");
  }
  if (requested == initial_penetration_policy_) return;
  initial_penetration_policy_ = requested;
  patch_initial_depth_.clear();
  if (engine_ptr_ && engine_ptr_->simLoopPtr()) {
    engine_ptr_->simLoopPtr()->contactPairManager()->clear();
  }
}

auto ContactSolver::initialPenetrationPolicy() const -> std::string {
  return initial_penetration_policy_ == InitialPenetrationPolicy::Eliminate
             ? "eliminate"
             : "retain_bounded";
}

auto ContactSolver::prepareSinglePointContacts(
    std::vector<common::PenetrationAsPointPair>& pairs,
    std::vector<std::array<double, 16>>& frames,
    std::vector<sire::Size>& preserved) -> void {
  // A newly detected contact may already be noticeably inside the surface
  // when the outer simulation step is large. Previously the whole detected
  // depth became the baseline, so the contact model started at exactly zero
  // compression and did not try to recover any of that penetration. Retain a
  // small, bounded amount for the contact dynamics to resolve. Shallower new
  // contacts keep their complete physical depth.
  constexpr double kNewContactRetainedDepth = 1e-4;
  // These IDs identify geometries, even though ContactPairManager uses the
  // PartId alias. Several geometries on one rigid body remain separate pairs.
  using Key = core::SortedPair<sire::PartId>;
  std::unordered_set<Key> detected;
  for (const auto& pair : pairs) {
    if (!detected.emplace(pair.id_A, pair.id_B).second) {
      throw std::invalid_argument(
          "single_point requires one contact point per geometry pair; pair (" +
          std::to_string(pair.id_A) + ", " + std::to_string(pair.id_B) +
          ") has multiple points. Use height_field mode for this scene.");
    }
  }

  auto& records =
      engine_ptr_->simLoopPtr()->contactPairManager()->contactPairMap();
  for (auto it = records.begin(); it != records.end();) {
    if (detected.count(it->first) == 0) {
      it = records.erase(it);
    } else {
      ++it;
    }
  }
  for (auto& pair : pairs) {
    const double newContactBaseline =
        initial_penetration_policy_ == InitialPenetrationPolicy::Eliminate
            ? pair.depth
            : std::max(0.0, pair.depth - kNewContactRetainedDepth);
    auto entry = records.emplace(
        Key(pair.id_A, pair.id_B),
        core::ContactPairValue(newContactBaseline, false));
    auto& initialDepth = entry.first->second.init_penetration_depth_;
    pair.modifiedDepth = pair.depth - initialDepth;
    // Preserve set6 semantics: a negative corrected depth is filtered this
    // step, while lowering the baseline for the next collision detection.
    if (pair.modifiedDepth < 0) initialDepth = pair.depth;
  }

  cptContactFrame(pairs, frames);
  preserved.clear();
  for (sire::Size i = 0; i < pairs.size(); ++i) {
    if (pairs[i].modifiedDepth < 0) continue;
    if (std::abs(pairs[i].modifiedDepth) < 1e-8) {
      std::array<double, 3> velocity;
      engine_ptr_->cptContactVelocityB2A(pairs[i], frames[i], velocity);
      if (velocity[2] >= 0) continue;
    }
    preserved.push_back(i);
  }
}

auto ContactSolver::updatePatchInitialDepths(
    std::vector<common::PenetrationAsPointPair>& pairs) -> void {
  constexpr double kNewContactRetainedDepth = 1e-4;
  std::unordered_set<std::uint64_t> detected;
  for (auto& pair : pairs) {
    if (pair.contact_id == 0) {
      // Non-manifold contacts retain the legacy bounded-depth treatment.
      pair.modifiedDepth = std::min(pair.depth, 1e-6);
      continue;
    }
    detected.insert(pair.contact_id);
    const double baseline =
        initial_penetration_policy_ == InitialPenetrationPolicy::Eliminate
            ? pair.depth
            : std::max(0.0, pair.depth - kNewContactRetainedDepth);
    auto [it, inserted] = patch_initial_depth_.emplace(pair.contact_id,
                                                        baseline);
    pair.modifiedDepth = pair.depth - it->second;
    if (pair.modifiedDepth < 0) it->second = pair.depth;
  }
  for (auto it = patch_initial_depth_.begin();
       it != patch_initial_depth_.end();) {
    if (detected.count(it->first) == 0)
      it = patch_initial_depth_.erase(it);
    else
      ++it;
  }
}

auto ContactSolver::preparePatchContacts(
    std::vector<common::PenetrationAsPointPair>& pairs,
    std::vector<std::array<double, 16>>& frames,
    std::vector<sire::Size>& preserved) -> void {
  updatePatchInitialDepths(pairs);
  cptContactFrame(pairs, frames);
  preserved.clear();
  for (sire::Size i = 0; i < pairs.size(); ++i) {
    if (pairs[i].modifiedDepth < 0) continue;
    if (std::abs(pairs[i].modifiedDepth) < 1e-8) {
      std::array<double, 3> velocity;
      engine_ptr_->cptContactVelocityB2A(pairs[i], frames[i], velocity);
      if (velocity[2] >= 0) continue;
    }
    preserved.push_back(i);
  }
}
ARIS_REGISTRATION {
  typedef sire::physics::collision::CollisionFilter& (
      ContactSolver::*CollisionFilterPoolFunc)();
  aris::core::class_<ContactSolver>("ContactSolver")
      .prop("contact_model_mode", &ContactSolver::setContactModelMode,
            &ContactSolver::contactModelMode)
      .prop("height_field_prism_max_contacts",
            &ContactSolver::setHeightFieldPrismMaxContacts,
            &ContactSolver::heightFieldPrismMaxContacts)
      .prop("contact_time_method", &ContactSolver::setContactTimeMethod,
            &ContactSolver::contactTimeMethod)
      .prop("initial_penetration_policy",
            &ContactSolver::setInitialPenetrationPolicy,
            &ContactSolver::initialPenetrationPolicy);
}
}  // namespace sire::physics::contact
